//! Unicorn (QEMU's ARM core) as a [`Cpu`] backend.
//!
//! Unicorn is GPL-2.0, so this crate is a development tool and the test oracle for our own core:
//! it is never part of a shipped build (`cargo deny` keeps the GPL inside this crate).
//!
//! Plain memory (flash, SRAM) is owned by the engine and shared with Unicorn by pointer, so both
//! see the same bytes without copies. Every other region is MMIO: Unicorn calls back for each
//! access and the call goes to the [`Bus`] passed to [`Cpu::run`]. Exception returns go to
//! [`Bus::exception_return`] from inside the run, the way the Python reference harness handled
//! them.
//!
//! Budgets: [`Quantum::Block`] (default) counts whole translated blocks and runs several times
//! faster; [`Quantum::Exact`] uses Unicorn's per-instruction counter and reproduces the Python
//! harness quantum for quantum, which is what the regression comparison uses.

// Unicorn is a C library: sharing engine-owned memory and reaching the bus from its callbacks
// needs raw pointers. Every unsafe block says why it is sound.
#![allow(unsafe_code)]

use std::collections::HashMap;
use std::ffi::c_void;
use std::ptr::NonNull;

use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Fault, Region, RegionKind, Size};
use thiserror::Error;
use unicorn_engine::unicorn_const::{Arch, HookType, MemType, Mode, Prot};
use unicorn_engine::{ArmCpuModel, RegisterARM, UcHookId, Unicorn};

/// QEMU's `EXCP_EXCEPTION_EXIT`: the core branched into the `EXC_RETURN` range.
const EXCP_EXCEPTION_EXIT: u32 = 8;
/// Lowest PC Unicorn reports for an exception return.
const EXC_RETURN_MIN: u32 = 0xFFFF_FF00;

/// QEMU exception numbers mapped to an [`Exit`].
const EXCP_UDEF: u32 = 1;
const EXCP_SWI: u32 = 2;
const EXCP_PREFETCH_ABORT: u32 = 3;
const EXCP_DATA_ABORT: u32 = 4;
const EXCP_BKPT: u32 = 7;
const EXCP_INVSTATE: u32 = 18;
const EXCP_UNALIGNED: u32 = 22;
const EXCP_DIVBYZERO: u32 = 23;

/// The xPSR bits a register write-back may change: N, Z, C, V, Q.
const APSR: u32 = 0xF800_0000;

/// How a run measures its instruction budget.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Quantum {
    /// Stop at the first translated-block boundary at or after the budget. Each block's
    /// instructions are counted once (from its Thumb encodings, cached), so `retired` stays exact
    /// and a run overshoots by less than one block. One callback per block.
    #[default]
    Block,
    /// Stop after exactly the budget, with Unicorn's per-instruction counter: identical quanta
    /// to the Python reference harness, at one callback per instruction (expensive on Apple
    /// Silicon, where every Unicorn callback toggles JIT write protection).
    Exact,
}

/// Unicorn refused a setup call.
#[derive(Debug, Clone, Error)]
#[error("unicorn: {0:?}")]
pub struct UnicornError(unicorn_engine::uc_error);

/// State the callbacks share with [`UnicornCpu`].
#[derive(Default)]
struct Shared {
    /// The bus of the current [`Cpu::run`]; `None` outside it.
    bus: Option<NonNull<dyn Bus>>,
    /// `IPSR` as the engine's exception unit keeps it. QEMU never takes an exception here, so
    /// its own `IPSR` stays 0; register reads put this one back.
    ipsr: u32,
    /// How the current run ended early, if it did.
    exit: Option<Exit>,
    /// Watched addresses not reached yet, with their hooks.
    watches: HashMap<u32, UcHookId>,
    /// Hooks of addresses reached during this run; removed once the run ends (a hook must not
    /// free itself from inside its own callback).
    spent: Vec<UcHookId>,
    /// How runs count their budget.
    quantum: Quantum,
    /// Instruction budget of the current run ([`Quantum::Block`]).
    budget: u64,
    /// Instructions of the blocks entered so far in the current run ([`Quantum::Block`]).
    executed: u64,
    /// Instructions per translated block, by `(address, size in bytes)`.
    block_sizes: HashMap<(u32, u32), u32>,
}

impl Shared {
    /// The bus of the current run.
    ///
    /// # Safety
    ///
    /// Only from a callback during [`Cpu::run`], which sets `bus` from a `&mut dyn Bus` that
    /// outlives the run and that nothing else uses while Unicorn executes.
    unsafe fn bus<'b>(&self) -> &'b mut dyn Bus {
        let bus = self.bus.expect("Unicorn callback outside Cpu::run");
        // SAFETY: see the function contract.
        unsafe { &mut *bus.as_ptr() }
    }
}

/// Ends the current run early with `exit` (the first reason wins).
///
/// With [`Quantum::Block`] a yield waits for the next block boundary: Unicorn would stop right
/// at the access, but without per-instruction hooks its PC is only synced at block starts, so a
/// run stopped mid-block would resume at the wrong instruction. The block hook stops instead,
/// where the PC is exact. Faults stop at once (the run is over anyway).
fn stop(uc: &mut Unicorn<'_, Shared>, exit: Exit) {
    let shared = uc.get_data_mut();
    let deferred = shared.quantum == Quantum::Block && exit == Exit::Yield;
    if shared.exit.is_none() {
        shared.exit = Some(exit);
    }
    if !deferred {
        // Stopping only fails when nothing runs, which cannot happen inside a callback.
        let _ = uc.emu_stop();
    }
}

/// Unicorn running a Cortex-M3.
pub struct UnicornCpu {
    uc: Unicorn<'static, Shared>,
    quantum: Quantum,
    regs: CoreRegs,
    synced: CoreRegs,
    retired: u64,
}

impl UnicornCpu {
    /// A Cortex-M3 with `regions` mapped. Unicorn needs every region 4 KiB aligned in base and
    /// size.
    ///
    /// # Safety
    ///
    /// Every [`RegionKind::Memory`] pointer must be valid for reads and writes of its region's
    /// `size` bytes for as long as this value lives and must not move. While [`Cpu::run`]
    /// executes, the memory may only be touched from inside that run's [`Bus`] callbacks (which
    /// is where an engine reads and writes guest RAM).
    pub unsafe fn new(regions: &[Region]) -> Result<Self, UnicornError> {
        // SAFETY: the caller upholds this function's contract, which is `with_quantum`'s.
        unsafe { Self::with_quantum(regions, Quantum::default()) }
    }

    /// [`UnicornCpu::new`] with a choice of [`Quantum`].
    ///
    /// # Safety
    ///
    /// As for [`UnicornCpu::new`].
    pub unsafe fn with_quantum(regions: &[Region], quantum: Quantum) -> Result<Self, UnicornError> {
        let shared = Shared { quantum, ..Shared::default() };
        let mut uc = Unicorn::new_with_data(Arch::ARM, Mode::THUMB | Mode::MCLASS, shared).map_err(UnicornError)?;
        uc.ctl_set_cpu_model(ArmCpuModel::CORTEX_M3 as i32).map_err(UnicornError)?;
        for region in regions {
            let (base, size) = (u64::from(region.base), u64::from(region.size));
            match region.kind {
                RegionKind::Memory { ptr, writable } => {
                    let perms = if writable { Prot::ALL } else { Prot::READ | Prot::EXEC };
                    // SAFETY: the caller guarantees the buffer per this function's contract.
                    unsafe { uc.mem_map_ptr(base, size, perms, ptr.as_ptr().cast::<c_void>()) }
                        .map_err(UnicornError)?;
                }
                RegionKind::Mmio => map_mmio(&mut uc, region.base, region.size)?,
            }
        }
        uc.add_intr_hook(on_interrupt).map_err(UnicornError)?;
        uc.add_mem_hook(HookType::MEM_UNMAPPED | HookType::MEM_WRITE_PROT, 1, 0, on_bad_access)
            .map_err(UnicornError)?;
        if quantum == Quantum::Block {
            uc.add_block_hook(1, 0, on_block).map_err(UnicornError)?;
        }
        let regs = read_regs(&uc);
        Ok(Self { uc, quantum, regs: regs.clone(), synced: regs, retired: 0 })
    }
}

const GPRS: [RegisterARM; 13] = [
    RegisterARM::R0,
    RegisterARM::R1,
    RegisterARM::R2,
    RegisterARM::R3,
    RegisterARM::R4,
    RegisterARM::R5,
    RegisterARM::R6,
    RegisterARM::R7,
    RegisterARM::R8,
    RegisterARM::R9,
    RegisterARM::R10,
    RegisterARM::R11,
    RegisterARM::R12,
];

fn read_reg(uc: &Unicorn<'_, Shared>, reg: RegisterARM) -> u32 {
    // Reads of valid registers of a configured core do not fail.
    uc.reg_read(reg).map_or(0, |value| value as u32)
}

fn write_reg(uc: &mut Unicorn<'_, Shared>, reg: RegisterARM, value: u32) {
    // Writes to valid registers of a configured core do not fail.
    let _ = uc.reg_write(reg, u64::from(value));
}

fn read_regs(uc: &Unicorn<'_, Shared>) -> CoreRegs {
    let mut regs = CoreRegs {
        lr: read_reg(uc, RegisterARM::LR),
        pc: read_reg(uc, RegisterARM::PC) & !1,
        xpsr: read_reg(uc, RegisterARM::XPSR),
        primask: read_reg(uc, RegisterARM::PRIMASK) & 1 != 0,
        faultmask: read_reg(uc, RegisterARM::FAULTMASK) & 1 != 0,
        basepri: read_reg(uc, RegisterARM::BASEPRI) as u8,
        control: read_reg(uc, RegisterARM::CONTROL) as u8,
        ..CoreRegs::default()
    };
    for (slot, reg) in regs.r.iter_mut().zip(GPRS) {
        *slot = read_reg(uc, reg);
    }
    regs.set_sp(read_reg(uc, RegisterARM::SP));
    regs
}

/// Writes the fields of `new` that differ from `old`. Of xPSR only the APSR flags are written,
/// as the Python harness did: IPSR and the IT state stay Unicorn's.
fn write_changes(uc: &mut Unicorn<'_, Shared>, old: &CoreRegs, new: &CoreRegs) {
    for ((&reg, &before), &after) in GPRS.iter().zip(&old.r).zip(&new.r) {
        if before != after {
            write_reg(uc, reg, after);
        }
    }
    let changes = [
        (RegisterARM::SP, old.sp(), new.sp()),
        (RegisterARM::LR, old.lr, new.lr),
        (RegisterARM::PC, old.pc | 1, new.pc | 1),
        (RegisterARM::APSR_NZCVQ, old.xpsr & APSR, new.xpsr & APSR),
        (RegisterARM::PRIMASK, u32::from(old.primask), u32::from(new.primask)),
        (RegisterARM::FAULTMASK, u32::from(old.faultmask), u32::from(new.faultmask)),
        (RegisterARM::BASEPRI, u32::from(old.basepri), u32::from(new.basepri)),
        (RegisterARM::CONTROL, u32::from(old.control), u32::from(new.control)),
    ];
    for (reg, before, after) in changes {
        if before != after {
            write_reg(uc, reg, after);
        }
    }
}

fn map_mmio(uc: &mut Unicorn<'static, Shared>, base: u32, size: u32) -> Result<(), UnicornError> {
    let read = move |uc: &mut Unicorn<'_, Shared>, offset: u64, width: usize| -> u64 {
        let addr = base.wrapping_add(offset as u32);
        let pc = read_reg(uc, RegisterARM::PC);
        // SAFETY: MMIO callbacks only run inside `Cpu::run`, which set the bus.
        let bus = unsafe { uc.get_data().bus() };
        bus.note_pc(pc);
        let value = split(addr, width, |part, size| bus.read(part, size));
        u64::from(finish(uc, bus, value))
    };
    let write = move |uc: &mut Unicorn<'_, Shared>, offset: u64, width: usize, value: u64| {
        let addr = base.wrapping_add(offset as u32);
        let pc = read_reg(uc, RegisterARM::PC);
        // SAFETY: as above.
        let bus = unsafe { uc.get_data().bus() };
        bus.note_pc(pc);
        let result = split(addr, width, |part, size| {
            let lane = (value >> (8 * (part - addr))) as u32 & size.mask();
            bus.write(part, size, lane).map(|()| 0)
        });
        finish(uc, bus, result);
    };
    uc.mmio_map(u64::from(base), u64::from(size), Some(read), Some(write)).map_err(UnicornError)
}

/// One MMIO access, split into byte transfers when Unicorn hands over an unaligned one (the bus
/// contract wants naturally aligned transfers). Read bytes are assembled little-endian.
fn split(addr: u32, width: usize, mut access: impl FnMut(u32, Size) -> Result<u32, BusFault>) -> Result<u32, BusFault> {
    let size = match width {
        1 => Size::Byte,
        2 => Size::Half,
        _ => Size::Word,
    };
    if addr.is_multiple_of(size.bytes()) {
        return access(addr, size);
    }
    (0..size.bytes()).try_fold(0, |value, i| Ok(value | (access(addr + i, Size::Byte)? & 0xFF) << (8 * i)))
}

/// Turns a bus fault into a stop, and honours a yield request after a good access.
fn finish(uc: &mut Unicorn<'_, Shared>, bus: &mut dyn Bus, result: Result<u32, BusFault>) -> u32 {
    match result {
        Ok(value) => {
            if bus.take_yield() {
                stop(uc, Exit::Yield);
            }
            value
        }
        Err(fault) => {
            stop(uc, Exit::Fault(Fault::Bus(fault)));
            0
        }
    }
}

fn on_interrupt(uc: &mut Unicorn<'_, Shared>, intno: u32) {
    let pc = read_reg(uc, RegisterARM::PC);
    if intno == EXCP_EXCEPTION_EXIT && pc >= EXC_RETURN_MIN {
        let mut before = read_regs(uc);
        before.xpsr |= uc.get_data().ipsr;
        let mut regs = before.clone();
        // SAFETY: interrupt callbacks only run inside `Cpu::run`.
        let bus = unsafe { uc.get_data().bus() };
        bus.exception_return(&mut regs, pc | 1);
        write_changes(uc, &before, &regs);
        uc.get_data_mut().ipsr = regs.xpsr & CoreRegs::XPSR_IPSR;
        if bus.take_yield() {
            stop(uc, Exit::Yield);
        }
        return;
    }
    let bus_fault = |access| Fault::Bus(BusFault { addr: pc, access, kind: BusFaultKind::Unmapped });
    let exit = match intno {
        EXCP_SWI => Exit::Svc(0),
        EXCP_BKPT => Exit::Breakpoint(0),
        EXCP_UDEF => Exit::Fault(Fault::Undefined(0)),
        EXCP_INVSTATE => Exit::Fault(Fault::InvalidState),
        EXCP_UNALIGNED => Exit::Fault(Fault::Unaligned(pc)),
        EXCP_DIVBYZERO => Exit::Fault(Fault::DivideByZero),
        EXCP_PREFETCH_ABORT => Exit::Fault(bus_fault(Access::Fetch)),
        EXCP_DATA_ABORT => Exit::Fault(bus_fault(Access::Read)),
        other => Exit::Fault(Fault::Unknown(other)),
    };
    stop(uc, exit);
}

fn on_bad_access(uc: &mut Unicorn<'_, Shared>, kind: MemType, addr: u64, _size: usize, _value: i64) -> bool {
    let (access, fault) = match kind {
        MemType::WRITE_PROT => (Access::Write, BusFaultKind::ReadOnly),
        MemType::WRITE_UNMAPPED => (Access::Write, BusFaultKind::Unmapped),
        MemType::FETCH_UNMAPPED => (Access::Fetch, BusFaultKind::Unmapped),
        _ => (Access::Read, BusFaultKind::Unmapped),
    };
    stop(uc, Exit::Fault(Fault::Bus(BusFault { addr: addr as u32, access, kind: fault })));
    false
}

/// Unicorn calls this before a translated block runs, ahead of the block's own exit check, so a
/// stop requested here takes effect before any of the block's instructions execute.
fn on_block(uc: &mut Unicorn<'_, Shared>, addr: u64, size: u32) {
    let shared = uc.get_data();
    if shared.executed >= shared.budget || shared.exit.is_some() {
        let _ = uc.emu_stop();
        return;
    }
    let key = (addr as u32, size);
    let count = if let Some(&count) = shared.block_sizes.get(&key) {
        count
    } else {
        let count = thumb_instructions(uc, addr, size);
        uc.get_data_mut().block_sizes.insert(key, count);
        count
    };
    uc.get_data_mut().executed += u64::from(count);
}

/// Instructions in `size` bytes of Thumb code at `addr`: a first halfword starting `0b11101`,
/// `0b11110` or `0b11111` opens a 32-bit instruction, anything else is 16-bit.
fn thumb_instructions(uc: &Unicorn<'_, Shared>, addr: u64, size: u32) -> u32 {
    let mut bytes = vec![0; size as usize];
    if uc.mem_read(addr, &mut bytes).is_err() {
        return size / 2;
    }
    let (mut at, mut count) = (0, 0);
    while at + 1 < bytes.len() {
        let first = u16::from_le_bytes([bytes[at], bytes[at + 1]]);
        at += if first >> 11 >= 0b11101 { 4 } else { 2 };
        count += 1;
    }
    count
}

fn on_watch(uc: &mut Unicorn<'_, Shared>, addr: u64, _size: u32) {
    let addr = addr as u32;
    let shared = uc.get_data_mut();
    if let Some(hook) = shared.watches.remove(&addr) {
        shared.spent.push(hook);
        // SAFETY: code hooks only run inside `Cpu::run`.
        unsafe { uc.get_data().bus() }.reached(addr);
    }
}

impl Cpu for UnicornCpu {
    /// Writes only what the Python harness wrote at reset (the masks, CONTROL, MSP, SP, LR, PC),
    /// so the other registers keep their values across a system reset, as on hardware.
    fn reset(&mut self, sp: u32, reset_vector: u32) {
        let reset = CoreRegs::at_reset(sp, reset_vector);
        for (reg, value) in [
            (RegisterARM::PRIMASK, 0),
            (RegisterARM::FAULTMASK, 0),
            (RegisterARM::BASEPRI, 0),
            (RegisterARM::CONTROL, 0),
            (RegisterARM::MSP, reset.msp),
            (RegisterARM::SP, reset.msp),
            (RegisterARM::LR, reset.lr),
            (RegisterARM::PC, reset.pc | 1),
        ] {
            write_reg(&mut self.uc, reg, value);
        }
        self.uc.get_data_mut().ipsr = 0;
        self.regs = read_regs(&self.uc);
        self.synced = self.regs.clone();
    }

    fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit {
        write_changes(&mut self.uc, &self.synced, &self.regs);
        let bus = NonNull::from(bus);
        // SAFETY: the pointer is only used by callbacks during the `emu_start` below and is
        // cleared before this function returns, so it never outlives the borrow; the transmute
        // only erases that borrow's lifetime so the pointer can be stored.
        let bus: NonNull<dyn Bus + 'static> = unsafe { std::mem::transmute(bus) };
        let shared = self.uc.get_data_mut();
        shared.bus = Some(bus);
        shared.ipsr = self.regs.xpsr & CoreRegs::XPSR_IPSR;
        shared.exit = None;
        shared.budget = u64::from(max_instr);
        shared.executed = 0;
        let count = if self.quantum == Quantum::Exact { max_instr as usize } else { 0 };
        let result = self.uc.emu_start(u64::from(self.regs.pc | 1), 0, 0, count);
        let shared = self.uc.get_data_mut();
        shared.bus = None;
        let exit = shared.exit.take();
        let executed = match self.quantum {
            Quantum::Block => shared.executed,
            Quantum::Exact if exit.is_none() => u64::from(max_instr),
            Quantum::Exact => 0,
        };
        for hook in std::mem::take(&mut shared.spent) {
            let _ = self.uc.remove_hook(hook);
        }
        self.regs = read_regs(&self.uc);
        self.regs.xpsr |= self.uc.get_data().ipsr;
        self.synced = self.regs.clone();
        self.retired += executed;
        match (exit, result) {
            (Some(exit), _) => exit,
            (None, Ok(())) => Exit::Budget,
            (None, Err(error)) => Exit::Fault(Fault::Unknown(error as u32)),
        }
    }

    fn regs(&self) -> &CoreRegs {
        &self.regs
    }

    fn regs_mut(&mut self) -> &mut CoreRegs {
        &mut self.regs
    }

    /// Exact with [`Quantum::Block`]. With [`Quantum::Exact`], a run stopped early adds nothing:
    /// Unicorn's counter cannot say how far it got.
    fn retired(&self) -> u64 {
        self.retired
    }

    fn watch(&mut self, addrs: &[u32]) {
        let old: Vec<UcHookId> = self.uc.get_data_mut().watches.drain().map(|(_, hook)| hook).collect();
        for hook in old {
            let _ = self.uc.remove_hook(hook);
        }
        for &addr in addrs {
            if let Ok(hook) = self.uc.add_code_hook(u64::from(addr), u64::from(addr), on_watch) {
                self.uc.get_data_mut().watches.insert(addr, hook);
            }
        }
    }

    fn invalidate(&mut self, addr: u32, len: u32) {
        let _ = self.uc.ctl_remove_cache(u64::from(addr), u64::from(addr) + u64::from(len));
    }
}

#[cfg(test)]
mod tests {
    use euc_core::Lane;

    use super::*;

    const FLASH: u32 = 0x0800_0000;
    const RAM: u32 = 0x2000_0000;
    const MMIO: u32 = 0x4000_0000;

    /// `ldr r0,[r1]; adds r0,#1; str r0,[r1,#4]; bx lr; movs r2,#7; b .; str r0,[r4]`
    const PROGRAM: [u16; 7] = [0x6808, 0x3001, 0x6048, 0x4770, 0x2207, 0xE7FE, 0x6020];

    struct Rig {
        flash: Box<[u8]>,
        ram: Box<[u8]>,
        cpu: UnicornCpu,
    }

    fn rig() -> Rig {
        rig_with(Quantum::Block)
    }

    fn rig_with(quantum: Quantum) -> Rig {
        let mut flash = vec![0xFF; 0x1000].into_boxed_slice();
        for (i, op) in PROGRAM.iter().enumerate() {
            flash[2 * i..2 * i + 2].copy_from_slice(&op.to_le_bytes());
        }
        let mut ram = vec![0; 0x1000].into_boxed_slice();
        let regions = [
            Region {
                base: FLASH,
                size: 0x1000,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut flash[0]), writable: false },
            },
            Region {
                base: RAM,
                size: 0x1000,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut ram[0]), writable: true },
            },
            Region { base: MMIO, size: 0x1000, kind: RegionKind::Mmio },
        ];
        // SAFETY: both buffers are boxed, live in the rig next to the CPU and are only touched
        // between runs.
        let mut cpu = unsafe { UnicornCpu::with_quantum(&regions, quantum) }.unwrap();
        cpu.reset(RAM + 0x800, FLASH | 1);
        let regs = cpu.regs_mut();
        regs.r[1] = MMIO + 4;
        regs.lr = 0xFFFF_FFF9;
        Rig { flash, ram, cpu }
    }

    #[derive(Default)]
    struct TestBus {
        writes: Vec<(u32, Size, u32)>,
        returns: Vec<u32>,
        reached: Vec<u32>,
        yield_on_read: bool,
        yield_pending: bool,
    }

    impl Bus for TestBus {
        fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
            self.yield_pending = self.yield_on_read;
            Ok(Lane::new(addr, size).extract(41))
        }
        fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
            self.writes.push((addr, size, value));
            Ok(())
        }
        fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
            Err(BusFault { addr, access: Access::Fetch, kind: BusFaultKind::NoExecute })
        }
        fn take_yield(&mut self) -> bool {
            std::mem::take(&mut self.yield_pending)
        }
        fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
            self.returns.push(exc_return);
            regs.pc = FLASH + 8;
            regs.r[3] = 99;
        }
        fn reached(&mut self, addr: u32) {
            self.reached.push(addr);
        }
    }

    #[test]
    fn mmio_goes_to_the_bus_and_exception_returns_resume_inline() {
        let mut rig = rig();
        let mut bus = TestBus::default();
        rig.cpu.watch(&[FLASH + 4, FLASH + 0x100]);
        assert_eq!(rig.cpu.run(&mut bus, 20), Exit::Budget);
        assert_eq!(bus.writes, [(MMIO + 8, Size::Word, 42)]);
        assert_eq!(bus.returns, [0xFFFF_FFF9]);
        assert_eq!(bus.reached, [FLASH + 4]);
        let regs = rig.cpu.regs();
        assert_eq!((regs.r[2], regs.r[3], regs.pc), (7, 99, FLASH + 10), "resumed at the bus's PC");
        assert_eq!(rig.cpu.retired(), 20);

        // Reached addresses are reported once.
        rig.cpu.regs_mut().pc = FLASH;
        rig.cpu.regs_mut().lr = 0xFFFF_FFF9;
        rig.cpu.run(&mut bus, 20);
        assert_eq!(bus.reached, [FLASH + 4]);
        assert_eq!(rig.ram.len(), 0x1000);
    }

    #[test]
    fn a_yield_request_ends_the_run_at_the_next_block() {
        let mut rig = rig();
        let mut bus = TestBus { yield_on_read: true, ..TestBus::default() };
        assert_eq!(rig.cpu.run(&mut bus, 20), Exit::Yield);
        assert_eq!(rig.cpu.retired(), 4, "the block that read runs to its end: ldr, adds, str, bx");
        assert_eq!(rig.cpu.regs().pc, FLASH + 8, "stopped where the next block starts");
    }

    #[test]
    fn exact_quanta_count_every_instruction_and_stop_at_the_access() {
        let mut rig = rig_with(Quantum::Exact);
        let mut bus = TestBus::default();
        assert_eq!(rig.cpu.run(&mut bus, 2), Exit::Budget);
        assert_eq!((rig.cpu.retired(), rig.cpu.regs().pc), (2, FLASH + 4), "exactly two instructions");

        let mut rig = rig_with(Quantum::Exact);
        let mut bus = TestBus { yield_on_read: true, ..TestBus::default() };
        assert_eq!(rig.cpu.run(&mut bus, 20), Exit::Yield);
        assert_eq!(rig.cpu.retired(), 0, "an early stop retires nothing Unicorn can count");
    }

    #[test]
    fn block_quanta_end_at_the_first_block_boundary_past_the_budget() {
        let mut rig = rig();
        let mut bus = TestBus::default();
        assert_eq!(rig.cpu.run(&mut bus, 2), Exit::Budget);
        assert_eq!(rig.cpu.retired(), 4, "the first block holds four instructions");
    }

    #[test]
    fn stores_to_flash_and_unmapped_reads_fault() {
        let mut rig = rig();
        let mut bus = TestBus::default();
        rig.cpu.regs_mut().pc = FLASH + 12; // str r0, [r4]
        rig.cpu.regs_mut().r[4] = FLASH + 0x20;
        let exit = rig.cpu.run(&mut bus, 1);
        let fault = BusFault { addr: FLASH + 0x20, access: Access::Write, kind: BusFaultKind::ReadOnly };
        assert_eq!(exit, Exit::Fault(Fault::Bus(fault)));
        assert_eq!(rig.flash[0x20], 0xFF);

        rig.cpu.regs_mut().pc = FLASH;
        rig.cpu.regs_mut().r[1] = 0x6000_0000;
        let fault = BusFault { addr: 0x6000_0000, access: Access::Read, kind: BusFaultKind::Unmapped };
        assert_eq!(rig.cpu.run(&mut bus, 1), Exit::Fault(Fault::Bus(fault)));
    }
}

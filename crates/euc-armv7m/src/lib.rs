//! Our own ARMv7-M interpreter: the Cortex-M3 CPU backend EUC Studio ships.
//!
//! It implements the full Thumb/Thumb-2 integer instruction set of the Cortex-M3 (ARMv7-M
//! without the DSP extension, no FPU) behind [`euc_core::Cpu`]. Unicorn stays a development
//! oracle only (GPL-2.0); this core is verified against it by differential fuzzing and by
//! lock-step on the stock firmware (see `tests/`), and against the ARM Architecture Reference
//! Manual by per-encoding tests.
//!
//! Structure:
//!
//! - [`decode`] (modules `decode16` and `decode32`, following ARM ARM A5.2 and A5.3) turns an
//!   encoding into an [`Op`] with operands extracted and PC-relative addresses resolved; `exec`
//!   runs an [`Op`] (A7, B5.2); `alu` holds the ARM ARM integer functions (`AddWithCarry`,
//!   `Shift_C`, `ThumbExpandImm_C`, saturation); [`it`] the condition codes and `ITSTATE`;
//!   [`timing`] the Cortex-M3 TRM cycle model.
//! - UNDEFINED encodings (including the DSP extension, coprocessor and ARMv7-A-only space) and
//!   UNPREDICTABLE ones both end the run with `Fault::Undefined`, so a decoding gap never runs
//!   silently; compiled firmware contains neither.
//! - Plain memory (flash, SRAM) runs in place with a decode cache of one slot per halfword,
//!   filled on first execution and emptied by [`Cpu::invalidate`] or by the core's own stores
//!   into a block that holds code. MMIO goes to the [`Bus`].
//! - Exception entry, return, priorities and tail-chaining belong to the engine
//!   ([`euc_core::ExceptionUnit`]). The core reports `SVC`, `BKPT`, `WFI`, `WFE` and faults
//!   through [`Exit`], and an `EXC_RETURN` load into PC through [`Bus::exception_return`],
//!   inline, as the trait describes.
//!
//! Behaviour the ARM ARM leaves to the implementation follows the Cortex-M3 TRM r2p1:
//! the local exclusive monitor holds no address (any `STREX` after `LDREX` succeeds), unaligned
//! `LDR`/`STR`/`LDRH`/`STRH` work unless `CCR.UNALIGN_TRP` is set (and reach MMIO as the
//! largest naturally aligned pieces, as the bus interface splits them), and `LDM`, `STM`,
//! `LDRD`, `STRD`, `LDREX` and `STREX` always need word (or natural) alignment.
//!
//! The crate's `README.md` records the verification results and every known difference from
//! Unicorn, with the ARM ARM reference that decides it.

mod alu;
mod decode16;
mod decode32;
mod exec;
pub mod it;
mod mem;
mod op;
pub mod timing;

use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Fault, Region};

pub use mem::LayoutError;
pub use op::{BitOp, Decoded, Dp, Extend, ImmCarry, MulLong, Op, SetFlags, Shift, Width, sysm};

use crate::it::Flags;

/// `CCR.UNALIGN_TRP`: unaligned single loads and stores fault.
pub const CCR_UNALIGN_TRP: u32 = 1 << 3;
/// `CCR.DIV_0_TRP`: `SDIV`/`UDIV` by zero fault instead of returning 0.
pub const CCR_DIV_0_TRP: u32 = 1 << 4;

/// The xPSR bits a run owns: N, Z, C, V, Q, T and the IT state. IPSR stays the engine's.
const XPSR_OWNED: u32 = 0xF800_0000 | CoreRegs::XPSR_T | it::XPSR_MASK;

/// True when `hw1` is the first halfword of a 32-bit instruction (A5.1).
pub const fn is_32bit(hw1: u16) -> bool {
    hw1 >> 11 >= 0b11101
}

/// Decodes the instruction `hw1` (and `hw2` when [`is_32bit`]) at address `pc`.
pub fn decode(hw1: u16, hw2: u16, pc: u32) -> Decoded {
    let (op, len) =
        if is_32bit(hw1) { (decode32::decode32(hw1, hw2, pc), 4) } else { (decode16::decode16(hw1, pc), 2) };
    Decoded { op, len, cycles: timing::base(&op), transfer: timing::is_single_transfer(&op) }
}

/// How an instruction ended when it did not simply retire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stop {
    /// It did not retire (a fault, `BKPT`): PC stays on it.
    Abort(Exit),
    /// It retired and the run ends (`SVC`, `WFI`, `WFE`).
    After(Exit),
    /// It retired by loading this `EXC_RETURN` value into PC.
    ExceptionReturn(u32),
}

/// A Cortex-M3 core.
#[allow(clippy::struct_excessive_bools)] // architectural flags and run state, not a configuration
pub struct Armv7m {
    /// Architectural state between runs; during a run the fields below own R0-R15, the flags
    /// and the IT state, and `regs` keeps the rest (masks, CONTROL, the inactive SP, IPSR).
    regs: CoreRegs,
    /// R0 to R15. R13 is the active stack pointer; R15 reads as the current instruction + 4.
    r: [u32; 16],
    /// Address of the next instruction to run (of the current one while it runs, until it
    /// moves PC).
    pc: u32,
    flags: Flags,
    /// `APSR.Q`, the sticky saturation flag.
    q: bool,
    /// `ITSTATE`.
    it: u8,
    /// `EPSR.T`.
    thumb: bool,
    /// The local exclusive monitor is in the Exclusive Access state.
    exclusive: bool,
    /// The event register (`SEV`, exception entry and return set it; `WFE` consumes it).
    event: bool,
    /// The `CCR` bits the core honours ([`CCR_UNALIGN_TRP`], [`CCR_DIV_0_TRP`]).
    ccr: u32,
    mem: mem::Memory,
    /// Plain region that held the last instruction: the hot loop checks it first.
    code_hint: usize,
    /// Watched addresses not reached yet.
    watches: Vec<u32>,
    /// The current instruction made a bus (MMIO) access: ask the bus whether it wants a yield.
    bus_touched: bool,
    /// The previous instruction was a load or store single (TRM pipelining).
    after_transfer: bool,
    /// PC at the end of the last run, to notice the engine redirecting the core.
    resume_pc: Option<u32>,
    retired: u64,
    cycles: u64,
}

impl Armv7m {
    /// A Cortex-M3 with `regions` mapped. Plain memory regions execute in place; MMIO regions
    /// go to the run's [`Bus`]; every other address is unmapped (a bus fault). Regions must be
    /// word aligned and must not overlap.
    ///
    /// # Safety
    ///
    /// The same contract as `UnicornCpu::new`: every [`euc_core::RegionKind::Memory`] pointer
    /// must be valid for reads and writes of its region's `size` bytes for as long as this value
    /// lives, and must not move. While [`Cpu::run`] executes, the memory may only be touched
    /// from inside that run's [`Bus`] callbacks.
    // The engine lends its memory by pointer (see `mem`), so building the core is unsafe.
    #[allow(unsafe_code)]
    pub unsafe fn new(regions: &[Region]) -> Result<Self, LayoutError> {
        // SAFETY: forwarded from this function's contract.
        let mem = unsafe { mem::Memory::new(regions) }?;
        Ok(Self {
            regs: CoreRegs::default(),
            r: [0; 16],
            pc: 0,
            flags: Flags::default(),
            q: false,
            it: 0,
            thumb: false,
            exclusive: false,
            event: false,
            ccr: 0,
            mem,
            code_hint: 0,
            watches: Vec::new(),
            bus_touched: false,
            after_transfer: false,
            resume_pc: None,
            retired: 0,
            cycles: 0,
        })
    }

    /// Core cycles since the backend was created, by the Cortex-M3 TRM model in [`timing`].
    pub const fn cycles(&self) -> u64 {
        self.cycles
    }

    /// Sets the `CCR` (`0xE000_ED14`) bits the core honours: [`CCR_UNALIGN_TRP`] and
    /// [`CCR_DIV_0_TRP`]. Both are clear at reset.
    pub const fn set_ccr(&mut self, ccr: u32) {
        self.ccr = ccr & (CCR_UNALIGN_TRP | CCR_DIV_0_TRP);
    }

    /// Copies the architectural registers into the run state.
    fn load_regs(&mut self) {
        let regs = &self.regs;
        self.r[..13].copy_from_slice(&regs.r);
        self.r[13] = regs.sp();
        self.r[14] = regs.lr;
        self.pc = regs.pc & !1;
        let x = regs.xpsr;
        self.flags = Flags { n: x >> 31 & 1 == 1, z: x >> 30 & 1 == 1, c: x >> 29 & 1 == 1, v: x >> 28 & 1 == 1 };
        self.q = x >> 27 & 1 == 1;
        self.thumb = x & CoreRegs::XPSR_T != 0;
        self.it = it::from_xpsr(x);
    }

    /// Copies the run state back into the architectural registers.
    fn store_regs(&mut self) {
        let regs = &mut self.regs;
        regs.r.copy_from_slice(&self.r[..13]);
        regs.set_sp(self.r[13]);
        regs.lr = self.r[14];
        regs.pc = self.pc;
        let f = self.flags;
        let apsr = u32::from(f.n) << 31 | u32::from(f.z) << 30 | u32::from(f.c) << 29 | u32::from(f.v) << 28;
        let epsr = u32::from(self.q) << 27 | if self.thumb { CoreRegs::XPSR_T } else { 0 } | it::to_xpsr(self.it);
        regs.xpsr = (regs.xpsr & !XPSR_OWNED) | apsr | epsr;
    }

    /// The decoded instruction at `pc`: from the cache for plain memory (decoding on a miss),
    /// or fetched through the bus and decoded each time elsewhere.
    #[inline(always)]
    #[allow(clippy::inline_always)] // once per instruction; see the note in `exec`
    fn fetch(&mut self, bus: &mut dyn Bus, pc: u32) -> Result<Decoded, Exit> {
        let hint = self.code_hint;
        if let Some(ram) = self.mem.rams.get(hint) {
            let off = pc.wrapping_sub(ram.base);
            if off < ram.size
                && let Some(&slot) = self.mem.code[hint].slots.get((off >> 1) as usize)
                && !matches!(slot.op, Op::Undecoded | Op::Watch)
            {
                return Ok(slot);
            }
        }
        self.fetch_slow(bus, pc)
    }

    /// Cache miss, watched address, region change or a fetch outside plain memory.
    #[cold]
    fn fetch_slow(&mut self, bus: &mut dyn Bus, pc: u32) -> Result<Decoded, Exit> {
        let Some((i, off)) = self.mem.find(pc, 2) else {
            if !self.mem.is_mmio(pc) {
                let fault = BusFault { addr: pc, access: Access::Fetch, kind: BusFaultKind::Unmapped };
                return Err(Exit::Fault(fault.into()));
            }
            // MMIO: whatever the bus fetches (System faults outside flash and SRAM).
            let hw1 = bus.fetch(pc).map_err(|fault| Exit::Fault(fault.into()))?;
            let hw2 =
                if is_32bit(hw1) { bus.fetch(pc.wrapping_add(2)).map_err(|f| Exit::Fault(f.into()))? } else { 0 };
            return Ok(decode(hw1, hw2, pc));
        };
        self.code_hint = i;
        let slot = self.mem.code(i).slots[(off >> 1) as usize];
        match slot.op {
            Op::Watch => {
                self.watches.retain(|&a| a != pc);
                bus.reached(pc);
                self.mem.code[i].slots[(off >> 1) as usize] = Decoded::EMPTY;
                self.fetch_slow(bus, pc)
            }
            Op::Undecoded => {
                let ram = &self.mem.rams[i];
                let hw1 = ram.read(off, 2) as u16;
                let hw2 = if !is_32bit(hw1) {
                    0
                } else if let Some(off2) = ram.offset(pc.wrapping_add(2), 2) {
                    ram.read(off2, 2) as u16
                } else {
                    bus.fetch(pc.wrapping_add(2)).map_err(|fault| Exit::Fault(fault.into()))?
                };
                let decoded = decode(hw1, hw2, pc);
                let watched = !self.watches.is_empty() && self.watches.contains(&pc);
                let code = &mut self.mem.code[i];
                code.mark(off);
                code.mark(off + u32::from(decoded.len) - 1);
                code.slots[(off >> 1) as usize] = if watched { Decoded { op: Op::Watch, ..decoded } } else { decoded };
                if watched { self.fetch_slow(bus, pc) } else { Ok(decoded) }
            }
            _ => Ok(slot),
        }
    }

    /// Runs up to `budget` instructions. The retired count, the base cycles and the pipelining
    /// state live in locals here and reach `self` when the run ends.
    fn execute(&mut self, bus: &mut dyn Bus, budget: u32) -> Exit {
        let mut left = budget;
        let mut cycles = 0_u64;
        let mut after_transfer = self.after_transfer;
        let exit = loop {
            if left == 0 {
                break Exit::Budget;
            }
            if !self.thumb {
                break Exit::Fault(Fault::InvalidState);
            }
            let pc = self.pc;
            let d = match self.fetch(bus, pc) {
                Ok(d) => d,
                Err(exit) => break exit,
            };
            self.r[15] = pc.wrapping_add(4);
            self.pc = pc.wrapping_add(u32::from(d.len));
            let it = self.it;
            let result = if it == 0 || self.flags.passes(it::condition(it)) { self.exec(bus, d.op) } else { Ok(()) };
            if let Err(Stop::Abort(exit)) = result {
                self.pc = pc;
                self.bus_touched = false;
                break exit;
            }
            // The instruction retired.
            if it != 0 {
                self.it = it::advance(it);
            }
            cycles += u64::from(d.cycles) - u64::from(d.transfer && after_transfer);
            after_transfer = d.transfer;
            left -= 1;
            match result {
                Ok(()) => {
                    if self.bus_touched {
                        self.bus_touched = false;
                        if bus.take_yield() {
                            break Exit::Yield;
                        }
                    }
                }
                Err(Stop::After(exit)) => break exit,
                Err(Stop::ExceptionReturn(value)) => {
                    self.bus_touched = false;
                    self.pc = value & !1;
                    self.store_regs();
                    // Valid EXC_RETURN values have bit 0 set; report it set, as the Unicorn
                    // backend does, since a tail-chain puts this value in LR.
                    bus.exception_return(&mut self.regs, value | 1);
                    self.load_regs();
                    // Exception return clears the local monitor and sets the event register.
                    self.exclusive = false;
                    self.event = true;
                    if bus.take_yield() {
                        break Exit::Yield;
                    }
                }
                Err(Stop::Abort(_)) => unreachable!("handled above"),
            }
        };
        self.retired += u64::from(budget - left);
        self.cycles += cycles;
        self.after_transfer = after_transfer;
        exit
    }
}

impl Cpu for Armv7m {
    fn reset(&mut self, sp: u32, reset_vector: u32) {
        self.regs = CoreRegs::at_reset(sp, reset_vector);
        self.exclusive = false;
        self.event = false;
        self.after_transfer = false;
        self.resume_pc = None;
    }

    fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit {
        if self.resume_pc != Some(self.regs.pc) {
            // The engine moved PC between runs (exception entry, reset, a debugger): exception
            // entry clears the local monitor and sets the event register.
            self.exclusive = false;
            self.event = true;
        }
        self.load_regs();
        let exit = self.execute(bus, max_instr);
        self.store_regs();
        self.resume_pc = Some(self.pc);
        exit
    }

    fn regs(&self) -> &CoreRegs {
        &self.regs
    }

    fn regs_mut(&mut self) -> &mut CoreRegs {
        &mut self.regs
    }

    fn retired(&self) -> u64 {
        self.retired
    }

    fn watch(&mut self, addrs: &[u32]) {
        // Empty the slots of the old and the new addresses: decoding re-arms the new ones.
        for addr in std::mem::take(&mut self.watches) {
            self.mem.invalidate(addr, 2);
        }
        for &addr in addrs {
            self.mem.invalidate(addr, 2);
        }
        self.watches = addrs.iter().map(|&addr| addr & !1).collect();
    }

    fn invalidate(&mut self, addr: u32, len: u32) {
        self.mem.invalidate(addr, len);
    }
}

//! Per-encoding behaviour from the ARM Architecture Reference Manual (ARMv7-M, DDI 0403E):
//! flags, shift edge cases, IT blocks, alignment rules, PC-relative addressing, interworking,
//! exception returns, the special registers, and the `Exit` contract of `euc_core::Cpu`.
//!
//! Encodings come from `llvm-mc -triple=thumbv7m-none-eabi -show-encoding`; the comment next to
//! each is its assembly.

#![allow(unsafe_code)]

use std::ptr::NonNull;

use euc_armv7m::{Armv7m, CCR_DIV_0_TRP, CCR_UNALIGN_TRP};
use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Fault, Region, RegionKind, Size};

const FLASH: u32 = 0x0800_0000;
const RAM: u32 = 0x2000_0000;
const MMIO: u32 = 0x4000_0000;
const SIZE: u32 = 0x1_0000;
const N: u32 = 1 << 31;
const Z: u32 = 1 << 30;
const C: u32 = 1 << 29;
const V: u32 = 1 << 28;
const Q: u32 = 1 << 27;
const T: u32 = CoreRegs::XPSR_T;

/// MMIO that reads `0x1000_0000 | addr` and records everything.
#[derive(Default)]
struct TestBus {
    reads: Vec<(u32, Size)>,
    writes: Vec<(u32, Size, u32)>,
    returns: Vec<(u32, u32)>,
    pcs: Vec<u32>,
    reached: Vec<u32>,
    yield_on_access: bool,
    yield_pending: bool,
}

impl Bus for TestBus {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.reads.push((addr, size));
        self.yield_pending = self.yield_on_access;
        Ok((0x1000_0000 | addr) & size.mask())
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.writes.push((addr, size, value));
        self.yield_pending = self.yield_on_access;
        Ok(())
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        Err(BusFault { addr, access: Access::Fetch, kind: BusFaultKind::NoExecute })
    }

    fn take_yield(&mut self) -> bool {
        std::mem::take(&mut self.yield_pending)
    }

    /// Records `(EXC_RETURN, SP)` and resumes at flash + 0x100.
    fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
        self.returns.push((exc_return, regs.sp()));
        regs.pc = FLASH + 0x100;
    }

    fn note_pc(&mut self, pc: u32) {
        self.pcs.push(pc);
    }

    fn reached(&mut self, addr: u32) {
        self.reached.push(addr);
    }
}

/// A core with 64 KiB of flash (code at its start), 64 KiB of RAM and a 64 KiB MMIO window.
struct Rig {
    flash: Box<[u8]>,
    ram: Box<[u8]>,
    cpu: Armv7m,
    bus: TestBus,
}

impl Rig {
    fn new(code: &[u16]) -> Self {
        let mut flash = vec![0; SIZE as usize].into_boxed_slice();
        let mut ram = vec![0; SIZE as usize].into_boxed_slice();
        for (i, h) in code.iter().enumerate() {
            flash[2 * i..2 * i + 2].copy_from_slice(&h.to_le_bytes());
        }
        let regions = [
            Region {
                base: FLASH,
                size: SIZE,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut flash[0]), writable: false },
            },
            Region {
                base: RAM,
                size: SIZE,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut ram[0]), writable: true },
            },
            Region { base: MMIO, size: SIZE, kind: RegionKind::Mmio },
        ];
        // SAFETY: both buffers are boxed, live in the rig next to the core and are only touched
        // between runs.
        let mut cpu = unsafe { Armv7m::new(&regions) }.unwrap();
        cpu.reset(RAM + 0x8000, FLASH | 1);
        Self { flash, ram, cpu, bus: TestBus::default() }
    }

    /// Runs `n` instructions, which must all retire normally.
    fn run(&mut self, n: u32) {
        assert_eq!(self.cpu.run(&mut self.bus, n), Exit::Budget, "at PC 0x{:08X}", self.cpu.regs().pc);
    }

    fn step(&mut self) -> Exit {
        self.cpu.run(&mut self.bus, 1)
    }

    fn r(&self, n: usize) -> u32 {
        self.cpu.regs().reg(n)
    }

    fn set(&mut self, n: usize, value: u32) {
        self.cpu.regs_mut().set_reg(n, value);
    }

    /// N, Z, C, V, Q as xPSR bits.
    fn flags(&self) -> u32 {
        self.cpu.regs().xpsr & 0xF800_0000
    }

    fn set_flags(&mut self, flags: u32) {
        let regs = self.cpu.regs_mut();
        regs.xpsr = (regs.xpsr & !0xF800_0000) | flags;
    }

    fn pc(&self) -> u32 {
        self.cpu.regs().pc
    }

    fn word(&self, addr: u32) -> u32 {
        let at = (addr - RAM) as usize;
        u32::from_le_bytes(self.ram[at..at + 4].try_into().unwrap())
    }

    fn put_word(&mut self, addr: u32, value: u32) {
        let at = (addr - RAM) as usize;
        self.ram[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn put_flash(&mut self, addr: u32, value: u32) {
        let at = (addr - FLASH) as usize;
        self.flash[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }
}

/// `AddWithCarry` flags of ADDS, SUBS, ADCS, SBCS, CMP, CMN, RSBS (A2.2.1, A7.7.3 and siblings).
#[test]
fn add_and_subtract_flags() {
    let case = |code: u16, r0: u32, r1: u32, r2: u32, flags: u32| {
        let mut rig = Rig::new(&[code]);
        rig.set(0, r0);
        rig.set(1, r1);
        rig.set(2, r2);
        rig.set_flags(flags);
        rig.run(1);
        (rig.r(0), rig.flags())
    };
    // adds r0, r1, r2
    assert_eq!(case(0x1888, 0, 0xFFFF_FFFF, 1, 0), (0, Z | C));
    assert_eq!(case(0x1888, 0, 0x7FFF_FFFF, 1, 0), (0x8000_0000, N | V));
    // subs r0, r1, r2: no borrow sets C.
    assert_eq!(case(0x1A88, 0, 5, 5, 0), (0, Z | C));
    assert_eq!(case(0x1A88, 0, 4, 5, 0), (0xFFFF_FFFF, N));
    assert_eq!(case(0x1A88, 0, 0x8000_0000, 1, 0), (0x7FFF_FFFF, C | V));
    // adcs r0, r1 adds the carry; sbcs r0, r1 subtracts NOT carry.
    assert_eq!(case(0x4148, 1, 2, 0, C), (4, 0));
    assert_eq!(case(0x4188, 5, 2, 0, 0), (2, C));
    assert_eq!(case(0x4188, 5, 2, 0, C), (3, C));
    // cmp r0, r1 and cmn r0, r1 keep r0.
    assert_eq!(case(0x4288, 3, 3, 0, 0), (3, Z | C));
    assert_eq!(case(0x42C8, 1, 0xFFFF_FFFF, 0, 0), (1, Z | C));
    // rsbs r0, r1, #0 (NEG).
    assert_eq!(case(0x4248, 0, 1, 0, 0), (0xFFFF_FFFF, N));
    assert_eq!(case(0x4248, 0, 0, 0, 0), (0, Z | C));
    // Q is sticky and untouched by arithmetic.
    assert_eq!(case(0x1888, 0, 1, 1, Q), (2, Q));
}

/// The 16-bit data-processing encodings set flags only outside an IT block (A7.3.3).
#[test]
fn sixteen_bit_encodings_set_flags_only_outside_it_blocks() {
    // it eq; addeq r0, r1, r2
    let mut rig = Rig::new(&[0xBF08, 0x1888]);
    rig.set(1, 0xFFFF_FFFF);
    rig.set(2, 1);
    rig.set_flags(Z);
    rig.run(2);
    assert_eq!(rig.r(0), 0);
    assert_eq!(rig.flags(), Z, "ADD inside the block leaves the flags alone");
}

/// `Shift_C` by a register amount: 0 keeps C, 32 and beyond follow `LSL_C`/`LSR_C`/`ASR_C`,
/// ROR by a multiple of 32 keeps the value and copies bit 31 to C (A7.4.2).
#[test]
fn shifts_by_register_at_the_edges() {
    let case = |code: u16, value: u32, amount: u32, flags: u32| {
        let mut rig = Rig::new(&[code]);
        rig.set(0, value);
        rig.set(1, amount);
        rig.set_flags(flags);
        rig.run(1);
        (rig.r(0), rig.flags() & (N | Z | C))
    };
    let shift_left = 0x4088; // lsls r0, r1
    assert_eq!(case(shift_left, 0x8000_0001, 0, C), (0x8000_0001, N | C), "amount 0 keeps C");
    assert_eq!(case(shift_left, 0x8000_0001, 0x100, 0), (0x8000_0001, N), "only Rm<7:0> counts");
    assert_eq!(case(shift_left, 0x8000_0001, 1, 0), (2, C));
    assert_eq!(case(shift_left, 0x8000_0001, 32, 0), (0, Z | C), "LSL #32 carries bit 0");
    assert_eq!(case(shift_left, 0xFFFF_FFFF, 33, C), (0, Z), "LSL #33 clears C");
    let shift_right = 0x40C8; // lsrs r0, r1
    assert_eq!(case(shift_right, 0x8000_0001, 32, 0), (0, Z | C), "LSR #32 carries bit 31");
    let arith_right = 0x4108; // asrs r0, r1
    assert_eq!(case(arith_right, 0x8000_0000, 40, 0), (0xFFFF_FFFF, N | C));
    let rotate_right = 0x41C8; // rors r0, r1
    assert_eq!(case(rotate_right, 0x8000_0001, 32, 0), (0x8000_0001, N | C));
    assert_eq!(case(rotate_right, 0x0000_0002, 33, C), (1, 0));
    assert_eq!(case(rotate_right, 0x0000_0002, 0, C), (2, C), "ROR by 0 keeps C");
}

/// `DecodeImmShift`: LSR #0 and ASR #0 encode 32, ROR #0 encodes RRX (A7.4.2).
#[test]
fn immediate_shifts_of_32_and_rrx() {
    let mut rig = Rig::new(&[0x0808, 0x1008, 0xEA5F, 0x0031]); // lsrs r0,r1,#32; asrs r0,r1,#32; rrxs r0,r1
    rig.set(1, 0x8000_0001);
    rig.run(1);
    assert_eq!((rig.r(0), rig.flags()), (0, Z | C));
    rig.run(1);
    assert_eq!((rig.r(0), rig.flags()), (0xFFFF_FFFF, N | C));
    rig.set_flags(0);
    rig.run(1);
    assert_eq!((rig.r(0), rig.flags()), (0x4000_0000, C), "RRX shifts C in and bit 0 out");
}

/// `ThumbExpandImm_C`: a rotated constant sets C from its bit 31; an unrotated one keeps C (A5.3.2).
#[test]
fn modified_immediate_carry() {
    let mut rig = Rig::new(&[0xF011, 0x407F, 0xF011, 0x00FF]); // ands r0,r1,#0xFF000000; ands r0,r1,#0xFF
    rig.set(1, 0xFFFF_FFFF);
    rig.run(1);
    assert_eq!((rig.r(0), rig.flags()), (0xFF00_0000, N | C));
    rig.run(1);
    assert_eq!((rig.r(0), rig.flags()), (0xFF, C), "C kept from before");
}

/// ITTE NE: then, then, else; the IT state survives run boundaries in xPSR (A7.3.3).
#[test]
fn it_blocks_select_slots_and_survive_runs() {
    let code = [0xBF1A, 0x2001, 0x2102, 0x2203]; // itte ne; movne r0,#1; movne r1,#2; moveq r2,#3
    let mut rig = Rig::new(&code);
    rig.run(1);
    assert_ne!(rig.cpu.regs().xpsr & 0x0600_FC00, 0, "ITSTATE is visible in xPSR between runs");
    for _ in 0..3 {
        rig.run(1);
    }
    assert_eq!((rig.r(0), rig.r(1), rig.r(2)), (1, 2, 0));
    assert_eq!(rig.cpu.regs().xpsr & 0x0600_FC00, 0, "the block is over");
    assert_eq!(rig.cpu.retired(), 4, "a skipped slot still retires");

    let mut rig = Rig::new(&code);
    rig.set_flags(Z);
    rig.run(4);
    assert_eq!((rig.r(0), rig.r(1), rig.r(2)), (0, 0, 3));
}

/// LDR (literal) and ADR use `Align(PC, 4)` (A7.7.43, A7.7.7).
#[test]
fn literals_align_the_pc() {
    // nop; ldr r0, [pc, #4]; adr r1, #8 — the LDR sits at +2, so PC reads +6 and aligns to +4.
    let mut rig = Rig::new(&[0xBF00, 0x4801, 0xA102]);
    rig.put_flash(FLASH + 8, 0xCAFE_F00D);
    rig.run(3);
    assert_eq!(rig.r(0), 0xCAFE_F00D);
    assert_eq!(rig.r(1), FLASH + 8 + 8, "ADR at +4: Align(+8, 4) + 8");
}

/// Unaligned LDR works unless `CCR.UNALIGN_TRP`; LDM and LDRD always need word alignment
/// (A3.2.1). A faulting instruction does not retire and changes no register.
#[test]
fn alignment_rules() {
    // ldr.w r0, [r1, #1]
    let mut rig = Rig::new(&[0xF8D1, 0x0001]);
    rig.put_word(RAM, 0x4433_2211);
    rig.put_word(RAM + 4, 0x8877_6655);
    rig.set(1, RAM);
    rig.run(1);
    assert_eq!(rig.r(0), 0x5544_3322);

    let mut rig = Rig::new(&[0xF8D1, 0x0001]);
    rig.cpu.set_ccr(CCR_UNALIGN_TRP);
    rig.set(1, RAM);
    assert_eq!(rig.step(), Exit::Fault(Fault::Unaligned(RAM + 1)));
    assert_eq!((rig.pc(), rig.cpu.retired()), (FLASH, 0));

    for code in [[0xE891, 0x000C], [0xE9D1, 0x2300]] {
        // ldm.w r1, {r2, r3}; ldrd r2, r3, [r1]
        let mut rig = Rig::new(&code);
        rig.set(1, RAM + 2);
        rig.set(2, 7);
        assert_eq!(rig.step(), Exit::Fault(Fault::Unaligned(RAM + 2)));
        assert_eq!((rig.r(2), rig.pc()), (7, FLASH));
    }
}

/// 16-bit LDM writes the base back only when the base is not loaded (A7.7.40).
#[test]
fn ldm_write_back() {
    let mut rig = Rig::new(&[0xC906, 0xC90C]); // ldm r1, {r1, r2}; ldm r1!, {r2, r3}
    rig.put_word(RAM, RAM + 0x10);
    rig.put_word(RAM + 4, 5);
    rig.put_word(RAM + 0x10, 6);
    rig.put_word(RAM + 0x14, 7);
    rig.set(1, RAM);
    rig.run(1);
    assert_eq!((rig.r(1), rig.r(2)), (RAM + 0x10, 5), "r1 holds the loaded value");
    rig.run(1);
    assert_eq!((rig.r(1), rig.r(2), rig.r(3)), (RAM + 0x18, 6, 7));
}

/// PUSH/POP with interworking: an odd PC value returns, an even one clears EPSR.T and the next
/// instruction raises INVSTATE (A7.7.99, A2.3.1).
#[test]
fn push_pop_and_interworking() {
    let mut rig = Rig::new(&[0xB510, 0xBD10]); // push {r4, lr}; pop {r4, pc}
    rig.set(4, 44);
    rig.cpu.regs_mut().lr = FLASH + 0x41;
    rig.run(1);
    assert_eq!(rig.r(13), RAM + 0x8000 - 8);
    assert_eq!((rig.word(RAM + 0x7FF8), rig.word(RAM + 0x7FFC)), (44, FLASH + 0x41));
    rig.set(4, 0);
    rig.run(1);
    assert_eq!((rig.r(4), rig.pc(), rig.r(13)), (44, FLASH + 0x40, RAM + 0x8000));

    let mut rig = Rig::new(&[0xBD10]);
    rig.set(13, RAM + 0x100);
    rig.put_word(RAM + 0x104, FLASH + 0x40);
    rig.run(1);
    assert_eq!(rig.cpu.regs().xpsr & T, 0);
    assert_eq!(rig.step(), Exit::Fault(Fault::InvalidState));
}

/// An `EXC_RETURN` loaded by POP or BX is an exception return, reported inline once the
/// instruction completes (SP already updated); MOV PC is a plain branch (B1.5.8).
#[test]
fn exception_returns() {
    let mut rig = Rig::new(&[0xBD10]); // pop {r4, pc}
    rig.set(13, RAM + 0x100);
    rig.put_word(RAM + 0x104, 0xFFFF_FFF9);
    rig.run(1);
    assert_eq!(rig.bus.returns, [(0xFFFF_FFF9, RAM + 0x108)]);
    assert_eq!(rig.pc(), FLASH + 0x100, "resumes where the engine put PC");

    let mut rig = Rig::new(&[0x4770]); // bx lr
    rig.cpu.regs_mut().lr = 0xFFFF_FFF1;
    rig.run(1);
    assert_eq!(rig.bus.returns, [(0xFFFF_FFF1, RAM + 0x8000)]);

    let mut rig = Rig::new(&[0x46F7]); // mov pc, lr
    rig.cpu.regs_mut().lr = 0xFFFF_FFF9;
    rig.run(1);
    assert_eq!(rig.bus.returns, []);
    assert_eq!(rig.pc(), 0xFFFF_FFF8);
    let fault = BusFault { addr: 0xFFFF_FFF8, access: Access::Fetch, kind: BusFaultKind::Unmapped };
    assert_eq!(rig.step(), Exit::Fault(Fault::Bus(fault)));
}

/// BL, BLX, B.W, B<c>.W, CBZ, CBNZ, ADD PC (A7.7.18-A7.7.21, A7.7.24).
#[test]
fn branches() {
    let mut rig = Rig::new(&[0xF000, 0xF804]); // bl #8
    rig.run(1);
    assert_eq!((rig.pc(), rig.cpu.regs().lr), (FLASH + 12, FLASH + 4 + 1));

    let mut rig = Rig::new(&[0xBF00, 0x4798]); // nop; blx r3
    rig.set(3, FLASH + 0x21);
    rig.run(2);
    assert_eq!((rig.pc(), rig.cpu.regs().lr), (FLASH + 0x20, FLASH + 4 + 1));

    let mut rig = Rig::new(&[0xF7FF, 0xBFFE]); // b.w . (to itself)
    rig.run(3);
    assert_eq!(rig.pc(), FLASH);

    for (flags, target) in [(Z, FLASH + 12), (0, FLASH + 4)] {
        let mut rig = Rig::new(&[0xF000, 0x8004]); // beq.w #8
        rig.set_flags(flags);
        rig.run(1);
        assert_eq!(rig.pc(), target);
    }

    for (code, r0, target) in [(0xB120, 0, 12), (0xB120, 1, 2), (0xB920, 0, 2), (0xB920, 1, 12)] {
        let mut rig = Rig::new(&[code]); // cbz/cbnz r0, #8 (from PC + 4)
        rig.set(0, r0);
        rig.run(1);
        assert_eq!(rig.pc(), FLASH + target);
    }

    let mut rig = Rig::new(&[0x4487]); // add pc, r0
    rig.set(0, 0x20);
    rig.run(1);
    assert_eq!(rig.pc(), FLASH + 4 + 0x20);
}

/// TBB and TBH branch forward by twice the table entry from PC + 4 (A7.7.185).
#[test]
fn table_branches() {
    let mut rig = Rig::new(&[0xE8DF, 0xF000, 0x0302]); // tbb [pc, r0]; table: 2, 3
    rig.set(0, 1);
    rig.run(1);
    assert_eq!(rig.pc(), FLASH + 4 + 6);

    let mut rig = Rig::new(&[0xE8DF, 0xF010, 0x0001, 0x0100]); // tbh [pc, r0, lsl #1]; table: 1, 0x100
    rig.set(0, 1);
    rig.run(1);
    assert_eq!(rig.pc(), FLASH + 4 + 0x200);
}

/// MUL, MULS (N and Z only), MLA, MLS and the long multiplies (A7.7.84 and siblings).
#[test]
fn multiplies() {
    let run = |code: &[u16], regs: [u32; 4], flags: u32| {
        let mut rig = Rig::new(code);
        for (n, value) in regs.into_iter().enumerate() {
            rig.set(n, value);
        }
        rig.set_flags(flags);
        rig.run(1);
        ([rig.r(0), rig.r(1)], rig.flags())
    };
    assert_eq!(run(&[0xFB01, 0xF002], [0, 0x1_0001, 0x1_0001, 0], 0).0[0], 0x2_0001); // mul r0, r1, r2
    // muls r0, r1, r0 keeps C and V.
    assert_eq!(run(&[0x4348], [0xFFFF_FFFF, 1, 0, 0], C | V), ([0xFFFF_FFFF, 1], N | C | V));
    assert_eq!(run(&[0xFB01, 0x3002], [0, 3, 4, 5], 0).0[0], 17); // mla r0, r1, r2, r3
    assert_eq!(run(&[0xFB01, 0x3012], [0, 3, 4, 5], 0).0[0], 0xFFFF_FFF9); // mls
    let big = [0, 0, 0xFFFF_FFFF, 2];
    assert_eq!(run(&[0xFBA2, 0x0103], big, 0).0, [0xFFFF_FFFE, 1]); // umull r0, r1, r2, r3
    assert_eq!(run(&[0xFB82, 0x0103], big, 0).0, [0xFFFF_FFFE, 0xFFFF_FFFF]); // smull
    assert_eq!(run(&[0xFBE2, 0x0103], [2, 0, 0xFFFF_FFFF, 2], 0).0, [0, 2]); // umlal
    assert_eq!(run(&[0xFBC2, 0x0103], [2, 0, 0xFFFF_FFFF, 2], 0).0, [0, 0]); // smlal
}

/// UDIV and SDIV round towards zero; division by zero gives 0, or a UsageFault with
/// `CCR.DIV_0_TRP`; `INT_MIN / -1` is `INT_MIN` (A7.7.127, A7.7.195).
#[test]
fn divides() {
    let div = |code: [u16; 2], n: u32, m: u32, ccr: u32| {
        let mut rig = Rig::new(&code);
        rig.cpu.set_ccr(ccr);
        rig.set(1, n);
        rig.set(2, m);
        let exit = rig.step();
        (exit, rig.r(0))
    };
    let (sdiv, udiv) = ([0xFB91, 0xF0F2], [0xFBB1, 0xF0F2]);
    assert_eq!(div(udiv, 7, 2, 0), (Exit::Budget, 3));
    assert_eq!(div(sdiv, -7_i32 as u32, 2, 0), (Exit::Budget, -3_i32 as u32));
    assert_eq!(div(sdiv, 0x8000_0000, u32::MAX, 0), (Exit::Budget, 0x8000_0000));
    assert_eq!(div(udiv, 7, 0, 0), (Exit::Budget, 0));
    assert_eq!(div(sdiv, 7, 0, CCR_DIV_0_TRP).0, Exit::Fault(Fault::DivideByZero));
}

/// SSAT, USAT and the sticky Q flag (A7.7.130, A7.7.205).
#[test]
fn saturation() {
    let sat = |code: [u16; 2], value: u32| {
        let mut rig = Rig::new(&code);
        rig.set(1, value);
        rig.run(1);
        (rig.r(0), rig.flags() & Q)
    };
    let (ssat8, usat8, ssat16_asr4) = ([0xF301, 0x0007], [0xF381, 0x0008], [0xF321, 0x100F]);
    assert_eq!(sat(ssat8, 100), (100, 0));
    assert_eq!(sat(ssat8, 200), (127, Q));
    assert_eq!(sat(ssat8, -200_i32 as u32), (-128_i32 as u32, Q));
    assert_eq!(sat(usat8, 300), (255, Q));
    assert_eq!(sat(usat8, -5_i32 as u32), (0, Q));
    assert_eq!(sat(ssat16_asr4, 0x7FFF_0000), (0x7FFF, Q));
}

/// BFI, BFC, UBFX, SBFX (A7.7.13-14, A7.7.106, A7.7.193).
#[test]
fn bit_fields() {
    let field = |code: [u16; 2], r0: u32, r1: u32| {
        let mut rig = Rig::new(&code);
        rig.set(0, r0);
        rig.set(1, r1);
        rig.run(1);
        rig.r(0)
    };
    assert_eq!(field([0xF361, 0x100B], 0xFFFF_FFFF, 0xAB), 0xFFFF_FABF); // bfi r0, r1, #4, #8
    assert_eq!(field([0xF36F, 0x100B], 0xFFFF_FFFF, 0), 0xFFFF_F00F); // bfc r0, #4, #8
    assert_eq!(field([0xF3C1, 0x1007], 0, 0x0000_0F80), 0xF8); // ubfx r0, r1, #4, #8
    assert_eq!(field([0xF341, 0x1007], 0, 0x0000_0F80), 0xFFFF_FFF8); // sbfx r0, r1, #4, #8
}

/// CLZ, RBIT, REV, REV16, REVSH and the extends with rotation (A7.7.24, A7.7.111-114, A7.7.181).
#[test]
fn bit_operations_and_extends() {
    let one = |code: &[u16], value: u32| {
        let mut rig = Rig::new(code);
        rig.set(1, value);
        rig.run(1);
        rig.r(0)
    };
    assert_eq!(one(&[0xFAB1, 0xF081], 0x0010_0000), 11); // clz
    assert_eq!(one(&[0xFAB1, 0xF081], 0), 32);
    assert_eq!(one(&[0xFA91, 0xF0A1], 1), 0x8000_0000); // rbit
    assert_eq!(one(&[0xBA08], 0x1122_3344), 0x4433_2211); // rev
    assert_eq!(one(&[0xBA48], 0x1122_3344), 0x2211_4433); // rev16
    assert_eq!(one(&[0xBAC8], 0x0000_80FF), 0xFFFF_FF80); // revsh
    assert_eq!(one(&[0xB248], 0x0000_0080), 0xFFFF_FF80); // sxtb
    assert_eq!(one(&[0xB288], 0xFFFF_1234), 0x1234); // uxth
    assert_eq!(one(&[0xFA0F, 0xF091], 0x0080_0000), 0xFFFF_8000); // sxth r0, r1, ror #8
    assert_eq!(one(&[0xFA5F, 0xF0A1], 0x00AB_0000), 0xAB); // uxtb r0, r1, ror #16
}

/// MOVW then MOVT build a 32-bit constant (A7.7.76, A7.7.79).
#[test]
fn movw_movt() {
    let mut rig = Rig::new(&[0xF241, 0x2034, 0xF2C5, 0x6078]);
    rig.set(0, 0xFFFF_FFFF);
    rig.run(2);
    assert_eq!(rig.r(0), 0x5678_1234);
}

/// MRS/MSR: APSR, PRIMASK, BASEPRI and the `BASEPRI_MAX` rule, CONTROL with the stack bank switch,
/// MSP/PSP keeping bits 1:0 clear, and unprivileged writes ignored (B5.2.2, B5.2.3).
#[test]
fn special_registers() {
    // msr apsr_nzcvq, r1; mrs r0, apsr
    let mut rig = Rig::new(&[0xF381, 0x8800, 0xF3EF, 0x8000]);
    rig.set(1, 0xA800_FFFF);
    rig.run(2);
    assert_eq!((rig.flags(), rig.r(0)), (0xA800_0000, 0xA800_0000));

    // msr basepri, r1; msr basepri_max, r1 (twice, with r1 changing between)
    let mut rig = Rig::new(&[0xF381, 0x8811, 0xF381, 0x8812, 0xF381, 0x8812]);
    rig.set(1, 0x40);
    rig.run(1);
    rig.set(1, 0x80);
    rig.run(1);
    assert_eq!(rig.cpu.regs().basepri, 0x40, "BASEPRI_MAX only raises the priority");
    rig.set(1, 0x20);
    rig.run(1);
    assert_eq!(rig.cpu.regs().basepri, 0x20);

    // msr psp, r1; msr control, r1 (SPSEL); then SP is PSP; mrs r0, msp
    let mut rig = Rig::new(&[0xF381, 0x8809, 0xF381, 0x8814, 0xF3EF, 0x8008]);
    rig.set(1, RAM + 0x403);
    rig.run(1);
    assert_eq!(rig.cpu.regs().psp, RAM + 0x400, "bits 1:0 of a stack pointer read as zero");
    rig.set(1, 2);
    rig.run(2);
    assert_eq!((rig.r(13), rig.r(0)), (RAM + 0x400, RAM + 0x8000));

    // Thread mode with CONTROL.nPRIV: msr primask, r1 and cpsid i are ignored.
    let mut rig = Rig::new(&[0xF381, 0x8810, 0xB672]);
    rig.cpu.regs_mut().control = 1;
    rig.set(1, 1);
    rig.run(2);
    assert!(!rig.cpu.regs().primask);

    // cpsid i; cpsie i; cpsid f
    let mut rig = Rig::new(&[0xB672, 0xB662, 0xB671]);
    rig.run(1);
    assert!(rig.cpu.regs().primask);
    rig.run(2);
    assert!(!rig.cpu.regs().primask && rig.cpu.regs().faultmask);
}

/// The local monitor: STREX fails without LDREX, succeeds after one, CLREX and an exception
/// (the engine moving PC) clear it (A3.4).
#[test]
fn exclusive_monitor() {
    // strex r2, r3, [r1]; ldrex r0, [r1]; strex r2, r3, [r1]; ldrex r0, [r1]; clrex; strex r2, r3, [r1]
    let code = [0xE841, 0x3200, 0xE851, 0x0F00, 0xE841, 0x3200, 0xE851, 0x0F00, 0xF3BF, 0x8F2F, 0xE841, 0x3200];
    let mut rig = Rig::new(&code);
    rig.set(1, RAM + 0x10);
    rig.set(3, 0x5A5A);
    rig.run(1);
    assert_eq!((rig.r(2), rig.word(RAM + 0x10)), (1, 0));
    rig.run(2);
    assert_eq!((rig.r(2), rig.word(RAM + 0x10)), (0, 0x5A5A));
    rig.run(3);
    assert_eq!(rig.r(2), 1, "CLREX opened the monitor");

    let mut rig = Rig::new(&[0xE851, 0x0F00, 0xE841, 0x3200]);
    rig.set(1, RAM + 0x10);
    rig.run(1);
    let pc = rig.pc();
    rig.cpu.regs_mut().pc = FLASH + 0x200; // an exception entry, as the engine does it
    rig.cpu.regs_mut().pc = pc;
    rig.run(1);
    assert_eq!(rig.r(2), 0, "the same PC again: no redirect, the monitor stays");
}

/// SVC retires with PC after it; BKPT and UDF do not retire; WFI retires and exits; WFE waits
/// unless the event register is set (SEV) (the `euc_core::Exit` contract).
#[test]
fn exits() {
    let mut rig = Rig::new(&[0xDF2A, 0xBE07, 0xDE03]);
    assert_eq!(rig.step(), Exit::Svc(42));
    assert_eq!(rig.pc(), FLASH + 2);
    assert_eq!(rig.step(), Exit::Breakpoint(7));
    assert_eq!((rig.pc(), rig.cpu.retired()), (FLASH + 2, 1));
    rig.cpu.regs_mut().pc = FLASH + 4;
    assert_eq!(rig.step(), Exit::Fault(Fault::Undefined(0xDE03)));

    let mut rig = Rig::new(&[0xBF00, 0xBF30, 0xBF40, 0xBF20, 0xBF20]); // nop; wfi; sev; wfe; wfe
    rig.run(1);
    assert_eq!(rig.cpu.run(&mut rig.bus, 10), Exit::WaitForInterrupt);
    assert_eq!(rig.pc(), FLASH + 4);
    assert_eq!(rig.cpu.run(&mut rig.bus, 10), Exit::WaitForEvent, "SEV, then WFE consumes the event");
    assert_eq!(rig.pc(), FLASH + 10);
}

/// MMIO goes to the bus with the PC noted; a yield request ends the run after the instruction;
/// unaligned MMIO splits as the Cortex-M3 bus does; unmapped and read-only accesses fault.
#[test]
fn bus_traffic_and_faults() {
    // str r0, [r1]; strb r0, [r1]; strh r0, [r1]; ldrsb r0, [r1, r2]
    let mut rig = Rig::new(&[0x6008, 0x7008, 0x8008, 0x5688]);
    rig.set(0, 0x1234_5678);
    rig.set(1, MMIO + 4);
    rig.set(2, 0x80);
    rig.run(4);
    assert_eq!(
        rig.bus.writes,
        [(MMIO + 4, Size::Word, 0x1234_5678), (MMIO + 4, Size::Byte, 0x78), (MMIO + 4, Size::Half, 0x5678)]
    );
    assert_eq!(rig.bus.reads, [(MMIO + 0x84, Size::Byte)]);
    assert_eq!(rig.r(0), 0xFFFF_FF84, "sign-extended byte");
    assert_eq!(rig.bus.pcs, [FLASH, FLASH + 2, FLASH + 4, FLASH + 6]);

    let mut rig = Rig::new(&[0xF8D1, 0x0001]); // ldr.w r0, [r1, #1]
    rig.set(1, MMIO);
    rig.run(1);
    assert_eq!(rig.bus.reads, [(MMIO + 1, Size::Byte), (MMIO + 2, Size::Half), (MMIO + 4, Size::Byte)]);

    let mut rig = Rig::new(&[0x6008, 0x6008]);
    rig.bus.yield_on_access = true;
    rig.set(1, MMIO);
    assert_eq!(rig.cpu.run(&mut rig.bus, 10), Exit::Yield);
    assert_eq!((rig.pc(), rig.cpu.retired()), (FLASH + 2, 1), "the store retired first");

    let mut rig = Rig::new(&[0x6008]);
    rig.set(1, FLASH + 0x100);
    let fault = BusFault { addr: FLASH + 0x100, access: Access::Write, kind: BusFaultKind::ReadOnly };
    assert_eq!(rig.step(), Exit::Fault(Fault::Bus(fault)));
    rig.set(1, 0x6000_0000);
    let fault = BusFault { addr: 0x6000_0000, access: Access::Write, kind: BusFaultKind::Unmapped };
    assert_eq!(rig.step(), Exit::Fault(Fault::Bus(fault)));
    assert_eq!(rig.cpu.retired(), 0);
}

/// Code in RAM: the core sees its own stores to decoded code at once; the engine's writes after
/// `Cpu::invalidate`.
#[test]
fn self_modifying_code() {
    // At RAM: movs r0, #1; bx lr. The flash program stores `movs r0, #2` over the first one,
    // then calls it twice around an engine write.
    let mut rig = Rig::new(&[0x6011, 0x47A0, 0x6019, 0x47A0]); // str r1,[r2]; blx r4; str r1,[r3]; blx r4
    rig.put_word(RAM, 0x4770_2001);
    rig.set(1, 0x4770_2002);
    rig.set(2, RAM);
    rig.set(3, RAM + 0x100);
    rig.set(4, RAM | 1);
    rig.cpu.regs_mut().pc = RAM;
    rig.cpu.regs_mut().lr = FLASH | 1;
    rig.run(2);
    assert_eq!((rig.r(0), rig.pc()), (1, FLASH), "decoded and ran the RAM code");
    rig.run(4);
    assert_eq!(rig.r(0), 2, "our own store replaced the decoded instruction");
    rig.put_word(RAM, 0x4770_2003);
    rig.cpu.invalidate(RAM, 4);
    rig.cpu.regs_mut().pc = RAM;
    rig.run(1);
    assert_eq!(rig.r(0), 3, "the engine's write, after invalidate");
}

/// Watched addresses are reported once, when first reached; an empty list stops watching.
#[test]
fn watches() {
    let mut rig = Rig::new(&[0xBF00, 0xBF00, 0xE7FC]); // nop; nop; b back to the first nop
    rig.cpu.watch(&[FLASH + 2, FLASH + 0x40]);
    rig.run(9);
    assert_eq!(rig.bus.reached, [FLASH + 2]);
    assert_eq!(rig.cpu.retired(), 9, "watching costs no instructions");
}

/// A run retires exactly its budget; cycles follow the Cortex-M3 TRM table.
#[test]
fn budget_and_cycles() {
    // movs r0, #1 (1); ldr r1, [r2] (2); ldr r3, [r2] (1, pipelined); b . (1 + P = 3)
    let mut rig = Rig::new(&[0x2001, 0x6811, 0x6813, 0xE7FE]);
    rig.set(2, RAM);
    rig.run(4);
    assert_eq!((rig.cpu.retired(), rig.cpu.cycles()), (4, 7));
    rig.run(1000);
    assert_eq!(rig.cpu.retired(), 1004);
}

/// Writes to SP clear bits 1:0 (B1.4.1); ADD/SUB SP immediates.
#[test]
fn stack_pointer_writes() {
    let mut rig = Rig::new(&[0x4685, 0xB001, 0xB081]); // mov sp, r0; add sp, #4; sub sp, #4
    rig.set(0, RAM + 0x123);
    rig.run(1);
    assert_eq!(rig.r(13), RAM + 0x120);
    rig.run(1);
    assert_eq!(rig.r(13), RAM + 0x124);
    rig.run(1);
    assert_eq!(rig.r(13), RAM + 0x120);
}

/// Pre- and post-indexed loads and stores write the base back (A7.7.42, A7.7.158).
#[test]
fn indexed_addressing() {
    // ldr r0, [r1, #4]!; ldr r0, [r1], #-4; str r0, [r1, #-8]!
    let mut rig = Rig::new(&[0xF851, 0x0F04, 0xF851, 0x0904, 0xF841, 0x0D08]);
    rig.put_word(RAM + 0x14, 0xAA);
    rig.set(1, RAM + 0x10);
    rig.run(1);
    assert_eq!((rig.r(0), rig.r(1)), (0xAA, RAM + 0x14));
    rig.run(1);
    assert_eq!((rig.r(0), rig.r(1)), (0xAA, RAM + 0x10));
    rig.run(1);
    assert_eq!((rig.r(1), rig.word(RAM + 8)), (RAM + 8, 0xAA));
}

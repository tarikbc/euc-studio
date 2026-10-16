//! Differential fuzzing of the core against Unicorn (QEMU's Cortex-M3), one instruction at a time.
//!
//! Each case draws a random register file and flags, random RAM, and an instruction: half the
//! time a fully random encoding, half the time one from a template per encoding group of the ARM
//! ARM (A5.2, A5.3) with its operand fields random. A case may put the instruction in an IT block
//! (alone or with up to three more), or pair `LDREX` with `STREX`. Both cores run it from the same
//! state; the test compares the exit, every register, the flags and IT state, RAM, and the MMIO
//! traffic.
//!
//! Where Unicorn and the ARM ARM disagree the ARM ARM decides. Those divergence classes are listed
//! in [`EXPLAINED`] with the reason; any other divergence fails the test.
//!
//! ```text
//! cargo test -p euc-armv7m --features unicorn --release --test fuzz_unicorn -- --nocapture
//! EUC_FUZZ_CASES=10000000 EUC_FUZZ_SEED=0x1234 cargo test ...   # more cases, a fixed seed
//! ```

#![allow(unsafe_code)]

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::ptr::NonNull;
use std::sync::Mutex;

use euc_armv7m::{Armv7m, Dp, Op, decode, is_32bit, it};
use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Fault, Region, RegionKind, Size};
use euc_unicorn::{Quantum, UnicornCpu};

const CODE: u32 = 0x0800_0000;
const CODE_SIZE: u32 = 0x1_0000;
const RAM: u32 = 0x2000_0000;
const RAM_SIZE: u32 = 0x4000;
const MMIO: u32 = 0x4000_0000;
const MMIO_SIZE: u32 = 0x1000;
/// Where the fuzz bus sends an exception return.
const RETURN_TO: u32 = CODE + 0x40;
/// Four NOPs that close an IT block left open in Unicorn (cases never write below `CODE + 0x100`).
const FLUSH: u32 = CODE + 0x10;

/// Divergence classes where Unicorn (QEMU 5) departs from the ARM ARM, or where its harness API
/// cannot observe the state, with the reason. A class is explained when it contains every
/// fragment of an entry. Anything else fails the test.
const EXPLAINED: &[(&[&str], &str)] = &[
    (&["Undefined [DSP]"], "DSP extension (ARMv7E-M) encodings are UNDEFINED on a Cortex-M3; QEMU 5's M3 runs them"),
    (
        &["Undefined [coprocessor]"],
        "the Cortex-M3 has no coprocessor or FPU (NOCP UsageFault); QEMU 5 runs VFP loads/stores",
    ),
    (&["Undefined [SIMD]"], "Advanced SIMD element load/store (1111 1001 xxx0) is ARMv7-A only; QEMU 5 runs it"),
    (&["Undefined [BLX imm]"], "BLX (immediate) needs ARM state, absent on ARMv7-M (UNDEFINED); QEMU 5 branches"),
    (&["Undefined [store Rn=PC]"], "stores with Rn = PC are UNDEFINED (A7.7.158-163); QEMU 5 stores"),
    (
        &["Unpredictable: exit", "vs Undefined"],
        "an UNPREDICTABLE encoding reached through a branch: our core faults on it by design, QEMU 5 runs it",
    ),
    (
        &["[isb in IT]"],
        "QEMU ends the translation block at an ISB inside an IT block and loses the IT state, so later slots run \
         unconditionally",
    ),
    (&["[ends in IT block]"], "the last step opened an IT block, and Unicorn does not stop inside one"),
    (
        &["[sp low bits]"],
        "SP written with bits 1:0 set: ARMv7-M keeps them zero (B1.4.1, CoreRegs::set_sp); QEMU 5 keeps them on ADD/MOV/LDR \
         into SP and on SP write-back, so later SP-relative results differ by 1 to 3",
    ),
    (&["Undefined [BXJ/SUBS PC]"], "BXJ and SUBS PC, LR are ARMv7-A/R only, UNDEFINED on ARMv7-M; QEMU 5 runs them"),
    (&["CONTROL[3:2]"], "Unicorn keeps CONTROL[3:2] (FPCA/SFPA); without the FP extension they are RAZ/WI (B1.4.4)"),
    (
        &["[undefined ahead]"],
        "Unicorn translates the next block before it stops and reports an UNDEFINED encoding later in it as an error of \
         this run; our core stops at the budget",
    ),
    (&["Undefined [SETEND]"], "SETEND is ARMv6/ARMv7-A only, UNDEFINED on ARMv7-M; QEMU 5 runs it"),
    (&["Undefined [exclusive doubleword]"], "LDREXD/STREXD are ARMv7-A only, UNDEFINED on ARMv7-M; QEMU 5 runs them"),
    (
        &["Undefined [ARMv8 acquire/release]"],
        "LDA/STL and their exclusive forms are ARMv8 only, UNDEFINED on ARMv7-M; QEMU 5 runs them",
    ),
    (
        &["Ldm: exit", "vs Unaligned"],
        "LDM/POP use MemA (A3.2.1): unaligned faults before any access; QEMU 5 does not check",
    ),
    (
        &["Stm: exit", "vs Unaligned"],
        "STM/PUSH use MemA (A3.2.1): unaligned faults before any access; QEMU 5 does not check",
    ),
    (
        &["Dual: exit", "vs Unaligned"],
        "LDRD/STRD use MemA (A3.2.1): unaligned faults before any access; QEMU 5 does not check",
    ),
    (
        &["Ldrex: exit", "vs Unaligned"],
        "LDREX checks alignment before the access (A3.4.3); QEMU 5 reports the bus fault first",
    ),
    (
        &["Strex: exit", "vs Unaligned"],
        "STREX checks alignment before the monitor and the access (A3.4.3); QEMU 5 does not",
    ),
    (
        &["after a fault in an IT block", ": pc"],
        "Unicorn reports PC past the IT block when an instruction in it faults; it stays on the faulting one",
    ),
    (
        &["exit Budget vs WaitForInterrupt"],
        "the Unicorn backend cannot report WFI; the Cpu trait asks for Exit::WaitForInterrupt",
    ),
    (
        &["[yield/wfe]", "exit Undefined vs"],
        "QEMU raises EXCP_YIELD for YIELD/WFE, which Unicorn ends as an error; YIELD is a hint, WFE with the event register set continues",
    ),
    (
        &["[msr control]", ": masks"],
        "Unicorn keeps CONTROL[3:2] (FPCA/SFPA, RAZ/WI without the FP extension) and reads masks as 0 once nPRIV is set",
    ),
    (&["[msr sp]"], "QEMU 5's MSR MSP/PSP keeps bits 1:0; ARMv7-M writes R[n]<31:2>:'00' (B5.2.3)"),
    (
        &["[exclusive]", "Strex"],
        "the Cortex-M3 local monitor holds no address, so STREX after any LDREX stores; QEMU tags the address",
    ),
    (
        &["[exclusive]", ": r"],
        "the Cortex-M3 local monitor holds no address, so STREX after any LDREX stores; QEMU tags the address",
    ),
    (
        &["[exc_return range]"],
        "PC loaded with 0xF0000000-0xFFFFFEFF is EXC_RETURN in handler mode (B1.5.8); QEMU starts the range at \
         0xFF000000 and the Unicorn backend reports only 0xFFFFFF00 and up",
    ),
    (
        &["INVSTATE or fetch fault"],
        "PC loaded with an even, unmapped address: QEMU raises the fetch fault, our core INVSTATE first; both fault",
    ),
];

/// Why `class` is a known Unicorn difference, if it is one.
fn explanation(class: &str) -> Option<&'static str> {
    EXPLAINED.iter().find(|(parts, _)| parts.iter().all(|part| class.contains(part))).map(|(_, why)| *why)
}

/// A small, fast, seedable generator (xoshiro256**).
struct Rng([u64; 4]);

impl Rng {
    fn new(seed: u64) -> Self {
        let mut z = seed;
        let mut next = || {
            z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            x ^ (x >> 31)
        };
        Self([next(), next(), next(), next()])
    }

    fn next(&mut self) -> u64 {
        let s = &mut self.0;
        let result = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        result
    }

    fn u32(&mut self) -> u32 {
        (self.next() >> 32) as u32
    }

    fn below(&mut self, n: u32) -> u32 {
        (((self.next() >> 32) * u64::from(n)) >> 32) as u32
    }

    fn chance(&mut self, percent: u32) -> bool {
        self.below(100) < percent
    }
}

/// What the bus saw, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    Read(u32, Size),
    Write(u32, Size, u32),
    Return(u32),
}

/// MMIO that answers reads with a hash of the address and records everything.
#[derive(Default)]
struct FuzzBus {
    log: Vec<Event>,
    salt: u32,
}

impl FuzzBus {
    fn byte(&self, addr: u32) -> u32 {
        (addr ^ self.salt).wrapping_mul(0x9E37_79B1).rotate_left(13).wrapping_mul(0x85EB_CA6B) >> 24
    }

    /// The MMIO traffic at byte level: bytes read, and the last value written to each byte.
    /// How an access was split into bus transfers does not show here.
    fn effects(&self) -> (std::collections::BTreeSet<u32>, BTreeMap<u32, u8>, Vec<u32>) {
        let (mut read, mut written, mut returns) = (std::collections::BTreeSet::new(), BTreeMap::new(), Vec::new());
        for event in &self.log {
            match *event {
                Event::Read(addr, size) => read.extend((0..size.bytes()).map(|i| addr + i)),
                Event::Write(addr, size, value) => {
                    for i in 0..size.bytes() {
                        written.insert(addr + i, (value >> (8 * i)) as u8);
                    }
                }
                Event::Return(value) => returns.push(value),
            }
        }
        (read, written, returns)
    }
}

impl Bus for FuzzBus {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.log.push(Event::Read(addr, size));
        // Every byte has its own value, so any split of an access reads the same bytes.
        Ok((0..size.bytes()).fold(0, |value, i| value | self.byte(addr + i) << (8 * i)))
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.log.push(Event::Write(addr, size, value));
        Ok(())
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        Err(BusFault { addr, access: Access::Fetch, kind: BusFaultKind::NoExecute })
    }

    fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
        self.log.push(Event::Return(exc_return));
        regs.pc = RETURN_TO;
    }
}

/// One core with its own copy of code and RAM.
struct Side {
    code: Box<[u8]>,
    ram: Box<[u8]>,
    cpu: Box<dyn Cpu>,
    bus: FuzzBus,
}

impl Side {
    fn new(code: &[u8], unicorn: bool) -> Self {
        let mut code = code.to_vec().into_boxed_slice();
        let mut ram = vec![0; RAM_SIZE as usize].into_boxed_slice();
        let regions = [
            Region {
                base: CODE,
                size: CODE_SIZE,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut code[0]), writable: false },
            },
            Region {
                base: RAM,
                size: RAM_SIZE,
                kind: RegionKind::Memory { ptr: NonNull::from(&mut ram[0]), writable: true },
            },
            Region { base: MMIO, size: MMIO_SIZE, kind: RegionKind::Mmio },
        ];
        // SAFETY: both buffers are boxed, live next to the CPU in this struct, and are only
        // written between runs.
        let cpu: Box<dyn Cpu> = if unicorn {
            Box::new(unsafe { UnicornCpu::with_quantum(&regions, Quantum::Exact) }.unwrap())
        } else {
            Box::new(unsafe { Armv7m::new(&regions) }.unwrap())
        };
        Self { code, ram, cpu, bus: FuzzBus::default() }
    }

    /// Writes `halves` at `pc` and drops what either core decoded there.
    fn place(&mut self, pc: u32, halves: &[u16]) {
        for (i, h) in halves.iter().enumerate() {
            let at = (pc - CODE) as usize + 2 * i;
            self.code[at..at + 2].copy_from_slice(&h.to_le_bytes());
        }
        self.cpu.invalidate(pc, 2 * halves.len() as u32);
    }
}

/// The instruction groups of the ARM ARM as `(name, fixed-bit mask, value)`; the other bits are
/// random. 16-bit entries have a zero high halfword.
const TEMPLATES: &[(&str, u32, u32)] = &[
    ("shift imm", 0xE000, 0x0000),
    ("add/sub reg/imm3", 0xF800, 0x1800),
    ("mov/cmp/add/sub imm8", 0xE000, 0x2000),
    ("data processing", 0xFC00, 0x4000),
    ("special/bx", 0xFC00, 0x4400),
    ("bx/blx", 0xFF00, 0x4700),
    ("ldr literal", 0xF800, 0x4800),
    ("load/store reg", 0xF000, 0x5000),
    ("load/store imm", 0xE000, 0x6000),
    ("ldrh/strh imm", 0xF000, 0x8000),
    ("load/store sp", 0xF000, 0x9000),
    ("adr/add sp", 0xF000, 0xA000),
    ("add/sub sp", 0xFF00, 0xB000),
    ("cbz/cbnz", 0xF500, 0xB100),
    ("extend", 0xFF00, 0xB200),
    ("push", 0xFE00, 0xB400),
    ("cps", 0xFFE8, 0xB660),
    ("rev", 0xFF00, 0xBA00),
    ("pop", 0xFE00, 0xBC00),
    ("it/hints", 0xFF00, 0xBF00),
    ("ldm/stm", 0xF000, 0xC000),
    ("b cond/svc/udf", 0xF000, 0xD000),
    ("b", 0xF800, 0xE000),
    ("stm.w/ldm.w", 0xFE40_0000, 0xE800_0000),
    ("strex", 0xFFF0_0000, 0xE840_0000),
    ("ldrex", 0xFFF0_0F00, 0xE850_0F00),
    ("ldrd/strd", 0xFE40_0000, 0xE840_0000),
    ("strexb/h", 0xFFF0_0FE0, 0xE8C0_0F40),
    ("tbb/tbh", 0xFFF0_FFE0, 0xE8D0_F000),
    ("ldrexb/h", 0xFFF0_0FEF, 0xE8D0_0F4F),
    ("dp shifted reg", 0xFE00_8000, 0xEA00_0000),
    ("dp modified imm", 0xFA00_8000, 0xF000_0000),
    ("dp plain imm", 0xFA00_8000, 0xF200_0000),
    ("b.w cond", 0xF800_D000, 0xF000_8000),
    ("b.w", 0xF800_D000, 0xF000_9000),
    ("bl", 0xF800_D000, 0xF000_D000),
    ("msr", 0xFFF0_FF00, 0xF380_8800),
    ("mrs", 0xFFFF_F000, 0xF3EF_8000),
    ("hints.w", 0xFFFF_FF00, 0xF3AF_8000),
    ("barriers", 0xFFFF_FF00, 0xF3BF_8F00),
    ("store single", 0xFF10_0000, 0xF800_0000),
    ("load byte", 0xFE70_0000, 0xF810_0000),
    ("load half", 0xFE70_0000, 0xF830_0000),
    ("load word", 0xFE70_0000, 0xF850_0000),
    ("dp register", 0xFF00_F000, 0xFA00_F000),
    ("multiply", 0xFF80_0000, 0xFB00_0000),
    ("long multiply/divide", 0xFF80_0000, 0xFB80_0000),
    // Valid forms that the groups above rarely hit with random fields.
    ("sdiv/udiv", 0xFFD0_F0F0, 0xFB90_F0F0),
    ("mul/mla/mls", 0xFFF0_00E0, 0xFB00_0000),
    ("smull/umull/smlal/umlal", 0xFF90_00F0, 0xFB80_0000),
    ("ssat/usat/sbfx/ubfx/bfi", 0xFF10_8020, 0xF300_0000),
    ("movw/movt", 0xFB70_8000, 0xF240_0000),
    ("clrex", 0xFFFF_FFFF, 0xF3BF_8F2F),
    ("ldrd literal", 0xFE5F_0000, 0xE85F_0000),
];

/// The `SYSm` values ARMv7-M defines, to steer `MSR`/`MRS` templates.
const SYSM: [u32; 14] = [0, 1, 2, 3, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20];

/// A random instruction as halfwords.
fn instruction(rng: &mut Rng) -> Vec<u16> {
    if rng.chance(50) {
        let hw1 = rng.u32() as u16;
        return if is_32bit(hw1) { vec![hw1, rng.u32() as u16] } else { vec![hw1] };
    }
    let (name, mask, value) = TEMPLATES[rng.below(TEMPLATES.len() as u32) as usize];
    let mut raw = (rng.u32() & !mask) | value;
    if (name == "msr" || name == "mrs") && rng.chance(80) {
        raw = (raw & !0xFF) | SYSM[rng.below(SYSM.len() as u32) as usize];
    }
    if mask >> 16 == 0 {
        let raw = raw as u16;
        // A 16-bit template must not wander into the 32-bit space.
        if is_32bit(raw) { vec![0xBF00] } else { vec![raw] }
    } else {
        vec![(raw >> 16) as u16, raw as u16]
    }
}

/// A register value: often an address in RAM or MMIO so loads and stores land somewhere.
fn register(rng: &mut Rng) -> u32 {
    match rng.below(10) {
        0..=3 => RAM + rng.below(RAM_SIZE),
        4 => MMIO + rng.below(MMIO_SIZE),
        5 => rng.below(64),
        6 => [0, 1, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF_FFF9, 31, 32, 33, 0xFF][rng.below(10) as usize],
        7 => CODE + rng.below(CODE_SIZE),
        _ => rng.u32(),
    }
}

/// Decoded form of an instruction, for classification and filtering.
fn op_of(halves: &[u16], pc: u32) -> Op {
    decode(halves[0], halves.get(1).copied().unwrap_or(0), pc).op
}

/// Instructions the ARM ARM makes UNPREDICTABLE (or IMPLEMENTATION DEFINED) inside an IT block, or
/// that are not conditional.
fn unfit_for_it(op: &Op, halves: &[u16]) -> bool {
    matches!(
        op,
        Op::It { .. }
            | Op::Cbz { .. }
            | Op::BCond { .. }
            | Op::Cps { .. }
            | Op::Bkpt { .. }
            | Op::Undefined { .. }
            | Op::Unpredictable { .. }
    ) || (halves.len() == 1 && halves[0] & 0xFFC0 == 0)
}

/// A whole case: the code, how many instructions to run, and the instruction under test.
struct Case {
    code: Vec<u16>,
    steps: u32,
    subject: Vec<u16>,
    /// Halfword offset of the subject in `code`.
    at: u32,
}

/// An IT block: `it` and as many instructions as its mask covers, each fit for an IT block and
/// all but the last free of branches. `None` when the draw does not fit.
fn it_block(rng: &mut Rng, pc: u32, it: u16) -> Option<Case> {
    let n = 4 - (it & 0xF).trailing_zeros();
    let mut code = vec![it];
    let mut at = pc + 2;
    for i in 0..n {
        let ins = instruction(rng);
        let op = op_of(&ins, at);
        if unfit_for_it(&op, &ins) || (i + 1 < n && op.branches()) {
            return None;
        }
        // A branch back into the block loops forever in Unicorn, which never stops inside an IT
        // block. Register targets are steered away in `worker`.
        let target = match op {
            Op::B { target } | Op::Bl { target } => Some(target),
            _ => None,
        };
        if target.is_some_and(|t| t.wrapping_sub(pc).wrapping_add(0x20) < 0x40 + 2 * 6) {
            return None;
        }
        at += 2 * ins.len() as u32;
        code.extend(ins);
    }
    // A NOP after the block: Unicorn stops before a final slot that fails its condition.
    code.push(0xBF00);
    Some(Case { code, steps: 2 + n, subject: vec![it], at: 0 })
}

fn case(rng: &mut Rng, pc: u32) -> Case {
    let first = instruction(rng);
    // An IT block always runs to its end: Unicorn does not stop inside one.
    if let Op::It { state } = op_of(&first, pc) {
        return (0..8).find_map(|_| it_block(rng, pc, 0xBF00 | u16::from(state))).unwrap_or(Case {
            code: vec![0xBF00],
            steps: 1,
            subject: vec![0xBF00],
            at: 0,
        });
    }
    match rng.below(100) {
        // A random instruction inside an IT block of one to four slots.
        0..=19 => {
            let n = 1 + rng.below(4);
            let firstcond = rng.below(14);
            let mut mask = 1 << (4 - n);
            for slot in 0..n - 1 {
                let then = rng.chance(50);
                let bit = if then { firstcond & 1 } else { !firstcond & 1 };
                mask |= bit << (3 - slot);
            }
            let it = (0xBF00 | firstcond << 4 | mask) as u16;
            match it_block(rng, pc, it) {
                Some(mut block) => {
                    block.subject = block.code[1..].to_vec();
                    block.at = 1;
                    block
                }
                None => Case { code: first.clone(), steps: 1, subject: first, at: 0 },
            }
        }
        // LDREX then STREX through the same base register.
        20..=24 => {
            let rn = rng.below(13);
            let (rt, rd) = (rng.below(13), rng.below(13));
            let ldrex = [0xE850 | rn as u16, (rt as u16) << 12 | 0x0F00];
            let strex = [0xE840 | rn as u16, (rng.below(13) as u16) << 12 | (rd as u16) << 8];
            let code = vec![0xF3BF, 0x8F2F, ldrex[0], ldrex[1], strex[0], strex[1]];
            Case { code, steps: 3, subject: strex.to_vec(), at: 4 }
        }
        _ => {
            // A STREX alone must not see an earlier case's LDREX: clear the monitor first.
            if matches!(op_of(&first, pc), Op::Strex { .. }) {
                let mut code = vec![0xF3BF, 0x8F2F];
                code.extend(&first);
                return Case { code, steps: 2, subject: first, at: 2 };
            }
            Case { code: first.clone(), steps: 1, subject: first, at: 0 }
        }
    }
}

/// The instruction a divergence is about: the one our core stopped on if it faulted, else the
/// one Unicorn stopped on if it faulted, else the instruction under test.
fn culprit(o: &Side, oe: Exit, u: &Side, ue: Exit, subject: &Op) -> Op {
    let at = |side: &Side| {
        let pc = side.cpu.regs().pc;
        let (bytes, base) = if pc.wrapping_sub(RAM) < RAM_SIZE { (&side.ram, RAM) } else { (&side.code, CODE) };
        let off = pc.wrapping_sub(base) as usize;
        (off + 4 <= bytes.len()).then(|| {
            let half = |i: usize| u16::from_le_bytes([bytes[off + i], bytes[off + i + 1]]);
            decode(half(0), half(2), pc).op
        })
    };
    let faulted = |exit: Exit| matches!(exit, Exit::Fault(_) | Exit::Breakpoint(_));
    if faulted(oe)
        && let Some(op) = at(o)
    {
        return op;
    }
    if faulted(ue)
        && let Some(op) = at(u)
    {
        return op;
    }
    *subject
}

/// The instructions of `code`, as halfword slices.
fn instructions(code: &[u16]) -> impl Iterator<Item = &[u16]> {
    let mut at = 0;
    std::iter::from_fn(move || {
        let len = if is_32bit(*code.get(at)?) { 2 } else { 1 };
        let ins = code.get(at..at + len)?;
        at += len;
        Some(ins)
    })
}

/// Tags for the instructions in a case that explain known Unicorn differences.
fn tags(code: &[u16], regs: &CoreRegs) -> String {
    let mut tags = String::new();
    for ins in instructions(code) {
        let raw = ins.iter().fold(0_u32, |raw, &h| raw << 16 | u32::from(h));
        let rn = (raw >> 16) as usize & 0xF;
        let tag = match ins.len() {
            1 if matches!(ins[0], 0xBF10 | 0xBF20) => "[yield/wfe]",
            2 if matches!(raw, 0xF3AF_8001 | 0xF3AF_8002) => "[yield/wfe]",
            2 if raw & 0xFFF0_FFFF == 0xF380_8814 => "[msr control]",
            2 if raw & 0xFFF0_FFFE == 0xF380_8808 && regs.r.get(rn).is_none_or(|v| v & 3 != 0) => "[msr sp]",
            2 if raw & 0xFFF0_0000 == 0xE850_0000 || raw & 0xFFF0_0FE0 == 0xE8D0_0F40 => "[exclusive]",
            2 if raw & 0xFFFF_FFF0 == 0xF3BF_8F60 && matches!(op_of(code, 0), Op::It { .. }) => "[isb in IT]",
            _ if writes_unaligned_sp(&op_of(ins, 0)) => "[sp low bits]",
            _ => continue,
        };
        if !tags.contains(tag) {
            tags.push(' ');
            tags.push_str(tag);
        }
    }
    tags
}

/// True for the instructions that can write SP with bits 1:0 set: a register or loaded value,
/// or an offset that is not a multiple of 4. PUSH, POP, LDM and STM move SP by whole words.
fn writes_unaligned_sp(op: &Op) -> bool {
    match *op {
        Op::DpImm { op, rd: 13, imm, .. } => !op.is_compare() && (imm % 4 != 0 || !matches!(op, Dp::Add | Dp::Sub)),
        Op::DpReg { op, rd: 13, .. } => !op.is_compare(),
        Op::Load { rt: 13, .. } | Op::LoadReg { rt: 13, .. } | Op::LoadLit { rt: 13, .. } => true,
        Op::LoadIdx { rt, rn, imm, .. } => rt == 13 || (rn == 13 && imm % 4 != 0),
        Op::StoreIdx { rn: 13, imm, .. } => imm % 4 != 0,
        _ => false,
    }
}

/// Why an encoding is UNDEFINED, by its bits (ARM ARM A5.3), for encodings Unicorn runs.
fn undefined_group(ins: &[u16]) -> &'static str {
    let [hw1, hw2] = *ins else {
        // SETEND (`1011 0110 0101 E000`) is ARMv6/ARMv7-A only.
        return if ins.first().is_some_and(|&h| h & 0xFFF7 == 0xB650) {
            "Undefined [SETEND]"
        } else {
            "Undefined [16-bit]"
        };
    };
    let (a, b) = (u32::from(hw1), u32::from(hw2));
    let (op1, op2) = ((a >> 11) & 3, (a >> 4) & 0x7F);
    let dsp = match op1 {
        // PKHBT/PKHTB.
        0b01 => op2 & 0x60 == 0x20 && (a >> 5) & 0xF == 0b0110,
        // SSAT16/USAT16.
        0b10 => b & 0x8000 == 0 && a & 0x0200 != 0 && matches!((a >> 4) & 0x1F, 0x12 | 0x1A) && b & 0x70C0 == 0,
        0b11 if op2 & 0x70 == 0x20 => {
            let (r1, r2) = ((a >> 4) & 0xF, (b >> 4) & 0xF);
            (r2 & 8 != 0 && r1 & 8 == 0 && (matches!(r1, 2 | 3) || a & 0xF != 0xF))
                || (r1 & 8 != 0 && r2 & 8 == 0)
                || (r1 & 0xC == 0x8 && r2 & 0xC == 0x8 && !matches!((r1 & 3, r2 & 3), (1, _) | (3, 0)))
        }
        0b11 if op2 & 0x78 == 0x30 => (a >> 4) & 7 != 0 || (b >> 4) & 0xF > 1,
        0b11 if op2 & 0x78 == 0x38 => !matches!(((a >> 4) & 7, (b >> 4) & 0xF), (0 | 2 | 4 | 6, 0) | (1 | 3, 0xF)),
        _ => false,
    };
    if dsp {
        "Undefined [DSP]"
    } else if op1 != 0b10 && op2 & 0x40 != 0 {
        "Undefined [coprocessor]"
    } else if op1 == 0b11 && op2 & 0x71 == 0x10 {
        "Undefined [SIMD]"
    } else if op1 == 0b10 && b & 0xD000 == 0xC000 {
        "Undefined [BLX imm]"
    } else if op1 == 0b10 && b & 0xD000 == 0x8000 && matches!(op2, 0x3C | 0x3D) {
        // BXJ and SUBS PC, LR (ERET).
        "Undefined [BXJ/SUBS PC]"
    } else if op1 == 0b11 && op2 & 0x71 == 0 && a & 0xF == 0xF {
        "Undefined [store Rn=PC]"
    } else if op1 == 0b01 && a & 0xFFE0 == 0xE8C0 && (b >> 4) & 0xF == 0b0111 {
        // LDREXD/STREXD (`1110 1000 110L Rn | .. 0111 ..`).
        "Undefined [exclusive doubleword]"
    } else if op1 == 0b01 && a & 0xFFE0 == 0xE8C0 && (b >> 4) & 0xF >= 0b1000 {
        // LDA/STL and their exclusive forms.
        "Undefined [ARMv8 acquire/release]"
    } else {
        "Undefined [other]"
    }
}

/// Examples kept per divergence class (`EUC_FUZZ_EXAMPLES`, default 3).
fn examples_kept() -> usize {
    std::env::var("EUC_FUZZ_EXAMPLES").ok().and_then(|v| v.parse().ok()).unwrap_or(3)
}

/// True when straight-line code from `pc` reaches an UNDEFINED or UNPREDICTABLE encoding within
/// a dozen instructions, before any branch: Unicorn translates such a block whole and reports the
/// bad encoding as soon as the block is entered.
fn undefined_ahead(side: &Side, pc: u32) -> bool {
    let (bytes, base) = if pc.wrapping_sub(RAM) < RAM_SIZE { (&side.ram, RAM) } else { (&side.code, CODE) };
    let mut at = pc;
    for _ in 0..12 {
        let off = at.wrapping_sub(base) as usize;
        let Some(b) = bytes.get(off..off + 4) else { return false };
        let (hw1, hw2) = (u16::from_le_bytes([b[0], b[1]]), u16::from_le_bytes([b[2], b[3]]));
        let decoded = decode(hw1, hw2, at);
        match decoded.op {
            Op::Undefined { .. } | Op::Unpredictable { .. } => return true,
            op if op.branches()
                || matches!(op, Op::BCond { .. } | Op::Cbz { .. } | Op::Svc { .. } | Op::Bkpt { .. }) =>
            {
                return false;
            }
            _ => at += u32::from(decoded.len),
        }
    }
    false
}

/// The two halfwords at `pc` in `side`'s code or RAM.
fn halfwords(side: &Side, pc: u32) -> Option<[u16; 2]> {
    let (bytes, base) = if pc.wrapping_sub(RAM) < RAM_SIZE { (&side.ram, RAM) } else { (&side.code, CODE) };
    let off = pc.wrapping_sub(base) as usize;
    let b = bytes.get(off..off + 4)?;
    Some([u16::from_le_bytes([b[0], b[1]]), u16::from_le_bytes([b[2], b[3]])])
}

/// The instructions from `pc` in `side`'s code or RAM, decoded, for examples.
fn halves_at(side: &Side, pc: u32) -> String {
    let (bytes, base) = if pc.wrapping_sub(RAM) < RAM_SIZE { (&side.ram, RAM) } else { (&side.code, CODE) };
    let mut at = pc;
    let mut text = String::new();
    for _ in 0..16 {
        let off = at.wrapping_sub(base) as usize;
        let Some(b) = bytes.get(off..off + 4) else { break };
        let (hw1, hw2) = (u16::from_le_bytes([b[0], b[1]]), u16::from_le_bytes([b[2], b[3]]));
        let decoded = decode(hw1, hw2, at);
        let hex = if decoded.len == 4 { format!("{hw1:04X} {hw2:04X}") } else { format!("{hw1:04X}") };
        let _ = write!(text, "\n        {at:08X}: {hex:<9} {:?}", decoded.op);
        at += u32::from(decoded.len);
    }
    text
}

/// The variant name of an op, e.g. `DpImm`.
fn op_name(op: &Op) -> String {
    let text = format!("{op:?}");
    text.split([' ', '{', '(']).next().unwrap_or("").to_owned()
}

/// Exit with its payloads reduced to what both backends report (Unicorn cannot say which SVC
/// or BKPT number, or which undefined encoding).
fn exit_class(exit: Exit) -> String {
    match exit {
        Exit::Svc(_) => "Svc".into(),
        Exit::Breakpoint(_) => "Breakpoint".into(),
        // Unicorn stops undefined encodings with UC_ERR_INSN_INVALID (10) and coprocessor ones with
        // QEMU's EXCP_NOCP (17); both are a UsageFault on a Cortex-M3, as is our Undefined.
        Exit::Fault(Fault::Undefined(_) | Fault::Unknown(10 | 17)) => "Undefined".into(),
        Exit::Fault(Fault::Unaligned(_)) => "Unaligned".into(),
        // A fetch outside plain memory faults on both sides; the fuzz bus calls MMIO execute-never.
        Exit::Fault(Fault::Bus(BusFault { access: Access::Fetch, .. })) => "Bus(Fetch)".into(),
        Exit::Fault(Fault::Bus(fault)) => format!("Bus({:?} {:?})", fault.access, fault.kind),
        other => format!("{other:?}"),
    }
}

/// The xPSR bits both backends own.
const XPSR_COMPARED: u32 = 0xF800_0000 | CoreRegs::XPSR_T | 0x0600_FC00;

/// Unicorn translates the instruction after its budget before it stops, so a fault of that next
/// instruction can end a run that our core ends with `Budget`. `next` is what our core does on
/// the following step: the two agree when it faults the same way (Unicorn reports a fetch in
/// ARM state, after an interworking branch to an even address, as an invalid instruction).
fn eager_match(ue: Exit, next: Exit) -> bool {
    match (ue, next) {
        (Exit::Fault(Fault::Unknown(10)), Exit::Fault(Fault::InvalidState))
        | (Exit::Fault(Fault::Unknown(8)), Exit::Fault(Fault::Bus(BusFault { access: Access::Fetch, .. }))) => true,
        (Exit::Fault(_) | Exit::Breakpoint(_) | Exit::Svc(_), _) => exit_class(ue) == exit_class(next),
        _ => false,
    }
}

/// What our core does on the step after the case, when Unicorn faulted where our core stopped
/// with `Budget` (see [`eager_match`]). Only a fault leaves the state as it was; anything else
/// gets its registers back, and then the comparison reports the difference.
fn next_step(o: &mut Side, oe: Exit, u: &Side, ue: Exit) -> Option<Exit> {
    let eager = oe == Exit::Budget
        && matches!(ue, Exit::Fault(_) | Exit::Breakpoint(_) | Exit::Svc(_))
        && u.cpu.regs().pc == o.cpu.regs().pc;
    if !eager {
        return None;
    }
    let saved = o.cpu.regs().clone();
    let next = o.cpu.run(&mut o.bus, 1);
    if !matches!(next, Exit::Fault(_) | Exit::Breakpoint(_)) {
        *o.cpu.regs_mut() = saved;
    }
    Some(next)
}

/// The first difference between the two sides, as a class name and a detail line.
fn compare(u: &Side, o: &Side, ue: Exit, oe: Exit, next_exit: Option<Exit>) -> Option<(String, String)> {
    let (ur, or) = (u.cpu.regs(), o.cpu.regs());
    let invstate = Some(Exit::Fault(Fault::InvalidState));
    if (next_exit == invstate || Some(oe) == invstate)
        && matches!(ue, Exit::Fault(Fault::Unknown(8 | 10) | Fault::Bus(BusFault { access: Access::Fetch, .. })))
    {
        return Some(("INVSTATE or fetch fault".into(), format!("unicorn {ue:?}")));
    }
    let ue = if next_exit.is_some_and(|next| eager_match(ue, next)) { Exit::Budget } else { ue };
    let (ux, ox) = (exit_class(ue), exit_class(oe));
    if ux != ox {
        return Some((format!("exit {ux} vs {ox}"), format!("unicorn {ue:?} core {oe:?}")));
    }
    let faulted = matches!(ue, Exit::Fault(_) | Exit::Breakpoint(_));
    if ur.pc != or.pc {
        return Some(("pc".into(), format!("unicorn 0x{:08X} core 0x{:08X}", ur.pc, or.pc)));
    }
    // Unicorn's register API reads the masks through QEMU's MRS path, which returns 0 once nPRIV
    // is set; only CONTROL is comparable then.
    let unprivileged = ur.control & 1 != 0;
    let masks = |r: &CoreRegs| {
        if unprivileged { (false, false, 0, r.control) } else { (r.primask, r.faultmask, r.basepri, r.control) }
    };
    let low = |r: &CoreRegs| CoreRegs { control: r.control & 3, ..r.clone() };
    if masks(&low(ur)) == masks(&low(or)) && ur.control & !3 != or.control & !3 {
        return Some(("CONTROL[3:2]".into(), format!("unicorn 0x{:X} core 0x{:X}", ur.control, or.control)));
    }
    if masks(ur) != masks(or) {
        return Some(("masks".into(), format!("unicorn {:?} core {:?}", masks(ur), masks(or))));
    }
    if faulted {
        // A faulting instruction may leave registers UNKNOWN; only PC and the exit are defined.
        return None;
    }
    for n in 0..15 {
        if ur.reg(n) != or.reg(n) {
            return Some((format!("r{n}"), format!("unicorn 0x{:08X} core 0x{:08X}", ur.reg(n), or.reg(n))));
        }
    }
    if ur.xpsr & XPSR_COMPARED != or.xpsr & XPSR_COMPARED {
        return Some(("xpsr".into(), format!("unicorn 0x{:08X} core 0x{:08X}", ur.xpsr, or.xpsr)));
    }
    // QEMU splits an unaligned MMIO read into two aligned reads of the access size, so it may read
    // bytes the Cortex-M3 does not; the bytes our core reads must be among them.
    let ((ur_bytes, uw, ureturns), (or_bytes, ow, oreturns)) = (u.bus.effects(), o.bus.effects());
    if uw != ow || ureturns != oreturns || !or_bytes.is_subset(&ur_bytes) {
        return Some(("mmio".into(), format!("unicorn {:?} core {:?}", u.bus.log, o.bus.log)));
    }
    if let Some(at) = u.ram.iter().zip(o.ram.iter()).position(|(a, b)| a != b) {
        let addr = RAM + at as u32;
        return Some(("ram".into(), format!("first difference at 0x{addr:08X}")));
    }
    None
}

#[derive(Default)]
struct Tally {
    cases: u64,
    compared: u64,
    skipped_unpredictable: u64,
    unicorn_rebuilds: u64,
    stale: u64,
    by_op: BTreeMap<String, u64>,
    divergences: BTreeMap<String, (u64, Vec<String>)>,
}

impl Tally {
    fn merge(&mut self, other: Self) {
        self.cases += other.cases;
        self.compared += other.compared;
        self.skipped_unpredictable += other.skipped_unpredictable;
        self.unicorn_rebuilds += other.unicorn_rebuilds;
        self.stale += other.stale;
        for (k, v) in other.by_op {
            *self.by_op.entry(k).or_default() += v;
        }
        for (k, (n, examples)) in other.divergences {
            let entry = self.divergences.entry(k).or_default();
            entry.0 += n;
            for e in examples {
                if entry.1.len() < examples_kept() {
                    entry.1.push(e);
                }
            }
        }
    }
}

#[allow(clippy::too_many_lines)] // one case end to end: draw, run both cores, compare, classify
fn worker(seed: u64, cases: u64) -> Tally {
    let mut rng = Rng::new(seed);
    let mut code = vec![0_u8; CODE_SIZE as usize];
    for chunk in code.chunks_mut(4) {
        chunk.copy_from_slice(&rng.u32().to_le_bytes());
    }
    let at = (FLUSH - CODE) as usize;
    code[at..at + 8].copy_from_slice(&[0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF]);
    let mut u = Side::new(&code, true);
    let mut o = Side::new(&code, false);
    let mut tally = Tally::default();
    for _ in 0..cases {
        tally.cases += 1;
        let pc = CODE + 0x100 + 2 * rng.below((CODE_SIZE - 0x200) / 2);
        let case = case(&mut rng, pc);
        let subject_pc = pc + 2 * case.at;
        let op = op_of(&case.subject, subject_pc);
        if matches!(op, Op::Unpredictable { .. }) {
            tally.skipped_unpredictable += 1;
            continue;
        }
        *tally.by_op.entry(op_name(&op)).or_default() += 1;

        // Same state on both sides.
        let mut regs = CoreRegs::default();
        for r in &mut regs.r {
            *r = register(&mut rng);
        }
        regs.msp = RAM + 0x400 + 4 * rng.below((RAM_SIZE - 0x800) / 4);
        regs.lr = register(&mut rng);
        regs.pc = pc;
        // No register may point back into the case: an IT block that branches to itself loops
        // forever in Unicorn (see `it_block`).
        let span = 2 * case.code.len() as u32 + 0x40;
        for n in 0..15 {
            if regs.reg(n).wrapping_sub(pc).wrapping_add(0x20) < span {
                regs.set_reg(n, regs.reg(n).wrapping_add(0x1000));
            }
        }
        regs.xpsr = (rng.u32() & 0xF800_0000) | CoreRegs::XPSR_T;
        regs.primask = rng.chance(20);
        regs.faultmask = rng.chance(10);
        regs.basepri = if rng.chance(50) { 0 } else { rng.u32() as u8 };
        let salt = rng.u32();
        if rng.chance(30) {
            for chunk in u.ram.chunks_mut(4) {
                chunk.copy_from_slice(&rng.u32().to_le_bytes());
            }
        }
        o.ram.copy_from_slice(&u.ram);
        let ram_before = u.ram.clone();
        for side in [&mut u, &mut o] {
            side.place(pc, &case.code);
            *side.cpu.regs_mut() = CoreRegs { msp: regs.msp, ..regs.clone() };
            side.bus.log.clear();
            side.bus.salt = salt;
        }
        // Our core runs one instruction per run, which also exercises IT state across runs.
        // Unicorn's instruction counter skips IT slots that fail their condition (ARM counts them
        // as executed), so it gets the count minus those.
        let mut oe = Exit::Budget;
        let mut skipped = 0;
        for _ in 0..case.steps {
            let x = o.cpu.regs().xpsr;
            let state = it::from_xpsr(x);
            let flags =
                it::Flags { n: x >> 31 & 1 == 1, z: x >> 30 & 1 == 1, c: x >> 29 & 1 == 1, v: x >> 28 & 1 == 1 };
            if state != 0 && !flags.passes(it::condition(state)) {
                skipped += 1;
            }
            oe = o.cpu.run(&mut o.bus, 1);
            if oe != Exit::Budget {
                break;
            }
        }
        let ue = u.cpu.run(&mut u.bus, case.steps - skipped);
        // Whether the last step opened an IT block, before any extra step below moves on.
        let ended_in_it = oe == Exit::Budget && it::from_xpsr(o.cpu.regs().xpsr) != 0;
        let mut next_exit = next_step(&mut o, oe, &u, ue);
        tally.compared += 1;
        let class_of = |o: &Side, u: &Side, ue: Exit, next_exit: Option<Exit>| {
            let (what, detail) = compare(u, o, ue, oe, next_exit)?;
            let in_it = matches!(op_of(&case.code, pc), Op::It { .. });
            let fault_in_it = if in_it && matches!(oe, Exit::Fault(_)) { " after a fault in an IT block" } else { "" };
            let name = match culprit(o, oe, u, ue, &op) {
                Op::Undefined { raw } if raw > 0xFFFF => undefined_group(&[(raw >> 16) as u16, raw as u16]).to_owned(),
                Op::Undefined { raw } => undefined_group(&[raw as u16]).to_owned(),
                other => op_name(&other),
            };
            let exc_return = o.bus.log.iter().any(|e| matches!(*e, Event::Return(v) if v < 0xFFFF_FF00));
            let range = if exc_return { " [exc_return range]" } else { "" };
            // Random code at a branch target can open an IT block on the last step.
            let open = ended_in_it;
            let open = if open { " [ends in IT block]" } else { "" };
            let ahead = if oe == Exit::Budget
                && ue == Exit::Fault(Fault::Unknown(10))
                && u.cpu.regs().pc == o.cpu.regs().pc
                && undefined_ahead(o, o.cpu.regs().pc)
            {
                " [undefined ahead]"
            } else {
                ""
            };
            let case_tags = tags(&case.code, &regs);
            // A YIELD or WFE that Unicorn just ran, reached through a branch rather than in the case.
            let hint = halfwords(o, u.cpu.regs().pc.wrapping_sub(4));
            let hint = matches!(hint, Some([_, 0xBF10 | 0xBF20] | [0xF3AF, 0x8001 | 0x8002]));
            let hint = if hint && !case_tags.contains("[yield/wfe]") { " [yield/wfe]" } else { "" };
            let open = format!("{open}{ahead}{hint}");
            Some((format!("{name}: {what}{fault_in_it}{case_tags}{range}{open}"), detail))
        };
        if let Some((mut class, mut detail)) = class_of(&o, &u, ue, next_exit) {
            let mut ue = ue;
            if explanation(&class).is_none() {
                // A fresh Unicorn tells a real difference from state an earlier case left behind.
                let mut fresh = Side::new(&o.code, true);
                fresh.ram.copy_from_slice(&ram_before);
                *fresh.cpu.regs_mut() = regs.clone();
                fresh.bus.salt = salt;
                ue = fresh.cpu.run(&mut fresh.bus, case.steps - skipped);
                if next_exit.is_none() {
                    next_exit = next_step(&mut o, oe, &fresh, ue);
                }
                let confirmed = class_of(&o, &fresh, ue, next_exit);
                u = fresh;
                let Some(found) = confirmed else {
                    tally.stale += 1;
                    o.ram.copy_from_slice(&u.ram);
                    continue;
                };
                (class, detail) = found;
            }
            let hex: Vec<String> = case.code.iter().map(|h| format!("{h:04X}")).collect();
            let mut example =
                format!("code [{}] at 0x{pc:08X} steps {} {detail}\n      in  {regs:08X?}", hex.join(" "), case.steps);
            let _ = write!(example, "\n      uni {:08X?} {ue:?}\n      own {:08X?} {oe:?}", u.cpu.regs(), o.cpu.regs());
            let _ =
                write!(example, "\n      from Unicorn's PC - 4: {}", halves_at(&o, u.cpu.regs().pc.wrapping_sub(4)));
            let entry = tally.divergences.entry(class).or_default();
            entry.0 += 1;
            if entry.1.len() < examples_kept() {
                entry.1.push(example);
            }
            // Resynchronise RAM for the next case.
            o.ram.copy_from_slice(&u.ram);
        }
        // Unicorn writes CONTROL through QEMU's MSR path, which ignores unprivileged writes: once
        // a case sets nPRIV it could never be cleared again. Start that side afresh.
        if u.cpu.regs().control != 0 {
            let ram = u.ram.clone();
            u = Side::new(&o.code, true);
            u.ram.copy_from_slice(&ram);
            tally.unicorn_rebuilds += 1;
        }
        // Leave no IT block open in Unicorn (its IT state cannot be written directly).
        if u.cpu.regs().xpsr & 0x0600_FC00 != 0 {
            u.cpu.regs_mut().pc = FLUSH;
            u.cpu.run(&mut u.bus, 4);
        }
    }
    tally
}

#[test]
fn matches_unicorn_instruction_by_instruction() {
    let cases: u64 = std::env::var("EUC_FUZZ_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(200_000);
    let seed = std::env::var("EUC_FUZZ_SEED")
        .ok()
        .and_then(|v| u64::from_str_radix(v.trim_start_matches("0x"), 16).ok())
        .unwrap_or_else(|| {
            std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64
        });
    let threads = std::env::var("EUC_FUZZ_THREADS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or_else(|| std::thread::available_parallelism().map_or(4, usize::from) as u64);
    eprintln!("fuzz: seed 0x{seed:X}, {cases} cases on {threads} threads (EUC_FUZZ_SEED=0x{seed:X} reproduces)");
    let total = Mutex::new(Tally::default());
    std::thread::scope(|scope| {
        for t in 0..threads {
            let total = &total;
            let share = cases / threads + u64::from(t < cases % threads);
            scope.spawn(move || {
                let tally = worker(seed ^ (t + 1).wrapping_mul(0xA076_1D64_78BD_642F), share);
                total.lock().unwrap().merge(tally);
            });
        }
    });
    let tally = total.into_inner().unwrap();
    eprintln!(
        "fuzz: {} cases, {} compared, {} UNPREDICTABLE encodings skipped, Unicorn rebuilt {} times, \
         {} differences gone on a fresh Unicorn",
        tally.cases, tally.compared, tally.skipped_unpredictable, tally.unicorn_rebuilds, tally.stale
    );
    let coverage: Vec<String> = tally.by_op.iter().map(|(k, v)| format!("{k} {v}")).collect();
    eprintln!("fuzz: coverage {}", coverage.join(", "));
    let mut unexplained = 0;
    for (class, (count, examples)) in &tally.divergences {
        let reason = explanation(class);
        if let Some(why) = reason {
            eprintln!("explained  {count:>8}  {class}  ({why})");
        } else {
            unexplained += count;
            eprintln!("DIVERGENCE {count:>8}  {class}");
            for e in examples {
                eprintln!("    {e}");
            }
        }
    }
    assert_eq!(unexplained, 0, "unexplained divergences from Unicorn (seed 0x{seed:X})");
}

//! The decoded form of one Thumb instruction.
//!
//! The decoders ([`crate::decode`]) turn every ARMv7-M encoding into an [`Op`] with its operands
//! already extracted, immediates expanded and PC-relative addresses resolved (a slot of the
//! decode cache belongs to exactly one address). The executor then never looks at instruction
//! bits again. Several encodings share one variant when the ARM ARM pseudocode is the same: all
//! sixteen data-processing operations with an immediate operand are [`Op::DpImm`], `PUSH` is a
//! [`Op::Stm`] that decrements before, `ADR` is a `MOV` of a constant.

/// Data-processing operation (ARM ARM A5.3.1, A5.3.11; the 16-bit forms in A5.2.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dp {
    /// `Rd = Rn AND op2`.
    And,
    /// `Rd = Rn AND NOT op2`.
    Bic,
    /// `Rd = Rn OR op2`.
    Orr,
    /// `Rd = Rn OR NOT op2`.
    Orn,
    /// `Rd = Rn EOR op2`.
    Eor,
    /// `Rd = op2`.
    Mov,
    /// `Rd = NOT op2`.
    Mvn,
    /// Flags of `Rn AND op2`.
    Tst,
    /// Flags of `Rn EOR op2`.
    Teq,
    /// `Rd = Rn + op2`.
    Add,
    /// `Rd = Rn + op2 + C`.
    Adc,
    /// `Rd = Rn + NOT op2 + C`.
    Sbc,
    /// `Rd = Rn - op2`.
    Sub,
    /// `Rd = op2 - Rn`.
    Rsb,
    /// Flags of `Rn - op2`.
    Cmp,
    /// Flags of `Rn + op2`.
    Cmn,
}

impl Dp {
    /// True for the operations whose flags come from `AddWithCarry`; the others are logical and
    /// take C from the shifter.
    pub const fn is_arithmetic(self) -> bool {
        matches!(self, Self::Add | Self::Adc | Self::Sbc | Self::Sub | Self::Rsb | Self::Cmp | Self::Cmn)
    }

    /// True for `TST`, `TEQ`, `CMP` and `CMN`, which only set flags.
    pub const fn is_compare(self) -> bool {
        matches!(self, Self::Tst | Self::Teq | Self::Cmp | Self::Cmn)
    }
}

/// When an instruction updates the APSR flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetFlags {
    /// Never (most 32-bit encodings with S clear).
    No,
    /// Always (S set, compares, `MOVS Rd, Rm`).
    Yes,
    /// `setflags = !InITBlock()`: the 16-bit data-processing encodings.
    OutsideIt,
}

/// Shift type (ARM ARM A7.4.2 `SRType`). `Rrx` always shifts by one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shift {
    /// Logical shift left.
    Lsl,
    /// Logical shift right.
    Lsr,
    /// Arithmetic shift right.
    Asr,
    /// Rotate right.
    Ror,
    /// Rotate right by one through carry.
    Rrx,
}

/// The carry a data-processing immediate carries (`ThumbExpandImm_C`, A5.3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImmCarry {
    /// The immediate is not rotated: C keeps its value.
    Keep,
    /// The rotated immediate has bit 31 clear: a logical operation clears C.
    Clear,
    /// The rotated immediate has bit 31 set: a logical operation sets C.
    Set,
}

/// Size and extension of a load or store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Width {
    /// 8 bits, zero-extended.
    Byte,
    /// 16 bits, zero-extended.
    Half,
    /// 32 bits.
    Word,
    /// 8 bits, sign-extended (loads only).
    SignedByte,
    /// 16 bits, sign-extended (loads only).
    SignedHalf,
}

impl Width {
    /// Bytes moved.
    pub const fn bytes(self) -> u32 {
        match self {
            Self::Byte | Self::SignedByte => 1,
            Self::Half | Self::SignedHalf => 2,
            Self::Word => 4,
        }
    }
}

/// Long multiplies (A5.3.17).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MulLong {
    /// `RdHi:RdLo = Rn * Rm`, unsigned.
    Umull,
    /// `RdHi:RdLo = Rn * Rm`, signed.
    Smull,
    /// `RdHi:RdLo += Rn * Rm`, unsigned.
    Umlal,
    /// `RdHi:RdLo += Rn * Rm`, signed.
    Smlal,
}

/// Single-register bit operations (A5.3.13).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOp {
    /// Count leading zeros.
    Clz,
    /// Reverse the bit order.
    Rbit,
    /// Reverse the byte order of the word.
    Rev,
    /// Reverse the bytes of each halfword.
    Rev16,
    /// Reverse the bytes of the low halfword and sign-extend.
    Revsh,
}

/// Extends with rotation (`SXTB`, `SXTH`, `UXTB`, `UXTH`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Extend {
    /// Sign-extend a byte.
    Sxtb,
    /// Sign-extend a halfword.
    Sxth,
    /// Zero-extend a byte.
    Uxtb,
    /// Zero-extend a halfword.
    Uxth,
}

/// Special registers by their `SYSm` number (ARM ARM B5.1.1).
pub mod sysm {
    /// `APSR`.
    pub const APSR: u8 = 0;
    /// `IAPSR`.
    pub const IAPSR: u8 = 1;
    /// `EAPSR`.
    pub const EAPSR: u8 = 2;
    /// `xPSR`.
    pub const XPSR: u8 = 3;
    /// `IPSR`.
    pub const IPSR: u8 = 5;
    /// `EPSR`.
    pub const EPSR: u8 = 6;
    /// `IEPSR`.
    pub const IEPSR: u8 = 7;
    /// `MSP`.
    pub const MSP: u8 = 8;
    /// `PSP`.
    pub const PSP: u8 = 9;
    /// `PRIMASK`.
    pub const PRIMASK: u8 = 16;
    /// `BASEPRI`.
    pub const BASEPRI: u8 = 17;
    /// `BASEPRI_MAX`.
    pub const BASEPRI_MAX: u8 = 18;
    /// `FAULTMASK`.
    pub const FAULTMASK: u8 = 19;
    /// `CONTROL`.
    pub const CONTROL: u8 = 20;

    /// True for the `SYSm` values ARMv7-M defines (`{0..3, 5..9, 16..20}`).
    pub const fn is_valid(sysm: u32) -> bool {
        matches!(sysm, 0..=3 | 5..=9 | 16..=20)
    }
}

/// One decoded instruction. Register fields are 0 to 15; 15 reads as the instruction's address
/// plus 4, 13 is the active stack pointer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    /// An empty decode-cache slot.
    Undecoded,
    /// A watched address not reached yet: report it, then decode and run the real instruction.
    Watch,

    // ---- Data processing ----
    /// `Rd = Rn <op> imm` (also `MOV`/`MVN` of an immediate, `ADR`, `ADDW`, `MOVW`).
    DpImm {
        /// Operation.
        op: Dp,
        /// Flag update.
        s: SetFlags,
        /// Destination (ignored by compares).
        rd: u8,
        /// First operand (ignored by `MOV`/`MVN`).
        rn: u8,
        /// Carry out of the immediate expansion, for logical operations.
        carry: ImmCarry,
        /// The expanded immediate.
        imm: u32,
    },
    /// `Rd = Rn <op> Shift(Rm, shift, amount)`.
    DpReg {
        /// Operation.
        op: Dp,
        /// Flag update.
        s: SetFlags,
        /// Destination.
        rd: u8,
        /// First operand.
        rn: u8,
        /// Shifted operand.
        rm: u8,
        /// Shift type.
        shift: Shift,
        /// Shift amount, 0 to 32.
        amount: u8,
    },
    /// `Rd = Shift(Rn, shift, Rm<7:0>)`: `LSL`/`LSR`/`ASR`/`ROR` by register.
    ShiftReg {
        /// Shift type (never `Rrx`).
        shift: Shift,
        /// Flag update.
        s: SetFlags,
        /// Destination.
        rd: u8,
        /// Value.
        rn: u8,
        /// Amount register.
        rm: u8,
    },
    /// `Rd = Rn * Rm` (16-bit `MULS` updates N and Z).
    Mul {
        /// Flag update.
        s: SetFlags,
        /// Destination.
        rd: u8,
        /// Operand.
        rn: u8,
        /// Operand.
        rm: u8,
    },
    /// `Rd = Ra + Rn * Rm` (`MLA`) or `Rd = Ra - Rn * Rm` (`MLS`).
    Mla {
        /// `MLS` when true.
        sub: bool,
        /// Destination.
        rd: u8,
        /// Operand.
        rn: u8,
        /// Operand.
        rm: u8,
        /// Accumulator.
        ra: u8,
    },
    /// 64-bit multiplies.
    MulLong {
        /// Which one.
        kind: MulLong,
        /// Low word of the result.
        rdlo: u8,
        /// High word of the result.
        rdhi: u8,
        /// Operand.
        rn: u8,
        /// Operand.
        rm: u8,
    },
    /// `Rd = Rn / Rm`, rounding towards zero.
    Div {
        /// `SDIV` when true.
        signed: bool,
        /// Destination.
        rd: u8,
        /// Dividend.
        rn: u8,
        /// Divisor.
        rm: u8,
    },
    /// `Rd<31:16> = imm`.
    Movt {
        /// Destination.
        rd: u8,
        /// The new top halfword.
        imm: u16,
    },
    /// `SSAT`/`USAT`: saturate `Shift(Rn)` to `bits` bits.
    Sat {
        /// `SSAT` when true.
        signed: bool,
        /// Destination.
        rd: u8,
        /// Operand.
        rn: u8,
        /// Saturation width: 1 to 32 (signed) or 0 to 31 (unsigned).
        bits: u8,
        /// `Lsl` or `Asr`.
        shift: Shift,
        /// Shift amount, 0 to 31 (`Asr` 1 to 31).
        amount: u8,
    },
    /// `Rd<msb:lsb> = Rn<width-1:0>` (`BFI`), or cleared when `rn` is 15 (`BFC`).
    Bfi {
        /// Destination.
        rd: u8,
        /// Source, 15 for `BFC`.
        rn: u8,
        /// Lowest bit.
        lsb: u8,
        /// Highest bit (at least `lsb`).
        msb: u8,
    },
    /// `Rd = Extend(Rn<lsb+width-1:lsb>)` (`SBFX`, `UBFX`).
    Bfx {
        /// `SBFX` when true.
        signed: bool,
        /// Destination.
        rd: u8,
        /// Source.
        rn: u8,
        /// Lowest bit.
        lsb: u8,
        /// Field width, 1 to 32 - lsb.
        width: u8,
    },
    /// `CLZ`, `RBIT`, `REV`, `REV16`, `REVSH`.
    Bits {
        /// Which one.
        op: BitOp,
        /// Destination.
        rd: u8,
        /// Source.
        rm: u8,
    },
    /// `SXTB`, `SXTH`, `UXTB`, `UXTH` of `ROR(Rm, rotation)`.
    Extend {
        /// Which one.
        kind: Extend,
        /// Destination.
        rd: u8,
        /// Source.
        rm: u8,
        /// Rotation: 0, 8, 16 or 24.
        rotation: u8,
    },

    // ---- Loads and stores ----
    /// `Rt = Mem[Rn + imm]`.
    Load {
        /// Width.
        w: Width,
        /// Destination (15 loads PC with interworking).
        rt: u8,
        /// Base.
        rn: u8,
        /// Signed offset.
        imm: i32,
    },
    /// `Rt = Mem[Rn + imm]` with write-back: pre-indexed (`[Rn, #imm]!`) or post-indexed
    /// (`[Rn], #imm`, the address is Rn itself).
    LoadIdx {
        /// Width.
        w: Width,
        /// Destination.
        rt: u8,
        /// Base, updated to `Rn + imm`.
        rn: u8,
        /// Signed offset.
        imm: i32,
        /// Pre-indexed when true.
        pre: bool,
    },
    /// `Rt = Mem[Rn + (Rm << lsl)]`.
    LoadReg {
        /// Width.
        w: Width,
        /// Destination.
        rt: u8,
        /// Base.
        rn: u8,
        /// Index.
        rm: u8,
        /// Index shift, 0 to 3.
        lsl: u8,
    },
    /// `Rt = Mem[addr]`, a literal pool load with its address resolved.
    LoadLit {
        /// Width.
        w: Width,
        /// Destination.
        rt: u8,
        /// `Align(PC, 4) ± imm`.
        addr: u32,
    },
    /// `Mem[Rn + imm] = Rt`.
    Store {
        /// Width (never signed).
        w: Width,
        /// Source.
        rt: u8,
        /// Base.
        rn: u8,
        /// Signed offset.
        imm: i32,
    },
    /// `Mem[Rn + imm] = Rt` with write-back (see [`Op::LoadIdx`]).
    StoreIdx {
        /// Width.
        w: Width,
        /// Source.
        rt: u8,
        /// Base, updated to `Rn + imm`.
        rn: u8,
        /// Signed offset.
        imm: i32,
        /// Pre-indexed when true.
        pre: bool,
    },
    /// `Mem[Rn + (Rm << lsl)] = Rt`.
    StoreReg {
        /// Width.
        w: Width,
        /// Source.
        rt: u8,
        /// Base.
        rn: u8,
        /// Index.
        rm: u8,
        /// Index shift, 0 to 3.
        lsl: u8,
    },
    /// `LDM`/`LDMDB`/`POP`: load the registers in `list` (bit 15 = PC) from consecutive words.
    Ldm {
        /// Base.
        rn: u8,
        /// Register list.
        list: u16,
        /// Update the base (skipped when the base is in the list).
        wback: bool,
        /// Decrement before (`LDMDB`) when true, increment after otherwise.
        db: bool,
    },
    /// `STM`/`STMDB`/`PUSH`.
    Stm {
        /// Base.
        rn: u8,
        /// Register list.
        list: u16,
        /// Update the base.
        wback: bool,
        /// Decrement before (`STMDB`, `PUSH`) when true.
        db: bool,
    },
    /// `LDRD`/`STRD` (immediate).
    Dual {
        /// `LDRD` when true.
        load: bool,
        /// First register (lower address).
        rt: u8,
        /// Second register.
        rt2: u8,
        /// Base.
        rn: u8,
        /// Signed offset.
        imm: i32,
        /// Offset applies before the access (`P`).
        index: bool,
        /// Base write-back (`W`).
        wback: bool,
    },
    /// `LDRD` (literal) with its address resolved.
    LoadDualLit {
        /// First register.
        rt: u8,
        /// Second register.
        rt2: u8,
        /// `Align(PC, 4) ± imm`.
        addr: u32,
    },
    /// `LDREX`, `LDREXB`, `LDREXH`.
    Ldrex {
        /// Width.
        w: Width,
        /// Destination.
        rt: u8,
        /// Base.
        rn: u8,
        /// Offset (word forms only).
        imm: u16,
    },
    /// `STREX`, `STREXB`, `STREXH`: `Rd` gets 0 on success, 1 on failure.
    Strex {
        /// Width.
        w: Width,
        /// Status destination.
        rd: u8,
        /// Source.
        rt: u8,
        /// Base.
        rn: u8,
        /// Offset (word form only).
        imm: u16,
    },
    /// `CLREX`.
    Clrex,
    /// `TBB`/`TBH`: branch forward by twice a table entry.
    Table {
        /// `TBH` when true.
        half: bool,
        /// Table base (15 = the table follows the instruction).
        rn: u8,
        /// Index.
        rm: u8,
    },

    // ---- Branches ----
    /// `B` (always).
    B {
        /// Target address.
        target: u32,
    },
    /// `B<c>` outside an IT block.
    BCond {
        /// Condition, 0 to 13.
        cond: u8,
        /// Target address.
        target: u32,
    },
    /// `BL`.
    Bl {
        /// Target address.
        target: u32,
    },
    /// `BX Rm` (may be an exception return).
    Bx {
        /// Target register.
        rm: u8,
    },
    /// `BLX Rm`.
    Blx {
        /// Target register.
        rm: u8,
    },
    /// `CBZ`/`CBNZ`.
    Cbz {
        /// `CBNZ` when true.
        nonzero: bool,
        /// Tested register.
        rn: u8,
        /// Target address.
        target: u32,
    },

    // ---- System ----
    /// `IT`: the new `ITSTATE` (`firstcond:mask`).
    It {
        /// `ITSTATE<7:0>`.
        state: u8,
    },
    /// `MRS Rd, <spec_reg>`.
    Mrs {
        /// Destination.
        rd: u8,
        /// Special register (see [`sysm`]).
        sysm: u8,
    },
    /// `MSR <spec_reg>, Rn`.
    Msr {
        /// Source.
        rn: u8,
        /// Special register (see [`sysm`]).
        sysm: u8,
    },
    /// `CPSIE`/`CPSID`.
    Cps {
        /// `CPSID` when true.
        disable: bool,
        /// Affects `PRIMASK`.
        i: bool,
        /// Affects `FAULTMASK`.
        f: bool,
    },
    /// `NOP`, `YIELD`, unallocated hints, `PLD`/`PLI`, `DMB`, `DSB`, `ISB`.
    Nop,
    /// `WFE`.
    Wfe,
    /// `WFI`.
    Wfi,
    /// `SEV`.
    Sev,
    /// `SVC #imm`.
    Svc {
        /// The comment field.
        imm: u8,
    },
    /// `BKPT #imm`.
    Bkpt {
        /// The comment field.
        imm: u8,
    },
    /// An UNDEFINED encoding (including `UDF`, coprocessor and DSP-extension instructions, none of
    /// which a Cortex-M3 implements): a UsageFault.
    Undefined {
        /// The encoding: a 16-bit value, or `hw1 << 16 | hw2`.
        raw: u32,
    },
    /// An UNPREDICTABLE encoding (bad register, a should-be-zero bit set). It faults like an
    /// undefined one, so a decoding gap never runs silently; compiled code never contains them.
    Unpredictable {
        /// The encoding: a 16-bit value, or `hw1 << 16 | hw2`.
        raw: u32,
    },
}

/// A decode-cache slot: the operation, its length and its base cycle count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Decoded {
    /// The operation.
    pub op: Op,
    /// Instruction length in bytes: 2 or 4.
    pub len: u8,
    /// Cycles it takes when it does not branch and has no data-dependent timing
    /// (see [`crate::timing`]).
    pub cycles: u8,
    /// A load or store of one register, which pipelines with a neighbouring one.
    pub transfer: bool,
}

impl Decoded {
    /// An empty cache slot.
    pub const EMPTY: Self = Self { op: Op::Undecoded, len: 2, cycles: 0, transfer: false };
}

impl Op {
    /// True when the instruction can write PC: branches, `BX`/`BLX`, `TBB`/`TBH`, loads into PC
    /// (`LDR PC`, `POP {.., PC}`) and data processing into PC (`ADD PC, ..`, `MOV PC, ..`).
    /// Inside an IT block such an instruction must be the last one (A7.3.3); the decoder cannot
    /// know the IT state, so the executor runs it as written and test generators use this to stay
    /// inside defined behaviour.
    pub const fn branches(&self) -> bool {
        match self {
            Self::B { .. }
            | Self::BCond { .. }
            | Self::Bl { .. }
            | Self::Bx { .. }
            | Self::Blx { .. }
            | Self::Cbz { .. }
            | Self::Table { .. } => true,
            Self::Load { rt, .. } | Self::LoadIdx { rt, .. } | Self::LoadReg { rt, .. } | Self::LoadLit { rt, .. } => {
                *rt == 15
            }
            Self::DpReg { op, rd, .. } | Self::DpImm { op, rd, .. } => *rd == 15 && !op.is_compare(),
            Self::Ldm { list, .. } => *list & 0x8000 != 0,
            _ => false,
        }
    }
}

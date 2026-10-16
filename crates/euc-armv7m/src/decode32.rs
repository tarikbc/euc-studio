//! 32-bit Thumb-2 encodings (ARM ARM ARMv7-M, A5.3 "32-bit Thumb instruction encoding").
//!
//! The Cortex-M3 implements ARMv7-M without the DSP extension and without a coprocessor, so the
//! DSP encodings (`SMLA<x><y>`, `QADD`, parallel add/subtract, `SXTAB`, `SSAT16`, ...) and the
//! coprocessor space are UNDEFINED here, as on the silicon (a UsageFault, `UNDEFINSTR` or
//! `NOCP`). Encodings with should-be-zero or should-be-one bits violated, or with registers the
//! ARM ARM forbids, decode to [`Op::Unpredictable`].

use crate::alu::{decode_imm_shift, thumb_expand_imm};
use crate::decode16::{literal_base, sign_extend};
use crate::op::{BitOp, Dp, Extend, ImmCarry, MulLong, Op, SetFlags, Shift, Width, sysm};

/// 13 (SP) or 15 (PC): most 32-bit encodings forbid them as operands.
const fn bad(r: u8) -> bool {
    r == 13 || r == 15
}

/// Decodes the 32-bit instruction `hw1:hw2` at address `pc`.
pub(crate) fn decode32(hw1: u16, hw2: u16, pc: u32) -> Op {
    let (a, b) = (u32::from(hw1), u32::from(hw2));
    let raw = a << 16 | b;
    let op2 = (a >> 4) & 0x7F;
    match (a >> 11) & 3 {
        0b01 if op2 & 0x64 == 0x00 => load_store_multiple(a, b, raw),
        0b01 if op2 & 0x64 == 0x04 => dual_exclusive_table(a, b, raw, pc),
        0b01 if op2 & 0x60 == 0x20 => shifted_register(a, b, raw),
        0b10 if b & 0x8000 != 0 => branch_and_control(a, b, raw, pc),
        0b10 if op2 & 0x20 == 0 => modified_immediate(a, b, raw),
        0b10 => plain_immediate(a, b, raw, pc),
        0b11 if op2 & 0x71 == 0x00 => store_single(a, b, raw),
        0b11 if op2 & 0x67 == 0x01 => load_byte_half(a, b, raw, pc, false),
        0b11 if op2 & 0x67 == 0x03 => load_byte_half(a, b, raw, pc, true),
        0b11 if op2 & 0x67 == 0x05 => load_word(a, b, raw, pc),
        0b11 if op2 & 0x70 == 0x20 => register(a, b, raw),
        0b11 if op2 & 0x78 == 0x30 => multiply(a, b, raw),
        0b11 if op2 & 0x78 == 0x38 => long_multiply_divide(a, b, raw),
        // Coprocessor instructions (A5.3.18) and the unallocated rest.
        _ => Op::Undefined { raw },
    }
}

/// A5.3.5 "Load Multiple and Store Multiple".
fn load_store_multiple(a: u32, b: u32, raw: u32) -> Op {
    let rn = (a & 0xF) as u8;
    let wback = a & (1 << 5) != 0;
    let list = b as u16;
    let unpredictable = Op::Unpredictable { raw };
    let count = list.count_ones();
    let base_listed = wback && list & (1 << rn) != 0;
    let load = a & (1 << 4) != 0;
    let db = match (a >> 7) & 3 {
        0b01 => false,
        0b10 => true,
        // SRS and RFE are ARMv7-A/R only.
        _ => return Op::Undefined { raw },
    };
    if load {
        // LDM T2, LDMDB T1, POP T2: `P M (0) list`; PC and LR are not both loaded.
        if b & 0x2000 != 0 || b & 0xC000 == 0xC000 || count < 2 || (rn == 15) || base_listed {
            return unpredictable;
        }
        Op::Ldm { rn, list, wback, db }
    } else {
        // STM T2, STMDB T1, PUSH T2: `(0) M (0) list`.
        if b & 0xA000 != 0 || count < 2 || rn == 15 || base_listed {
            return unpredictable;
        }
        Op::Stm { rn, list, wback, db }
    }
}

/// A5.3.6 "Load/store dual or exclusive, table branch".
fn dual_exclusive_table(a: u32, b: u32, raw: u32, pc: u32) -> Op {
    let (op1, op2, op3) = ((a >> 7) & 3, (a >> 4) & 3, (b >> 4) & 0xF);
    let rn = (a & 0xF) as u8;
    let (rt, rt2_or_rd) = ((b >> 12) as u8, ((b >> 8) & 0xF) as u8);
    let unpredictable = Op::Unpredictable { raw };
    let imm8x4 = ((b & 0xFF) << 2) as u16;
    match (op1, op2) {
        // STREX T1.
        (0b00, 0b00) => {
            let rd = rt2_or_rd;
            if bad(rd) || bad(rt) || rn == 15 || rd == rn || rd == rt {
                return unpredictable;
            }
            Op::Strex { w: Width::Word, rd, rt, rn, imm: imm8x4 }
        }
        // LDREX T1: `Rt (1)(1)(1)(1) imm8`.
        (0b00, 0b01) => {
            if bad(rt) || rn == 15 || rt2_or_rd != 0xF {
                return unpredictable;
            }
            Op::Ldrex { w: Width::Word, rt, rn, imm: imm8x4 }
        }
        // STREXB/STREXH T1: `Rt (1)(1)(1)(1) 010x Rd`.
        (0b01, 0b00) => {
            let w = match op3 {
                0b0100 => Width::Byte,
                0b0101 => Width::Half,
                _ => return Op::Undefined { raw },
            };
            let rd = (b & 0xF) as u8;
            if bad(rd) || bad(rt) || rn == 15 || rd == rn || rd == rt || rt2_or_rd != 0xF {
                return unpredictable;
            }
            Op::Strex { w, rd, rt, rn, imm: 0 }
        }
        // TBB/TBH, LDREXB/LDREXH.
        (0b01, 0b01) => match op3 {
            0b0000 | 0b0001 => {
                let rm = (b & 0xF) as u8;
                // `(1)(1)(1)(1)(0)(0)(0)(0) 000H Rm`.
                if b & 0xFFE0 != 0xF000 || rn == 13 || bad(rm) {
                    return unpredictable;
                }
                Op::Table { half: op3 == 1, rn, rm }
            }
            0b0100 | 0b0101 => {
                if bad(rt) || rn == 15 || b & 0x0F0F != 0x0F0F {
                    return unpredictable;
                }
                Op::Ldrex { w: if op3 == 0b0100 { Width::Byte } else { Width::Half }, rt, rn, imm: 0 }
            }
            _ => Op::Undefined { raw },
        },
        // LDRD/STRD (immediate, literal) T1: `1110 100P U1WL Rn | Rt Rt2 imm8`.
        _ => {
            let (index, wback, load) = (a & (1 << 8) != 0, a & (1 << 5) != 0, a & (1 << 4) != 0);
            let add = a & (1 << 7) != 0;
            let rt2 = rt2_or_rd;
            let offset = (b & 0xFF) << 2;
            if load && rn == 15 {
                if bad(rt) || bad(rt2) || rt == rt2 || wback {
                    return unpredictable;
                }
                let base = literal_base(pc);
                let addr = if add { base.wrapping_add(offset) } else { base.wrapping_sub(offset) };
                return Op::LoadDualLit { rt, rt2, addr };
            }
            if (wback && (rn == rt || rn == rt2)) || bad(rt) || bad(rt2) || (load && rt == rt2) || (!load && rn == 15) {
                return unpredictable;
            }
            let imm = if add { offset as i32 } else { -(offset as i32) };
            Op::Dual { load, rt, rt2, rn, imm, index, wback }
        }
    }
}

/// A5.3.11 "Data processing (shifted register)".
fn shifted_register(a: u32, b: u32, raw: u32) -> Op {
    let s = a & (1 << 4) != 0;
    let (rn, rd, rm) = ((a & 0xF) as u8, ((b >> 8) & 0xF) as u8, (b & 0xF) as u8);
    let (shift, amount) = decode_imm_shift(b >> 4, ((b >> 10) & 0x1C) | ((b >> 6) & 3));
    let unpredictable = Op::Unpredictable { raw };
    if b & 0x8000 != 0 {
        return unpredictable;
    }
    let flags = if s { SetFlags::Yes } else { SetFlags::No };
    let op = |op, rd, rn| Op::DpReg { op, s: flags, rd, rn, rm, shift, amount };
    let compare = rd == 15 && s;
    match (a >> 5) & 0xF {
        // AND/TST, EOR/TEQ.
        0b0000 | 0b0100 if compare => {
            if bad(rn) || bad(rm) {
                return unpredictable;
            }
            op(if a & (0xF << 5) == 0 { Dp::Tst } else { Dp::Teq }, 0, rn)
        }
        0b0000 | 0b0100 => {
            if rd == 13 || rd == 15 || bad(rn) || bad(rm) {
                return unpredictable;
            }
            op(if a & (0xF << 5) == 0 { Dp::And } else { Dp::Eor }, rd, rn)
        }
        0b0001 | 0b1010 | 0b1011 | 0b1110 => {
            if bad(rd) || bad(rn) || bad(rm) {
                return unpredictable;
            }
            let kind = match (a >> 5) & 0xF {
                0b0001 => Dp::Bic,
                0b1010 => Dp::Adc,
                0b1011 => Dp::Sbc,
                _ => Dp::Rsb,
            };
            op(kind, rd, rn)
        }
        // MOV (register) T3 and the immediate shifts (LSL, LSR, ASR, ROR, RRX).
        0b0010 if rn == 15 => {
            let plain = shift == Shift::Lsl && amount == 0;
            let forbidden =
                if !plain || s { bad(rd) || bad(rm) } else { rd == 15 || rm == 15 || (rd == 13 && rm == 13) };
            if forbidden { unpredictable } else { op(Dp::Mov, rd, 0) }
        }
        // MVN (register) T2.
        0b0011 if rn == 15 => {
            if bad(rd) || bad(rm) {
                unpredictable
            } else {
                op(Dp::Mvn, rd, 0)
            }
        }
        // ORR, ORN.
        0b0010 | 0b0011 => {
            if bad(rd) || rn == 13 || bad(rm) {
                return unpredictable;
            }
            op(if (a >> 5) & 0xF == 0b0010 { Dp::Orr } else { Dp::Orn }, rd, rn)
        }
        // ADD/CMN, SUB/CMP.
        0b1000 | 0b1101 => {
            let add = (a >> 5) & 0xF == 0b1000;
            if compare {
                if rn == 15 || bad(rm) {
                    return unpredictable;
                }
                return op(if add { Dp::Cmn } else { Dp::Cmp }, 0, rn);
            }
            let forbidden = if rn == 13 {
                // ADD (SP plus register) T3, SUB (SP minus register) T1.
                (rd == 13 && (shift != Shift::Lsl || amount > 3)) || rd == 15 || bad(rm)
            } else {
                rd == 13 || rd == 15 || rn == 15 || bad(rm)
            };
            if forbidden { unpredictable } else { op(if add { Dp::Add } else { Dp::Sub }, rd, rn) }
        }
        _ => Op::Undefined { raw },
    }
}

/// A5.3.1 "Data processing (modified immediate)".
fn modified_immediate(a: u32, b: u32, raw: u32) -> Op {
    let s = a & (1 << 4) != 0;
    let (rn, rd) = ((a & 0xF) as u8, ((b >> 8) & 0xF) as u8);
    let imm12 = ((a >> 10) & 1) << 11 | ((b >> 12) & 7) << 8 | (b & 0xFF);
    let unpredictable = Op::Unpredictable { raw };
    let Some((imm, carry)) = thumb_expand_imm(imm12) else { return unpredictable };
    let flags = if s { SetFlags::Yes } else { SetFlags::No };
    let op = |op, rd, rn| Op::DpImm { op, s: flags, rd, rn, carry, imm };
    let compare = rd == 15 && s;
    match (a >> 5) & 0xF {
        0b0000 | 0b0100 if compare => {
            if bad(rn) {
                return unpredictable;
            }
            op(if a & (0xF << 5) == 0 { Dp::Tst } else { Dp::Teq }, 0, rn)
        }
        0b0000 | 0b0100 => {
            if rd == 13 || rd == 15 || bad(rn) {
                return unpredictable;
            }
            op(if a & (0xF << 5) == 0 { Dp::And } else { Dp::Eor }, rd, rn)
        }
        0b0001 | 0b1010 | 0b1011 | 0b1110 => {
            if bad(rd) || bad(rn) {
                return unpredictable;
            }
            let kind = match (a >> 5) & 0xF {
                0b0001 => Dp::Bic,
                0b1010 => Dp::Adc,
                0b1011 => Dp::Sbc,
                _ => Dp::Rsb,
            };
            op(kind, rd, rn)
        }
        // MOV (immediate) T2, MVN (immediate) T1.
        0b0010 | 0b0011 if rn == 15 => {
            if bad(rd) {
                return unpredictable;
            }
            op(if (a >> 5) & 0xF == 0b0010 { Dp::Mov } else { Dp::Mvn }, rd, 0)
        }
        0b0010 | 0b0011 => {
            if bad(rd) || rn == 13 {
                return unpredictable;
            }
            op(if (a >> 5) & 0xF == 0b0010 { Dp::Orr } else { Dp::Orn }, rd, rn)
        }
        0b1000 | 0b1101 => {
            let add = (a >> 5) & 0xF == 0b1000;
            if compare {
                if rn == 15 {
                    return unpredictable;
                }
                return op(if add { Dp::Cmn } else { Dp::Cmp }, 0, rn);
            }
            // With SP as Rn (ADD SP plus immediate T3, SUB SP minus immediate T2) Rd may be SP.
            let forbidden = rd == 15 || (rn != 13 && (rd == 13 || rn == 15));
            if forbidden { unpredictable } else { op(if add { Dp::Add } else { Dp::Sub }, rd, rn) }
        }
        _ => Op::Undefined { raw },
    }
}

/// A5.3.3 "Data processing (plain binary immediate)".
fn plain_immediate(a: u32, b: u32, raw: u32, pc: u32) -> Op {
    let (rn, rd) = ((a & 0xF) as u8, ((b >> 8) & 0xF) as u8);
    let imm12 = ((a >> 10) & 1) << 11 | ((b >> 12) & 7) << 8 | (b & 0xFF);
    let imm16 = (a & 0xF) << 12 | imm12;
    // `imm3:imm2`, the shift or least significant bit of the saturate and bit-field group.
    let imm5 = ((b >> 10) & 0x1C) | ((b >> 6) & 3);
    let low5 = (b & 0x1F) as u8;
    let unpredictable = Op::Unpredictable { raw };
    let mov = |rd, imm| Op::DpImm { op: Dp::Mov, s: SetFlags::No, rd, rn: 0, carry: ImmCarry::Keep, imm };
    let op = (a >> 4) & 0x1F;
    // The saturate and bit-field group has `(0)` at hw1 bit 10 and hw2 bit 5.
    if op >= 0b1_0000 && (a & (1 << 10) != 0 || b & (1 << 5) != 0) {
        return unpredictable;
    }
    match op {
        // ADDW T4 / ADR T3, SUBW T4 / ADR T2.
        0b0_0000 | 0b0_1010 => {
            let add = op == 0;
            if rn == 15 {
                if bad(rd) {
                    return unpredictable;
                }
                let base = literal_base(pc);
                return mov(rd, if add { base.wrapping_add(imm12) } else { base.wrapping_sub(imm12) });
            }
            if rd == 15 || (rn != 13 && rd == 13) {
                return unpredictable;
            }
            let kind = if add { Dp::Add } else { Dp::Sub };
            Op::DpImm { op: kind, s: SetFlags::No, rd, rn, carry: ImmCarry::Keep, imm: imm12 }
        }
        // MOVW T3, MOVT T1.
        0b0_0100 if !bad(rd) => mov(rd, imm16),
        0b0_1100 if !bad(rd) => Op::Movt { rd, imm: imm16 as u16 },
        0b0_0100 | 0b0_1100 => unpredictable,
        // SSAT, USAT (the ASR #0 forms are SSAT16/USAT16, DSP extension).
        0b1_0000 | 0b1_0010 | 0b1_1000 | 0b1_1010 => {
            let asr = op & 0b10 != 0;
            if asr && imm5 == 0 {
                return Op::Undefined { raw };
            }
            if bad(rd) || bad(rn) {
                return unpredictable;
            }
            let signed = op & 0b1000 == 0;
            Op::Sat {
                signed,
                rd,
                rn,
                bits: if signed { low5 + 1 } else { low5 },
                shift: if asr { Shift::Asr } else { Shift::Lsl },
                amount: imm5 as u8,
            }
        }
        // SBFX, UBFX.
        0b1_0100 | 0b1_1100 => {
            if bad(rd) || bad(rn) || imm5 + u32::from(low5) > 31 {
                return unpredictable;
            }
            Op::Bfx { signed: op == 0b1_0100, rd, rn, lsb: imm5 as u8, width: low5 + 1 }
        }
        // BFI, BFC (Rn = PC).
        0b1_0110 => {
            if bad(rd) || rn == 13 || u32::from(low5) < imm5 {
                return unpredictable;
            }
            Op::Bfi { rd, rn, lsb: imm5 as u8, msb: low5 }
        }
        _ => Op::Undefined { raw },
    }
}

/// A5.3.4 "Branches and miscellaneous control".
fn branch_and_control(a: u32, b: u32, raw: u32, pc: u32) -> Op {
    let op = (a >> 4) & 0x7F;
    let op1 = (b >> 12) & 7;
    let unpredictable = Op::Unpredictable { raw };
    let next = pc.wrapping_add(4);
    let s = (a >> 10) & 1;
    let (j1, j2) = ((b >> 13) & 1, (b >> 11) & 1);
    match op1 {
        // Permanently UNDEFINED (`UDF.W`).
        0b010 if op == 0x7F => Op::Undefined { raw },
        0b000 | 0b010 if op & 0x38 != 0x38 => {
            // B T3: `S:J2:J1:imm6:imm11:0`.
            let imm = s << 20 | j2 << 19 | j1 << 18 | (a & 0x3F) << 12 | (b & 0x7FF) << 1;
            Op::BCond { cond: ((a >> 6) & 0xF) as u8, target: next.wrapping_add(sign_extend(imm, 21)) }
        }
        // MSR, hints, CLREX/barriers, MRS: hw2 bit 13 is `(0)`.
        0b010 => unpredictable,
        0b000 => match op {
            // MSR T1: `11110 0 1110 0 (0) Rn | 10 (0) 0 mask (0)(0) SYSm`.
            0x38 | 0x39 => {
                let (rn, mask, sysm) = ((a & 0xF) as u8, (b >> 10) & 3, b & 0xFF);
                // Without the DSP extension only `mask = 10` (APSR_nzcvq) is defined.
                if op == 0x39 || b & 0x0300 != 0 || mask != 0b10 || bad(rn) || !sysm::is_valid(sysm) {
                    return unpredictable;
                }
                Op::Msr { rn, sysm: sysm as u8 }
            }
            // Hints: `11110 0 1110 1 0 (1)(1)(1)(1) | 10 (0) 0 (0) op1 op2`; CPS.W is ARMv7-A.
            0x3A => {
                if b & 0x0700 != 0 {
                    return Op::Undefined { raw };
                }
                if a & 0xF != 0xF || b & 0x0800 != 0 {
                    return unpredictable;
                }
                match b & 0xFF {
                    2 => Op::Wfe,
                    3 => Op::Wfi,
                    4 => Op::Sev,
                    // NOP, YIELD, DBG and the unallocated hints.
                    _ => Op::Nop,
                }
            }
            // Miscellaneous control: `... (1)(1)(1)(1) | 10 (0) 0 (1)(1)(1)(1) op option`.
            0x3B => {
                let kind = match (b >> 4) & 0xF {
                    // CLREX: the option field is `(1)(1)(1)(1)`.
                    0b0010 if b & 0xF != 0xF => return unpredictable,
                    0b0010 => Op::Clrex,
                    // DSB, DMB, ISB: the core has no buffers to drain, so they are NOPs.
                    0b0100..=0b0110 => Op::Nop,
                    _ => return Op::Undefined { raw },
                };
                if a & 0xF != 0xF || b & 0x0F00 != 0x0F00 {
                    return unpredictable;
                }
                kind
            }
            // MRS T1: `11110 0 1111 1 (0) (1)(1)(1)(1) | 10 (0) 0 Rd SYSm`.
            0x3E | 0x3F => {
                let (rd, sysm) = (((b >> 8) & 0xF) as u8, b & 0xFF);
                if op == 0x3F || a & 0xF != 0xF || bad(rd) || !sysm::is_valid(sysm) {
                    return unpredictable;
                }
                Op::Mrs { rd, sysm: sysm as u8 }
            }
            _ => Op::Undefined { raw },
        },
        // B T4 and BL T1: `I1 = NOT(J1 EOR S)`, `I2 = NOT(J2 EOR S)`.
        0b001 | 0b011 | 0b101 | 0b111 => {
            let (i1, i2) = (!(j1 ^ s) & 1, !(j2 ^ s) & 1);
            let imm = s << 24 | i1 << 23 | i2 << 22 | (a & 0x3FF) << 12 | (b & 0x7FF) << 1;
            let target = next.wrapping_add(sign_extend(imm, 25));
            if op1 & 0b100 == 0 { Op::B { target } } else { Op::Bl { target } }
        }
        // BLX (immediate) is ARM-state only.
        _ => Op::Undefined { raw },
    }
}

/// A5.3.10 "Store single data item".
fn store_single(a: u32, b: u32, raw: u32) -> Op {
    let (rn, rt) = ((a & 0xF) as u8, (b >> 12) as u8);
    let unpredictable = Op::Unpredictable { raw };
    if rn == 15 {
        return Op::Undefined { raw };
    }
    let w = match (a >> 5) & 3 {
        0 => Width::Byte,
        1 => Width::Half,
        2 => Width::Word,
        _ => return Op::Undefined { raw },
    };
    // STR may store SP; STRB and STRH may not.
    let bad_rt = if w == Width::Word { rt == 15 } else { bad(rt) };
    if a & (1 << 7) != 0 {
        // T2/T3 with a 12-bit positive offset.
        if bad_rt {
            return unpredictable;
        }
        return Op::Store { w, rt, rn, imm: (b & 0xFFF) as i32 };
    }
    if b & (1 << 11) == 0 {
        // Register offset: `Rt 0 00000 imm2 Rm`.
        let rm = (b & 0xF) as u8;
        if b & 0x07C0 != 0 {
            return Op::Undefined { raw };
        }
        if bad_rt || bad(rm) {
            return unpredictable;
        }
        return Op::StoreReg { w, rt, rn, rm, lsl: ((b >> 4) & 3) as u8 };
    }
    let (index, add, wback) = (b & (1 << 10) != 0, b & (1 << 9) != 0, b & (1 << 8) != 0);
    let imm8 = (b & 0xFF) as i32;
    if index && add && !wback {
        // STRT, STRBT, STRHT: unprivileged, which is the same access with no MPU.
        if bad(rt) {
            return unpredictable;
        }
        return Op::Store { w, rt, rn, imm: imm8 };
    }
    if !index && !wback {
        return Op::Undefined { raw };
    }
    if bad_rt || (wback && rn == rt) {
        return unpredictable;
    }
    let imm = if add { imm8 } else { -imm8 };
    if wback { Op::StoreIdx { w, rt, rn, imm, pre: index } } else { Op::Store { w, rt, rn, imm } }
}

/// A5.3.7 "Load word".
fn load_word(a: u32, b: u32, raw: u32, pc: u32) -> Op {
    let (rn, rt) = ((a & 0xF) as u8, (b >> 12) as u8);
    let unpredictable = Op::Unpredictable { raw };
    let w = Width::Word;
    if a & (1 << 8) != 0 {
        return Op::Undefined { raw };
    }
    if rn == 15 {
        let (base, imm12) = (literal_base(pc), b & 0xFFF);
        let addr = if a & (1 << 7) != 0 { base.wrapping_add(imm12) } else { base.wrapping_sub(imm12) };
        return Op::LoadLit { w, rt, addr };
    }
    if a & (1 << 7) != 0 {
        return Op::Load { w, rt, rn, imm: (b & 0xFFF) as i32 };
    }
    if b & (1 << 11) == 0 {
        let rm = (b & 0xF) as u8;
        if b & 0x07C0 != 0 {
            return Op::Undefined { raw };
        }
        if bad(rm) {
            return unpredictable;
        }
        return Op::LoadReg { w, rt, rn, rm, lsl: ((b >> 4) & 3) as u8 };
    }
    let (index, add, wback) = (b & (1 << 10) != 0, b & (1 << 9) != 0, b & (1 << 8) != 0);
    let imm8 = (b & 0xFF) as i32;
    if index && add && !wback {
        // LDRT.
        if bad(rt) {
            return unpredictable;
        }
        return Op::Load { w, rt, rn, imm: imm8 };
    }
    if !index && !wback {
        return Op::Undefined { raw };
    }
    if wback && rn == rt {
        return unpredictable;
    }
    let imm = if add { imm8 } else { -imm8 };
    if wback { Op::LoadIdx { w, rt, rn, imm, pre: index } } else { Op::Load { w, rt, rn, imm } }
}

/// A5.3.8 "Load halfword, memory hints" and A5.3.9 "Load byte, memory hints": the same layout.
/// `Rt = 15` turns most forms into `PLD`, `PLI` or an unallocated hint, all NOPs here.
fn load_byte_half(a: u32, b: u32, raw: u32, pc: u32, half: bool) -> Op {
    let (rn, rt) = ((a & 0xF) as u8, (b >> 12) as u8);
    let unpredictable = Op::Unpredictable { raw };
    let signed = a & (1 << 8) != 0;
    let w = match (half, signed) {
        (false, false) => Width::Byte,
        (false, true) => Width::SignedByte,
        (true, false) => Width::Half,
        (true, true) => Width::SignedHalf,
    };
    if rn == 15 {
        if rt == 15 {
            return Op::Nop;
        }
        if rt == 13 {
            return unpredictable;
        }
        let (base, imm12) = (literal_base(pc), b & 0xFFF);
        let addr = if a & (1 << 7) != 0 { base.wrapping_add(imm12) } else { base.wrapping_sub(imm12) };
        return Op::LoadLit { w, rt, addr };
    }
    if a & (1 << 7) != 0 {
        // 12-bit positive offset.
        return match rt {
            15 => Op::Nop,
            13 => unpredictable,
            _ => Op::Load { w, rt, rn, imm: (b & 0xFFF) as i32 },
        };
    }
    if b & (1 << 11) == 0 {
        let rm = (b & 0xF) as u8;
        if b & 0x07C0 != 0 {
            return Op::Undefined { raw };
        }
        if rt == 15 {
            return if bad(rm) { unpredictable } else { Op::Nop };
        }
        if rt == 13 || bad(rm) {
            return unpredictable;
        }
        return Op::LoadReg { w, rt, rn, rm, lsl: ((b >> 4) & 3) as u8 };
    }
    let (index, add, wback) = (b & (1 << 10) != 0, b & (1 << 9) != 0, b & (1 << 8) != 0);
    let imm8 = (b & 0xFF) as i32;
    if index && !add && !wback && rt == 15 {
        return Op::Nop;
    }
    if index && add && !wback {
        // LDRBT, LDRHT, LDRSBT, LDRSHT.
        if bad(rt) {
            return unpredictable;
        }
        return Op::Load { w, rt, rn, imm: imm8 };
    }
    if !index && !wback {
        return Op::Undefined { raw };
    }
    if bad(rt) || (wback && rn == rt) {
        return unpredictable;
    }
    let imm = if add { imm8 } else { -imm8 };
    if wback { Op::LoadIdx { w, rt, rn, imm, pre: index } } else { Op::Load { w, rt, rn, imm } }
}

/// A5.3.12 "Data processing (register)" and A5.3.13 "Miscellaneous operations".
fn register(a: u32, b: u32, raw: u32) -> Op {
    let (op1, op2) = ((a >> 4) & 0xF, (b >> 4) & 0xF);
    let (rn, rd, rm) = ((a & 0xF) as u8, ((b >> 8) & 0xF) as u8, (b & 0xF) as u8);
    let unpredictable = Op::Unpredictable { raw };
    if b >> 12 != 0xF {
        return Op::Undefined { raw };
    }
    if op2 == 0 && op1 & 0x8 == 0 {
        // LSL, LSR, ASR, ROR (register) T2.
        if bad(rd) || bad(rn) || bad(rm) {
            return unpredictable;
        }
        let shift = [Shift::Lsl, Shift::Lsr, Shift::Asr, Shift::Ror][(op1 >> 1) as usize];
        let s = if op1 & 1 != 0 { SetFlags::Yes } else { SetFlags::No };
        return Op::ShiftReg { shift, s, rd, rn, rm };
    }
    if op2 & 0x8 != 0 && op1 & 0x8 == 0 {
        // SXTH, UXTH, SXTB, UXTB (T2); the accumulating and 16-bit forms are DSP.
        let kind = match op1 {
            0b0000 => Extend::Sxth,
            0b0001 => Extend::Uxth,
            0b0100 => Extend::Sxtb,
            0b0101 => Extend::Uxtb,
            _ => return Op::Undefined { raw },
        };
        if rn != 15 {
            return Op::Undefined { raw };
        }
        if b & (1 << 6) != 0 || bad(rd) || bad(rm) {
            return unpredictable;
        }
        return Op::Extend { kind, rd, rm, rotation: (((b >> 4) & 3) * 8) as u8 };
    }
    if op1 & 0xC == 0x8 && op2 & 0xC == 0x8 {
        let op = match (op1 & 3, op2 & 3) {
            (0b01, 0b00) => BitOp::Rev,
            (0b01, 0b01) => BitOp::Rev16,
            (0b01, 0b10) => BitOp::Rbit,
            (0b01, 0b11) => BitOp::Revsh,
            (0b11, 0b00) => BitOp::Clz,
            // QADD, QSUB, SEL, ...: DSP.
            _ => return Op::Undefined { raw },
        };
        // The two Rm fields must match (`Consistent(Rm)`).
        if rn != rm || bad(rd) || bad(rm) {
            return unpredictable;
        }
        return Op::Bits { op, rd, rm };
    }
    // Parallel addition and subtraction: DSP.
    Op::Undefined { raw }
}

/// A5.3.16 "Multiply, multiply accumulate, and absolute difference".
fn multiply(a: u32, b: u32, raw: u32) -> Op {
    let (rn, ra, rd, rm) = ((a & 0xF) as u8, (b >> 12) as u8, ((b >> 8) & 0xF) as u8, (b & 0xF) as u8);
    let unpredictable = Op::Unpredictable { raw };
    if (a >> 4) & 7 != 0 || b & 0xE0 != 0 {
        // SMLA<x><y>, SMUL<x><y>, SMMLA, USAD8, ...: DSP.
        return Op::Undefined { raw };
    }
    let sub = b & 0x10 != 0;
    if bad(rd) || bad(rn) || bad(rm) || ra == 13 || (sub && ra == 15) {
        return unpredictable;
    }
    match (sub, ra) {
        (false, 15) => Op::Mul { s: SetFlags::No, rd, rn, rm },
        _ => Op::Mla { sub, rd, rn, rm, ra },
    }
}

/// A5.3.17 "Long multiply, long multiply accumulate, and divide".
fn long_multiply_divide(a: u32, b: u32, raw: u32) -> Op {
    let (rn, rdlo, rdhi, rm) = ((a & 0xF) as u8, (b >> 12) as u8, ((b >> 8) & 0xF) as u8, (b & 0xF) as u8);
    let unpredictable = Op::Unpredictable { raw };
    let kind = match ((a >> 4) & 7, (b >> 4) & 0xF) {
        (0b000, 0b0000) => MulLong::Smull,
        (0b010, 0b0000) => MulLong::Umull,
        (0b100, 0b0000) => MulLong::Smlal,
        (0b110, 0b0000) => MulLong::Umlal,
        (0b001 | 0b011, 0b1111) => {
            // SDIV, UDIV: `(1)(1)(1)(1) Rd 1111 Rm`.
            let rd = rdhi;
            if rdlo != 0xF || bad(rd) || bad(rn) || bad(rm) {
                return unpredictable;
            }
            return Op::Div { signed: (a >> 4) & 7 == 0b001, rd, rn, rm };
        }
        _ => return Op::Undefined { raw },
    };
    if bad(rdlo) || bad(rdhi) || bad(rn) || bad(rm) || rdhi == rdlo {
        return unpredictable;
    }
    Op::MulLong { kind, rdlo, rdhi, rn, rm }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PC: u32 = 0x0800_2000;

    fn d(hw1: u16, hw2: u16) -> Op {
        decode32(hw1, hw2, PC)
    }

    #[test]
    fn branches() {
        // bl +0x1234: S=0, imm10=1, imm11=0x11A, J1=J2=1.
        assert_eq!(d(0xF001, 0xF91A), Op::Bl { target: PC + 4 + 0x1234 });
        // b.w -4 (to itself): S=1, imm10=0x3FF, J1=J2=1, imm11=0x7FE.
        assert_eq!(d(0xF7FF, 0xBFFE), Op::B { target: PC });
        // beq.w +0x100.
        assert_eq!(d(0xF000, 0x8080), Op::BCond { cond: 0, target: PC + 4 + 0x100 });
        assert_eq!(d(0xF7F0, 0xA000), Op::Undefined { raw: 0xF7F0_A000 }, "UDF.W");
    }

    #[test]
    fn data_processing() {
        // movw r1, #0xBEEF
        assert!(matches!(d(0xF64B, 0x61EF), Op::DpImm { op: Dp::Mov, rd: 1, imm: 0xBEEF, .. }));
        // movt r1, #0x1234
        assert_eq!(d(0xF2C1, 0x2134), Op::Movt { rd: 1, imm: 0x1234 });
        // adds.w r0, r1, #0x00FF00FF
        assert!(matches!(
            d(0xF111, 0x10FF),
            Op::DpImm { op: Dp::Add, s: SetFlags::Yes, rd: 0, rn: 1, imm: 0x00FF_00FF, .. }
        ));
        // cmp.w r2, #1 (Rd = PC with S set).
        assert!(matches!(d(0xF1B2, 0x0F01), Op::DpImm { op: Dp::Cmp, rn: 2, imm: 1, .. }));
        // add.w sp, sp, #8 is allowed, add.w sp, r0, #8 is not.
        assert!(matches!(d(0xF10D, 0x0D08), Op::DpImm { op: Dp::Add, rd: 13, rn: 13, .. }));
        assert_eq!(d(0xF100, 0x0D08), Op::Unpredictable { raw: 0xF100_0D08 });
        // lsl.w r0, r1, #3 is MOV with a shift.
        assert!(matches!(d(0xEA4F, 0x00C1), Op::DpReg { op: Dp::Mov, rd: 0, rm: 1, shift: Shift::Lsl, amount: 3, .. }));
        // ubfx r0, r1, #4, #8
        assert_eq!(d(0xF3C1, 0x1007), Op::Bfx { signed: false, rd: 0, rn: 1, lsb: 4, width: 8 });
        // bfc r0, #8, #4
        assert_eq!(d(0xF36F, 0x200B), Op::Bfi { rd: 0, rn: 15, lsb: 8, msb: 11 });
        // ssat r0, #16, r1
        assert!(matches!(d(0xF301, 0x000F), Op::Sat { signed: true, bits: 16, .. }));
        // sdiv r0, r1, r2; udiv r0, r1, r2
        assert_eq!(d(0xFB91, 0xF0F2), Op::Div { signed: true, rd: 0, rn: 1, rm: 2 });
        assert_eq!(d(0xFBB1, 0xF0F2), Op::Div { signed: false, rd: 0, rn: 1, rm: 2 });
        // smulbb is DSP: UNDEFINED on a Cortex-M3.
        assert_eq!(d(0xFB11, 0xF002), Op::Undefined { raw: 0xFB11_F002 });
        // clz r0, r1 needs both Rm fields equal.
        assert_eq!(d(0xFAB1, 0xF081), Op::Bits { op: BitOp::Clz, rd: 0, rm: 1 });
        assert_eq!(d(0xFAB2, 0xF081), Op::Unpredictable { raw: 0xFAB2_F081 });
    }

    #[test]
    fn loads_and_stores() {
        // ldr.w r0, [r1, #0x123]
        assert_eq!(d(0xF8D1, 0x0123), Op::Load { w: Width::Word, rt: 0, rn: 1, imm: 0x123 });
        // ldr r0, [r1], #4 (post-indexed)
        assert_eq!(d(0xF851, 0x0B04), Op::LoadIdx { w: Width::Word, rt: 0, rn: 1, imm: 4, pre: false });
        // str r0, [sp, #-4]! (PUSH T3)
        assert_eq!(d(0xF84D, 0x0D04), Op::StoreIdx { w: Width::Word, rt: 0, rn: 13, imm: -4, pre: true });
        // ldr.w pc, [pc, #-8] at a word-aligned address.
        assert_eq!(d(0xF85F, 0xF008), Op::LoadLit { w: Width::Word, rt: 15, addr: PC + 4 - 8 });
        // pld [r0, #4]
        assert_eq!(d(0xF890, 0xF004), Op::Nop);
        // ldrsh r0, [r1, r2, lsl #1]
        assert_eq!(d(0xF931, 0x0012), Op::LoadReg { w: Width::SignedHalf, rt: 0, rn: 1, rm: 2, lsl: 1 });
        // push.w {r4-r11, lr}
        assert_eq!(d(0xE92D, 0x4FF0), Op::Stm { rn: 13, list: 0x4FF0, wback: true, db: true });
        // pop.w {r4-r11, pc}
        assert_eq!(d(0xE8BD, 0x8FF0), Op::Ldm { rn: 13, list: 0x8FF0, wback: true, db: false });
        assert_eq!(d(0xE8BD, 0xC000), Op::Unpredictable { raw: 0xE8BD_C000 }, "PC and LR together");
        // ldrd r0, r1, [r2, #8]
        assert_eq!(d(0xE9D2, 0x0102), Op::Dual { load: true, rt: 0, rt2: 1, rn: 2, imm: 8, index: true, wback: false });
        // tbb [pc, r1]
        assert_eq!(d(0xE8DF, 0xF001), Op::Table { half: false, rn: 15, rm: 1 });
        // ldrex r0, [r1]; strex r2, r0, [r1]
        assert_eq!(d(0xE851, 0x0F00), Op::Ldrex { w: Width::Word, rt: 0, rn: 1, imm: 0 });
        assert_eq!(d(0xE841, 0x0200), Op::Strex { w: Width::Word, rd: 2, rt: 0, rn: 1, imm: 0 });
        assert_eq!(d(0xF80F, 0x0000), Op::Undefined { raw: 0xF80F_0000 }, "stores never use PC as base");
    }

    #[test]
    fn system() {
        // mrs r0, primask; msr basepri, r1
        assert_eq!(d(0xF3EF, 0x8010), Op::Mrs { rd: 0, sysm: sysm::PRIMASK });
        assert_eq!(d(0xF381, 0x8811), Op::Msr { rn: 1, sysm: sysm::BASEPRI });
        assert_eq!(d(0xF381, 0x8411), Op::Unpredictable { raw: 0xF381_8411 }, "mask 01 needs the DSP extension");
        assert_eq!(d(0xF3BF, 0x8F5F), Op::Nop, "dmb sy");
        assert_eq!(d(0xF3BF, 0x8F2F), Op::Clrex);
        assert_eq!(d(0xF3AF, 0x8003), Op::Wfi);
    }
}

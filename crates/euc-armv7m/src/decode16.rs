//! 16-bit Thumb encodings (ARM ARM ARMv7-M, A5.2 "16-bit Thumb instruction encoding").
//!
//! Section numbers in comments refer to DDI 0403E. Encodings follow the ARM ARM tables one to
//! one; UNPREDICTABLE register choices decode to [`Op::Unpredictable`].

use crate::alu::decode_imm_shift;
use crate::op::{BitOp, Dp, Extend, ImmCarry, Op, SetFlags, Shift, Width};

/// The value the PC reads as for a literal or `ADR`: `Align(PC, 4)` with PC = address + 4.
pub(crate) const fn literal_base(pc: u32) -> u32 {
    pc.wrapping_add(4) & !3
}

/// Sign-extends the low `bits` bits of `value`.
pub(crate) const fn sign_extend(value: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    (((value << shift) as i32) >> shift) as u32
}

/// Decodes the 16-bit instruction `hw` at address `pc`.
pub(crate) fn decode16(hw: u16, pc: u32) -> Op {
    let h = u32::from(hw);
    let lo3 = |at: u32| ((h >> at) & 7) as u8;
    match h >> 10 {
        0b00_0000..=0b00_1111 => shift_add_sub_move_compare(h),
        0b01_0000 => data_processing(h),
        0b01_0001 => special_and_branch_exchange(h),
        // LDR (literal) T1.
        0b01_0010 | 0b01_0011 => {
            Op::LoadLit { w: Width::Word, rt: lo3(8), addr: literal_base(pc).wrapping_add((h & 0xFF) << 2) }
        }
        0b01_0100..=0b10_0111 => load_store_single(h),
        // ADR T1: a MOV of the resolved address.
        0b10_1000 | 0b10_1001 => Op::DpImm {
            op: Dp::Mov,
            s: SetFlags::No,
            rd: lo3(8),
            rn: 0,
            carry: ImmCarry::Keep,
            imm: literal_base(pc).wrapping_add((h & 0xFF) << 2),
        },
        // ADD (SP plus immediate) T1.
        0b10_1010 | 0b10_1011 => {
            Op::DpImm { op: Dp::Add, s: SetFlags::No, rd: lo3(8), rn: 13, carry: ImmCarry::Keep, imm: (h & 0xFF) << 2 }
        }
        0b10_1100..=0b10_1111 => miscellaneous(h, pc),
        // STM T1: always writes back.
        0b11_0000 | 0b11_0001 => {
            let list = (h & 0xFF) as u16;
            if list == 0 { Op::Unpredictable { raw: h } } else { Op::Stm { rn: lo3(8), list, wback: true, db: false } }
        }
        // LDM T1: writes back unless the base is loaded.
        0b11_0010 | 0b11_0011 => {
            let (rn, list) = (lo3(8), (h & 0xFF) as u16);
            if list == 0 {
                Op::Unpredictable { raw: h }
            } else {
                Op::Ldm { rn, list, wback: list & (1 << rn) == 0, db: false }
            }
        }
        0b11_0100..=0b11_0111 => conditional_branch_and_svc(h, pc),
        // B T2.
        0b11_1000 | 0b11_1001 => Op::B { target: pc.wrapping_add(4).wrapping_add(sign_extend((h & 0x7FF) << 1, 12)) },
        _ => Op::Undefined { raw: h },
    }
}

/// A5.2.1 "Shift (immediate), add, subtract, move, and compare".
fn shift_add_sub_move_compare(h: u32) -> Op {
    let (r0, r3, r6, r8) = ((h & 7) as u8, ((h >> 3) & 7) as u8, ((h >> 6) & 7) as u8, ((h >> 8) & 7) as u8);
    let imm5 = (h >> 6) & 0x1F;
    let shifted = |kind: u32| {
        let (shift, amount) = decode_imm_shift(kind, imm5);
        Op::DpReg { op: Dp::Mov, s: SetFlags::OutsideIt, rd: r0, rn: 0, rm: r3, shift, amount }
    };
    let imm = |op, s, rd, rn, imm| Op::DpImm { op, s, rd, rn, carry: ImmCarry::Keep, imm };
    let reg = |op, rd, rn, rm| Op::DpReg { op, s: SetFlags::OutsideIt, rd, rn, rm, shift: Shift::Lsl, amount: 0 };
    match (h >> 9) & 0x1F {
        // LSL #0 is MOV (register) T2, `MOVS Rd, Rm`: flags always, C unchanged.
        0b0_0000..=0b0_0011 if imm5 == 0 => {
            Op::DpReg { op: Dp::Mov, s: SetFlags::Yes, rd: r0, rn: 0, rm: r3, shift: Shift::Lsl, amount: 0 }
        }
        0b0_0000..=0b0_0011 => shifted(0),
        0b0_0100..=0b0_0111 => shifted(1),
        0b0_1000..=0b0_1011 => shifted(2),
        0b0_1100 => reg(Dp::Add, r0, r3, r6),
        0b0_1101 => reg(Dp::Sub, r0, r3, r6),
        0b0_1110 => imm(Dp::Add, SetFlags::OutsideIt, r0, r3, u32::from(r6)),
        0b0_1111 => imm(Dp::Sub, SetFlags::OutsideIt, r0, r3, u32::from(r6)),
        0b1_0000..=0b1_0011 => imm(Dp::Mov, SetFlags::OutsideIt, r8, 0, h & 0xFF),
        0b1_0100..=0b1_0111 => imm(Dp::Cmp, SetFlags::Yes, 0, r8, h & 0xFF),
        0b1_1000..=0b1_1011 => imm(Dp::Add, SetFlags::OutsideIt, r8, r8, h & 0xFF),
        _ => imm(Dp::Sub, SetFlags::OutsideIt, r8, r8, h & 0xFF),
    }
}

/// A5.2.2 "Data processing": two low registers, `Rdn` and `Rm`.
fn data_processing(h: u32) -> Op {
    let (rdn, rm) = ((h & 7) as u8, ((h >> 3) & 7) as u8);
    let reg = |op, s| Op::DpReg { op, s, rd: rdn, rn: rdn, rm, shift: Shift::Lsl, amount: 0 };
    let by_reg = |shift| Op::ShiftReg { shift, s: SetFlags::OutsideIt, rd: rdn, rn: rdn, rm };
    let it = SetFlags::OutsideIt;
    match (h >> 6) & 0xF {
        0b0000 => reg(Dp::And, it),
        0b0001 => reg(Dp::Eor, it),
        0b0010 => by_reg(Shift::Lsl),
        0b0011 => by_reg(Shift::Lsr),
        0b0100 => by_reg(Shift::Asr),
        0b0101 => reg(Dp::Adc, it),
        0b0110 => reg(Dp::Sbc, it),
        0b0111 => by_reg(Shift::Ror),
        0b1000 => reg(Dp::Tst, SetFlags::Yes),
        // RSB (immediate) T1: `RSBS Rd, Rn, #0`.
        0b1001 => Op::DpImm { op: Dp::Rsb, s: it, rd: rdn, rn: rm, carry: ImmCarry::Keep, imm: 0 },
        0b1010 => reg(Dp::Cmp, SetFlags::Yes),
        0b1011 => reg(Dp::Cmn, SetFlags::Yes),
        0b1100 => reg(Dp::Orr, it),
        // MUL T1: `MULS Rdm, Rn, Rdm`.
        0b1101 => Op::Mul { s: it, rd: rdn, rn: rm, rm: rdn },
        0b1110 => reg(Dp::Bic, it),
        _ => Op::DpReg { op: Dp::Mvn, s: it, rd: rdn, rn: 0, rm, shift: Shift::Lsl, amount: 0 },
    }
}

/// A5.2.3 "Special data instructions and branch and exchange": high registers.
fn special_and_branch_exchange(h: u32) -> Op {
    let rdn = (((h >> 4) & 8) | (h & 7)) as u8;
    let rm = ((h >> 3) & 0xF) as u8;
    let unpredictable = Op::Unpredictable { raw: h };
    let reg = |op, s| Op::DpReg { op, s, rd: rdn, rn: rdn, rm, shift: Shift::Lsl, amount: 0 };
    match (h >> 6) & 0xF {
        // ADD (register) T2 and ADD (SP plus register) T1/T2.
        0b0000..=0b0011 if rdn == 15 && rm == 15 => unpredictable,
        0b0000..=0b0011 => reg(Dp::Add, SetFlags::No),
        0b0100 => unpredictable,
        // CMP (register) T2.
        0b0101..=0b0111 if (rdn < 8 && rm < 8) || rdn == 15 || rm == 15 => unpredictable,
        0b0101..=0b0111 => reg(Dp::Cmp, SetFlags::Yes),
        // MOV (register) T1.
        0b1000..=0b1011 => Op::DpReg { op: Dp::Mov, s: SetFlags::No, rd: rdn, rn: 0, rm, shift: Shift::Lsl, amount: 0 },
        // BX and BLX (register): bits 2:0 are should-be-zero.
        _ if h & 7 != 0 => unpredictable,
        0b1100 | 0b1101 => Op::Bx { rm },
        _ if rm == 15 => unpredictable,
        _ => Op::Blx { rm },
    }
}

/// A5.2.4 "Load/store single data item".
fn load_store_single(h: u32) -> Op {
    let (rt, rn, rm) = ((h & 7) as u8, ((h >> 3) & 7) as u8, ((h >> 6) & 7) as u8);
    let imm5 = ((h >> 6) & 0x1F) as i32;
    let load = h & (1 << 11) != 0;
    let single = |w: Width, scale: i32| {
        let imm = imm5 * scale;
        if load { Op::Load { w, rt, rn, imm } } else { Op::Store { w, rt, rn, imm } }
    };
    match h >> 12 {
        0b0101 => {
            let (w, is_load) = match (h >> 9) & 7 {
                0 => (Width::Word, false),
                1 => (Width::Half, false),
                2 => (Width::Byte, false),
                3 => (Width::SignedByte, true),
                4 => (Width::Word, true),
                5 => (Width::Half, true),
                6 => (Width::Byte, true),
                _ => (Width::SignedHalf, true),
            };
            if is_load { Op::LoadReg { w, rt, rn, rm, lsl: 0 } } else { Op::StoreReg { w, rt, rn, rm, lsl: 0 } }
        }
        0b0110 => single(Width::Word, 4),
        0b0111 => single(Width::Byte, 1),
        0b1000 => single(Width::Half, 2),
        // SP-relative STR/LDR T2.
        _ => {
            let (rt, imm) = (((h >> 8) & 7) as u8, ((h & 0xFF) << 2) as i32);
            if load {
                Op::Load { w: Width::Word, rt, rn: 13, imm }
            } else {
                Op::Store { w: Width::Word, rt, rn: 13, imm }
            }
        }
    }
}

/// A5.2.5 "Miscellaneous 16-bit instructions".
fn miscellaneous(h: u32, pc: u32) -> Op {
    let (rd, rm) = ((h & 7) as u8, ((h >> 3) & 7) as u8);
    let undefined = Op::Undefined { raw: h };
    let sp = |op| Op::DpImm { op, s: SetFlags::No, rd: 13, rn: 13, carry: ImmCarry::Keep, imm: (h & 0x7F) << 2 };
    let extend = |kind| Op::Extend { kind, rd, rm, rotation: 0 };
    let bits = |op| Op::Bits { op, rd, rm };
    match (h >> 5) & 0x7F {
        0b000_0000..=0b000_0011 => sp(Dp::Add),
        0b000_0100..=0b000_0111 => sp(Dp::Sub),
        // CBZ/CBNZ: `1011 o0i1 iiii innn`.
        0b000_1000..=0b000_1111 | 0b001_1000..=0b001_1111 | 0b100_1000..=0b100_1111 | 0b101_1000..=0b101_1111 => {
            let imm = ((h >> 9) & 1) << 6 | ((h >> 3) & 0x1F) << 1;
            Op::Cbz { nonzero: h & (1 << 11) != 0, rn: rd, target: pc.wrapping_add(4).wrapping_add(imm) }
        }
        0b001_0000 | 0b001_0001 => extend(Extend::Sxth),
        0b001_0010 | 0b001_0011 => extend(Extend::Sxtb),
        0b001_0100 | 0b001_0101 => extend(Extend::Uxth),
        0b001_0110 | 0b001_0111 => extend(Extend::Uxtb),
        // PUSH T1: R0-R7 and LR.
        0b010_0000..=0b010_1111 => {
            let list = (h & 0xFF) as u16 | (((h >> 8) & 1) as u16) << 14;
            if list == 0 { Op::Unpredictable { raw: h } } else { Op::Stm { rn: 13, list, wback: true, db: true } }
        }
        // CPS: `1011 0110 011 im (0)(0) I F`; no A bit on ARMv7-M.
        0b011_0011 => {
            let (i, f) = (h & 2 != 0, h & 1 != 0);
            if h & 0xC != 0 || !(i || f) {
                Op::Unpredictable { raw: h }
            } else {
                Op::Cps { disable: h & 0x10 != 0, i, f }
            }
        }
        0b101_0000 | 0b101_0001 => bits(BitOp::Rev),
        0b101_0010 | 0b101_0011 => bits(BitOp::Rev16),
        0b101_0110 | 0b101_0111 => bits(BitOp::Revsh),
        // POP T1: R0-R7 and PC.
        0b110_0000..=0b110_1111 => {
            let list = (h & 0xFF) as u16 | (((h >> 8) & 1) as u16) << 15;
            if list == 0 { Op::Unpredictable { raw: h } } else { Op::Ldm { rn: 13, list, wback: true, db: false } }
        }
        0b111_0000..=0b111_0111 => Op::Bkpt { imm: h as u8 },
        // IT and hints (A5.2.5, table A5-8).
        0b111_1000..=0b111_1111 => {
            let (firstcond, mask) = ((h >> 4) & 0xF, h & 0xF);
            if mask != 0 {
                if firstcond == 0xF || (firstcond == 0xE && mask.count_ones() != 1) {
                    Op::Unpredictable { raw: h }
                } else {
                    Op::It { state: h as u8 }
                }
            } else {
                match firstcond {
                    0b0010 => Op::Wfe,
                    0b0011 => Op::Wfi,
                    0b0100 => Op::Sev,
                    // NOP, YIELD, and the unallocated hints, which execute as NOPs.
                    _ => Op::Nop,
                }
            }
        }
        _ => undefined,
    }
}

/// A5.2.6 "Conditional branch, and Supervisor Call".
fn conditional_branch_and_svc(h: u32, pc: u32) -> Op {
    match (h >> 8) & 0xF {
        // Permanently UNDEFINED (`UDF #imm8`).
        0b1110 => Op::Undefined { raw: h },
        0b1111 => Op::Svc { imm: h as u8 },
        cond => {
            Op::BCond { cond: cond as u8, target: pc.wrapping_add(4).wrapping_add(sign_extend((h & 0xFF) << 1, 9)) }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PC: u32 = 0x0800_1002;

    #[test]
    fn literal_addresses_align_the_pc() {
        // ldr r3, [pc, #8] at an address that is not word aligned.
        assert_eq!(decode16(0x4B02, PC), Op::LoadLit { w: Width::Word, rt: 3, addr: 0x0800_1004 + 8 });
        // adr r1, #12
        let Op::DpImm { op: Dp::Mov, rd: 1, imm, .. } = decode16(0xA103, PC) else { panic!() };
        assert_eq!(imm, 0x0800_1004 + 12);
    }

    #[test]
    fn branch_targets() {
        // b . (0xE7FE) branches to itself.
        assert_eq!(decode16(0xE7FE, PC), Op::B { target: PC });
        // bne -8: imm8 = 0xFC.
        assert_eq!(decode16(0xD1FC, PC), Op::BCond { cond: 1, target: PC + 4 - 8 });
        // cbnz r2, +0x7E (i=1, imm5=0x1F).
        assert_eq!(decode16(0xBBFA, PC), Op::Cbz { nonzero: true, rn: 2, target: PC + 4 + 0x7E });
    }

    #[test]
    fn special_cases() {
        assert_eq!(
            decode16(0x0008, PC),
            Op::DpReg { op: Dp::Mov, s: SetFlags::Yes, rd: 0, rn: 0, rm: 1, shift: Shift::Lsl, amount: 0 }
        );
        assert_eq!(decode16(0x4770, PC), Op::Bx { rm: 14 });
        assert_eq!(decode16(0x4771, PC), Op::Unpredictable { raw: 0x4771 }, "BX bits 2:0 are SBZ");
        assert_eq!(decode16(0x47F8, PC), Op::Unpredictable { raw: 0x47F8 }, "BLX PC");
        assert_eq!(decode16(0xB500, PC), Op::Stm { rn: 13, list: 0x4000, wback: true, db: true });
        assert_eq!(decode16(0xBD10, PC), Op::Ldm { rn: 13, list: 0x8010, wback: true, db: false });
        assert_eq!(decode16(0xC901, PC), Op::Ldm { rn: 1, list: 1, wback: true, db: false });
        assert_eq!(decode16(0xC903, PC), Op::Ldm { rn: 1, list: 3, wback: false, db: false });
        assert_eq!(decode16(0xBF08, PC), Op::It { state: 0x08 });
        assert_eq!(decode16(0xBFE3, PC), Op::Unpredictable { raw: 0xBFE3 }, "IT AL with an else slot");
        assert_eq!(decode16(0xBF30, PC), Op::Wfi);
        assert_eq!(decode16(0xBF50, PC), Op::Nop, "unallocated hint");
        assert_eq!(decode16(0xB672, PC), Op::Cps { disable: true, i: true, f: false });
        assert_eq!(decode16(0xDE00, PC), Op::Undefined { raw: 0xDE00 });
        assert_eq!(decode16(0xDF05, PC), Op::Svc { imm: 5 });
        assert_eq!(decode16(0xBA80, PC), Op::Undefined { raw: 0xBA80 }, "HLT is not ARMv7-M");
        assert_eq!(
            decode16(0x4485, PC),
            Op::DpReg { op: Dp::Add, s: SetFlags::No, rd: 13, rn: 13, rm: 0, shift: Shift::Lsl, amount: 0 }
        );
    }
}

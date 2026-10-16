//! Integer pseudocode functions of the ARM ARM (ARMv7-M, A2.2 "Pseudocode details of operations
//! on integers and floating-point values" and A5.3.2 "Modified immediate constants").

use crate::op::{ImmCarry, Shift};

/// `AddWithCarry(x, y, carry_in)` (A2.2.1): the sum and the carry and overflow flags.
#[inline]
pub const fn add_with_carry(x: u32, y: u32, carry_in: bool) -> (u32, bool, bool) {
    let (partial, c1) = x.overflowing_add(y);
    let (result, c2) = partial.overflowing_add(carry_in as u32);
    // Signed overflow: both operands have the same sign and the result has the other one.
    let overflow = ((x ^ result) & (y ^ result)) >> 31 == 1;
    (result, c1 | c2, overflow)
}

/// `Shift_C(value, type, amount, carry_in)` (A7.4.2). An amount of 0 leaves the value and the
/// carry alone; amounts of 32 and more follow `LSL_C`, `LSR_C`, `ASR_C` and `ROR_C`.
#[inline]
pub const fn shift_c(value: u32, shift: Shift, amount: u32, carry_in: bool) -> (u32, bool) {
    if amount == 0 && !matches!(shift, Shift::Rrx) {
        return (value, carry_in);
    }
    match shift {
        Shift::Lsl => match amount {
            1..=31 => (value << amount, (value >> (32 - amount)) & 1 == 1),
            32 => (0, value & 1 == 1),
            _ => (0, false),
        },
        Shift::Lsr => match amount {
            1..=31 => (value >> amount, (value >> (amount - 1)) & 1 == 1),
            32 => (0, value >> 31 == 1),
            _ => (0, false),
        },
        Shift::Asr => {
            let amount = if amount > 32 { 32 } else { amount };
            let result = if amount == 32 { ((value as i32) >> 31) as u32 } else { ((value as i32) >> amount) as u32 };
            (result, ((value as i32) >> (amount - 1)) & 1 == 1)
        }
        Shift::Ror => {
            let result = value.rotate_right(amount % 32);
            (result, result >> 31 == 1)
        }
        Shift::Rrx => ((value >> 1) | ((carry_in as u32) << 31), value & 1 == 1),
    }
}

/// `Shift(value, type, amount, carry_in)`: [`shift_c`] without the carry.
#[inline]
pub const fn shift(value: u32, shift: Shift, amount: u32, carry_in: bool) -> u32 {
    shift_c(value, shift, amount, carry_in).0
}

/// `DecodeImmShift(type, imm5)` (A7.4.2): `LSR #0` and `ASR #0` mean 32, `ROR #0` means `RRX`.
pub const fn decode_imm_shift(kind: u32, imm5: u32) -> (Shift, u8) {
    match kind & 3 {
        0 => (Shift::Lsl, imm5 as u8),
        1 => (Shift::Lsr, if imm5 == 0 { 32 } else { imm5 as u8 }),
        2 => (Shift::Asr, if imm5 == 0 { 32 } else { imm5 as u8 }),
        _ if imm5 == 0 => (Shift::Rrx, 1),
        _ => (Shift::Ror, imm5 as u8),
    }
}

/// `ThumbExpandImm_C(imm12, carry_in)` (A5.3.2), with the carry as an [`ImmCarry`] so it can be
/// stored at decode time. `None` for the UNPREDICTABLE patterns (a zero byte replicated).
pub const fn thumb_expand_imm(imm12: u32) -> Option<(u32, ImmCarry)> {
    let byte = imm12 & 0xFF;
    if imm12 >> 10 == 0 {
        let value = match (imm12 >> 8) & 3 {
            0 => byte,
            _ if byte == 0 => return None,
            1 => byte << 16 | byte,
            2 => byte << 24 | byte << 8,
            _ => byte << 24 | byte << 16 | byte << 8 | byte,
        };
        Some((value, ImmCarry::Keep))
    } else {
        let value = (0x80 | (imm12 & 0x7F)).rotate_right(imm12 >> 7);
        Some((value, if value >> 31 == 1 { ImmCarry::Set } else { ImmCarry::Clear }))
    }
}

/// `SignedSatQ(i, n)` (A2.2.1): `i` saturated to an `n`-bit signed value, and whether it
/// saturated. `n` is 1 to 32.
#[inline]
pub const fn signed_sat(i: i64, n: u32) -> (u32, bool) {
    let max = (1_i64 << (n - 1)) - 1;
    let min = -(1_i64 << (n - 1));
    if i > max {
        (max as u32, true)
    } else if i < min {
        (min as u32, true)
    } else {
        (i as u32, false)
    }
}

/// `UnsignedSatQ(i, n)` (A2.2.1): `i` saturated to an `n`-bit unsigned value. `n` is 0 to 31.
#[inline]
pub const fn unsigned_sat(i: i64, n: u32) -> (u32, bool) {
    let max = (1_i64 << n) - 1;
    if i > max {
        (max as u32, true)
    } else if i < 0 {
        (0, true)
    } else {
        (i as u32, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_with_carry_flags() {
        // Unsigned carry, no signed overflow.
        assert_eq!(add_with_carry(0xFFFF_FFFF, 1, false), (0, true, false));
        // Signed overflow, no carry.
        assert_eq!(add_with_carry(0x7FFF_FFFF, 1, false), (0x8000_0000, false, true));
        // Both.
        assert_eq!(add_with_carry(0x8000_0000, 0x8000_0000, false), (0, true, true));
        // The carry-in alone carries out.
        assert_eq!(add_with_carry(0xFFFF_FFFF, 0, true), (0, true, false));
        // Subtraction is x + NOT y + 1: equal operands give Z and C set (no borrow).
        assert_eq!(add_with_carry(5, !5, true), (0, true, false));
        assert_eq!(add_with_carry(4, !5, true), (0xFFFF_FFFF, false, false));
        // 0x80000000 - 1 overflows.
        assert_eq!(add_with_carry(0x8000_0000, !1, true), (0x7FFF_FFFF, true, true));
    }

    #[test]
    fn shifts_follow_the_c_variants() {
        let v = 0x8000_0001;
        assert_eq!(shift_c(v, Shift::Lsl, 0, true), (v, true), "amount 0 keeps the carry");
        assert_eq!(shift_c(v, Shift::Lsl, 1, false), (2, true));
        assert_eq!(shift_c(v, Shift::Lsl, 32, false), (0, true), "LSL #32 carries bit 0");
        assert_eq!(shift_c(v, Shift::Lsl, 33, true), (0, false));
        assert_eq!(shift_c(v, Shift::Lsr, 1, false), (0x4000_0000, true));
        assert_eq!(shift_c(v, Shift::Lsr, 32, false), (0, true), "LSR #32 carries bit 31");
        assert_eq!(shift_c(v, Shift::Lsr, 200, true), (0, false));
        assert_eq!(shift_c(v, Shift::Asr, 1, false), (0xC000_0000, true));
        assert_eq!(shift_c(v, Shift::Asr, 32, false), (0xFFFF_FFFF, true));
        assert_eq!(shift_c(0x7FFF_FFFF, Shift::Asr, 255, true), (0, false));
        assert_eq!(shift_c(v, Shift::Ror, 1, false), (0xC000_0000, true));
        assert_eq!(shift_c(v, Shift::Ror, 32, false), (v, true), "ROR by 32 keeps the value, C = bit 31");
        assert_eq!(shift_c(v, Shift::Ror, 33, false), (0xC000_0000, true));
        assert_eq!(shift_c(v, Shift::Rrx, 1, false), (0x4000_0000, true));
        assert_eq!(shift_c(2, Shift::Rrx, 1, true), (0x8000_0001, false));
    }

    #[test]
    fn imm_shift_decoding() {
        assert_eq!(decode_imm_shift(0, 0), (Shift::Lsl, 0));
        assert_eq!(decode_imm_shift(1, 0), (Shift::Lsr, 32));
        assert_eq!(decode_imm_shift(2, 0), (Shift::Asr, 32));
        assert_eq!(decode_imm_shift(3, 0), (Shift::Rrx, 1));
        assert_eq!(decode_imm_shift(3, 7), (Shift::Ror, 7));
    }

    #[test]
    fn modified_immediates() {
        assert_eq!(thumb_expand_imm(0x0AB), Some((0xAB, ImmCarry::Keep)));
        assert_eq!(thumb_expand_imm(0x1AB), Some((0x00AB_00AB, ImmCarry::Keep)));
        assert_eq!(thumb_expand_imm(0x2AB), Some((0xAB00_AB00, ImmCarry::Keep)));
        assert_eq!(thumb_expand_imm(0x3AB), Some((0xABAB_ABAB, ImmCarry::Keep)));
        assert_eq!(thumb_expand_imm(0x100), None, "a zero byte replicated is UNPREDICTABLE");
        // Rotation 8 of 1:0101010 is 0xD5 << 24, bit 31 set.
        assert_eq!(thumb_expand_imm(0x455), Some((0xD500_0000, ImmCarry::Set)));
        // Rotation 31 of 0x80 = 0x100, carry clear.
        assert_eq!(thumb_expand_imm(0xF80), Some((0x100, ImmCarry::Clear)));
    }

    #[test]
    fn saturation() {
        assert_eq!(signed_sat(200, 8), (127, true));
        assert_eq!(signed_sat(-200, 8), (-128_i32 as u32, true));
        assert_eq!(signed_sat(-128, 8), (-128_i32 as u32, false));
        assert_eq!(signed_sat(i64::from(i32::MIN), 32), (0x8000_0000, false));
        assert_eq!(signed_sat(1, 1), (0, true));
        assert_eq!(unsigned_sat(256, 8), (255, true));
        assert_eq!(unsigned_sat(-1, 8), (0, true));
        assert_eq!(unsigned_sat(5, 0), (0, true));
        assert_eq!(unsigned_sat(0, 0), (0, false));
    }
}

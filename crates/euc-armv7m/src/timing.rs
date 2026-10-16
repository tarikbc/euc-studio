//! Cycle counts from the Cortex-M3 Technical Reference Manual r2p1 (ARM DDI 0337I), section 18.2
//! "Instruction set summary", Table 18-1.
//!
//! The model is the TRM's, with three choices where the TRM gives a range:
//!
//! - `P`, the pipeline refill after a taken branch or a PC write, is "1 to 3 depending on the
//!   alignment and width of the target instruction"; we use 2.
//! - Long multiplies (3 to 5 or 4 to 7 cycles) and divides (2 to 12) terminate early depending
//!   on the operands; [`long_multiply`] and [`divide`] spread that range over the operand sizes.
//! - "Neighboring load and store single instructions can pipeline their address and data phases":
//!   a load or store single that follows another one takes 1 cycle instead of 2.
//!
//! Flash wait states (2 at 72 MHz on an STM32F1, mostly hidden by its prefetch buffer) and
//! exception entry/return (12 cycles each, the engine's business) are not counted here: this is
//! the core at zero wait states.

use crate::op::Op;

/// Pipeline refill after a taken branch.
pub const P: u64 = 2;

/// Base cycles of `op` when it does not branch and its timing does not depend on data.
/// Branch refills, register counts of `LDM`/`STM` (1 + N, the N added when they run), and early
/// termination of long multiplies and divides are added when the instruction runs.
pub const fn base(op: &Op) -> u8 {
    match op {
        Op::Load { .. }
        | Op::LoadIdx { .. }
        | Op::LoadReg { .. }
        | Op::LoadLit { .. }
        | Op::Store { .. }
        | Op::StoreIdx { .. }
        | Op::StoreReg { .. }
        | Op::Ldrex { .. }
        | Op::Strex { .. }
        | Op::Table { .. }
        | Op::Mla { .. } => 2,
        // 1 + N with N = 2.
        Op::Dual { .. } | Op::LoadDualLit { .. } => 3,
        // Exceptions: their cost is the engine's exception entry.
        Op::Svc { .. } | Op::Bkpt { .. } | Op::Undefined { .. } | Op::Unpredictable { .. } => 0,
        _ => 1,
    }
}

/// True for the loads and stores of a single register, which pipeline with a neighbour.
pub const fn is_single_transfer(op: &Op) -> bool {
    matches!(
        op,
        Op::Load { .. }
            | Op::LoadIdx { .. }
            | Op::LoadReg { .. }
            | Op::LoadLit { .. }
            | Op::Store { .. }
            | Op::StoreIdx { .. }
            | Op::StoreReg { .. }
    )
}

/// `UMULL`/`SMULL` take 3 to 5 cycles, `UMLAL`/`SMLAL` 4 to 7: the multiplier terminates early
/// when the operands are small. Returns the cycles for operands `a` and `b` (as unsigned or
/// two's complement magnitudes).
pub const fn long_multiply(a: u32, b: u32, signed: bool, accumulate: bool) -> u64 {
    let (wide_a, wide_b) = (wide(a, signed), wide(b, signed));
    let base = 3 + wide_a as u64 + wide_b as u64;
    if accumulate { base + 1 + (wide_a && wide_b) as u64 } else { base }
}

/// True when `x` needs more than 16 bits (as a magnitude when `signed`).
const fn wide(x: u32, signed: bool) -> bool {
    let x = if signed && (x as i32) < 0 { !x } else { x };
    x.leading_zeros() < 16
}

/// `SDIV`/`UDIV` take 2 to 12 cycles, terminating early on the quotient's size: 2 when the
/// divisor exceeds the dividend, up to 12 for a 32-bit quotient.
pub const fn divide(dividend: u32, divisor: u32) -> u64 {
    if divisor == 0 || dividend < divisor {
        return 2;
    }
    let quotient_bits = (divisor.leading_zeros() - dividend.leading_zeros() + 1) as u64;
    2 + (quotient_bits * 10).div_ceil(32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divide_spans_the_trm_range() {
        assert_eq!(divide(5, 10), 2);
        assert_eq!(divide(5, 0), 2);
        assert!(divide(u32::MAX, 1) <= 12);
        assert_eq!(divide(u32::MAX, 1), 12);
        assert!(divide(1000, 7) > divide(10, 7));
    }

    #[test]
    fn long_multiply_spans_the_trm_range() {
        assert_eq!(long_multiply(3, 4, false, false), 3);
        assert_eq!(long_multiply(u32::MAX, u32::MAX, false, false), 5);
        assert_eq!(long_multiply(-3_i32 as u32, 7, true, false), 3, "small negative numbers terminate early");
        assert_eq!(long_multiply(u32::MAX, 0x1_0000, false, true), 7);
        assert_eq!(long_multiply(1, 1, false, true), 4);
    }
}

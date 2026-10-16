//! Condition codes and the IT block state (ARM ARM A7.3 "Conditional execution" and A7.3.3
//! `ITSTATE`).
//!
//! `ITSTATE<7:0>` holds the base condition in bits 7:5 and a 5-bit mask whose leading part also
//! supplies the low condition bit of each remaining instruction. It lives in `EPSR` as
//! `ICI/IT`: bits 1:0 at xPSR 26:25 and bits 7:2 at xPSR 15:10.

/// The APSR condition flags.
#[allow(clippy::struct_excessive_bools)] // the four architectural flags
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Flags {
    /// Negative.
    pub n: bool,
    /// Zero.
    pub z: bool,
    /// Carry (no borrow).
    pub c: bool,
    /// Signed overflow.
    pub v: bool,
}

impl Flags {
    /// `ConditionPassed()` for condition `cond` (A7.3.1). 14 and 15 always pass.
    #[inline]
    pub const fn passes(self, cond: u8) -> bool {
        let base = match cond >> 1 {
            0 => self.z,
            1 => self.c,
            2 => self.n,
            3 => self.v,
            4 => self.c && !self.z,
            5 => self.n == self.v,
            6 => self.n == self.v && !self.z,
            _ => true,
        };
        // Odd conditions invert, except 0b1111 which is "always" too.
        if cond & 1 == 1 && cond != 15 { !base } else { base }
    }
}

/// `ITAdvance()` (A7.3.3): the state for the next instruction.
#[inline]
pub const fn advance(state: u8) -> u8 {
    if state.trailing_zeros() >= 3 { 0 } else { (state & 0xE0) | ((state << 1) & 0x1F) }
}

/// The condition of the current instruction inside an IT block.
#[inline]
pub const fn condition(state: u8) -> u8 {
    state >> 4
}

/// True when the current instruction is the last one of its IT block.
pub const fn last_in_block(state: u8) -> bool {
    state & 0x0F == 0x08
}

/// `ITSTATE` from an xPSR value.
pub const fn from_xpsr(xpsr: u32) -> u8 {
    (((xpsr >> 25) & 0x03) | ((xpsr >> 8) & 0xFC)) as u8
}

/// The xPSR bits that hold `state`.
pub const fn to_xpsr(state: u8) -> u32 {
    let state = state as u32;
    ((state & 0x03) << 25) | ((state & 0xFC) << 8)
}

/// The xPSR bits of the IT state.
pub const XPSR_MASK: u32 = 0x0600_FC00;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conditions() {
        let f = |n, z, c, v| Flags { n, z, c, v };
        let eq = f(false, true, false, false);
        assert!(eq.passes(0) && !eq.passes(1));
        let hi = f(false, false, true, false);
        assert!(hi.passes(8) && !hi.passes(9) && hi.passes(2));
        let ls = f(false, true, true, false);
        assert!(!ls.passes(8) && ls.passes(9));
        // GE: N == V; LT: N != V; GT: Z clear and N == V; LE: Z set or N != V.
        let lt = f(true, false, false, false);
        assert!(!lt.passes(10) && lt.passes(11) && !lt.passes(12) && lt.passes(13));
        let gt = f(true, false, false, true);
        assert!(gt.passes(10) && gt.passes(12) && !gt.passes(13));
        for cond in [14, 15] {
            assert!(Flags::default().passes(cond));
        }
    }

    #[test]
    fn it_state_walks_the_mask() {
        // ITTE EQ: firstcond 0000, mask 0110 (T, E, end).
        let mut state = 0x06;
        assert_eq!(condition(state), 0);
        state = advance(state);
        assert_eq!((state, condition(state)), (0x0C, 0));
        state = advance(state);
        assert_eq!((state, condition(state)), (0x18, 1), "the else slot inverts the condition");
        assert!(last_in_block(state));
        assert_eq!(advance(state), 0, "the block ends");
    }

    #[test]
    fn xpsr_round_trip() {
        for state in 0..=255_u8 {
            assert_eq!(from_xpsr(to_xpsr(state) | 0xF800_0000 | (1 << 24) | 0x25), state);
            assert_eq!(to_xpsr(state) & !XPSR_MASK, 0);
        }
    }
}

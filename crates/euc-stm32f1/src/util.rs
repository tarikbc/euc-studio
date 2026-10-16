//! Small helpers shared by the models.

use serde::Deserialize;

/// Parameters of a kind that takes none: any key in its `board.toml` table is an error.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct NoParams {}

/// `old` with the bits under `mask` replaced by `value`: a plain store of the CPU's byte lanes.
pub(crate) const fn merge(old: u32, value: u32, mask: u32) -> u32 {
    old & !mask | value & mask
}

/// A status register whose flags software clears by writing 0 (`rc_w0`): writing 1 leaves a
/// flag alone, writing 0 clears it. Only bits in `flags` and in the written lanes take part.
pub(crate) const fn clear_w0(old: u32, value: u32, mask: u32, flags: u32) -> u32 {
    old & !(mask & flags & !value)
}

/// Converts between engine cycles (HCLK) and ticks of a peripheral clock, with integer math so
/// that long runs never drift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Rate {
    hclk: u64,
    clk: u64,
}

impl Rate {
    /// Ticks of `clk` Hz measured on an engine running at `hclk` cycles per second.
    pub(crate) fn new(hclk: u64, clk: u64) -> Self {
        Self { hclk, clk: clk.max(1) }
    }

    /// The peripheral clock in Hz.
    pub(crate) const fn clk(self) -> u64 {
        self.clk
    }

    /// Whole ticks in `cycles` cycles (rounded down).
    pub(crate) const fn ticks(self, cycles: u64) -> u64 {
        (cycles as u128 * self.clk as u128 / self.hclk as u128) as u64
    }

    /// The first cycle at which `ticks` whole ticks have elapsed (rounded up).
    pub(crate) const fn cycles(self, ticks: u64) -> u64 {
        (ticks as u128 * self.hclk as u128).div_ceil(self.clk as u128) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_and_clear() {
        assert_eq!(merge(0x1234_5678, 0x0000_AB00, 0x0000_FF00), 0x1234_AB78);
        // SR = TXE|TC|RXNE; write ~RXNE in the low byte: only RXNE clears.
        assert_eq!(clear_w0(0xE0, !0x20, 0xFF, 0x60), 0xC0);
        // A byte write to another lane clears nothing.
        assert_eq!(clear_w0(0xE0, 0, 0xFF00, 0x60), 0xE0);
    }

    #[test]
    fn rates_round_trip_without_drift() {
        let rtc = Rate::new(72_000_000, 62_500);
        assert_eq!(rtc.cycles(126), 145_152, "one 2.016 ms RTC second at PRL 125");
        assert_eq!(rtc.cycles(126 * 1_000_000), 145_152_000_000);
        assert_eq!(rtc.ticks(145_151), 125);
        assert_eq!(rtc.ticks(145_152), 126);
        let same = Rate::new(72_000_000, 72_000_000);
        assert_eq!((same.ticks(1234), same.cycles(1234)), (1234, 1234));
        let fast = Rate::new(72_000_000, 144_000_000);
        assert_eq!((fast.ticks(10), fast.cycles(3)), (20, 2));
    }
}

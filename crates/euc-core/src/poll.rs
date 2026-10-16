/// Spots firmware spinning on peripheral reads that never change.
///
/// Busy-wait loops (`while (!(SR & TXE));`, `delay_ms` on SysTick, a DMA counter poll) would
/// otherwise cost millions of emulated instructions per simulated second. When the same few
/// `(address, value)` pairs are read [`PollDetector::WINDOW`] times in one quantum with no write in
/// between, the engine skips time to the next scheduled event, which is the first moment the
/// value could change. Only thread-mode reads count: a handler never waits on itself.
#[derive(Debug, Clone, Default)]
pub struct PollDetector {
    keys: [(u32, u32); Self::MAX_KEYS],
    len: usize,
    reads: u32,
}

impl PollDetector {
    /// Identical reads in a row, within one quantum, before a loop counts as a poll.
    pub const WINDOW: u32 = 16;
    /// A loop may alternate between this many registers and still count.
    pub const MAX_KEYS: usize = 3;

    /// Forgets everything (on any write and at each quantum start).
    pub fn reset(&mut self) {
        self.len = 0;
        self.reads = 0;
    }

    /// Records a read of `value` at `addr`; true once the reads look like a poll.
    pub fn observe(&mut self, addr: u32, value: u32) -> bool {
        let key = (addr, value);
        if !self.keys().contains(&key) {
            if self.len == Self::MAX_KEYS {
                self.reset();
            }
            self.keys[self.len] = key;
            self.len += 1;
        }
        self.reads += 1;
        self.reads >= Self::WINDOW
    }

    /// The `(address, value)` pairs of the current loop.
    pub fn keys(&self) -> &[(u32, u32)] {
        &self.keys[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sixteen_identical_reads_make_a_poll() {
        let mut poll = PollDetector::default();
        for _ in 0..15 {
            assert!(!poll.observe(0x4001_3800, 0x80));
        }
        assert!(poll.observe(0x4001_3800, 0x80));
        assert_eq!(poll.keys(), [(0x4001_3800, 0x80)]);
    }

    #[test]
    fn loops_over_three_registers_still_count() {
        let mut poll = PollDetector::default();
        let keys = [(1, 0), (2, 0), (3, 0)];
        let hits: Vec<bool> = (0..16).map(|i| poll.observe(keys[i % 3].0, keys[i % 3].1)).collect();
        assert!(hits[15] && !hits[14]);
    }

    #[test]
    fn a_fourth_register_or_a_changing_value_restarts_the_count() {
        let mut poll = PollDetector::default();
        for i in 0..40 {
            assert!(!poll.observe(0x100, i), "a counter that moves is not a poll");
        }
        poll.reset();
        for i in 0..12 {
            poll.observe(i % 3, 0);
        }
        assert!(!poll.observe(7, 0));
        assert_eq!(poll.keys(), [(7, 0)], "a fourth register starts a new loop");
    }
}

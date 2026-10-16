//! A register block with no behavior.

use std::collections::BTreeMap;

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind};

use crate::util::{NoParams, merge};

/// Registers that read back what was written and do nothing else.
///
/// For blocks a board has but no firmware here relies on (WWDG, CRC, DAC on the Begode Extreme):
/// mapping them keeps accesses visible in traces and reports under the block's name instead of
/// as unmapped addresses. Every register resets to 0.
#[derive(Debug, Clone, Default)]
pub struct Plain {
    regs: BTreeMap<u32, u32>,
}

impl Plain {
    /// A block with every register at 0.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds the `stm32f1.plain` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }
}

impl Peripheral for Plain {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        self.regs.clear();
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        self.regs.get(&off).copied().unwrap_or(0)
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, _ctx: &mut Ctx<'_>) {
        let slot = self.regs.entry(off).or_default();
        *slot = merge(*slot, value, mask);
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn reads_back_writes_until_reset() {
        let mut host = TestHost::new(72_000_000);
        let mut plain = Plain::new();
        plain.write(0x04, 0x1234_5678, u32::MAX, &mut host.ctx());
        plain.write(0x04, 0xAB00, 0xFF00, &mut host.ctx());
        assert_eq!(plain.read(0x04, u32::MAX, &mut host.ctx()), 0x1234_AB78);
        assert_eq!(plain.read(0x08, u32::MAX, &mut host.ctx()), 0);
        plain.reset(ResetKind::System, &mut host.ctx());
        assert_eq!(plain.read(0x04, u32::MAX, &mut host.ctx()), 0);
    }
}

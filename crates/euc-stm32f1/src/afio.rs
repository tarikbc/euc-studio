//! Alternate-function I/O: remapping and EXTI source selection (RM0008 §9.4).

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind};

use crate::util::{NoParams, merge};

const EVCR: u32 = 0x00;
const MAPR: u32 = 0x04;
const EXTICR1: u32 = 0x08;
const EXTICR4: u32 = 0x14;
const MAPR2: u32 = 0x1C;

/// `MAPR.SWJ_CFG`: write-only, reads 0.
const SWJ_CFG: u32 = 7 << 24;

/// AFIO: event output, pin remapping and the EXTI line multiplexer.
///
/// The remap registers read back what was written (pin remapping has no effect on the models:
/// wiring in `board.toml` already says which pin goes where). [`exti_source`](Self::exti_source)
/// tells the EXTI which GPIO port drives each of lines 0-15.
#[derive(Debug, Clone, Default)]
pub struct Afio {
    evcr: u32,
    mapr: u32,
    exticr: [u32; 4],
    mapr2: u32,
}

impl Afio {
    /// AFIO after reset: no remap, every EXTI line on port A.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds the `stm32f1.afio` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }

    /// The GPIO port (0 = A, 1 = B, ...) selected for EXTI line `line` (0..=15).
    pub const fn exti_source(&self, line: u8) -> u8 {
        (self.exticr[(line / 4) as usize] >> (4 * (line % 4)) & 0xF) as u8
    }
}

impl Peripheral for Afio {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new();
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            EVCR => self.evcr,
            MAPR => self.mapr & !SWJ_CFG,
            EXTICR1..=EXTICR4 => self.exticr[((off - EXTICR1) / 4) as usize],
            MAPR2 => self.mapr2,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, _ctx: &mut Ctx<'_>) {
        match off {
            EVCR => self.evcr = merge(self.evcr, value, mask) & 0xFF,
            MAPR => self.mapr = merge(self.mapr, value, mask),
            EXTICR1..=EXTICR4 => {
                let slot = &mut self.exticr[((off - EXTICR1) / 4) as usize];
                *slot = merge(*slot, value, mask) & 0xFFFF;
            }
            MAPR2 => self.mapr2 = merge(self.mapr2, value, mask),
            _ => {}
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            EVCR => "EVCR",
            MAPR => "MAPR",
            0x08 => "EXTICR1",
            0x0C => "EXTICR2",
            0x10 => "EXTICR3",
            0x14 => "EXTICR4",
            MAPR2 => "MAPR2",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn exti_sources_and_write_only_swj() {
        let mut host = TestHost::new(72_000_000);
        let mut afio = Afio::new();
        afio.write(0x0C, 0x1230, u32::MAX, &mut host.ctx()); // EXTICR2: line 5 = D, 6 = C, 7 = B
        assert_eq!((afio.exti_source(4), afio.exti_source(5), afio.exti_source(7)), (0, 3, 1));
        afio.write(0x14, 0x1000, u32::MAX, &mut host.ctx()); // EXTICR4: line 15 = B
        assert_eq!(afio.exti_source(15), 1);
        // The Extreme's only remap: SWJ_CFG = 010 (JTAG off, SWD on), frees PA15/PB3/PB4.
        afio.write(MAPR, 0x0200_0000, u32::MAX, &mut host.ctx());
        assert_eq!(afio.read(MAPR, u32::MAX, &mut host.ctx()), 0);
        assert_eq!(afio.reg_name(0x10), Some("EXTICR3"));
    }
}

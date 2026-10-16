//! Power control (RM0008 §5.4).

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind};

use crate::util::{NoParams, merge};

const CR: u32 = 0x00;
const CSR: u32 = 0x04;

/// `PWR_CR.DBP`: disable backup domain write protection.
pub const DBP: u32 = 1 << 8;
const CWUF: u32 = 1 << 2;
const CSBF: u32 = 1 << 3;
const EWUP: u32 = 1 << 8;

/// Power control.
///
/// Its job here is `PWR_CR.DBP`, the write gate of the backup domain: the RCC (`BDCR`), BKP and
/// RTC models ask [`backup_access`](Self::backup_access) through a peer call before accepting
/// writes. `CWUF`/`CSBF` clear the (never set) wake-up and standby flags.
///
/// Not modeled: the voltage detector (`PVDO` reads 0), sleep, stop and standby modes.
#[derive(Debug, Clone, Default)]
pub struct Pwr {
    cr: u32,
    csr: u32,
}

impl Pwr {
    /// PWR after reset: backup domain write-protected.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds the `stm32f1.pwr` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }

    /// True while `DBP` allows writes to the backup domain.
    pub const fn backup_access(&self) -> bool {
        self.cr & DBP != 0
    }
}

impl Peripheral for Pwr {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new();
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            CR => self.cr,
            CSR => self.csr,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, _ctx: &mut Ctx<'_>) {
        match off {
            // CWUF and CSBF always read 0: they only clear the flags.
            CR => {
                let new = merge(self.cr, value, mask);
                if new & CWUF != 0 {
                    self.csr &= !1;
                }
                if new & CSBF != 0 {
                    self.csr &= !2;
                }
                self.cr = new & 0x1F3;
            }
            CSR => self.csr = merge(self.csr, value, mask) & EWUP | self.csr & 7,
            _ => {}
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        match off {
            CR => Some("CR"),
            CSR => Some("CSR"),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn dbp_opens_the_backup_domain() {
        let mut host = TestHost::new(72_000_000);
        let mut pwr = Pwr::new();
        assert!(!pwr.backup_access());
        // The firmware sets DBP through the bit-band alias: a word write of the whole register.
        pwr.write(CR, DBP, u32::MAX, &mut host.ctx());
        assert!(pwr.backup_access());
        assert_eq!(pwr.read(CR, u32::MAX, &mut host.ctx()), DBP);
        pwr.write(CR, DBP | CWUF | CSBF, u32::MAX, &mut host.ctx());
        assert_eq!(pwr.read(CR, u32::MAX, &mut host.ctx()), DBP, "clear bits read 0");
        pwr.write(CSR, EWUP | 3, u32::MAX, &mut host.ctx());
        assert_eq!(pwr.read(CSR, u32::MAX, &mut host.ctx()), EWUP, "flags are read-only");
        pwr.reset(ResetKind::System, &mut host.ctx());
        assert!(!pwr.backup_access());
    }
}

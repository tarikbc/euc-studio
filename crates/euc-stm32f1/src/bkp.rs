//! Backup registers (RM0008 §6.4, high-density layout with 42 data registers).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind};
use serde::Deserialize;

use crate::pwr::Pwr;

const RTCCR: u32 = 0x2C;
const CR: u32 = 0x30;
const CSR: u32 = 0x34;

/// Data registers on high-density parts.
pub const DATA_REGS: usize = 42;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// The PWR instance whose `DBP` bit guards writes.
    pwr: Option<String>,
    /// A cell on VBAT keeps the registers through power cycles.
    #[serde(default)]
    battery_backed: bool,
}

/// Backup registers: 42 × 16-bit data registers plus the RTC calibration and tamper registers.
///
/// Everything here lives in the backup domain: it survives a system reset and, when
/// `battery_backed` is set (a VBAT cell on the board), a power cycle too. While a PWR instance is
/// configured, writes need `PWR_CR.DBP`; without it they are ignored and reported.
///
/// The Begode Extreme uses DR1 (trip), DR2 (boot count), DR3-DR10 (settings mirror) and reads
/// DR11-DR13. Whether its board has a VBAT cell is UNKNOWN.
///
/// Not modeled: the tamper pin (`TEF` never sets), RTC clock output and calibration effects (the
/// registers read back).
#[derive(Debug, Clone)]
pub struct Bkp {
    pwr: Option<DeviceId>,
    battery_backed: bool,
    data: [u16; DATA_REGS],
    rtccr: u32,
    cr: u32,
    csr: u32,
}

impl Bkp {
    /// Cleared backup registers.
    pub fn new(pwr: Option<DeviceId>, battery_backed: bool) -> Self {
        Self { pwr, battery_backed, data: [0; DATA_REGS], rtccr: 0, cr: 0, csr: 0 }
    }

    /// Builds the `stm32f1.bkp` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let pwr = config.pwr.as_deref().map(|name| build.id_of(name)).transpose()?;
        Ok(Box::new(Self::new(pwr, config.battery_backed)))
    }

    /// Data register `n` (1..=42).
    pub fn data(&self, n: usize) -> Option<u16> {
        self.data.get(n.checked_sub(1)?).copied()
    }

    /// Sets data register `n` (1..=42) from outside the firmware (a scenario seeding the boot
    /// count, say). Returns `false` for a register that does not exist.
    pub fn set_data(&mut self, n: usize, value: u16) -> bool {
        n.checked_sub(1).and_then(|i| self.data.get_mut(i)).map(|slot| *slot = value).is_some()
    }

    /// Index into `data` of the data register at `off`.
    const fn data_index(off: u32) -> Option<usize> {
        match off {
            0x04..=0x28 => Some((off / 4 - 1) as usize),
            0x40..=0xBC => Some((off / 4 - 6) as usize),
            _ => None,
        }
    }
}

impl Peripheral for Bkp {
    fn reset(&mut self, kind: ResetKind, _ctx: &mut Ctx<'_>) {
        if kind == ResetKind::Power && !self.battery_backed {
            *self = Self::new(self.pwr, self.battery_backed);
        }
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            RTCCR => self.rtccr,
            CR => self.cr,
            CSR => self.csr,
            _ => Self::data_index(off).map_or(0, |i| u32::from(self.data[i])),
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let writable = self.pwr.is_none_or(|id| ctx.peer::<Pwr>(id).is_some_and(|pwr| pwr.backup_access()));
        if !writable {
            let name = self.reg_name(off).unwrap_or("?");
            ctx.blocker("backup-locked", format!("BKP {name} write without PWR_CR.DBP (ignored)"), false);
            return;
        }
        let store = |old: u32| old & !mask | value & mask;
        match off {
            RTCCR => self.rtccr = store(self.rtccr) & 0x3FF,
            CR => self.cr = store(self.cr) & 3,
            // CTE and CTI (bits 0-1) only clear the never-set tamper flags; TPIE is kept.
            CSR => self.csr = store(self.csr) & 0x04,
            _ => {
                if let Some(i) = Self::data_index(off) {
                    self.data[i] = store(u32::from(self.data[i])) as u16;
                }
            }
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        const DR: [&str; DATA_REGS] = [
            "DR1", "DR2", "DR3", "DR4", "DR5", "DR6", "DR7", "DR8", "DR9", "DR10", "DR11", "DR12", "DR13", "DR14",
            "DR15", "DR16", "DR17", "DR18", "DR19", "DR20", "DR21", "DR22", "DR23", "DR24", "DR25", "DR26", "DR27",
            "DR28", "DR29", "DR30", "DR31", "DR32", "DR33", "DR34", "DR35", "DR36", "DR37", "DR38", "DR39", "DR40",
            "DR41", "DR42",
        ];
        match off {
            RTCCR => Some("RTCCR"),
            CR => Some("CR"),
            CSR => Some("CSR"),
            _ => Self::data_index(off).map(|i| DR[i]),
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn data_registers_map_to_both_windows() {
        let mut host = TestHost::new(72_000_000);
        let mut bkp = Bkp::new(None, false);
        bkp.write(0x08, 0x0000_0007, u32::MAX, &mut host.ctx()); // DR2
        bkp.write(0x40, 0x1234_5678, u32::MAX, &mut host.ctx()); // DR11, 16 bits
        bkp.write(0xBC, 0xBEEF, 0xFFFF, &mut host.ctx()); // DR42
        assert_eq!(bkp.data(2), Some(7));
        assert_eq!(bkp.read(0x40, u32::MAX, &mut host.ctx()), 0x5678);
        assert_eq!(bkp.data(42), Some(0xBEEF));
        assert_eq!((bkp.data(0), bkp.data(43)), (None, None));
        assert_eq!(bkp.read(0x00, u32::MAX, &mut host.ctx()), 0, "offset 0 is reserved");
        assert_eq!(
            (bkp.reg_name(0x04), bkp.reg_name(0x28), bkp.reg_name(0x44)),
            (Some("DR1"), Some("DR10"), Some("DR12"))
        );
        assert_eq!(bkp.reg_name(0x3C), None);
    }

    #[test]
    fn survives_system_reset_and_battery_backed_power_cycles() {
        let mut host = TestHost::new(72_000_000);
        let mut bkp = Bkp::new(None, false);
        assert!(bkp.set_data(1, 42));
        bkp.reset(ResetKind::System, &mut host.ctx());
        assert_eq!(bkp.data(1), Some(42));
        bkp.reset(ResetKind::Power, &mut host.ctx());
        assert_eq!(bkp.data(1), Some(0));
        let mut cell = Bkp::new(None, true);
        cell.set_data(1, 42);
        cell.reset(ResetKind::Power, &mut host.ctx());
        assert_eq!(cell.data(1), Some(42));
    }

    #[test]
    fn writes_need_backup_access() {
        let mut host = TestHost::new(72_000_000);
        host.peers.insert(3, Box::new(Pwr::new()));
        let mut bkp = Bkp::new(Some(3), false);
        bkp.write(0x04, 5, u32::MAX, &mut host.ctx());
        assert_eq!(bkp.data(1), Some(0));
        assert_eq!(host.blockers[0].1, "BKP DR1 write without PWR_CR.DBP (ignored)");
    }
}

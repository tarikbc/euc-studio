//! Independent watchdog (RM0008 §19).

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind};

use crate::rcc::LSI_HZ;
use crate::util::{NoParams, Rate};

const KR: u32 = 0x00;
const PR: u32 = 0x04;
const RLR: u32 = 0x08;
const SR: u32 = 0x0C;

const KEY_ACCESS: u32 = 0x5555;
const KEY_RELOAD: u32 = 0xAAAA;
const KEY_START: u32 = 0xCCCC;

const TAG_TIMEOUT: u32 = 0;

/// The independent watchdog: a 12-bit down-counter on the 40 kHz LSI that resets the chip when it
/// reaches zero.
///
/// `0xCCCC` in `KR` starts it, `0xAAAA` reloads it, `0x5555` unlocks `PR` and `RLR` until the next
/// key. The timeout is `(RLR + 1) × 4 × 2^PR` LSI ticks; when it expires the model asks the engine
/// for a system reset. Once started it runs until a reset. Updates of `PR`/`RLR` complete at once
/// (`SR` reads 0). The Begode Extreme never starts it.
#[derive(Debug, Clone)]
pub struct Iwdg {
    pr: u32,
    rlr: u32,
    unlocked: bool,
    running: bool,
}

impl Default for Iwdg {
    fn default() -> Self {
        Self::new()
    }
}

impl Iwdg {
    /// A stopped watchdog after reset.
    pub fn new() -> Self {
        Self { pr: 0, rlr: 0xFFF, unlocked: false, running: false }
    }

    /// Builds the `stm32f1.iwdg` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }

    /// True once started.
    pub const fn running(&self) -> bool {
        self.running
    }

    fn reload(&self, ctx: &mut Ctx<'_>) {
        let ticks = (u64::from(self.rlr) + 1) * (4 << self.pr.min(6));
        let at = ctx.now() + Rate::new(ctx.hclk(), u64::from(LSI_HZ)).cycles(ticks);
        ctx.schedule(at, TAG_TIMEOUT);
    }
}

impl Peripheral for Iwdg {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new();
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            PR => self.pr,
            RLR => self.rlr,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let value = value & mask;
        match off {
            KR => {
                let key = value & 0xFFFF;
                self.unlocked = key == KEY_ACCESS;
                if key == KEY_START && !self.running {
                    self.running = true;
                    ctx.log("IWDG started");
                }
                if self.running && matches!(key, KEY_START | KEY_RELOAD) {
                    self.reload(ctx);
                }
            }
            PR if self.unlocked => self.pr = value & 7,
            RLR if self.unlocked => self.rlr = value & 0xFFF,
            _ => {}
        }
    }

    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        if tag == TAG_TIMEOUT && self.running {
            ctx.request_reset(ResetKind::System, "IWDG timeout");
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            KR => "KR",
            PR => "PR",
            RLR => "RLR",
            SR => "SR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn starts_reloads_and_resets_on_timeout() {
        let mut host = TestHost::new(72_000_000);
        let mut iwdg = Iwdg::new();
        iwdg.write(PR, 3, u32::MAX, &mut host.ctx());
        assert_eq!(iwdg.read(PR, u32::MAX, &mut host.ctx()), 0, "locked without 0x5555");
        iwdg.write(KR, KEY_ACCESS, u32::MAX, &mut host.ctx());
        iwdg.write(PR, 3, u32::MAX, &mut host.ctx());
        iwdg.write(RLR, 999, u32::MAX, &mut host.ctx());
        iwdg.write(KR, KEY_START, u32::MAX, &mut host.ctx());
        // 1000 × 32 LSI ticks = 0.8 s.
        assert_eq!(host.scheduled.get(&TAG_TIMEOUT), Some(&57_600_000));
        host.now = 10_000_000;
        iwdg.write(KR, KEY_RELOAD, u32::MAX, &mut host.ctx());
        assert_eq!(host.scheduled.get(&TAG_TIMEOUT), Some(&67_600_000));
        iwdg.write(RLR, 5, u32::MAX, &mut host.ctx());
        assert_eq!(iwdg.read(RLR, u32::MAX, &mut host.ctx()), 999, "relocked by the reload key");
        host.now = 67_600_000;
        iwdg.on_event(TAG_TIMEOUT, &mut host.ctx());
        assert_eq!(host.resets, [(ResetKind::System, "IWDG timeout".to_owned())]);
        iwdg.reset(ResetKind::System, &mut host.ctx());
        assert!(!iwdg.running());
    }
}

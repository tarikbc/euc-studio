//! Real-time clock (RM0008 §18).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::pwr::Pwr;
use crate::rcc::Rcc;
use crate::util::{Rate, merge};

const CRH: u32 = 0x00;
const CRL: u32 = 0x04;
const PRLH: u32 = 0x08;
const PRLL: u32 = 0x0C;
const DIVH: u32 = 0x10;
const DIVL: u32 = 0x14;
const CNTH: u32 = 0x18;
const CNTL: u32 = 0x1C;
const ALRH: u32 = 0x20;
const ALRL: u32 = 0x24;

// CRL
const SECF: u32 = 1 << 0;
const ALRF: u32 = 1 << 1;
const OWF: u32 = 1 << 2;
const RSF: u32 = 1 << 3;
const CNF: u32 = 1 << 4;
const RTOFF: u32 = 1 << 5;

const TAG_SECOND: u32 = 0;
const TAG_SYNC: u32 = 1;

const PORT_ALARM: u8 = 0;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// The RCC instance whose `BDCR` selects and enables the RTC clock.
    clock: String,
    /// The PWR instance whose `DBP` bit guards writes.
    pwr: Option<String>,
    /// A cell on VBAT keeps the counter running through power cycles.
    #[serde(default)]
    battery_backed: bool,
}

/// The RTC: a 20-bit prescaler that divides RTCCLK into seconds, a 32-bit counter and an alarm.
///
/// RTCCLK comes from the RCC (`BDCR.RTCSEL`, `RTCEN`); the RTC notices it at its next register
/// access. Each `PRL + 1` RTCCLK ticks the counter increments and `SECF` sets (the Begode
/// Extreme's application tick: HSE/128 = 62.5 kHz, `PRL` 125, one second every 2.016 ms). `ALRF`
/// sets when the counter reaches `ALR` and pulses `"alarm"` (EXTI line 17); `OWF` sets when it
/// wraps. `DIV` reads the prescaler's countdown. Interrupt role `irq`: `SECF`/`ALRF`/`OWF` masked
/// by `CRH`.
///
/// `RSF` behaves as on hardware: software clears it, and it sets again two RTCCLK ticks later, so
/// `RTC_WaitForSynchro` waits for a running clock. Writes complete at once (`RTOFF` reads 1).
/// `PRL`, `CNT` and `ALR` take writes only in configuration mode (`CNF`); a new `PRL` applies from
/// the next second. While a PWR instance is configured, writes need `PWR_CR.DBP`.
///
/// Backup domain: a system reset clears `CRH`/`CRL` (and with them `RSF`) but the prescaler,
/// counter and alarm run on; a power cycle resets everything unless `battery_backed`.
#[derive(Debug, Clone)]
pub struct Rtc {
    rcc: DeviceId,
    pwr: Option<DeviceId>,
    battery_backed: bool,
    irq: Option<(u16, bool)>,
    crh: u32,
    crl: u32,
    prl: u32,
    cnt: u32,
    alr: u32,
    /// RTCCLK in use, and the engine cycle its tick count starts from.
    clock: Option<(Rate, u64)>,
    /// RTCCLK ticks from the epoch to the start of the current second.
    second_start: u64,
    /// `PRL` of the current second (a new value applies from the next one).
    prl_active: u32,
    seconds: u64,
}

impl Rtc {
    /// An RTC after a backup domain reset.
    pub fn new(rcc: DeviceId, pwr: Option<DeviceId>, battery_backed: bool, irq: Option<u16>) -> Self {
        Self {
            rcc,
            pwr,
            battery_backed,
            irq: irq.map(|irq| (irq, false)),
            crh: 0,
            crl: RTOFF,
            prl: 0x8000,
            cnt: 0,
            alr: u32::MAX,
            clock: None,
            second_start: 0,
            prl_active: 0x8000,
            seconds: 0,
        }
    }

    /// Builds the `stm32f1.rtc` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let pwr = config.pwr.as_deref().map(|name| build.id_of(name)).transpose()?;
        Ok(Box::new(Self::new(build.id_of(&config.clock)?, pwr, config.battery_backed, build.irq("irq").ok())))
    }

    /// The counter.
    pub const fn counter(&self) -> u32 {
        self.cnt
    }

    /// Seconds counted since the RTC clock started.
    pub const fn seconds(&self) -> u64 {
        self.seconds
    }

    /// Follows the RCC: starts, stops or re-times the prescaler when RTCCLK changes.
    fn follow_clock(&mut self, ctx: &mut Ctx<'_>) {
        let hz = ctx.peer::<Rcc>(self.rcc).and_then(|rcc| rcc.clocks().rtcclk).map(u64::from);
        let now = ctx.now();
        match (hz, self.clock) {
            (Some(hz), Some((rate, _))) if rate.clk() == hz => {}
            (Some(hz), _) => {
                self.clock = Some((Rate::new(ctx.hclk(), hz), now));
                self.second_start = 0;
                self.schedule_second(ctx);
            }
            (None, Some(_)) => {
                self.clock = None;
                ctx.cancel(TAG_SECOND);
            }
            (None, None) => {}
        }
    }

    fn schedule_second(&self, ctx: &mut Ctx<'_>) {
        if let Some((rate, epoch)) = self.clock {
            ctx.schedule(epoch + rate.cycles(self.second_start + u64::from(self.prl_active) + 1), TAG_SECOND);
        }
    }

    /// RTCCLK ticks since the epoch.
    fn ticks(&self, now: u64) -> Option<u64> {
        self.clock.map(|(rate, epoch)| rate.ticks(now.saturating_sub(epoch)))
    }

    /// Every second due by `now`.
    fn run_seconds(&mut self, ctx: &mut Ctx<'_>) {
        let Some(now_ticks) = self.ticks(ctx.now()) else { return };
        let mut alarm = false;
        while now_ticks > self.second_start + u64::from(self.prl_active) {
            self.second_start += u64::from(self.prl_active) + 1;
            self.prl_active = self.prl;
            self.seconds += 1;
            self.cnt = self.cnt.wrapping_add(1);
            self.crl |= SECF;
            if self.cnt == 0 {
                self.crl |= OWF;
            }
            if self.cnt == self.alr {
                self.crl |= ALRF;
                alarm = true;
            }
        }
        if alarm {
            ctx.emit(PORT_ALARM, Signal::Pulse);
        }
        self.schedule_second(ctx);
    }

    fn div(&self, now: u64) -> u32 {
        let elapsed = self.ticks(now).map_or(0, |ticks| ticks.saturating_sub(self.second_start));
        self.prl_active.saturating_sub(elapsed as u32)
    }

    fn writable(&self, ctx: &mut Ctx<'_>) -> bool {
        self.pwr.is_none_or(|id| ctx.peer::<Pwr>(id).is_some_and(|pwr| pwr.backup_access()))
    }

    fn update_irq(&mut self, ctx: &mut Ctx<'_>) {
        let Some((irq, level)) = &mut self.irq else { return };
        let high = self.crl & self.crh & (SECF | ALRF | OWF) != 0;
        if high != *level {
            *level = high;
            ctx.set_irq(*irq, high);
        }
    }
}

impl Peripheral for Rtc {
    fn reset(&mut self, kind: ResetKind, ctx: &mut Ctx<'_>) {
        if kind == ResetKind::Power && !self.battery_backed {
            *self = Self::new(self.rcc, self.pwr, self.battery_backed, self.irq.map(|(irq, _)| irq));
        } else {
            // The engine dropped the interrupt line and the pending second; the prescaler runs on.
            self.crh = 0;
            self.crl = RTOFF;
            self.irq = self.irq.map(|(irq, _)| (irq, false));
            self.schedule_second(ctx);
        }
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        self.follow_clock(ctx);
        let now = ctx.now();
        match off {
            CRH => self.crh,
            CRL => self.crl | RTOFF,
            PRLH => self.prl >> 16,
            PRLL => self.prl & 0xFFFF,
            DIVH => self.div(now) >> 16,
            DIVL => self.div(now) & 0xFFFF,
            CNTH => self.cnt >> 16,
            CNTL => self.cnt & 0xFFFF,
            ALRH => self.alr >> 16,
            ALRL => self.alr & 0xFFFF,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        self.follow_clock(ctx);
        if !self.writable(ctx) {
            ctx.blocker(
                "backup-locked",
                format!("RTC {} write without PWR_CR.DBP (ignored)", self.reg_name(off).unwrap_or("?")),
                false,
            );
            return;
        }
        let half = |old: u32| merge(old, value, mask) & 0xFFFF;
        let configuring = self.crl & CNF != 0;
        match off {
            CRH => self.crh = half(self.crh) & 7,
            CRL => {
                let new = merge(self.crl, value, mask);
                // SECF, ALRF, OWF and RSF clear by writing 0; CNF is read/write.
                let cleared = mask & !value & (SECF | ALRF | OWF | RSF);
                self.crl = (self.crl & !cleared & !CNF) | new & CNF;
                if cleared & RSF != 0
                    && let (Some((rate, epoch)), Some(ticks)) = (self.clock, self.ticks(ctx.now()))
                {
                    ctx.schedule(epoch + rate.cycles(ticks + 2), TAG_SYNC);
                }
            }
            PRLH | PRLL | CNTH | CNTL | ALRH | ALRL if !configuring => {
                let name = self.reg_name(off).unwrap_or("?");
                ctx.blocker(
                    "rtc-not-configuring",
                    format!("RTC {name} write outside configuration mode (ignored)"),
                    false,
                );
            }
            PRLH => self.prl = (half(self.prl >> 16) & 0xF) << 16 | self.prl & 0xFFFF,
            PRLL => self.prl = self.prl & 0xF_0000 | half(self.prl),
            CNTH => self.cnt = half(self.cnt >> 16) << 16 | self.cnt & 0xFFFF,
            CNTL => self.cnt = self.cnt & 0xFFFF_0000 | half(self.cnt),
            ALRH => self.alr = half(self.alr >> 16) << 16 | self.alr & 0xFFFF,
            ALRL => self.alr = self.alr & 0xFFFF_0000 | half(self.alr),
            _ => return,
        }
        self.update_irq(ctx);
    }

    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        match tag {
            TAG_SECOND => self.run_seconds(ctx),
            TAG_SYNC => self.crl |= RSF,
            _ => {}
        }
        self.update_irq(ctx);
    }

    fn port(&self, name: &str) -> Option<u8> {
        (name == "alarm").then_some(PORT_ALARM)
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            CRH => "CRH",
            CRL => "CRL",
            PRLH => "PRLH",
            PRLL => "PRLL",
            DIVH => "DIVH",
            DIVL => "DIVL",
            CNTH => "CNTH",
            CNTL => "CNTL",
            ALRH => "ALRH",
            ALRL => "ALRL",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const RCC: DeviceId = 1;

    /// One RTC second at HSE/128 = 62.5 kHz with PRL 125: 126 ticks, 145,152 cycles of 72 MHz.
    const SECOND: u64 = 145_152;

    /// The RCC with HSE on and the RTC clocked from HSE/128, as the Begode Extreme sets it.
    fn host() -> TestHost {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        rcc.write(0x00, 0x0101_0083, u32::MAX, &mut host.ctx());
        rcc.write(0x20, 0x8300, u32::MAX, &mut host.ctx());
        host.peers.insert(RCC, Box::new(rcc));
        host
    }

    fn write(rtc: &mut Rtc, host: &mut TestHost, off: u32, value: u32) {
        rtc.write(off, value, u32::MAX, &mut host.ctx());
    }

    fn read(rtc: &mut Rtc, host: &mut TestHost, off: u32) -> u32 {
        rtc.read(off, u32::MAX, &mut host.ctx())
    }

    fn fire(rtc: &mut Rtc, host: &mut TestHost, tag: u32) -> u64 {
        let at = host.scheduled.remove(&tag).expect("event scheduled");
        host.now = at;
        rtc.on_event(tag, &mut host.ctx());
        at
    }

    /// `rtc_tick_init` of the Begode Extreme: sync, PRL = 125, second interrupt.
    fn extreme_rtc(host: &mut TestHost) -> Rtc {
        let mut rtc = Rtc::new(RCC, None, false, Some(3));
        write(&mut rtc, host, CRL, !RSF & 0xFFFF);
        assert_eq!(read(&mut rtc, host, CRL) & RSF, 0);
        fire(&mut rtc, host, TAG_SYNC);
        assert_eq!(read(&mut rtc, host, CRL) & RSF, RSF);
        write(&mut rtc, host, CRL, CNF);
        write(&mut rtc, host, PRLH, 0);
        write(&mut rtc, host, PRLL, 125);
        write(&mut rtc, host, CRL, 0);
        write(&mut rtc, host, CRH, 1);
        rtc
    }

    #[test]
    fn seconds_tick_at_496_hz() {
        let mut host = host();
        let mut rtc = extreme_rtc(&mut host);
        // The first second still runs with the reset PRL (0x8000); then 126 ticks per second.
        let first = fire(&mut rtc, &mut host, TAG_SECOND);
        assert_eq!(first, Rate::new(72_000_000, 62_500).cycles(0x8001));
        assert!(host.irq(3));
        write(&mut rtc, &mut host, CRL, !SECF & 0xFFFF);
        assert!(!host.irq(3));
        assert_eq!(fire(&mut rtc, &mut host, TAG_SECOND) - first, SECOND);
        assert_eq!(fire(&mut rtc, &mut host, TAG_SECOND) - first, 2 * SECOND);
        assert_eq!(rtc.counter(), 3);
        host.now = first + 2 * SECOND + SECOND / 2;
        assert_eq!(read(&mut rtc, &mut host, DIVL), 125 - 63);
    }

    #[test]
    fn rsf_needs_a_running_clock() {
        let mut host = TestHost::new(72_000_000);
        host.peers.insert(RCC, Box::new(Rcc::new(8_000_000, None)));
        let mut rtc = Rtc::new(RCC, None, false, None);
        write(&mut rtc, &mut host, CRL, 0);
        assert!(host.scheduled.is_empty(), "no RTCCLK: RSF never sets, as on hardware");
    }

    #[test]
    fn configuration_mode_guards_the_counter_and_the_alarm_fires() {
        let mut host = host();
        let mut rtc = extreme_rtc(&mut host);
        write(&mut rtc, &mut host, CNTL, 9);
        assert_eq!(rtc.counter(), 0);
        assert_eq!(host.blockers[0].0, "rtc-not-configuring");
        write(&mut rtc, &mut host, CRL, CNF);
        write(&mut rtc, &mut host, CNTL, 9);
        write(&mut rtc, &mut host, ALRH, 0);
        write(&mut rtc, &mut host, ALRL, 10);
        write(&mut rtc, &mut host, CRL, 0);
        write(&mut rtc, &mut host, CRH, 2);
        fire(&mut rtc, &mut host, TAG_SECOND);
        assert_eq!(read(&mut rtc, &mut host, CRL) & ALRF, ALRF);
        assert!(host.irq(3));
        assert!(host.emitted.contains(&(PORT_ALARM, Signal::Pulse)));
    }

    #[test]
    fn counter_survives_a_system_reset_but_not_a_power_cycle() {
        let mut host = host();
        let mut rtc = extreme_rtc(&mut host);
        fire(&mut rtc, &mut host, TAG_SECOND);
        rtc.reset(ResetKind::System, &mut host.ctx());
        assert_eq!((rtc.counter(), read(&mut rtc, &mut host, CRH)), (1, 0));
        assert_eq!(read(&mut rtc, &mut host, CRL) & RSF, 0, "must resynchronise");
        assert!(host.scheduled.contains_key(&TAG_SECOND), "the prescaler runs on");
        rtc.reset(ResetKind::Power, &mut host.ctx());
        assert_eq!(rtc.counter(), 0);
    }

    #[test]
    fn overflow_flag_on_wrap() {
        let mut host = host();
        let mut rtc = extreme_rtc(&mut host);
        write(&mut rtc, &mut host, CRL, CNF);
        write(&mut rtc, &mut host, CNTH, 0xFFFF);
        write(&mut rtc, &mut host, CNTL, 0xFFFF);
        write(&mut rtc, &mut host, CRL, 0);
        fire(&mut rtc, &mut host, TAG_SECOND);
        assert_eq!((rtc.counter(), read(&mut rtc, &mut host, CRL) & OWF), (0, OWF));
    }
}

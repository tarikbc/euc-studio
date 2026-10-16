//! Timers TIM1-TIM8: advanced (RM0008 §14), general-purpose (§15) and basic (§17).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::rcc::{Apb, Rcc};
use crate::util::{Rate, clear_w0, merge};

const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const SMCR: u32 = 0x08;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const RCR: u32 = 0x30;
const CCR1: u32 = 0x34;
const CCR4: u32 = 0x40;
const BDTR: u32 = 0x44;
const DCR: u32 = 0x48;
const DMAR: u32 = 0x4C;

// CR1
const CEN: u32 = 1 << 0;
const UDIS: u32 = 1 << 1;
const URS: u32 = 1 << 2;
const OPM: u32 = 1 << 3;
const DIR: u32 = 1 << 4;
const CMS: u32 = 3 << 5;
const ARPE: u32 = 1 << 7;
// SR / DIER / EGR share bit positions 0-7.
const UIF: u32 = 1 << 0;
const COMIF: u32 = 1 << 5;
const TIF: u32 = 1 << 6;
const BIF: u32 = 1 << 7;
// BDTR
const AOE: u32 = 1 << 14;
const MOE: u32 = 1 << 15;

/// Event tag of the update event; tags 1-4 are compare matches of channels 1-4.
const TAG_UPDATE: u32 = 0;

/// Output port of the trigger output.
const PORT_TRGO: u8 = 0;

/// Which register set a timer has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Class {
    /// TIM6, TIM7: up-counter, update event, no channels.
    Basic,
    /// TIM2-TIM5: four channels, up/down/centre-aligned counting.
    General,
    /// TIM1, TIM8: general plus repetition counter, complementary outputs and break (`BDTR`).
    Advanced,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    class: Class,
    /// The bus the timer sits on; defaults to APB2 for advanced timers (TIM1, TIM8) and APB1 for
    /// the others.
    bus: Option<Apb>,
    /// The RCC instance that gives the timer clock.
    clock: String,
}

/// A timer's time base, compare channels and trigger output.
///
/// **Counting** is computed from time, not ticked: the counter's phase is a function of the
/// engine cycle since the last reference point (enable, `UG`, a write to `CNT`, or an update
/// event that changed `PSC`/`ARR`/`RCR`). `CNT`, `DIR`, `UIF` and the compare flags are brought
/// up to date whenever the firmware reads them, so a timer nobody listens to costs nothing.
/// Real events are scheduled only when something must happen on time: an enabled interrupt
/// (`UIE`, `CCxIE`), a trigger output in update mode, or a preload transfer (a new `CCRx`, `ARR`
/// or `PSC` becomes active at the next update event, as with the hardware shadow registers).
///
/// Up, down and centre-aligned modes (centre-aligned: update events at both ends, `DIR` read-only,
/// compare flags in the direction `CMS` selects), repetition counter, one-pulse mode, `UDIS` and
/// `URS`, software event generation (`EGR`) and the `AOE`/`MOE` rule are modeled.
///
/// Outputs: `"trgo"` pulses on `UG` (`MMS` 000), on enable (001) and on each update event (010).
/// `"oc1"`..`"oc4"` carry each channel's waveform as a [`Signal::Pwm`] whenever it changes (PWM,
/// forced and toggle modes, after polarity, `CCxE` and, on advanced timers, `MOE`), or
/// [`Signal::Released`] while the output is disabled; see [`output`](Self::output).
///
/// Interrupt roles: `irq` for basic and general timers; `up`, `cc`, `trg_com`, `brk` for advanced
/// timers. All optional.
///
/// Not modeled: the slave mode controller (`SMCR`, reported if used), input capture and encoder
/// mode, DMA requests (`UDE`, `CCxDE`; the Begode Extreme's TIM3 → DMA LED stream is decoded from
/// RAM instead), the COM event's preloaded output enables, break input, `TRGO` modes 011-111,
/// dead time on the outputs.
#[derive(Debug, Clone)]
pub struct Timer {
    class: Class,
    bus: Apb,
    rcc: DeviceId,
    /// Interrupt line, the `SR` bits it reports, and the level last driven.
    irqs: Vec<(u16, u32, bool)>,
    // Registers as the firmware sees them (preload values for PSC, ARR, RCR, CCRx).
    cr1: u32,
    cr2: u32,
    smcr: u32,
    dier: u32,
    sr: u32,
    ccmr: [u32; 2],
    ccer: u32,
    psc: u32,
    arr: u32,
    rcr: u32,
    ccr: [u32; 4],
    bdtr: u32,
    dcr: u32,
    dmar: u32,
    // Shadow (active) registers.
    psc_active: u32,
    arr_active: u32,
    rcr_active: u32,
    ccr_active: [u32; 4],
    // Counter state.
    rate: Rate,
    /// Engine cycle of the reference point.
    base: u64,
    /// Counter phase at `base`, in counter ticks within one counting cycle.
    phase0: u64,
    /// Repetition counter at `base`: over/underflows still to go before the next update event.
    rep0: u64,
    /// Engine cycle up to which flags and update counts are applied.
    synced: u64,
    /// `CNT` while the counter is stopped.
    stopped_cnt: u32,
    /// Update events since reset.
    updates: u64,
    /// What each channel output last announced.
    outputs: [Signal; 4],
}

impl Timer {
    /// A timer after reset. `irqs` = (interrupt line, `SR` bits it reports).
    pub fn new(class: Class, bus: Apb, rcc: DeviceId, irqs: Vec<(u16, u32)>, hclk: u64) -> Self {
        Self {
            class,
            bus,
            rcc,
            irqs: irqs.into_iter().map(|(irq, bits)| (irq, bits, false)).collect(),
            cr1: 0,
            cr2: 0,
            smcr: 0,
            dier: 0,
            sr: 0,
            ccmr: [0; 2],
            ccer: 0,
            psc: 0,
            arr: 0xFFFF,
            rcr: 0,
            ccr: [0; 4],
            bdtr: 0,
            dcr: 0,
            dmar: 0,
            psc_active: 0,
            arr_active: 0xFFFF,
            rcr_active: 0,
            ccr_active: [0; 4],
            rate: Rate::new(hclk, hclk),
            base: 0,
            phase0: 0,
            rep0: 0,
            synced: 0,
            stopped_cnt: 0,
            updates: 0,
            outputs: [Signal::Released; 4],
        }
    }

    /// Builds the `stm32f1.tim` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let roles: &[(&str, u32)] = match config.class {
            Class::Basic => &[("irq", UIF)],
            Class::General => &[("irq", 0x5F)],
            Class::Advanced => &[("up", UIF), ("cc", 0x1E), ("trg_com", COMIF | TIF), ("brk", BIF)],
        };
        let irqs = roles.iter().filter_map(|&(role, bits)| build.irq(role).ok().map(|irq| (irq, bits))).collect();
        let bus = config.bus.unwrap_or(if config.class == Class::Advanced { Apb::Apb2 } else { Apb::Apb1 });
        // The engine clock is not known yet; the first reference point reads it from the RCC.
        Ok(Box::new(Self::new(config.class, bus, build.id_of(&config.clock)?, irqs, 72_000_000)))
    }

    /// Update events since reset (computed up to the last register access or event).
    pub const fn updates(&self) -> u64 {
        self.updates
    }

    /// The compare value in effect for channel `ch` (1..=4), after preload.
    pub const fn ccr_active(&self, ch: usize) -> u32 {
        self.ccr_active[ch - 1]
    }

    /// The auto-reload value in effect, after preload.
    pub const fn arr_active(&self) -> u32 {
        self.arr_active
    }

    /// Main output enable (`BDTR.MOE`); always true for timers without a break unit.
    pub const fn moe(&self) -> bool {
        !matches!(self.class, Class::Advanced) || self.bdtr & MOE != 0
    }

    /// The waveform on output `OCx` of channel `ch` (1..=4) as of the last sync: a [`Signal::Pwm`],
    /// or [`Signal::Released`] while the output is disabled (`CCxE` clear, `MOE` clear, an input
    /// channel, or a basic timer).
    pub fn output(&self, ch: usize) -> Signal {
        if self.class == Class::Basic || self.ccer >> (4 * (ch - 1)) & 1 == 0 || !self.moe() {
            return Signal::Released;
        }
        let ccmr = self.ccmr[(ch - 1) / 2] >> (8 * ((ch - 1) % 2)) & 0xFF;
        if ccmr & 3 != 0 {
            return Signal::Released;
        }
        let tick_hz = u32::try_from(self.rate.clk() / (u64::from(self.psc_active) + 1)).unwrap_or(u32::MAX);
        let arr = self.arr_active;
        let compare = self.ccr_active[ch - 1];
        let centre = self.centre();
        let period = if centre { 2 * arr } else { arr.saturating_add(1) };
        let pwm1 = if centre {
            2 * compare.min(arr)
        } else if self.cr1 & DIR != 0 {
            (compare.saturating_add(1)).min(period)
        } else {
            compare.min(period)
        };
        let running = self.cr1 & CEN != 0 && period > 0;
        let (period, high) = match ccmr >> 4 & 7 {
            0b110 if running => (period, pwm1),
            0b111 if running => (period, period - pwm1),
            0b011 if running => (period.saturating_mul(2), period),
            0b101 => (period.max(1), period.max(1)),
            _ => (period.max(1), 0),
        };
        let inverted = self.ccer >> (4 * (ch - 1) + 1) & 1 == 1;
        Signal::Pwm { tick_hz, period, high: if inverted { period - high } else { high } }
    }

    /// Announces every channel output that changed.
    fn emit_outputs(&mut self, ctx: &mut Ctx<'_>) {
        for ch in 1..=4 {
            let output = self.output(ch);
            if output != self.outputs[ch - 1] {
                self.outputs[ch - 1] = output;
                ctx.emit(ch as u8, output);
            }
        }
    }

    fn centre(&self) -> bool {
        self.class != Class::Basic && self.cr1 & CMS != 0
    }

    fn counting_down(&self) -> bool {
        self.class != Class::Basic && self.cr1 & DIR != 0
    }

    /// Ticks in one counting cycle.
    fn len(&self) -> u64 {
        let arr = u64::from(self.arr_active);
        if self.centre() { 2 * arr } else { arr + 1 }
    }

    /// Ticks between two over/underflows (0: the counter is blocked by `ARR` = 0).
    fn step(&self) -> u64 {
        let arr = u64::from(self.arr_active);
        if self.centre() {
            arr
        } else if arr == 0 {
            0
        } else {
            arr + 1
        }
    }

    fn running(&self) -> bool {
        self.cr1 & CEN != 0 && self.step() > 0
    }

    /// Counter ticks between `base` and `now`.
    fn ticks(&self, now: u64) -> u64 {
        self.rate.ticks(now.saturating_sub(self.base)) / (u64::from(self.psc_active) + 1)
    }

    /// The engine cycle at which `ticks` counter ticks after `base` have elapsed.
    fn cycle_at(&self, ticks: u64) -> u64 {
        self.base + self.rate.cycles(ticks * (u64::from(self.psc_active) + 1))
    }

    /// Ticks from `base` to the first update event, and between update events.
    fn uev_ticks(&self) -> (u64, u64) {
        let step = self.step();
        let first = step - self.phase0 % step + self.rep0 * step;
        (first, step * (u64::from(self.rcr_active) + 1))
    }

    /// Update events at ticks `1..=ticks` after `base`.
    fn uevs_until(&self, ticks: u64) -> u64 {
        let (first, period) = self.uev_ticks();
        if ticks < first { 0 } else { 1 + (ticks - first) / period }
    }

    /// The tick of update event number `n` (1-based) after `base`.
    fn uev_tick(&self, n: u64) -> u64 {
        let (first, period) = self.uev_ticks();
        first + (n - 1) * period
    }

    fn cnt_of(&self, phase: u64) -> u32 {
        let arr = u64::from(self.arr_active);
        let phase = phase % self.len().max(1);
        let cnt = if self.centre() {
            if phase <= arr { phase } else { 2 * arr - phase }
        } else if self.counting_down() {
            arr - phase
        } else {
            phase
        };
        cnt as u32
    }

    fn phase_of(&self, cnt: u32, down: bool) -> u64 {
        let arr = u64::from(self.arr_active);
        let cnt = u64::from(cnt).min(arr);
        if self.centre() {
            if down && cnt > 0 { 2 * arr - cnt } else { cnt }
        } else if self.counting_down() {
            arr - cnt
        } else {
            cnt
        }
    }

    fn phase_at(&self, now: u64) -> u64 {
        (self.phase0 + self.ticks(now)) % self.len().max(1)
    }

    fn cnt_at(&self, now: u64) -> u32 {
        if self.running() { self.cnt_of(self.phase_at(now)) } else { self.stopped_cnt }
    }

    /// `CR1.DIR` as read: in centre-aligned mode the counting direction right now.
    fn dir_at(&self, now: u64) -> bool {
        if self.centre() {
            self.running() && self.phase_at(now) >= u64::from(self.arr_active)
        } else {
            self.counting_down()
        }
    }

    fn timer_clock(&self, ctx: &mut Ctx<'_>) -> u64 {
        let hclk = ctx.hclk();
        ctx.peer::<Rcc>(self.rcc).map_or(hclk, |rcc| u64::from(rcc.clocks().timer(self.bus)))
    }

    /// Starts counting at `now` from `cnt`, with `rep` over/underflows to the next update event.
    fn rebase(&mut self, now: u64, cnt: u32, down: bool, rep: u64, ctx: &mut Ctx<'_>) {
        self.rate = Rate::new(ctx.hclk(), self.timer_clock(ctx));
        self.base = now;
        self.synced = now;
        self.phase0 = self.phase_of(cnt, down);
        self.rep0 = rep;
        self.stopped_cnt = cnt;
    }

    fn preload_pending(&self) -> bool {
        let ccr_preloaded = (0..4).any(|ch| self.oc_preload(ch) && self.ccr[ch] != self.ccr_active[ch]);
        self.psc != self.psc_active || self.arr != self.arr_active || self.rcr != self.rcr_active || ccr_preloaded
    }

    fn oc_preload(&self, ch: usize) -> bool {
        self.ccmr[ch / 2] >> (8 * (ch % 2) + 3) & 1 == 1
    }

    fn transfer_preloads(&mut self) {
        self.psc_active = self.psc;
        self.arr_active = self.arr;
        self.rcr_active = self.rcr;
        self.ccr_active = self.ccr;
    }

    /// Brings flags and counters up to `now`: update events, preload transfers at them, compare
    /// matches, one-pulse stop.
    fn sync(&mut self, now: u64, ctx: &mut Ctx<'_>) {
        while self.running() && self.synced < now {
            let done = self.uevs_until(self.ticks(self.synced));
            // With a preload pending, stop at the next update event: it changes the parameters.
            let next = self.cycle_at(self.uev_tick(done + 1));
            let split = self.cr1 & UDIS == 0 && self.preload_pending() && next <= now;
            let end = if split { next } else { now };
            self.scan(end);
            if !split || !self.running() {
                continue;
            }
            // `end` is an update event: transfer the shadows and count on with the new values.
            let down = self.dir_at(end);
            let cnt = self.cnt_at(end);
            self.transfer_preloads();
            self.rebase(end, cnt, down, u64::from(self.rcr_active), ctx);
        }
        self.synced = self.synced.max(now);
    }

    /// Applies what happened in `(synced, end]` with the current parameters.
    fn scan(&mut self, end: u64) {
        let from = self.ticks(self.synced);
        let mut to = self.ticks(end);
        let uevs = self.uevs_until(to) - self.uevs_until(from);
        if uevs > 0 {
            if self.cr1 & OPM != 0 {
                // One-pulse mode: the counter stops at the first update event.
                to = self.uev_tick(self.uevs_until(from) + 1);
                self.stopped_cnt = self.cnt_of(self.phase0 + to);
                self.cr1 &= !CEN;
            }
            self.updates += if self.cr1 & OPM != 0 { 1 } else { uevs };
            if self.cr1 & UDIS == 0 {
                self.sr |= UIF;
                if self.bdtr & AOE != 0 {
                    self.bdtr |= MOE;
                }
            }
        }
        for ch in 0..4 {
            if self.next_match(ch, from).is_some_and(|tick| tick <= to) {
                self.sr |= 2 << ch;
            }
        }
        self.synced = end;
    }

    /// The first tick after `after` (relative to `base`) at which channel `ch` (0-based) sets its
    /// compare flag, if it is an output compare channel and its value is reachable.
    fn next_match(&self, ch: usize, after: u64) -> Option<u64> {
        if self.class == Class::Basic || self.ccmr[ch / 2] >> (8 * (ch % 2)) & 3 != 0 {
            return None;
        }
        let arr = u64::from(self.arr_active);
        let ccr = u64::from(self.ccr_active[ch]);
        let len = self.len();
        if ccr > arr || len == 0 || self.step() == 0 {
            return None;
        }
        let cms = self.cr1 >> 5 & 3;
        let phases: [Option<u64>; 2] = if self.centre() {
            let up = (ccr < arr && cms != 1).then_some(ccr);
            let down = (ccr > 0 && cms != 2).then_some(2 * arr - ccr);
            [up, down]
        } else if self.counting_down() {
            [Some(arr - ccr), None]
        } else {
            [Some(ccr), None]
        };
        phases
            .into_iter()
            .flatten()
            .map(|phase| {
                let target = (phase + len - self.phase0 % len) % len;
                after + 1 + (target + len - (after + 1) % len) % len
            })
            .min()
    }

    /// Schedules the events that must happen on time and cancels the others.
    fn reschedule(&mut self, now: u64, ctx: &mut Ctx<'_>) {
        let needs_update = self.dier & UIF != 0
            || self.cr2 >> 4 & 7 == 0b010
            || self.preload_pending()
            || self.bdtr & (AOE | MOE) == AOE;
        if self.running() && needs_update {
            let at = self.cycle_at(self.uev_tick(self.uevs_until(self.ticks(now)) + 1));
            ctx.schedule(at, TAG_UPDATE);
        } else {
            ctx.cancel(TAG_UPDATE);
        }
        for ch in 0..4 {
            let tag = ch as u32 + 1;
            let wanted = self.dier & 2 << ch != 0 && self.running();
            match self.next_match(ch, self.ticks(now)).filter(|_| wanted) {
                Some(tick) => ctx.schedule(self.cycle_at(tick), tag),
                None => ctx.cancel(tag),
            }
        }
    }

    fn update_irqs(&mut self, ctx: &mut Ctx<'_>) {
        let active = self.sr & self.dier;
        for (irq, bits, level) in &mut self.irqs {
            let high = active & *bits != 0;
            if high != *level {
                *level = high;
                ctx.set_irq(*irq, high);
            }
        }
    }

    /// Update generation (`EGR.UG`, or the reset of the slave controller).
    fn generate_update(&mut self, now: u64, ctx: &mut Ctx<'_>) {
        if self.cr1 & UDIS == 0 {
            self.transfer_preloads();
            if self.cr1 & URS == 0 {
                self.sr |= UIF;
            }
        }
        let cnt = if self.counting_down() && !self.centre() { self.arr_active } else { 0 };
        self.rebase(now, cnt, false, u64::from(self.rcr_active), ctx);
        if self.cr2 & 0x70 == 0 {
            ctx.emit(PORT_TRGO, Signal::Pulse);
        }
    }

    fn write_cr1(&mut self, new: u32, ctx: &mut Ctx<'_>) {
        let now = ctx.now();
        let was_running = self.cr1 & CEN != 0;
        let cnt = self.cnt_at(now);
        let down = self.dir_at(now);
        let mut new = new & 0x3FF;
        if self.class == Class::Basic {
            new &= CEN | UDIS | URS | OPM | ARPE;
        } else if self.centre() && new & CMS != 0 {
            // DIR is read-only in centre-aligned mode.
            new = new & !DIR | self.cr1 & DIR;
        }
        let restart = (self.cr1 ^ new) & (CMS | DIR) != 0;
        self.cr1 = new;
        let running = new & CEN != 0;
        if running && (!was_running || restart) {
            self.rebase(now, cnt, down, self.rep0, ctx);
            if !was_running && self.cr2 >> 4 & 7 == 0b001 {
                ctx.emit(PORT_TRGO, Signal::Pulse);
            }
        } else if !running {
            self.stopped_cnt = cnt;
        }
    }

    fn write_arr(&mut self, value: u32, ctx: &mut Ctx<'_>) {
        self.arr = value & 0xFFFF;
        if self.cr1 & ARPE == 0 {
            let now = ctx.now();
            let (cnt, down) = (self.cnt_at(now), self.dir_at(now));
            self.arr_active = self.arr;
            if self.cr1 & CEN != 0 {
                self.rebase(now, cnt, down, self.rep0, ctx);
            }
        }
    }

    fn generate_events(&mut self, egr: u32, ctx: &mut Ctx<'_>) {
        if egr & UIF != 0 {
            self.generate_update(ctx.now(), ctx);
        }
        // CCxG, COMG, TG and BG set their flags, where the timer has them.
        self.sr |= egr
            & match self.class {
                Class::Basic => 0,
                Class::General => 0x1E | TIF,
                Class::Advanced => 0x1E | COMIF | TIF | BIF,
            };
        if self.class == Class::Advanced && egr & BIF != 0 {
            self.bdtr &= !MOE;
        }
    }

    fn reg(&self, off: u32) -> Option<u32> {
        let adv = self.class == Class::Advanced;
        let channels = self.class != Class::Basic;
        Some(match off {
            CR1 => self.cr1,
            CR2 => self.cr2,
            SMCR if channels => self.smcr,
            DIER => self.dier,
            SR => self.sr,
            EGR => 0,
            CCMR1 | CCMR2 if channels => self.ccmr[((off - CCMR1) / 4) as usize],
            CCER if channels => self.ccer,
            CNT => self.stopped_cnt,
            PSC => self.psc,
            ARR => self.arr,
            RCR if adv => self.rcr,
            CCR1..=CCR4 if channels => self.ccr[((off - CCR1) / 4) as usize],
            BDTR if adv => self.bdtr,
            DCR if channels => self.dcr,
            DMAR if channels => self.dmar,
            _ => return None,
        })
    }
}

impl Peripheral for Timer {
    fn reset(&mut self, _kind: ResetKind, ctx: &mut Ctx<'_>) {
        let irqs = self.irqs.iter().map(|&(irq, bits, _)| (irq, bits)).collect();
        let outputs = self.outputs;
        *self = Self::new(self.class, self.bus, self.rcc, irqs, ctx.hclk());
        // Outputs that were driving let go (a buzzer stops, the inverter sees its gates released).
        self.outputs = outputs;
        self.emit_outputs(ctx);
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        let now = ctx.now();
        match off {
            CR1 => {
                self.sync(now, ctx);
                self.cr1 & !DIR | if self.dir_at(now) { DIR } else { 0 }
            }
            SR => {
                self.sync(now, ctx);
                self.update_irqs(ctx);
                self.sr
            }
            CNT => self.cnt_at(now),
            _ => self.reg(off).unwrap_or(0),
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let now = ctx.now();
        self.sync(now, ctx);
        let Some(old) = self.reg(off) else { return };
        let old = if off == CNT { self.cnt_at(now) } else { old };
        let new = merge(old, value, mask);
        match off {
            CR1 => self.write_cr1(new, ctx),
            CR2 => self.cr2 = new & if self.class == Class::Advanced { 0x7FFD } else { 0xF8 },
            SMCR => {
                self.smcr = new & 0xFFF7;
                if self.smcr & 7 != 0 {
                    ctx.blocker("timer-slave-mode", format!("slave mode {} is not modeled", self.smcr & 7), false);
                }
            }
            DIER => self.dier = new & 0x7FFF,
            SR => self.sr = clear_w0(self.sr, value, mask, 0x1EFF),
            EGR => self.generate_events(value & mask & 0xFF, ctx),
            CCMR1 | CCMR2 => self.ccmr[((off - CCMR1) / 4) as usize] = new & 0xFFFF,
            CCER => self.ccer = new & 0x3FFF,
            CNT => {
                self.stopped_cnt = new & 0xFFFF;
                if self.running() {
                    let down = self.dir_at(now);
                    self.rebase(now, new & 0xFFFF, down, self.rep0, ctx);
                }
            }
            PSC => self.psc = new & 0xFFFF,
            ARR => self.write_arr(new, ctx),
            RCR => self.rcr = new & 0xFF,
            CCR1..=CCR4 => {
                let ch = ((off - CCR1) / 4) as usize;
                self.ccr[ch] = new & 0xFFFF;
                if !self.oc_preload(ch) {
                    self.ccr_active[ch] = self.ccr[ch];
                }
            }
            BDTR => self.bdtr = new & 0xFFFF,
            DCR => self.dcr = new & 0x1F1F,
            DMAR => self.dmar = new,
            _ => {}
        }
        self.update_irqs(ctx);
        self.reschedule(now, ctx);
        self.emit_outputs(ctx);
    }

    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        let now = ctx.now();
        let before = self.updates;
        self.sync(now, ctx);
        if tag == TAG_UPDATE && self.updates > before && self.cr2 >> 4 & 7 == 0b010 {
            ctx.emit(PORT_TRGO, Signal::Pulse);
        }
        self.update_irqs(ctx);
        self.reschedule(now, ctx);
        self.emit_outputs(ctx);
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "trgo" => Some(PORT_TRGO),
            "oc1" => Some(1),
            "oc2" => Some(2),
            "oc3" => Some(3),
            "oc4" => Some(4),
            _ => None,
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        self.reg(off)?;
        Some(match off {
            CR1 => "CR1",
            CR2 => "CR2",
            SMCR => "SMCR",
            DIER => "DIER",
            SR => "SR",
            EGR => "EGR",
            CCMR1 => "CCMR1",
            CCMR2 => "CCMR2",
            CCER => "CCER",
            CNT => "CNT",
            PSC => "PSC",
            ARR => "ARR",
            RCR => "RCR",
            0x34 => "CCR1",
            0x38 => "CCR2",
            0x3C => "CCR3",
            CCR4 => "CCR4",
            BDTR => "BDTR",
            DCR => "DCR",
            DMAR => "DMAR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const RCC: DeviceId = 1;

    /// A timer under test with an RCC peer set up like the Begode Extreme (72 MHz timer clocks).
    struct Bench {
        host: TestHost,
        tim: Timer,
    }

    impl Bench {
        fn new(class: Class, irqs: Vec<(u16, u32)>) -> Self {
            let mut host = TestHost::new(72_000_000);
            let mut rcc = Rcc::new(8_000_000, None);
            for (off, value) in [(0x00, 0x0101_0083), (0x04, 0x001D_8402)] {
                rcc.write(off, value, u32::MAX, &mut host.ctx());
            }
            host.peers.insert(RCC, Box::new(rcc));
            let tim = Timer::new(class, Apb::Apb2, RCC, irqs, 72_000_000);
            Self { host, tim }
        }

        fn at(&mut self, now: u64) -> &mut Self {
            self.host.now = now;
            self
        }

        fn write(&mut self, off: u32, value: u32) -> &mut Self {
            self.tim.write(off, value, u32::MAX, &mut self.host.ctx());
            self
        }

        fn read(&mut self, off: u32) -> u32 {
            self.tim.read(off, u32::MAX, &mut self.host.ctx())
        }

        /// Fires every scheduled event due by `now`, in time order, like the engine.
        fn run_to(&mut self, now: u64) {
            while let Some((&tag, &at)) = self.host.scheduled.iter().min_by_key(|&(_, &at)| at) {
                if at > now {
                    break;
                }
                self.host.scheduled.remove(&tag);
                self.host.now = at;
                self.tim.on_event(tag, &mut self.host.ctx());
            }
            self.host.now = now;
        }
    }

    /// The Begode Extreme's TIM8: centre-aligned 1, ARR 3600 with preload, update interrupt.
    fn tim8() -> Bench {
        let mut bench = Bench::new(Class::Advanced, vec![(44, UIF), (46, 0x1E)]);
        bench.write(ARR, 3600).write(PSC, 0).write(RCR, 0).write(EGR, UIF);
        bench.write(SR, 0).write(CR1, ARPE | 1 << 5).write(DIER, UIF).write(CR1, ARPE | 1 << 5 | CEN);
        bench
    }

    #[test]
    fn centre_aligned_updates_at_both_ends() {
        let mut b = tim8();
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), Some(&3600));
        assert_eq!(b.at(1000).read(CNT), 1000);
        assert_eq!(b.read(CR1) & DIR, 0, "counting up");
        b.run_to(3600);
        assert!(b.host.irq(44));
        assert_eq!(b.read(CR1) & DIR, DIR, "overflow: counting down");
        assert_eq!(b.at(5000).read(CNT), 2200);
        b.write(SR, !UIF);
        assert!(!b.host.irq(44));
        b.run_to(7200);
        assert!(b.host.irq(44));
        assert_eq!(b.read(CR1) & DIR, 0, "underflow: counting up again");
        assert_eq!(b.tim.updates(), 2);
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), Some(&10_800), "20 kHz");
    }

    #[test]
    fn late_events_catch_up_whole_periods() {
        let mut b = tim8();
        b.run_to(36_000 + 100);
        assert_eq!(b.tim.updates(), 10);
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), Some(&39_600));
        assert_eq!(b.read(CNT), 100);
    }

    #[test]
    fn compare_values_load_at_the_update_event_with_preload() {
        let mut b = tim8();
        b.write(CCMR1, 0x6868).write(CCMR2, 0x0068).write(CCER, 0x555).write(BDTR, MOE | 1);
        b.at(100).write(CCR1, 2227).write(0x38, 1800).write(0x3C, 1374);
        assert_eq!((b.tim.ccr_active(1), b.read(CCR1)), (0, 2227), "shadow keeps the old value");
        b.run_to(3600);
        assert_eq!([b.tim.ccr_active(1), b.tim.ccr_active(2), b.tim.ccr_active(3)], [2227, 1800, 1374]);
        let wave = Signal::Pwm { tick_hz: 72_000_000, period: 7200, high: 2 * 2227 };
        assert_eq!(b.tim.output(1), wave);
        assert!(b.host.emitted.contains(&(1, wave)), "announced on oc1");
        b.write(BDTR, 1);
        assert_eq!(b.tim.output(1), Signal::Released, "MOE off releases the outputs");
        assert_eq!(b.host.emitted.last(), Some(&(3, Signal::Released)));
    }

    #[test]
    fn lazy_timer_flags_and_preload_event() {
        // The buzzer timer TIM4: PSC 39, ARR 666, PWM mode 1 on CH4 with preload, no interrupt.
        let mut b = Bench::new(Class::General, vec![(30, 0x5F)]);
        b.write(PSC, 39).write(ARR, 666).write(EGR, UIF).write(SR, 0);
        b.write(CCMR2, 0x6800).write(CCER, 1 << 12).write(CR1, ARPE | CEN);
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), None, "nothing to do on time");
        assert_eq!(b.at(40 * 667 * 3 + 5).read(SR) & UIF, UIF, "UIF set lazily");
        assert!(!b.host.irq(30), "UIE is off");
        b.write(0x40, 151);
        let at = *b.host.scheduled.get(&TAG_UPDATE).expect("preload transfer is scheduled");
        assert_eq!(at, 40 * 667 * 4);
        b.run_to(at);
        assert_eq!(b.tim.ccr_active(4), 151);
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), None);
        let tone = Signal::Pwm { tick_hz: 1_800_000, period: 667, high: 151 };
        assert_eq!(b.host.emitted.last(), Some(&(4, tone)), "the buzzer hears it at the update event");
    }

    #[test]
    fn edge_aligned_down_counting() {
        let mut b = Bench::new(Class::General, vec![(28, 0x5F)]);
        // As TIM_TimeBaseInit does it: direction first, then UG loads CNT = ARR.
        b.write(ARR, 99).write(CR1, DIR).write(EGR, UIF).write(SR, 0).write(DIER, UIF).write(CR1, DIR | CEN);
        assert_eq!(b.read(CNT), 99);
        assert_eq!(b.at(30).read(CNT), 69);
        b.run_to(100);
        assert!(b.host.irq(28));
        assert_eq!(b.read(CNT), 99, "reloaded at underflow");
        assert_eq!(b.read(CR1) & DIR, DIR);
    }

    #[test]
    fn one_pulse_mode_stops_after_one_period() {
        let mut b = Bench::new(Class::General, vec![]);
        b.write(ARR, 9).write(EGR, UIF).write(SR, 0).write(CR1, OPM | CEN);
        assert_eq!(b.at(25).read(CR1) & CEN, 0);
        assert_eq!(b.read(CNT), 0);
        assert_eq!(b.read(SR) & UIF, UIF);
        assert_eq!(b.tim.updates(), 1);
    }

    #[test]
    fn repetition_counter_divides_update_events() {
        let mut b = Bench::new(Class::Advanced, vec![(25, UIF)]);
        b.write(ARR, 9).write(RCR, 2).write(EGR, UIF).write(SR, 0).write(DIER, UIF).write(CR1, CEN);
        assert_eq!(b.host.scheduled.get(&TAG_UPDATE), Some(&30));
        b.run_to(29);
        assert!(!b.host.irq(25));
        b.run_to(30);
        assert!(b.host.irq(25));
    }

    #[test]
    fn update_generation_honours_udis_and_urs() {
        let mut b = Bench::new(Class::General, vec![]);
        b.write(PSC, 3).write(CR1, URS).write(EGR, UIF);
        assert_eq!((b.read(SR) & UIF, b.tim.psc_active), (0, 3), "URS: transfer without UIF");
        b.write(PSC, 5).write(CR1, UDIS).write(EGR, UIF);
        assert_eq!(b.tim.psc_active, 3, "UDIS: no transfer");
        b.write(CR1, 0).write(EGR, UIF);
        assert_eq!((b.read(SR) & UIF, b.tim.psc_active), (UIF, 5));
    }

    #[test]
    fn compare_flags_follow_the_counting_direction() {
        let mut b = Bench::new(Class::General, vec![(29, 0x5F)]);
        b.write(ARR, 999).write(EGR, UIF).write(SR, 0).write(CCR1, 100).write(CR1, CEN);
        assert_eq!(b.at(99).read(SR) & 2, 0);
        assert_eq!(b.at(100).read(SR) & 2, 2);
        b.write(SR, !2).write(DIER, 2);
        assert_eq!(b.host.scheduled.get(&1), Some(&1100), "next match one period later");
        // Centre-aligned mode 1: compare flags only while counting down (CNT 100 at phase 1898).
        let mut c = Bench::new(Class::General, vec![]);
        c.write(ARR, 999).write(EGR, UIF).write(SR, 0).write(CCR1, 100).write(CR1, 1 << 5 | CEN);
        assert_eq!(c.at(500).read(SR) & 2, 0, "passed CNT = 100 counting up");
        assert_eq!(c.at(1897).read(SR) & 2, 0);
        assert_eq!(c.at(1898).read(SR) & 2, 2);
    }

    #[test]
    fn trigger_output_pulses_on_update() {
        let mut b = Bench::new(Class::General, vec![]);
        b.write(ARR, 9).write(CR2, 0b010 << 4).write(EGR, UIF).write(CR1, CEN);
        b.run_to(35);
        let pulses = b.host.emitted.iter().filter(|e| **e == (PORT_TRGO, Signal::Pulse)).count();
        assert_eq!(pulses, 3);
        assert_eq!(b.tim.port("trgo"), Some(PORT_TRGO));
    }

    #[test]
    fn reset_releases_driving_outputs() {
        let mut b = Bench::new(Class::General, vec![]);
        b.write(ARR, 99).write(CCR1, 30).write(CCMR1, 0x60).write(CCER, 1).write(EGR, UIF).write(CR1, CEN);
        assert_eq!(b.host.emitted.last(), Some(&(1, Signal::Pwm { tick_hz: 72_000_000, period: 100, high: 30 })));
        b.tim.reset(ResetKind::System, &mut b.host.ctx());
        assert_eq!(b.host.emitted.last(), Some(&(1, Signal::Released)));
    }

    #[test]
    fn basic_timer_has_no_channels() {
        let mut b = Bench::new(Class::Basic, vec![(54, UIF)]);
        b.write(CCR1, 5).write(CR1, CEN | 1 << 5);
        assert_eq!(b.read(CCR1), 0);
        assert_eq!(b.read(CR1), CEN, "no centre-aligned mode");
        assert_eq!((b.tim.reg_name(CCR1), b.tim.reg_name(CNT)), (None, Some("CNT")));
        assert_eq!(b.tim.output(1), Signal::Released);
    }

    #[test]
    fn zero_arr_blocks_the_counter_and_cnt_writes_restart_it() {
        let mut b = Bench::new(Class::General, vec![]);
        b.write(ARR, 0).write(EGR, UIF).write(CR1, CEN);
        assert_eq!(b.at(1000).read(CNT), 0);
        b.write(ARR, 499);
        assert_eq!(b.at(1100).read(CNT), 100);
        b.write(CNT, 400);
        assert_eq!(b.at(1150).read(CNT), 450);
        b.write(CR1, 0);
        assert_eq!(b.at(5000).read(CNT), 450, "stopped");
    }

    #[test]
    fn timer_clock_follows_the_apb_prescaler() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        rcc.write(0x00, 0x0101_0083, u32::MAX, &mut host.ctx());
        rcc.write(0x04, 0x001D_0402 | 0b100 << 11, u32::MAX, &mut host.ctx()); // APB2 /2: timers x2 = 72 MHz
        host.peers.insert(RCC, Box::new(rcc));
        let mut tim = Timer::new(Class::General, Apb::Apb1, RCC, vec![], 72_000_000);
        tim.write(PSC, 1, u32::MAX, &mut host.ctx());
        tim.write(EGR, 1, u32::MAX, &mut host.ctx());
        tim.write(CR1, CEN, u32::MAX, &mut host.ctx());
        host.now = 1000;
        assert_eq!(tim.read(CNT, u32::MAX, &mut host.ctx()), 500, "72 MHz / 2");
    }
}

//! Passive buzzer driven by one PWM channel.

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind, Signal};

use crate::NoParams;

const PORT_PWM: u8 = 0;

/// A periodic PWM waveform, in timer ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Pwm {
    /// Timer counter clock in Hz (after the prescaler).
    pub tick_hz: u32,
    /// Period in ticks (`ARR + 1` for an edge-aligned timer).
    pub period: u32,
    /// High time per period in ticks (the compare value in PWM mode 1).
    pub high: u32,
}

impl Pwm {
    /// Waveform frequency in Hz.
    pub fn frequency_hz(&self) -> f64 {
        f64::from(self.tick_hz) / f64::from(self.period.max(1))
    }

    /// High fraction of the period, 0.0..=1.0.
    pub fn duty(&self) -> f64 {
        f64::from(self.high.min(self.period)) / f64::from(self.period.max(1))
    }

    /// True when the pin actually toggles: a duty of 0 or 100 % is a steady level, which a
    /// passive buzzer does not hear.
    pub const fn toggles(&self) -> bool {
        self.tick_hz > 0 && self.high > 0 && self.high < self.period
    }
}

/// The buzzer started, stopped, or changed its tone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuzzerEvent {
    /// Simulation time of the change (engine clock cycles).
    pub at: u64,
    /// The tone now sounding, or `None` for silence.
    pub tone: Option<Pwm>,
}

/// A passive buzzer: it sounds while its PWM pin toggles, at the PWM frequency, louder with a
/// duty closer to 50 %.
///
/// The board model calls [`drive`](Self::drive) whenever the timer channel's waveform may have
/// changed (a write to its prescaler, period, compare or enable registers). The buzzer reports
/// only real changes, so a stream of identical writes produces no events.
///
/// The Begode Extreme drives its buzzer from TIM4 channel 4 (PB9): 72 MHz / 40 / 667 ≈ 2.70 kHz,
/// with the compare value 1..=270 as the volume.
#[derive(Debug, Clone, Default)]
pub struct Buzzer {
    tone: Option<Pwm>,
    beeps: u64,
}

impl Buzzer {
    /// A silent buzzer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Applies the channel's current waveform (`None` when the output is disabled).
    pub fn drive(&mut self, at: u64, pwm: Option<Pwm>) -> Option<BuzzerEvent> {
        let tone = pwm.filter(Pwm::toggles);
        if tone == self.tone {
            return None;
        }
        if self.tone.is_none() {
            self.beeps += 1;
        }
        self.tone = tone;
        Some(BuzzerEvent { at, tone })
    }

    /// The tone sounding now, if any.
    pub const fn tone(&self) -> Option<Pwm> {
        self.tone
    }

    /// Times the buzzer started sounding.
    pub const fn beeps(&self) -> u64 {
        self.beeps
    }
}

impl Buzzer {
    /// Builds the `buzzer` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }
}

/// As a board component (kind `buzzer`): `"pwm"` takes the timer channel's [`Signal::Pwm`] (or
/// [`Signal::Released`] when the output is off), and every change goes into the run log.
impl Peripheral for Buzzer {
    fn reset(&mut self, _kind: ResetKind, ctx: &mut Ctx<'_>) {
        if let Some(event) = self.drive(ctx.now(), None) {
            ctx.log(event.to_string());
        }
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let pwm = match (port, signal) {
            (PORT_PWM, Signal::Pwm { tick_hz, period, high }) => Some(Pwm { tick_hz, period, high }),
            (PORT_PWM, Signal::Released) => None,
            _ => return,
        };
        if let Some(event) = self.drive(ctx.now(), pwm) {
            ctx.log(event.to_string());
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        (name == "pwm").then_some(PORT_PWM)
    }
}

impl std::fmt::Display for BuzzerEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.tone {
            Some(tone) => write!(f, "buzzer on, {:.0} Hz, duty {:.0} %", tone.frequency_hz(), 100.0 * tone.duty()),
            None => f.write_str("buzzer off"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Extreme's buzzer: TIM4 PSC 39 (1.8 MHz ticks), ARR 666.
    fn tim4(ccr: u32) -> Pwm {
        Pwm { tick_hz: 72_000_000 / 40, period: 667, high: ccr }
    }

    #[test]
    fn horn_command_beeps_twice() {
        let mut buzzer = Buzzer::new();
        let on = buzzer.drive(100, Some(tim4(151))).expect("starts");
        assert!((on.tone.unwrap().frequency_hz() - 2698.65).abs() < 0.01);
        assert_eq!(buzzer.drive(150, Some(tim4(151))), None, "same waveform, no event");
        assert_eq!(buzzer.drive(200, Some(tim4(0))), Some(BuzzerEvent { at: 200, tone: None }));
        assert!(buzzer.drive(300, Some(tim4(151))).is_some());
        assert!(buzzer.drive(400, None).is_some());
        assert_eq!(buzzer.beeps(), 2);
        assert_eq!(buzzer.tone(), None);
    }

    #[test]
    fn volume_change_while_sounding_is_an_event_but_not_a_beep() {
        let mut buzzer = Buzzer::new();
        buzzer.drive(0, Some(tim4(1)));
        let louder = buzzer.drive(10, Some(tim4(270))).unwrap();
        assert!((louder.tone.unwrap().duty() - 270.0 / 667.0).abs() < 1e-12);
        assert_eq!(buzzer.beeps(), 1);
    }

    #[test]
    fn steady_levels_are_silent() {
        let mut buzzer = Buzzer::new();
        assert_eq!(buzzer.drive(0, Some(tim4(667))), None, "100 % duty");
        assert_eq!(buzzer.drive(0, Some(Pwm { tick_hz: 0, period: 667, high: 100 })), None, "timer stopped");
        assert_eq!(buzzer.beeps(), 0);
    }

    #[test]
    fn as_a_component_it_logs_tone_changes() {
        use euc_core::testing::TestHost;

        let mut host = TestHost::new(72_000_000);
        let mut buzzer = Buzzer::new();
        let pwm = buzzer.port("pwm").unwrap();
        buzzer.input(pwm, Signal::Pwm { tick_hz: 1_800_000, period: 667, high: 151 }, &mut host.ctx());
        buzzer.input(pwm, Signal::Pwm { tick_hz: 1_800_000, period: 667, high: 0 }, &mut host.ctx());
        assert_eq!(host.log, ["buzzer on, 2699 Hz, duty 23 %", "buzzer off"]);
        assert_eq!(buzzer.beeps(), 1);
    }
}

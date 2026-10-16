//! The board's analog front end: what each ADC channel converts, as a raw 12-bit count.
//!
//! The ADC model applies its own offset registers and data alignment; this module delivers the
//! count a perfect converter would produce from the sensor's output voltage.

use serde::Deserialize;

use crate::rng::Rng;

/// Count of a bipolar sensor at zero input (half the 3.3 V reference).
const MID_SCALE: f64 = 2048.0;

/// A motor phase (`phase = "a"` in `plant.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    /// Phase a.
    A,
    /// Phase b.
    B,
    /// Phase c.
    C,
}

impl Phase {
    /// Index 0, 1, 2.
    pub const fn index(self) -> usize {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
        }
    }
}

/// A current sensor around mid-scale: `count = 2048 + i / amps_per_count + offset_counts`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CurrentChannel {
    /// ADC number (1-based).
    pub adc: u8,
    /// ADC input channel.
    pub channel: u8,
    /// Amperes per count; negative for an inverted sensor.
    pub amps_per_count: f64,
    /// Zero error, counts.
    #[serde(default)]
    pub offset_counts: f64,
}

/// A phase current sensor: which phase and how it converts.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhaseCurrentChannel {
    /// The motor phase whose current it measures (positive into the motor).
    pub phase: Phase,
    /// ADC number (1-based).
    pub adc: u8,
    /// ADC input channel.
    pub channel: u8,
    /// Amperes per count; negative for an inverted sensor.
    pub amps_per_count: f64,
    /// Zero error, counts.
    #[serde(default)]
    pub offset_counts: f64,
}

/// A unipolar voltage divider: `count = V / volts_per_count`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VoltageChannel {
    /// ADC number (1-based).
    pub adc: u8,
    /// ADC input channel.
    pub channel: u8,
    /// Volts at the source per count.
    pub volts_per_count: f64,
}

/// An NTC thermistor to ground under a pull-up to the ADC reference.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NtcChannel {
    /// ADC number (1-based).
    pub adc: u8,
    /// ADC input channel.
    pub channel: u8,
    /// Resistance at 25 °C, ohm.
    pub r25_ohm: f64,
    /// B constant, kelvin.
    pub beta_k: f64,
    /// Pull-up resistor, ohm.
    pub pullup_ohm: f64,
}

/// Sensor wiring (`[sensors]` in `plant.toml`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorParams {
    /// Phase current sensors.
    pub phase_current: Vec<PhaseCurrentChannel>,
    /// DC bus (battery) current sensor; positive = the battery discharges.
    pub bus_current: Option<CurrentChannel>,
    /// DC bus voltage divider.
    pub bus_voltage: Option<VoltageChannel>,
    /// Power-stage (board) NTC.
    pub board_ntc: Option<NtcChannel>,
    /// Standard deviation of the noise added to every count.
    #[serde(default)]
    pub noise_counts: f64,
}

/// The physical values the sensors see.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measurands {
    /// Phase currents a, b, c, ampere (positive into the motor).
    pub phase_a: [f64; 3],
    /// Battery current, ampere (positive = discharge).
    pub bus_a: f64,
    /// Bus voltage, volt.
    pub bus_v: f64,
    /// Power-stage temperature, °C.
    pub board_c: f64,
}

impl SensorParams {
    /// The count ADC `adc` converts on `channel`, or `None` if no sensor of this plant is wired
    /// there.
    pub(crate) fn count(&self, adc: u8, channel: u8, m: &Measurands, rng: &mut Rng) -> Option<u16> {
        let at = |a: u8, c: u8| a == adc && c == channel;
        let value = (self.phase_current.iter().find(|s| at(s.adc, s.channel)))
            .map(|s| MID_SCALE + m.phase_a[s.phase.index()] / s.amps_per_count + s.offset_counts)
            .or_else(|| {
                let s = self.bus_current.filter(|s| at(s.adc, s.channel))?;
                Some(MID_SCALE + m.bus_a / s.amps_per_count + s.offset_counts)
            })
            .or_else(|| self.bus_voltage.filter(|s| at(s.adc, s.channel)).map(|s| m.bus_v / s.volts_per_count))
            .or_else(|| self.board_ntc.filter(|s| at(s.adc, s.channel)).map(|s| ntc_count(m.board_c, s)))?;
        Some(counts(value + rng.gaussian(self.noise_counts)))
    }
}

/// A 12-bit count, rounded half to even like the reference harness.
fn counts(value: f64) -> u16 {
    value.round_ties_even().clamp(0.0, 4095.0) as u16
}

/// The (unrounded) count of an NTC at `temp_c`.
fn ntc_count(temp_c: f64, ntc: NtcChannel) -> f64 {
    let r = ntc.r25_ohm * (ntc.beta_k * (1.0 / (temp_c + 273.15) - 1.0 / 298.15)).exp();
    4096.0 * r / (r + ntc.pullup_ohm)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The Begode Extreme's front end (see `profiles/begode-extreme/plant.toml`).
    pub(crate) fn extreme() -> SensorParams {
        let amps = 437.5 / 2048.0;
        SensorParams {
            phase_current: vec![
                PhaseCurrentChannel { phase: Phase::A, adc: 1, channel: 4, amps_per_count: -amps, offset_counts: 0.0 },
                PhaseCurrentChannel { phase: Phase::B, adc: 2, channel: 5, amps_per_count: amps, offset_counts: 0.0 },
            ],
            bus_current: Some(CurrentChannel { adc: 2, channel: 10, amps_per_count: amps, offset_counts: 0.0 }),
            bus_voltage: Some(VoltageChannel { adc: 1, channel: 0, volts_per_count: 1377.0 / 32768.0 }),
            board_ntc: Some(NtcChannel {
                adc: 1,
                channel: 11,
                r25_ohm: 47_000.0,
                beta_k: 4150.0,
                pullup_ohm: 10_000.0,
            }),
            noise_counts: 0.0,
        }
    }

    fn at(currents: [f64; 3], bus_a: f64) -> Measurands {
        Measurands { phase_a: currents, bus_a, bus_v: 130.0, board_c: 25.0 }
    }

    #[test]
    fn idle_counts_match_the_m1_reference() {
        let (s, mut rng) = (extreme(), Rng::new(0));
        let m = at([0.0; 3], 0.0);
        assert_eq!(s.count(1, 0, &m, &mut rng), Some(3094), "130 V");
        assert_eq!(s.count(1, 11, &m, &mut rng), Some(3377), "25 °C");
        assert_eq!(s.count(1, 4, &m, &mut rng), Some(2048));
        assert_eq!(s.count(2, 10, &m, &mut rng), Some(2048));
        assert_eq!(s.count(3, 4, &m, &mut rng), None, "not this plant's");
    }

    #[test]
    fn phase_a_is_inverted_and_scales_like_the_firmware() {
        let (s, mut rng) = (extreme(), Rng::new(0));
        let m = at([42.725, -21.3623, -21.3627], 0.0);
        let a = s.count(1, 4, &m, &mut rng).unwrap();
        let b = s.count(2, 5, &m, &mut rng).unwrap();
        assert_eq!((a, b), (2048 - 200, 2048 - 100));
        // The firmware: Ia = −(16·raw − 16·2048) counts of 437.5 A / 32768.
        let firmware_ia = -(16.0 * f64::from(a) - 32768.0) * 437.5 / 32768.0;
        assert!((firmware_ia - 42.725).abs() < 0.11);
        assert_eq!(s.count(1, 4, &at([1e4, 0.0, 0.0], 0.0), &mut rng), Some(0), "clamped");
    }

    #[test]
    fn hot_board_reads_lower() {
        let (s, mut rng) = (extreme(), Rng::new(0));
        let hot = Measurands { board_c: 80.0, ..at([0.0; 3], 0.0) };
        assert!(s.count(1, 11, &hot, &mut rng).unwrap() < 3377 - 1500);
    }

    #[test]
    fn parses_from_toml() {
        let text = r#"
            phase_current = [{ phase = "a", adc = 1, channel = 4, amps_per_count = -0.2136 }]
            bus_voltage = { adc = 1, channel = 0, volts_per_count = 0.042 }
            noise_counts = 1.5
        "#;
        let s: SensorParams = toml::from_str(text).unwrap();
        assert_eq!(s.phase_current[0].phase, Phase::A);
        assert_eq!((s.bus_current, s.noise_counts), (None, 1.5));
    }
}

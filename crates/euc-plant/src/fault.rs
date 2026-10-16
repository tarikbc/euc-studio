//! Faults to inject into the plant: failing hall and current sensors, an open motor phase, an
//! overheating power stage, a weak battery.
//!
//! A scenario lists faults on its timeline, one `[[timeline]]` table each with `fault = "<kind>"`
//! and the kind's fields (`docs/models/electrical.md` has the list). The simulator hands each one
//! to [`ElectricalPlant::add_fault`](crate::ElectricalPlant::add_fault) with its window in engine
//! cycles; the plant applies it from the first grid point at or after the window's start to the
//! first grid point at or after its end.

use serde::Deserialize;

use crate::PlantParams;
use crate::battery::PackCondition;
use crate::halls::HallParams;
use crate::sensors::Phase;

/// A hall sensor line (`line = "a"`): a is code bit 0, b bit 1, c bit 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HallLine {
    /// Sensor a (PB7 on the Begode Extreme).
    A,
    /// Sensor b (PB6).
    B,
    /// Sensor c (PB8).
    C,
}

/// A logic level (`level = "high"`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Level {
    /// Logic high.
    High,
    /// Logic low.
    Low,
}

/// A current sensor (`sensor = "a"`: the sensor on phase a; `"bus"`: the battery current).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CurrentSensor {
    /// The sensor on phase a.
    A,
    /// The sensor on phase b.
    B,
    /// The sensor on phase c.
    C,
    /// The battery (DC bus) current sensor.
    Bus,
}

/// A fault of the plant, as a scenario's `[[timeline]]` entry writes it (`fault = "hall_stuck"`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(tag = "fault", rename_all = "snake_case", deny_unknown_fields)]
pub enum PlantFault {
    /// A hall line held at a level (a shorted sensor output or wire).
    HallStuck {
        /// The line.
        line: HallLine,
        /// The level it reads.
        level: Level,
    },
    /// A hall line disconnected. It reads high: hall sensors have open-collector outputs, so the
    /// board pulls their inputs up (INFERRED).
    HallLost {
        /// The line.
        line: HallLine,
    },
    /// Extra jitter on the sensed angle (a noisy supply, a loose magnet ring), added to the
    /// configured noise.
    HallNoise {
        /// Standard deviation, electrical degrees.
        noise_deg: f64,
    },
    /// A current sensor reads `gain · i + offset_a`, plus Gaussian noise.
    CurrentSensor {
        /// Which sensor.
        sensor: CurrentSensor,
        /// Zero error, ampere.
        #[serde(default)]
        offset_a: f64,
        /// Gain error: 1 is correct, 0.9 reads 10 % low, −1 reads with the wrong sign.
        #[serde(default = "unity")]
        gain: f64,
        /// Standard deviation of the noise, ampere.
        #[serde(default)]
        noise_a: f64,
    },
    /// A motor phase wire open: no current flows in it, whatever its inverter leg does.
    OpenPhase {
        /// The phase.
        phase: Phase,
    },
    /// The power stage heats up as if its cooling failed: it climbs `rate_k_per_s` above what
    /// its losses give, up to `max_c`, and holds there. After the window it cools back with the
    /// stage's own time constant.
    StageOvertemp {
        /// Extra heating, kelvin per second.
        rate_k_per_s: f64,
        /// Highest temperature it reaches, °C.
        max_c: f64,
    },
    /// One cell group sits `drop_v` below the open-circuit voltage curve (a weak or damaged
    /// cell): its BMS reading drops, and so does the pack voltage.
    CellLow {
        /// Pack, from 0.
        pack: u8,
        /// Cell group in series order, from 0.
        cell: u8,
        /// Volts below the curve.
        drop_v: f64,
    },
    /// One pack's state of charge jumps by `soc_delta` when the window starts (packs charged
    /// apart). The jump stays: the window's end does not undo it.
    PackImbalance {
        /// Pack, from 0.
        pack: u8,
        /// Change of state of charge, −1..=1.
        soc_delta: f64,
    },
    /// One pack's internal resistance (R0 and R1) is `factor` times normal (aged cells, a loose
    /// connector): it sags more and carries less of the load.
    HighResistance {
        /// Pack, from 0.
        pack: u8,
        /// Resistance multiplier, above 0.
        factor: f64,
    },
}

const fn unity() -> f64 {
    1.0
}

impl PlantFault {
    /// Every `fault` name of this enum, for a timeline parser that dispatches on it.
    pub const KINDS: &[&str] = &[
        "hall_stuck",
        "hall_lost",
        "hall_noise",
        "current_sensor",
        "open_phase",
        "stage_overtemp",
        "cell_low",
        "pack_imbalance",
        "high_resistance",
    ];

    /// The `fault` name of this fault.
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::HallStuck { .. } => "hall_stuck",
            Self::HallLost { .. } => "hall_lost",
            Self::HallNoise { .. } => "hall_noise",
            Self::CurrentSensor { .. } => "current_sensor",
            Self::OpenPhase { .. } => "open_phase",
            Self::StageOvertemp { .. } => "stage_overtemp",
            Self::CellLow { .. } => "cell_low",
            Self::PackImbalance { .. } => "pack_imbalance",
            Self::HighResistance { .. } => "high_resistance",
        }
    }

    /// Checks the fault against the plant it goes into.
    pub fn validate(&self, params: &PlantParams) -> Result<(), String> {
        let bad = |why: String| Err(format!("fault {}: {why}", self.kind()));
        let pack_ok = |pack: u8| pack < params.battery.packs;
        match *self {
            Self::HallNoise { noise_deg } if !(noise_deg >= 0.0 && noise_deg.is_finite()) => {
                bad("noise_deg must be 0 or more".into())
            }
            Self::CurrentSensor { sensor, offset_a, gain, noise_a } => {
                if !(offset_a.is_finite() && gain.is_finite() && noise_a >= 0.0 && noise_a.is_finite()) {
                    return bad("offset_a and gain must be finite, noise_a 0 or more".into());
                }
                let wired = match sensor {
                    CurrentSensor::A => params.sensors.phase_current.iter().any(|s| s.phase == Phase::A),
                    CurrentSensor::B => params.sensors.phase_current.iter().any(|s| s.phase == Phase::B),
                    CurrentSensor::C => params.sensors.phase_current.iter().any(|s| s.phase == Phase::C),
                    CurrentSensor::Bus => params.sensors.bus_current.is_some(),
                };
                if wired { Ok(()) } else { bad(format!("this board has no {sensor:?} current sensor")) }
            }
            Self::StageOvertemp { rate_k_per_s, max_c } if !(rate_k_per_s > 0.0 && max_c.is_finite()) => {
                bad("rate_k_per_s must be above 0 and max_c finite".into())
            }
            Self::CellLow { pack, cell, drop_v } => {
                if !pack_ok(pack) || cell >= params.battery.series || !drop_v.is_finite() {
                    return bad(format!(
                        "needs pack < {}, cell < {} and a finite drop_v",
                        params.battery.packs, params.battery.series
                    ));
                }
                Ok(())
            }
            Self::PackImbalance { pack, soc_delta } if !pack_ok(pack) || !(-1.0..=1.0).contains(&soc_delta) => {
                bad(format!("needs pack < {} and soc_delta in -1..=1", params.battery.packs))
            }
            Self::HighResistance { pack, factor } if !(pack_ok(pack) && factor > 0.0 && factor.is_finite()) => {
                bad(format!("needs pack < {} and factor above 0", params.battery.packs))
            }
            _ => Ok(()),
        }
    }
}

/// A sensor's error while a `current_sensor` fault is active.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SensorError {
    pub(crate) gain: f64,
    pub(crate) offset_a: f64,
    pub(crate) noise_a: f64,
}

impl SensorError {
    const NONE: Self = Self { gain: 1.0, offset_a: 0.0, noise_a: 0.0 };
}

/// What the active faults amount to.
#[derive(Debug, Clone)]
pub(crate) struct Effects {
    /// The hall sensors with stuck lines and extra noise.
    pub(crate) halls: HallParams,
    /// Current sensor errors: phases a, b, c, then the bus.
    pub(crate) sensors: [SensorError; 4],
    /// Open motor phases.
    pub(crate) cut: [bool; 3],
    /// Stage overheating: (kelvin per second, ceiling °C).
    pub(crate) overtemp: Option<(f64, f64)>,
    /// Each pack's condition.
    pub(crate) packs: Vec<PackCondition>,
}

/// A fault with its window.
#[derive(Debug, Clone)]
struct Scheduled {
    from: u64,
    until: Option<u64>,
    fault: PlantFault,
    started: bool,
}

impl Scheduled {
    fn active(&self, now: u64) -> bool {
        self.from <= now && self.until.is_none_or(|until| now < until)
    }
}

/// The plant's faults and what they amount to now.
#[derive(Debug, Clone)]
pub(crate) struct Faults {
    healthy: Effects,
    effects: Effects,
    list: Vec<Scheduled>,
    /// The earliest cycle at which the active set may change; `u64::MAX` when it never does.
    next_change: u64,
}

impl Faults {
    pub(crate) fn new(params: &PlantParams) -> Self {
        let healthy = Effects {
            halls: params.halls,
            sensors: [SensorError::NONE; 4],
            cut: [false; 3],
            overtemp: None,
            packs: vec![PackCondition::healthy(params.battery.series); usize::from(params.battery.packs)],
        };
        Self { effects: healthy.clone(), healthy, list: Vec::new(), next_change: u64::MAX }
    }

    pub(crate) const fn effects(&self) -> &Effects {
        &self.effects
    }

    /// The earliest cycle at which the active set may change.
    pub(crate) fn next_change(&self) -> Option<u64> {
        (self.next_change != u64::MAX).then_some(self.next_change)
    }

    pub(crate) fn add(&mut self, from: u64, until: Option<u64>, fault: PlantFault) {
        self.list.push(Scheduled { from, until, fault, started: false });
        self.next_change = 0;
    }

    /// Recomputes the effects if the active set may have changed by `now`. Returns `None` when
    /// nothing changed, else the state-of-charge jumps that start now (`(pack, delta)`).
    pub(crate) fn refresh(&mut self, now: u64) -> Option<Vec<(usize, f64)>> {
        if now < self.next_change {
            return None;
        }
        let mut effects = self.healthy.clone();
        let mut jumps = Vec::new();
        let mut extra_noise = 0.0;
        for scheduled in self.list.iter_mut().filter(|s| s.active(now)) {
            match scheduled.fault {
                PlantFault::HallStuck { line, level } => set_line(&mut effects.halls, line, level == Level::High),
                PlantFault::HallLost { line } => set_line(&mut effects.halls, line, true),
                PlantFault::HallNoise { noise_deg } => extra_noise += noise_deg * noise_deg,
                PlantFault::CurrentSensor { sensor, offset_a, gain, noise_a } => {
                    effects.sensors[sensor as usize] = SensorError { gain, offset_a, noise_a };
                }
                PlantFault::OpenPhase { phase } => effects.cut[phase.index()] = true,
                PlantFault::StageOvertemp { rate_k_per_s, max_c } => effects.overtemp = Some((rate_k_per_s, max_c)),
                PlantFault::CellLow { pack, cell, drop_v } => {
                    effects.packs[usize::from(pack)].cell_drop_v[usize::from(cell)] += drop_v;
                }
                PlantFault::PackImbalance { pack, soc_delta } => {
                    if !scheduled.started {
                        jumps.push((usize::from(pack), soc_delta));
                    }
                }
                PlantFault::HighResistance { pack, factor } => {
                    effects.packs[usize::from(pack)].resistance_factor *= factor;
                }
            }
            scheduled.started = true;
        }
        effects.halls.noise_deg = (effects.halls.noise_deg.powi(2) + extra_noise).sqrt();
        self.effects = effects;
        self.next_change = self
            .list
            .iter()
            .flat_map(|s| [Some(s.from), s.until])
            .flatten()
            .filter(|&at| at > now)
            .min()
            .unwrap_or(u64::MAX);
        Some(jumps)
    }
}

fn set_line(halls: &mut HallParams, line: HallLine, high: bool) {
    let slot = match line {
        HallLine::A => &mut halls.stuck.a,
        HallLine::B => &mut halls.stuck.b,
        HallLine::C => &mut halls.stuck.c,
    };
    *slot = Some(high);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A timeline entry as a scenario parser would hold it: its window, then the fault's fields.
    #[derive(Debug, Deserialize)]
    struct Entry {
        at_s: f64,
        for_s: Option<f64>,
        #[serde(flatten)]
        fault: PlantFault,
    }

    fn entry(text: &str) -> Result<Entry, String> {
        toml::from_str(text).map_err(|e| e.to_string())
    }

    #[test]
    fn timeline_entries_parse() {
        let e = entry("at_s = 12.0\nfor_s = 3.0\nfault = \"hall_stuck\"\nline = \"a\"\nlevel = \"low\"").unwrap();
        assert_eq!((e.at_s, e.for_s), (12.0, Some(3.0)));
        assert_eq!(e.fault, PlantFault::HallStuck { line: HallLine::A, level: Level::Low });
        let e = entry("at_s = 1.0\nfault = \"current_sensor\"\nsensor = \"bus\"\noffset_a = 4.0").unwrap();
        assert_eq!(
            e.fault,
            PlantFault::CurrentSensor { sensor: CurrentSensor::Bus, offset_a: 4.0, gain: 1.0, noise_a: 0.0 }
        );
        assert_eq!(e.for_s, None, "no duration: to the end of the run");
        let wrong = entry("at_s = 1.0\nfault = \"open_phase\"\nphase = \"a\"\nlevel = \"low\"").unwrap_err();
        assert!(wrong.contains("level"), "unknown fields are errors: {wrong}");
        assert!(entry("at_s = 1.0\nfault = \"melted\"").is_err());
    }

    #[test]
    fn kinds_list_every_variant() {
        let samples = [
            "fault = \"hall_stuck\"\nline = \"b\"\nlevel = \"high\"",
            "fault = \"hall_lost\"\nline = \"c\"",
            "fault = \"hall_noise\"\nnoise_deg = 3.0",
            "fault = \"current_sensor\"\nsensor = \"a\"\ngain = 0.9",
            "fault = \"open_phase\"\nphase = \"c\"",
            "fault = \"stage_overtemp\"\nrate_k_per_s = 2.0\nmax_c = 95.0",
            "fault = \"cell_low\"\npack = 1\ncell = 7\ndrop_v = 0.5",
            "fault = \"pack_imbalance\"\npack = 0\nsoc_delta = -0.2",
            "fault = \"high_resistance\"\npack = 1\nfactor = 4.0",
        ];
        let kinds: Vec<&str> = samples.iter().map(|s| toml::from_str::<PlantFault>(s).unwrap().kind()).collect();
        assert_eq!(kinds, PlantFault::KINDS);
    }

    #[test]
    fn windows_and_effects() {
        let params = crate::plant::tests::params();
        let mut faults = Faults::new(&params);
        faults.add(100, Some(200), PlantFault::HallStuck { line: HallLine::B, level: Level::High });
        faults.add(150, None, PlantFault::PackImbalance { pack: 1, soc_delta: -0.1 });
        faults.add(150, None, PlantFault::HallNoise { noise_deg: 4.0 });
        assert_eq!(faults.refresh(0), Some(vec![]));
        assert_eq!(faults.next_change(), Some(100));
        assert_eq!(faults.refresh(50), None, "nothing changes before 100");
        faults.refresh(100);
        assert_eq!(faults.effects().halls.stuck.b, Some(true));
        assert_eq!(faults.refresh(160), Some(vec![(1, -0.1)]), "the jump happens once");
        assert!((faults.effects().halls.noise_deg - 4.0).abs() < 1e-12);
        assert_eq!(faults.refresh(200), Some(vec![]));
        assert_eq!(faults.effects().halls.stuck.b, None, "the stuck line is released at 200");
        assert_eq!(faults.next_change(), None);
    }

    #[test]
    fn validation_names_the_problem() {
        let params = crate::plant::tests::params();
        let c_sensor = PlantFault::CurrentSensor { sensor: CurrentSensor::C, offset_a: 1.0, gain: 1.0, noise_a: 0.0 };
        assert_eq!(c_sensor.validate(&params).unwrap_err(), "fault current_sensor: this board has no C current sensor");
        let cell = PlantFault::CellLow { pack: 2, cell: 0, drop_v: 0.3 };
        assert_eq!(
            cell.validate(&params).unwrap_err(),
            "fault cell_low: needs pack < 2, cell < 32 and a finite drop_v"
        );
        assert!(PlantFault::HighResistance { pack: 0, factor: 0.0 }.validate(&params).is_err());
        assert!(PlantFault::OpenPhase { phase: Phase::B }.validate(&params).is_ok());
    }
}

//! Scenarios: a scripted ride with expectations, as `scenarios/*.toml`.
//!
//! ```toml
//! schema = 1
//! name = "ride-20kmh-stop"
//! description = "Lean to 20 km/h, hold it, brake to a stop."
//! wheel = "begode-extreme"
//! seconds = 40.0
//! release_s = 6.0                    # the stand lets the wheel go
//! speed_kmh = [[0, 0], [10, 0], [18, 20], [28, 20], [34, 0]]   # [s, km/h], linear in between
//!
//! [[timeline]]                     # a fault: its window, then the fault's own fields
//! at_s = 15.0
//! for_s = 2.0                        # absent: until the end
//! fault = "hall_stuck"
//! line = "a"
//! level = "low"
//!
//! [[expect]]
//! check = "speed_reached"
//! kmh = 19.0
//! ```
//!
//! The rider follows the target speed by leaning (see `euc-physics`). Faults go to the electrical
//! plant (`euc_plant::fault::PlantFault`) or to a device (`euc_devices::DeviceFault`, with
//! `device = "<instance>"` when the board has several). Checks look at the whole run after it
//! ends; each says what it measured. [`Ride`] runs a scenario one sample at a time; [`run`] runs
//! it to the end.

use euc_devices::{DeviceFault, Mpu6500, SmartBms};
use euc_physics::{Assist, Intent, Rider, Terrain};
use euc_plant::fault::PlantFault;
use euc_telemetry::{CHANNELS, EventKind, Signals, Snapshot};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::session::Session;

/// A scenario file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    /// Format version, 1.
    pub schema: u32,
    /// Short name, as the file name.
    pub name: String,
    /// What it shows, for people.
    pub description: String,
    /// Wheel profile id.
    pub wheel: String,
    /// Simulated seconds from power-on.
    pub seconds: f64,
    /// When the stand lets the wheel go, seconds. The firmware must be balancing by then.
    pub release_s: f64,
    /// Target speed: `[seconds, km/h]` points, linear in between, held after the last.
    pub speed_kmh: Vec<[f64; 2]>,
    /// The rider (defaults for anything not given).
    #[serde(default)]
    pub rider: Rider,
    /// Aids for the rigid rider.
    #[serde(default)]
    pub assist: Assist,
    /// Settings preset the wheel boots with (`profiles/<wheel>/settings.toml`); none: a blank
    /// settings journal.
    pub settings: Option<String>,
    /// The ground: a slope, bumps, potholes (`euc_physics::Terrain`); flat by default.
    #[serde(default)]
    pub terrain: Terrain,
    /// BLE commands to send.
    #[serde(default)]
    pub inject: Vec<Inject>,
    /// Pushes on the rider: `[start s, newtons forward, duration s]`.
    #[serde(default)]
    pub push: Vec<[f64; 3]>,
    /// What must hold.
    #[serde(default, rename = "expect")]
    pub expects: Vec<Expect>,
    /// `[[timeline]]` as written; [`Scenario::from_toml`] parses it into [`Scenario::faults`].
    #[serde(default, rename = "timeline")]
    timeline: Vec<toml::Table>,
    /// Faults to inject, from `[[timeline]]`.
    #[serde(skip)]
    pub faults: Vec<TimedFault>,
}

/// A fault and its window.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedFault {
    /// Start, seconds.
    pub at_s: f64,
    /// Length, seconds (`None`: to the end).
    pub for_s: Option<f64>,
    /// What breaks.
    pub fault: Fault,
}

/// What a timeline entry breaks.
#[derive(Debug, Clone, PartialEq)]
pub enum Fault {
    /// Part of the electrical plant (halls, current sensors, a phase, the battery, the stage).
    Plant(PlantFault),
    /// A board device; `device` names the instance when the board has several of its kind.
    Device {
        /// Instance name in `board.toml`.
        device: Option<String>,
        /// The fault.
        fault: DeviceFault,
    },
}

impl Fault {
    /// The `fault` name.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Plant(fault) => fault.kind(),
            Self::Device { fault, .. } => fault.kind(),
        }
    }
}

/// Parses one `[[timeline]]` table: the window keys, then the fault by its `fault` name.
fn timed_fault(mut table: toml::Table) -> Result<TimedFault, String> {
    let number = |value: toml::Value| value.as_float().or_else(|| value.as_integer().map(|i| i as f64));
    let at_s = table.remove("at_s").and_then(number).ok_or("needs at_s (seconds)")?;
    let for_s = match table.remove("for_s") {
        Some(value) => Some(number(value).ok_or("for_s must be a number")?),
        None => None,
    };
    let device = match table.remove("device") {
        Some(toml::Value::String(name)) => Some(name),
        Some(_) => return Err("device must be an instance name".into()),
        None => None,
    };
    let kind = table.get("fault").and_then(toml::Value::as_str).ok_or("needs fault = \"<kind>\"")?.to_owned();
    let value = toml::Value::Table(table);
    let fault = if PlantFault::KINDS.contains(&kind.as_str()) {
        if device.is_some() {
            return Err(format!("{kind} is a plant fault; device does not apply"));
        }
        Fault::Plant(value.try_into().map_err(|e: toml::de::Error| format!("{kind}: {e}"))?)
    } else if DeviceFault::KINDS.contains(&kind.as_str()) {
        let fault: DeviceFault = value.try_into().map_err(|e: toml::de::Error| format!("{kind}: {e}"))?;
        fault.validate().map_err(|e| format!("{kind}: {e}"))?;
        Fault::Device { device, fault }
    } else {
        let known: Vec<&str> = PlantFault::KINDS.iter().chain(DeviceFault::KINDS).copied().collect();
        return Err(format!("unknown fault \"{kind}\" (known: {})", known.join(", ")));
    };
    if at_s.is_nan() || at_s < 0.0 || for_s.is_some_and(|f| f.is_nan() || f <= 0.0) {
        return Err(format!("{kind}: at_s must not be negative and for_s must be positive"));
    }
    Ok(TimedFault { at_s, for_s, fault })
}

/// BLE bytes the phone sends at a time.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Inject {
    /// When, seconds.
    pub at_s: f64,
    /// The bytes, as text.
    pub bytes: String,
}

/// One expectation.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "check", rename_all = "snake_case", deny_unknown_fields)]
pub enum Expect {
    /// The wheel never falls over (pitch or roll beyond 45°).
    Upright {
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
    },
    /// The shell's pitch stays within a bound.
    Pitch {
        /// Largest allowed |pitch|, degrees.
        max_abs_deg: f64,
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
    },
    /// The firmware never cuts the motor (none of its cutoff flags is set).
    NoCutoff {
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
    },
    /// The ground speed reaches at least this.
    SpeedReached {
        /// km/h.
        kmh: f64,
    },
    /// The wheel stands still at the end.
    StoppedAtEnd {
        /// Largest allowed |speed| at the end, km/h.
        max_abs_kmh: f64,
    },
    /// The firmware cuts the motor: its first cut at or after `from_s` comes within `within_s`,
    /// with the stock-map cutoff flag `flag` among those set, if given (a fault the firmware
    /// must catch).
    Cutoff {
        /// From this time on, seconds.
        from_s: f64,
        /// Seconds the firmware may take.
        within_s: f64,
        /// The flag that must be set at the cut, by stock-map name (`hall_fault`).
        flag: Option<String>,
    },
    /// The shell's pitch reaches `deg` (nose up positive): at least `deg` if it is positive, at
    /// most `deg` if negative (a tiltback, or a nose dive).
    PitchReaches {
        /// Degrees, nose up positive.
        deg: f64,
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
        /// By this time, seconds (default: the end).
        by_s: Option<f64>,
    },
    /// A telemetry signal stays steady: its standard deviation over the window is at most `max`
    /// (torque or current ripple, a pitch disturbance).
    Ripple {
        /// Telemetry key (`torque_nm`, `iq_a`, `pitch_deg`...).
        signal: String,
        /// Largest allowed standard deviation, in the signal's unit.
        max: f64,
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
        /// Until this time, seconds (default: the end).
        by_s: Option<f64>,
    },
    /// A firmware estimate follows the truth: a stock-map variable times `scale` stays within an
    /// RMS error of `max_rms` of a telemetry signal (the firmware's speed against the true speed).
    FirmwareTracks {
        /// Stock-map RAM variable name (`speed_cms`).
        var: String,
        /// Factor from the variable's unit to the signal's (`0.036` for cm/s to km/h).
        scale: f64,
        /// Telemetry key of the truth (`speed_kmh`).
        signal: String,
        /// Largest allowed RMS error, in the signal's unit.
        max_rms: f64,
        /// From this time on, seconds (default: the release).
        from_s: Option<f64>,
        /// Until this time, seconds (default: the end).
        by_s: Option<f64>,
    },
    /// Records a measurement and always passes: a statistic of a telemetry signal or of a
    /// stock-map variable (times `scale`) over a window.
    Record {
        /// Telemetry key; give this or `var`.
        signal: Option<String>,
        /// Stock-map RAM variable; give this or `signal`.
        var: Option<String>,
        /// Factor applied to `var` (default 1).
        scale: Option<f64>,
        /// What to record.
        stat: Stat,
        /// The level for `first_above` and `first_below`.
        threshold: Option<f64>,
        /// From this time on, seconds (default: power-on).
        from_s: Option<f64>,
        /// Until this time, seconds (default: the end).
        by_s: Option<f64>,
    },
    /// A firmware variable of the stock map has every bit of `bits` set (an alarm flag).
    FirmwareBits {
        /// Stock-map RAM variable name (`alerts`).
        var: String,
        /// Bits that must all be set.
        bits: u64,
        /// From this time on, seconds (default: power-on).
        from_s: Option<f64>,
        /// By this time, seconds (default: the end).
        by_s: Option<f64>,
    },
}

/// What a `record` check measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stat {
    /// The largest value, and when.
    Max,
    /// The smallest value, and when.
    Min,
    /// The mean.
    Mean,
    /// When the value first reaches `threshold` or more.
    FirstAbove,
    /// When the value first falls to `threshold` or less.
    FirstBelow,
    /// The value when the wheel first falls over.
    AtFall,
}

impl Expect {
    /// The stock-map variable this check reads, if any.
    pub fn firmware_var(&self) -> Option<&str> {
        match self {
            Self::FirmwareBits { var, .. } | Self::FirmwareTracks { var, .. } => Some(var),
            Self::Record { var, .. } => var.as_deref(),
            _ => None,
        }
    }

    /// The telemetry signal this check reads, if any.
    fn signal(&self) -> Option<&str> {
        match self {
            Self::Ripple { signal, .. } | Self::FirmwareTracks { signal, .. } => Some(signal),
            Self::Record { signal, .. } => signal.as_deref(),
            _ => None,
        }
    }

    /// Why this check cannot work as written, if it cannot.
    fn problem(&self) -> Option<String> {
        let Self::Record { signal, var, stat, threshold, .. } = self else { return None };
        if signal.is_some() == var.is_some() {
            return Some("a record check needs either signal or var".into());
        }
        let needs_level = matches!(stat, Stat::FirstAbove | Stat::FirstBelow);
        (needs_level && threshold.is_none()).then(|| "first_above and first_below need a threshold".into())
    }
}

/// The value of telemetry signal `key` in `signals`, if `key` names one.
fn signal_value(signals: &Signals, key: &str) -> Option<f64> {
    CHANNELS.iter().find(|c| c.key == key).map(|c| f64::from(signals.get(c.channel)))
}

/// What the firmware showed at one sample, for the checks that read it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Observation {
    /// Cutoff flags set, by stock-map name.
    pub cutoff_flags: Vec<String>,
    /// The stock-map variables the checks read, by name, raw.
    pub vars: Vec<(String, u64)>,
    /// The same variables as numbers, read as their stock-map kind says.
    pub values: Vec<(String, f64)>,
    /// The telemetry signals.
    pub signals: Signals,
}

/// A scenario that cannot run.
#[derive(Debug, Error)]
pub enum ScenarioError {
    /// Not valid TOML for a scenario.
    #[error("{0}")]
    Parse(#[from] toml::de::Error),
    /// A value out of range.
    #[error("{0}")]
    Invalid(String),
}

impl Scenario {
    /// Parses and checks a scenario.
    pub fn from_toml(text: &str) -> Result<Self, ScenarioError> {
        let mut scenario: Self = toml::from_str(text)?;
        let bad = |why: String| Err(ScenarioError::Invalid(why));
        if scenario.schema != 1 {
            return bad(format!("schema {} is not supported (expected 1)", scenario.schema));
        }
        if scenario.seconds.is_nan()
            || scenario.seconds <= 0.0
            || !(0.0..scenario.seconds).contains(&scenario.release_s)
        {
            return bad("seconds must be positive and release_s inside the run".into());
        }
        if scenario.speed_kmh.is_empty() || scenario.speed_kmh.windows(2).any(|w| w[1][0] < w[0][0]) {
            return bad("speed_kmh needs at least one [s, km/h] point, in time order".into());
        }
        if scenario.push.iter().any(|&[at, n, length]| !(at >= 0.0 && n.is_finite() && length > 0.0)) {
            return bad("push needs [start s, newtons, duration s] with a positive duration".into());
        }
        if let Some(problem) = scenario.expects.iter().find_map(Expect::problem) {
            return bad(problem);
        }
        if let Some(signal) =
            scenario.expects.iter().filter_map(Expect::signal).find(|s| !CHANNELS.iter().any(|c| c.key == *s))
        {
            let keys: Vec<&str> = CHANNELS.iter().map(|c| c.key).collect();
            return bad(format!("unknown telemetry signal \"{signal}\" (known: {})", keys.join(", ")));
        }
        for (i, table) in std::mem::take(&mut scenario.timeline).into_iter().enumerate() {
            let fault =
                timed_fault(table).map_err(|why| ScenarioError::Invalid(format!("timeline entry {}: {why}", i + 1)))?;
            scenario.faults.push(fault);
        }
        Ok(scenario)
    }

    /// The push on the rider at `t_s`, newtons forward (pushes that overlap add up).
    pub fn push_n(&self, t_s: f64) -> f64 {
        self.push.iter().filter(|&&[at, _, length]| (at..at + length).contains(&t_s)).map(|p| p[1]).sum()
    }

    /// The target speed at `t_s`, km/h.
    pub fn target_kmh(&self, t_s: f64) -> f64 {
        let points = &self.speed_kmh;
        let i = points.partition_point(|p| p[0] <= t_s);
        match (i.checked_sub(1).map(|k| points[k]), points.get(i)) {
            (Some([t0, v0]), Some(&[t1, v1])) if t1 > t0 => v0 + (v1 - v0) * (t_s - t0) / (t1 - t0),
            (Some([_, v0]), _) => v0,
            (None, Some(&[_, v1])) => v1,
            (None, None) => 0.0,
        }
    }
}

/// What a run looked like, sample by sample (only what the checks need).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Simulated seconds.
    pub t_s: f64,
    /// Ground speed, km/h.
    pub speed_kmh: f64,
    /// Shell pitch, degrees, nose up positive.
    pub pitch_deg: f64,
    /// The wheel is down.
    pub fallen: bool,
    /// A firmware cutoff flag is set.
    pub cutoff: bool,
}

/// The verdict of one expectation.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CheckResult {
    /// The check, with its bound (`pitch within 10°`).
    pub what: String,
    /// It held.
    pub passed: bool,
    /// What was measured.
    pub measured: String,
}

/// The verdicts of a run, with a few numbers that summarise it.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Outcome {
    /// One per expectation, in file order.
    pub checks: Vec<CheckResult>,
    /// Highest ground speed, km/h.
    pub top_speed_kmh: f64,
    /// Largest |pitch| after the release, degrees.
    pub max_abs_pitch_deg: f64,
    /// Distance travelled, metres.
    pub distance_m: f64,
}

impl Outcome {
    /// Every check held.
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|c| c.passed)
    }
}

impl Expect {
    /// Judges `samples` (the whole run) against this expectation. Checks that read the firmware
    /// need [`Expect::judge_observed`].
    pub fn judge(&self, samples: &[Sample], release_s: f64) -> CheckResult {
        self.judge_observed(samples, &[], release_s)
    }

    /// Judges `samples` with what the firmware showed at each (`observed`, one per sample, or
    /// empty when nothing was observed).
    pub fn judge_observed(&self, samples: &[Sample], observed: &[Observation], release_s: f64) -> CheckResult {
        let after = |from: Option<f64>| {
            let from = from.unwrap_or(release_s);
            samples.iter().filter(move |s| s.t_s >= from)
        };
        let from_text = |from: Option<f64>| format!("from {:.1} s", from.unwrap_or(release_s));
        match *self {
            Self::Upright { from_s } => {
                let fell = after(from_s).find(|s| s.fallen);
                CheckResult {
                    what: format!("upright {}", from_text(from_s)),
                    passed: fell.is_none(),
                    measured: fell.map_or_else(|| "stayed up".to_owned(), |s| format!("fell at {:.2} s", s.t_s)),
                }
            }
            Self::Pitch { max_abs_deg, from_s } => {
                let worst = after(from_s).max_by(|a, b| a.pitch_deg.abs().total_cmp(&b.pitch_deg.abs()));
                let max = worst.map_or(0.0, |s| s.pitch_deg.abs());
                CheckResult {
                    what: format!("|pitch| <= {max_abs_deg}° {}", from_text(from_s)),
                    passed: max <= max_abs_deg,
                    measured: worst.map_or_else(
                        || "no samples".to_owned(),
                        |s| format!("max {:.2}° at {:.2} s", s.pitch_deg, s.t_s),
                    ),
                }
            }
            Self::NoCutoff { from_s } => {
                let cut = after(from_s).find(|s| s.cutoff);
                CheckResult {
                    what: format!("no cutoff {}", from_text(from_s)),
                    passed: cut.is_none(),
                    measured: cut.map_or_else(|| "none".to_owned(), |s| format!("cut at {:.2} s", s.t_s)),
                }
            }
            Self::SpeedReached { kmh } => {
                let best = samples.iter().max_by(|a, b| a.speed_kmh.total_cmp(&b.speed_kmh));
                let top = best.map_or(0.0, |s| s.speed_kmh);
                CheckResult {
                    what: format!("speed reaches {kmh} km/h"),
                    passed: top >= kmh,
                    measured: best.map_or_else(
                        || "no samples".to_owned(),
                        |s| format!("top {:.2} km/h at {:.2} s", s.speed_kmh, s.t_s),
                    ),
                }
            }
            Self::StoppedAtEnd { max_abs_kmh } => {
                let end = samples.last().map_or(f64::NAN, |s| s.speed_kmh);
                CheckResult {
                    what: format!("|speed| <= {max_abs_kmh} km/h at the end"),
                    passed: end.abs() <= max_abs_kmh,
                    measured: format!("{end:.2} km/h"),
                }
            }
            Self::Cutoff { from_s, within_s, ref flag } => {
                judge_cutoff(samples, observed, from_s, within_s, flag.as_deref())
            }
            Self::PitchReaches { deg, from_s, by_s } => {
                judge_pitch_reaches(samples, deg, from_s.unwrap_or(release_s), by_s)
            }
            Self::FirmwareBits { ref var, bits, from_s, by_s } => {
                judge_bits(samples, observed, var, bits, from_s.unwrap_or(0.0), by_s)
            }
            Self::Ripple { ref signal, max, from_s, by_s } => {
                judge_ripple(samples, observed, signal, max, from_s.unwrap_or(release_s), by_s)
            }
            Self::Record { ref signal, ref var, scale, stat, threshold, from_s, by_s } => {
                let source = match (signal, var) {
                    (Some(signal), _) => Source::Signal(signal),
                    (None, var) => Source::Var(var.as_deref().unwrap_or(""), scale.unwrap_or(1.0)),
                };
                judge_record(samples, observed, source, (stat, threshold), (from_s.unwrap_or(0.0), by_s))
            }
            Self::FirmwareTracks { ref var, scale, ref signal, max_rms, from_s, by_s } => {
                let window = (from_s.unwrap_or(release_s), by_s);
                judge_tracks(samples, observed, (var, scale), signal, max_rms, window)
            }
        }
    }
}

/// Samples from `from` to `by` (the end if `None`), with their index.
fn window(samples: &[Sample], from: f64, by: Option<f64>) -> impl Iterator<Item = (&Sample, usize)> {
    samples.iter().zip(0..).filter(move |(s, _)| s.t_s >= from && by.is_none_or(|by| s.t_s <= by))
}

fn by_text(by: Option<f64>) -> String {
    by.map_or_else(|| "by the end".to_owned(), |by| format!("by {by:.1} s"))
}

/// [`Expect::Cutoff`].
fn judge_cutoff(
    samples: &[Sample],
    observed: &[Observation],
    from_s: f64,
    within_s: f64,
    flag: Option<&str>,
) -> CheckResult {
    let what =
        format!("cutoff{} within {within_s} s of {from_s:.1} s", flag.map_or_else(String::new, |f| format!(" ({f})")));
    let Some((cut, i)) = window(samples, from_s, None).find(|(s, _)| s.cutoff) else {
        return CheckResult { what, passed: false, measured: "no cutoff".to_owned() };
    };
    let flags = observed.get(i).map(|o| o.cutoff_flags.clone());
    let flag_ok = match (flag, &flags) {
        (None, _) => true,
        (Some(flag), Some(flags)) => flags.iter().any(|f| f == flag),
        (Some(_), None) => false,
    };
    let shown = flags.map_or_else(|| "flags not observed".to_owned(), |f| f.join(", "));
    CheckResult {
        what,
        passed: cut.t_s - from_s <= within_s && flag_ok,
        measured: format!("cut at {:.2} s ({shown})", cut.t_s),
    }
}

/// [`Expect::PitchReaches`].
fn judge_pitch_reaches(samples: &[Sample], deg: f64, from: f64, by_s: Option<f64>) -> CheckResult {
    let up = deg >= 0.0;
    let hit = window(samples, from, by_s).find(|(s, _)| if up { s.pitch_deg >= deg } else { s.pitch_deg <= deg });
    let pitches = window(samples, from, by_s).map(|(s, _)| s.pitch_deg);
    let extreme = if up { pitches.fold(f64::NEG_INFINITY, f64::max) } else { pitches.fold(f64::INFINITY, f64::min) };
    CheckResult {
        what: format!("pitch reaches {deg}° from {from:.1} s {}", by_text(by_s)),
        passed: hit.is_some(),
        measured: hit
            .map_or_else(|| format!("only {extreme:.2}°"), |(s, _)| format!("{:.2}° at {:.2} s", s.pitch_deg, s.t_s)),
    }
}

/// [`Expect::FirmwareBits`].
fn judge_bits(
    samples: &[Sample],
    observed: &[Observation],
    var: &str,
    bits: u64,
    from: f64,
    by_s: Option<f64>,
) -> CheckResult {
    let what = format!("{var} has bits 0x{bits:X} {}", by_text(by_s));
    if observed.is_empty() {
        return CheckResult { what, passed: false, measured: "the firmware was not observed".to_owned() };
    }
    let value = |i: usize| observed.get(i)?.vars.iter().find(|(name, _)| name == var).map(|&(_, v)| v);
    let hit = window(samples, from, by_s).find(|&(_, i)| value(i).is_some_and(|v| v & bits == bits));
    CheckResult {
        what,
        passed: hit.is_some(),
        measured: hit
            .map_or_else(|| "never".to_owned(), |(s, i)| format!("0x{:X} at {:.2} s", value(i).unwrap_or(0), s.t_s)),
    }
}

/// [`Expect::Ripple`].
fn judge_ripple(
    samples: &[Sample],
    observed: &[Observation],
    signal: &str,
    max: f64,
    from: f64,
    by_s: Option<f64>,
) -> CheckResult {
    let what = format!("{signal} spread <= {max} from {from:.1} s {}", by_text(by_s));
    let values: Vec<f64> = window(samples, from, by_s)
        .filter_map(|(_, i)| signal_value(&observed.get(i)?.signals, signal))
        .filter(|v| v.is_finite())
        .collect();
    if values.is_empty() {
        return CheckResult { what, passed: false, measured: "not observed".to_owned() };
    }
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let spread = (values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n).sqrt();
    CheckResult { what, passed: spread <= max, measured: format!("standard deviation {spread:.2}, mean {mean:.2}") }
}

/// Where a `record` check takes its values.
#[derive(Clone, Copy)]
enum Source<'a> {
    Signal(&'a str),
    Var(&'a str, f64),
}

impl Source<'_> {
    /// The value at one observed sample.
    fn value(self, seen: &Observation) -> Option<f64> {
        match self {
            Self::Signal(key) => signal_value(&seen.signals, key),
            Self::Var(var, scale) => Some(seen.values.iter().find(|(name, _)| name == var)?.1 * scale),
        }
    }
}

/// [`Expect::Record`].
fn judge_record(
    samples: &[Sample],
    observed: &[Observation],
    source: Source<'_>,
    (stat, threshold): (Stat, Option<f64>),
    (from, by_s): (f64, Option<f64>),
) -> CheckResult {
    let name = match source {
        Source::Signal(key) => key.to_owned(),
        Source::Var(var, scale) if (scale - 1.0).abs() < f64::EPSILON => var.to_owned(),
        Source::Var(var, scale) => format!("{var} x {scale}"),
    };
    let points: Vec<(f64, f64)> = window(samples, from, by_s)
        .filter_map(|(s, i)| Some((s.t_s, source.value(observed.get(i)?)?)))
        .filter(|(_, v)| v.is_finite())
        .collect();
    let level = threshold.unwrap_or(0.0);
    let fall = window(samples, from, by_s)
        .find(|(s, _)| s.fallen)
        .map(|(s, i)| (s.t_s, observed.get(i).and_then(|o| source.value(o))));
    let measured = if points.is_empty() {
        "not observed".to_owned()
    } else {
        match stat {
            Stat::Max | Stat::Min => {
                let pick = |a: &&(f64, f64), b: &&(f64, f64)| a.1.total_cmp(&b.1);
                let (t, v) = if stat == Stat::Max { points.iter().max_by(pick) } else { points.iter().min_by(pick) }
                    .copied()
                    .unwrap_or_default();
                format!("{} {v:.2} at {t:.2} s", if stat == Stat::Max { "max" } else { "min" })
            }
            Stat::Mean => format!("mean {:.2}", points.iter().map(|p| p.1).sum::<f64>() / points.len() as f64),
            Stat::AtFall => match fall {
                Some((t, Some(v))) => format!("{v:.2} when the wheel fell at {t:.2} s"),
                Some((t, None)) => format!("not observed when the wheel fell at {t:.2} s"),
                None => "the wheel did not fall".to_owned(),
            },
            Stat::FirstAbove | Stat::FirstBelow => {
                let above = stat == Stat::FirstAbove;
                let word = if above { "above" } else { "below" };
                points.iter().find(|(_, v)| if above { *v >= level } else { *v <= level }).map_or_else(
                    || format!("never {word} {level}"),
                    |(t, v)| format!("first {word} {level} at {t:.2} s ({v:.2})"),
                )
            }
        }
    };
    let what = format!("record {name} from {from:.1} s {}", by_text(by_s));
    CheckResult { what, passed: true, measured }
}

/// [`Expect::FirmwareTracks`].
fn judge_tracks(
    samples: &[Sample],
    observed: &[Observation],
    (var, scale): (&str, f64),
    signal: &str,
    max_rms: f64,
    (from, by_s): (f64, Option<f64>),
) -> CheckResult {
    let what = format!("{var} x {scale} follows {signal} within {max_rms} RMS from {from:.1} s {}", by_text(by_s));
    let pairs: Vec<(f64, f64)> = window(samples, from, by_s)
        .filter_map(|(_, i)| {
            let seen = observed.get(i)?;
            let estimate = seen.values.iter().find(|(name, _)| name == var)?.1 * scale;
            Some((estimate, signal_value(&seen.signals, signal)?))
        })
        .collect();
    if pairs.is_empty() {
        return CheckResult { what, passed: false, measured: "not observed".to_owned() };
    }
    let n = pairs.len() as f64;
    let rms = (pairs.iter().map(|(e, t)| (e - t).powi(2)).sum::<f64>() / n).sqrt();
    let (estimate, truth) = pairs.iter().fold((0.0, 0.0), |(a, b), (e, t)| (a + e / n, b + t / n));
    CheckResult {
        what,
        passed: rms <= max_rms,
        measured: format!("RMS error {rms:.2}; mean {estimate:.2} against {truth:.2}"),
    }
}

/// A scenario in progress, one sample at a time (for viewers that step it themselves).
#[derive(Debug, Clone)]
pub struct Ride {
    samples: Vec<Sample>,
    /// What the firmware showed at each sample.
    observed: Vec<Observation>,
    /// The stock-map variables the checks read.
    vars: Vec<String>,
    last_target: f64,
    was_ramping: bool,
    last_push: f64,
    /// Fault windows already announced: (started, ended) per timeline entry.
    announced: Vec<(bool, bool)>,
    distance_m: f64,
}

impl Ride {
    /// Starts `scenario` on `session` (built for it, at power-on): notes it on the timeline and
    /// hands its faults to the plant and the devices.
    pub fn new(scenario: &Scenario, session: &mut Session) -> Result<Self, ScenarioError> {
        session.note(format!("scenario {}", scenario.name));
        let hclk = session.machine().hclk() as f64;
        for timed in &scenario.faults {
            let from = (timed.at_s * hclk) as u64;
            let until = timed.for_s.map(|length| ((timed.at_s + length) * hclk) as u64);
            inject(session, from, until, &timed.fault).map_err(ScenarioError::Invalid)?;
        }
        let mut vars: Vec<String> = Vec::new();
        for var in scenario.expects.iter().filter_map(Expect::firmware_var) {
            if session.firmware_var(var).is_none() {
                return Err(ScenarioError::Invalid(format!(
                    "a check reads {var}, which the stock map of this build does not name"
                )));
            }
            if !vars.iter().any(|v| v == var) {
                vars.push(var.to_owned());
            }
        }
        Ok(Self {
            samples: Vec::new(),
            observed: Vec::new(),
            vars,
            last_target: f64::NAN,
            was_ramping: false,
            last_push: 0.0,
            announced: vec![(false, false); scenario.faults.len()],
            distance_m: 0.0,
        })
    }

    /// The scenario has run its time.
    pub fn finished(&self, scenario: &Scenario, session: &Session) -> bool {
        session.time_s() + 0.5 * session.sample_period_s() >= scenario.seconds
    }

    /// Runs one sample period: stand, target speed, pushes, then the machine.
    pub fn sample(&mut self, scenario: &Scenario, session: &mut Session) -> Snapshot {
        let t = session.time_s();
        if t >= scenario.release_s && session.machine().plant().live_part().is_some_and(|l| l.physics().held()) {
            session.set_held(false);
        }
        let target = scenario.target_kmh(t);
        // A note per change of the target: at each ramp's start and end, not every sample on it.
        let ramping = (target - self.last_target).abs() > 1e-9;
        if ramping != self.was_ramping || self.last_target.is_nan() {
            session.note(format!("target {target:.1} km/h"));
        }
        self.was_ramping = ramping;
        self.last_target = target;
        session.set_intent(Intent::Speed(target / 3.6));
        let push = scenario.push_n(t);
        if (push - self.last_push).abs() > f64::EPSILON {
            session.note(if push == 0.0 { "push ends".to_owned() } else { format!("push {push:.0} N") });
            session.set_push(push);
            self.last_push = push;
        }
        for (timed, (started, ended)) in scenario.faults.iter().zip(&mut self.announced) {
            if !*started && t >= timed.at_s {
                *started = true;
                session.log(EventKind::Fault, format!("fault {} starts", timed.fault.kind()));
            }
            if !*ended && timed.for_s.is_some_and(|length| t >= timed.at_s + length) {
                *ended = true;
                session.log(EventKind::Fault, format!("fault {} ends", timed.fault.kind()));
            }
        }
        let mut snapshot = session.step();
        snapshot.signals.target_speed_kmh = target as f32;
        let state = session.machine().plant().live_part().map(|l| l.physics().state());
        self.samples.push(Sample {
            t_s: snapshot.t_s,
            speed_kmh: f64::from(snapshot.signals.speed_kmh),
            pitch_deg: f64::from(snapshot.signals.pitch_deg),
            fallen: state.is_some_and(|s| s.fallen),
            cutoff: session.cutoff().is_some(),
        });
        self.observed.push(Observation {
            cutoff_flags: session.cutoff_flags(),
            vars: self.vars.iter().map(|v| (v.clone(), session.firmware_var(v).unwrap_or(0))).collect(),
            values: self.vars.iter().map(|v| (v.clone(), session.firmware_value(v).unwrap_or(f64::NAN))).collect(),
            signals: snapshot.signals,
        });
        self.distance_m = f64::from(snapshot.signals.distance_m);
        snapshot
    }

    /// The samples so far.
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    /// Judges the run so far (the whole run once [`Ride::finished`]).
    pub fn outcome(&self, scenario: &Scenario) -> Outcome {
        let samples = &self.samples;
        let released = samples.iter().filter(|s| s.t_s >= scenario.release_s);
        Outcome {
            checks: scenario
                .expects
                .iter()
                .map(|e| e.judge_observed(samples, &self.observed, scenario.release_s))
                .collect(),
            top_speed_kmh: samples.iter().map(|s| s.speed_kmh).fold(0.0, f64::max),
            max_abs_pitch_deg: released.map(|s| s.pitch_deg.abs()).fold(0.0, f64::max),
            distance_m: self.distance_m,
        }
    }
}

/// Hands `fault` to its target for the cycles `from..until`.
fn inject(session: &mut Session, from: u64, until: Option<u64>, fault: &Fault) -> Result<(), String> {
    let machine = session.machine_mut();
    match fault {
        Fault::Plant(fault) => {
            let live = machine.plant_mut().live_part_mut().ok_or("plant faults need a live plant")?;
            live.electrical_mut().add_fault(from, until, *fault).map_err(|e| e.to_string())
        }
        Fault::Device { device, fault } => {
            let kind = fault.device_kind();
            let names = if kind == "mpu6500" {
                machine.component_names::<Mpu6500>()
            } else {
                machine.component_names::<SmartBms>()
            };
            let name = match device {
                Some(name) if names.contains(name) => name.clone(),
                Some(name) => {
                    return Err(format!("{}: no {kind} named {name} (have: {})", fault.kind(), names.join(", ")));
                }
                None if names.len() == 1 => names[0].clone(),
                None => {
                    return Err(format!(
                        "{}: say which {kind} with device = (have: {})",
                        fault.kind(),
                        names.join(", ")
                    ));
                }
            };
            if kind == "mpu6500" {
                machine.component_mut::<Mpu6500>(&name).ok_or("device vanished")?.add_fault(from, until, *fault)
            } else {
                machine.component_mut::<SmartBms>(&name).ok_or("device vanished")?.add_fault(from, until, *fault)
            }
        }
    }
}

/// Runs `scenario` on `session` (built for it, at power-on) to the end, calling `on_sample`
/// with every snapshot, and judges the run.
pub fn run(
    scenario: &Scenario,
    session: &mut Session,
    mut on_sample: impl FnMut(&Snapshot, &mut Session),
) -> Result<Outcome, ScenarioError> {
    let mut ride = Ride::new(scenario, session)?;
    while !ride.finished(scenario, session) {
        let snapshot = ride.sample(scenario, session);
        on_sample(&snapshot, session);
    }
    Ok(ride.outcome(scenario))
}

#[cfg(test)]
mod tests {
    use super::*;

    const RIDE: &str = r#"
schema = 1
name = "t"
description = "test"
wheel = "begode-extreme"
seconds = 30.0
release_s = 5.0
speed_kmh = [[0, 0], [10, 0], [20, 20]]
[[expect]]
check = "pitch"
max_abs_deg = 10.0
[[expect]]
check = "speed_reached"
kmh = 19.0
"#;

    #[test]
    fn parses_and_interpolates_the_speed_profile() {
        let s = Scenario::from_toml(RIDE).unwrap();
        assert_eq!(
            (s.target_kmh(0.0), s.target_kmh(10.0), s.target_kmh(15.0), s.target_kmh(25.0)),
            (0.0, 0.0, 10.0, 20.0)
        );
        assert_eq!(s.expects[0], Expect::Pitch { max_abs_deg: 10.0, from_s: None });
        assert_eq!(s.rider, Rider::default());
    }

    #[test]
    fn refuses_bad_files() {
        let bad = RIDE.replace("release_s = 5.0", "release_s = 50.0");
        assert!(Scenario::from_toml(&bad).unwrap_err().to_string().contains("release_s"));
        let typo = RIDE.replace("kmh = 19.0", "kmph = 19.0");
        assert!(Scenario::from_toml(&typo).is_err());
    }

    #[test]
    fn timeline_faults_parse_by_kind() {
        let text = format!(
            "{RIDE}\n[[timeline]]\nat_s = 12\nfor_s = 3.0\nfault = \"hall_stuck\"\nline = \"a\"\nlevel = \"low\"\n\
             [[timeline]]\nat_s = 20.0\nfault = \"imu_stuck\"\ndevice = \"imu\"\n"
        );
        let s = Scenario::from_toml(&text).unwrap();
        assert_eq!(s.faults.len(), 2);
        assert_eq!((s.faults[0].at_s, s.faults[0].for_s, s.faults[0].fault.kind()), (12.0, Some(3.0), "hall_stuck"));
        assert_eq!(s.faults[1].fault, Fault::Device { device: Some("imu".into()), fault: DeviceFault::ImuStuck });
        let error =
            |entry: &str| Scenario::from_toml(&format!("{RIDE}\n[[timeline]]\n{entry}\n")).unwrap_err().to_string();
        assert!(error("at_s = 1.0\nfault = \"gremlins\"").contains("unknown fault \"gremlins\" (known: hall_stuck"));
        assert!(error("fault = \"imu_stuck\"").contains("needs at_s"));
        assert!(
            error("at_s = 1.0\nfault = \"hall_stuck\"\nline = \"a\"").contains("hall_stuck: missing field `level`")
        );
        assert!(
            error("at_s = 1.0\nfault = \"hall_stuck\"\nline = \"a\"\nlevel = \"low\"\ndevice = \"x\"")
                .contains("device does not apply")
        );
    }

    #[test]
    fn judges_fault_reactions() {
        let at = |t_s, pitch_deg, cutoff| Sample { t_s, speed_kmh: 0.0, pitch_deg, fallen: false, cutoff };
        let samples = [at(1.0, 0.0, false), at(2.0, 9.0, false), at(2.1, 12.0, true)];
        let seen = |flags: &[&str], alerts| Observation {
            cutoff_flags: flags.iter().map(|&f| f.to_owned()).collect(),
            vars: vec![("alerts".to_owned(), alerts)],
            ..Observation::default()
        };
        let observed = [seen(&[], 0), seen(&[], 0x08), seen(&["hall_fault"], 0x48)];
        let cut = Expect::Cutoff { from_s: 2.0, within_s: 0.2, flag: Some("hall_fault".into()) };
        assert_eq!(cut.judge_observed(&samples, &observed, 0.0).measured, "cut at 2.10 s (hall_fault)");
        assert!(cut.judge_observed(&samples, &observed, 0.0).passed);
        assert!(!cut.judge(&samples, 0.0).passed, "the flag needs the firmware observed");
        assert!(!Expect::Cutoff { from_s: 1.0, within_s: 0.5, flag: None }.judge(&samples, 0.0).passed, "too late");
        let tilt = Expect::PitchReaches { deg: 8.0, from_s: None, by_s: Some(2.0) }.judge(&samples, 0.0);
        assert_eq!((tilt.passed, tilt.measured.as_str()), (true, "9.00° at 2.00 s"));
        let dive = Expect::PitchReaches { deg: -5.0, from_s: None, by_s: None }.judge(&samples, 0.0);
        assert_eq!((dive.passed, dive.measured.as_str()), (false, "only 0.00°"));
        let alarm = Expect::FirmwareBits { var: "alerts".into(), bits: 0x08, from_s: None, by_s: None };
        assert_eq!(alarm.judge_observed(&samples, &observed, 0.0).measured, "0x8 at 2.00 s");
        assert!(!alarm.judge(&samples, 0.0).passed, "not observed");
        let parsed = Scenario::from_toml(&format!(
            "{RIDE}\n[[expect]]\ncheck = \"firmware_bits\"\nvar = \"alerts\"\nbits = 0x40\nby_s = 20.0\n\
             [[expect]]\ncheck = \"cutoff\"\nfrom_s = 15.0\nwithin_s = 0.2\nflag = \"hall_fault\"\n"
        ))
        .unwrap();
        assert_eq!(
            parsed.expects[2],
            Expect::FirmwareBits { var: "alerts".into(), bits: 0x40, from_s: None, by_s: Some(20.0) }
        );
    }

    #[test]
    fn judges_ripple_and_tracking() {
        let at = |t_s| Sample { t_s, speed_kmh: 20.0, pitch_deg: 0.0, fallen: false, cutoff: false };
        let samples = [at(1.0), at(2.0), at(3.0), at(4.0)];
        let seen = |torque: f32, speed_cms: f64| Observation {
            values: vec![("speed_cms".to_owned(), speed_cms)],
            signals: Signals { torque_nm: torque, speed_kmh: 20.0, ..Signals::default() },
            ..Observation::default()
        };
        let observed = [seen(10.0, 556.0), seen(14.0, 556.0), seen(6.0, 400.0), seen(10.0, 400.0)];
        let ripple = |from_s| Expect::Ripple { signal: "torque_nm".into(), max: 1.0, from_s: Some(from_s), by_s: None };
        let steady = ripple(4.0).judge_observed(&samples, &observed, 0.0);
        assert_eq!((steady.passed, steady.measured.as_str()), (true, "standard deviation 0.00, mean 10.00"));
        assert!(!ripple(1.0).judge_observed(&samples, &observed, 0.0).passed, "10, 14, 6, 10: 2.83");
        let tracks = Expect::FirmwareTracks {
            var: "speed_cms".into(),
            scale: 0.036,
            signal: "speed_kmh".into(),
            max_rms: 1.0,
            from_s: Some(3.0),
            by_s: None,
        };
        let lagging = tracks.judge_observed(&samples, &observed, 0.0);
        assert_eq!((lagging.passed, lagging.measured.as_str()), (false, "RMS error 5.60; mean 14.40 against 20.00"));
        let typo = format!("{RIDE}\n[[expect]]\ncheck = \"ripple\"\nsignal = \"torque\"\nmax = 1.0\n");
        assert!(Scenario::from_toml(&typo).unwrap_err().to_string().contains("unknown telemetry signal \"torque\""));
    }

    #[test]
    fn records_always_pass_and_say_what_they_saw() {
        let at = |t_s| Sample { t_s, speed_kmh: 0.0, pitch_deg: 0.0, fallen: false, cutoff: false };
        let samples = [at(1.0), at(2.0), at(3.0)];
        let seen = |speed: f32, vq: f64| Observation {
            values: vec![("vq_active".to_owned(), vq)],
            signals: Signals { speed_kmh: speed, ..Signals::default() },
            ..Observation::default()
        };
        let observed = [seen(10.0, 6300.0), seen(30.0, 31500.0), seen(20.0, 9450.0)];
        let record = |signal: Option<&str>, var: Option<&str>, stat, threshold| Expect::Record {
            signal: signal.map(str::to_owned),
            var: var.map(str::to_owned),
            scale: var.map(|_| 1.0 / 315.0),
            stat,
            threshold,
            from_s: None,
            by_s: None,
        };
        let measured = |e: Expect| {
            let result = e.judge_observed(&samples, &observed, 0.0);
            assert!(result.passed);
            result.measured
        };
        assert_eq!(measured(record(Some("speed_kmh"), None, Stat::Max, None)), "max 30.00 at 2.00 s");
        assert_eq!(measured(record(Some("speed_kmh"), None, Stat::Mean, None)), "mean 20.00");
        let pwm = record(None, Some("vq_active"), Stat::FirstAbove, Some(80.0));
        assert_eq!(measured(pwm), "first above 80 at 2.00 s (100.00)");
        assert_eq!(measured(record(None, Some("vq_active"), Stat::FirstBelow, Some(5.0))), "never below 5");
        let both =
            format!("{RIDE}\n[[expect]]\ncheck = \"record\"\nsignal = \"speed_kmh\"\nvar = \"x\"\nstat = \"max\"\n");
        assert!(Scenario::from_toml(&both).unwrap_err().to_string().contains("either signal or var"));
    }

    #[test]
    fn judges_samples() {
        let at = |t_s, speed_kmh, pitch_deg| Sample { t_s, speed_kmh, pitch_deg, fallen: false, cutoff: false };
        let samples = [at(1.0, 0.0, 30.0), at(6.0, 10.0, -4.0), at(7.0, 19.5, 2.0), at(8.0, 0.4, 1.0)];
        let pitch = Expect::Pitch { max_abs_deg: 5.0, from_s: None }.judge(&samples, 5.0);
        assert!(pitch.passed, "the stand phase does not count: {pitch:?}");
        assert_eq!(pitch.measured, "max -4.00° at 6.00 s");
        assert!(Expect::SpeedReached { kmh: 19.0 }.judge(&samples, 5.0).passed);
        assert!(Expect::StoppedAtEnd { max_abs_kmh: 0.5 }.judge(&samples, 5.0).passed);
        let mut cut = samples;
        cut[2].cutoff = true;
        let verdict = Expect::NoCutoff { from_s: None }.judge(&cut, 5.0);
        assert!(!verdict.passed && verdict.measured == "cut at 7.00 s", "{verdict:?}");
    }
}

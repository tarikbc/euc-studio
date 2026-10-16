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
use euc_physics::{Assist, Intent, Rider};
use euc_plant::fault::PlantFault;
use euc_telemetry::{EventKind, Snapshot};
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
    if !(at_s >= 0.0) || for_s.is_some_and(|f| !(f > 0.0)) {
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
    /// Judges `samples` (the whole run) against this expectation.
    pub fn judge(&self, samples: &[Sample], release_s: f64) -> CheckResult {
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
        }
    }
}

/// A scenario in progress, one sample at a time (for viewers that step it themselves).
#[derive(Debug, Clone)]
pub struct Ride {
    samples: Vec<Sample>,
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
        Ok(Self {
            samples: Vec::new(),
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
        let snapshot = session.step();
        let state = session.machine().plant().live_part().map(|l| l.physics().state());
        self.samples.push(Sample {
            t_s: snapshot.t_s,
            speed_kmh: f64::from(snapshot.signals.speed_kmh),
            pitch_deg: f64::from(snapshot.signals.pitch_deg),
            fallen: state.is_some_and(|s| s.fallen),
            cutoff: session.cutoff().is_some(),
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
            checks: scenario.expects.iter().map(|e| e.judge(samples, scenario.release_s)).collect(),
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

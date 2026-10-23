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
use euc_proto_begode::{Assembler, Segment, Telemetry};
use euc_telemetry::{CHANNELS, EventKind, Signals, Snapshot};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::session::Session;

/// Physical observation after a phase sequence completes or fails (no fabricated stop).
const PHASE_OBSERVE_S: f64 = 2.0;

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
    /// Start of measured-speed lean phases; defaults to release time.
    #[serde(default)]
    pub phase_start_s: Option<f64>,
    /// Optional bounded lean commands, advanced by measured speed crossings (the ground speed,
    /// or the speed the wheel shows: [`GateSpeed`]).
    #[serde(default, rename = "phase")]
    pub phases: Vec<Phase>,
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
    /// A streaming world instead of `[terrain]` (`docs/models/world.md`).
    #[serde(default)]
    pub world: Option<WorldScenario>,
    /// Why the scenario rides the plain flat floor on purpose (`[world] flat = "<reason>"`; then
    /// [`Scenario::world`] is `None`).
    #[serde(skip)]
    pub flat: Option<String>,
    /// When the rider steps off the pedals, seconds (the wheel goes on alone).
    #[serde(default)]
    pub step_off_s: Option<f64>,
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

/// `[world]`: ride a generated world instead of a terrain strip.
///
/// ```toml
/// [world]
/// seed = 12345
/// revision = 2                 # the generator's; a scenario written for another one is refused
/// biome = "city"               # default
/// start = { x_m = 0.0, y_m = 0.0, heading_deg = 0.0 }   # default: the world's spawn
/// slope_pct = -2.0             # optional: the whole world graded along the start heading
/// slope_start_m = 25.0         # where the grade starts, metres ahead of the start
/// slope_ramp_m = 10.0          # over which it builds up (default 10)
///
/// [world.densities]            # optional, per euc_world::Densities (hills = 0: a level city)
/// potholes = 3.0
/// ```
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldScenario {
    /// Seed.
    pub seed: u64,
    /// Generator revision the scenario was written for.
    pub revision: u32,
    /// Biome.
    #[serde(default = "city")]
    pub biome: euc_world::Biome,
    /// Feature densities.
    #[serde(default)]
    pub densities: euc_world::Densities,
    /// Where the wheel starts; the world's spawn when absent.
    #[serde(default)]
    pub start: Option<WorldStartToml>,
    /// Waypoints to steer through after release, world coordinates `[x, y]`, metres. The rider
    /// heads for the next one and moves on within 1 m of it; after the last, it holds its heading.
    #[serde(default)]
    pub route: Vec<[f64; 2]>,
    /// A grade over the whole world along the start heading, percent (negative: downhill), for a
    /// slope test in the city ([`euc_world::Incline`]). The ground behind its start is unchanged.
    #[serde(default)]
    pub slope_pct: f64,
    /// Distance ahead of the start where the grade begins, metres.
    #[serde(default)]
    pub slope_start_m: f64,
    /// Distance over which the grade builds up, metres (as `[terrain]`'s `slope_ramp_m`).
    #[serde(default = "slope_ramp")]
    pub slope_ramp_m: f64,
}

const fn slope_ramp() -> f64 {
    10.0
}

/// A start pose in world coordinates.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldStartToml {
    /// Ground point under the wheel, metres.
    pub x_m: f64,
    /// Ground point under the wheel, metres.
    pub y_m: f64,
    /// Heading, degrees, left positive from world +x.
    pub heading_deg: f64,
}

/// The unit vector of a heading, exact at whole quarter turns: the generator then sees the same
/// bits on every platform (sine and cosine come from the platform's libm).
fn unit(heading_deg: f64) -> [f64; 2] {
    let quarter = heading_deg / 90.0;
    #[allow(clippy::float_cmp)]
    if quarter == quarter.round() {
        return match (quarter.round() as i64).rem_euclid(4) {
            0 => [1.0, 0.0],
            1 => [0.0, 1.0],
            2 => [-1.0, 0.0],
            _ => [0.0, -1.0],
        };
    }
    let rad = heading_deg.to_radians();
    [rad.cos(), rad.sin()]
}

const fn city() -> euc_world::Biome {
    euc_world::Biome::City
}

/// `[world] flat = "<reason>"`: the scenario rides the plain flat floor on purpose, and says why.
/// Returns the scenario's table without its `[world]`, and the reason; `None` for any other
/// scenario.
fn flat_floor(text: &str) -> Result<Option<(toml::Table, String)>, ScenarioError> {
    let mut table: toml::Table = toml::from_str(text)?;
    let Some(toml::Value::Table(world)) = table.get("world") else { return Ok(None) };
    let Some(flat) = world.get("flat") else { return Ok(None) };
    let invalid = |why: &str| ScenarioError::Invalid(why.into());
    let reason = flat
        .as_str()
        .map(str::trim)
        .filter(|reason| !reason.is_empty())
        .ok_or_else(|| invalid("[world] flat takes the reason for the flat floor, as text"))?
        .to_owned();
    if world.len() > 1 {
        return Err(invalid("[world] flat rides the plain flat floor: it takes no other [world] key"));
    }
    table.remove("world");
    Ok(Some((table, reason)))
}

impl WorldScenario {
    /// The world's specification.
    pub fn spec(&self) -> euc_world::WorldSpec {
        // The spawn is the world's origin, heading +x (euc_world::World::spawn).
        let (origin, heading_deg) = self.start.map_or(([0.0, 0.0], 0.0), |s| ([s.x_m, s.y_m], s.heading_deg));
        let incline = (self.slope_pct != 0.0).then(|| euc_world::Incline {
            origin,
            direction: unit(heading_deg),
            slope_pct: self.slope_pct,
            start_m: self.slope_start_m,
            ramp_m: self.slope_ramp_m,
        });
        euc_world::WorldSpec {
            seed: self.seed,
            revision: self.revision,
            biome: self.biome,
            densities: self.densities,
            incline,
        }
    }

    /// The ground for physics.
    pub fn ground(&self) -> Result<euc_physics::WorldGround, String> {
        match self.start {
            None => euc_physics::WorldGround::at_spawn(self.spec()),
            Some(s) => euc_physics::WorldGround::new(
                self.spec(),
                euc_physics::WorldStart { xy_m: [s.x_m, s.y_m], heading_rad: s.heading_deg.to_radians() },
            ),
        }
    }
}

/// A rider lean command with a measured-speed gate, never a velocity or motor torque override.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Phase {
    /// Unique name used in events and outcomes.
    pub name: String,
    /// Desired lean from vertical, forward positive, subject to the rider's existing limits.
    pub lean_deg: f64,
    /// Signed forward speed at which to advance, km/h: the ground speed, or what the wheel shows
    /// (see [`speed`](Self::speed)).
    pub until_speed_kmh: f64,
    /// Direction of the measured threshold crossing.
    pub crossing: Crossing,
    /// Which speed the gate reads (default: the ground speed).
    #[serde(default)]
    pub speed: GateSpeed,
    /// Maximum simulated time in this phase, seconds.
    pub timeout_s: f64,
}

/// Which speed a phase's gate reads.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GateSpeed {
    /// The true ground speed.
    #[default]
    Ground,
    /// The speed the wheel shows: its telemetry frame 00, what the app and a rider see (about
    /// 3 Hz). It differs from the ground speed by the firmware's own scale (the hall count and the
    /// wheel's pole pairs), so a gate a rider judges by the display, next to a firmware speed
    /// threshold, keeps its meaning through any plant calibration.
    Shown,
}

/// Direction of a signed speed crossing.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Crossing {
    /// Previous speed below the threshold, new speed at or above it.
    Above,
    /// Previous speed above the threshold, new speed at or below it.
    Below,
}

impl Phase {
    fn crossed(&self, previous: f64, current: f64) -> bool {
        match self.crossing {
            Crossing::Above => previous < self.until_speed_kmh && current >= self.until_speed_kmh,
            Crossing::Below => previous > self.until_speed_kmh && current <= self.until_speed_kmh,
        }
    }
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
    /// The rider stays on the pedals: never comes off them while the wheel stays up. Welded feet
    /// and the rigid rider cannot come off that way, so it bites only with contact feet
    /// (`articulated_contact_v1`). Every scenario without a `step_off_s` gets it by default.
    RiderOnPedals {
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
    /// The value when the firmware first cuts the motor (any cutoff flag).
    AtCutoff,
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
        let mut scenario: Self = match flat_floor(text)? {
            Some((table, reason)) => Self { flat: Some(reason), ..toml::Value::Table(table).try_into()? },
            None => toml::from_str(text)?,
        };
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
        if scenario.phase_start_s.is_some() && scenario.phases.is_empty() {
            return bad("phase_start_s needs at least one phase".into());
        }
        if !scenario.phases.is_empty() {
            let start = scenario.phase_start_s.unwrap_or(scenario.release_s);
            if !scenario.seconds.is_finite()
                || !start.is_finite()
                || !(scenario.release_s..scenario.seconds).contains(&start)
                || scenario.phases.len() > 16
            {
                return bad("phases need finite times, a start at or after release, and at most 16 entries".into());
            }
            scenario.rider.control.validate().map_err(|e| ScenarioError::Invalid(e.to_string()))?;
            for (i, phase) in scenario.phases.iter().enumerate() {
                if phase.name.trim().is_empty()
                    || phase.name.len() > 64
                    || scenario.phases[..i].iter().any(|p| p.name == phase.name)
                    || !phase.lean_deg.is_finite()
                    || phase.lean_deg.abs() > scenario.rider.control.lean_max_deg
                    || !phase.until_speed_kmh.is_finite()
                    || !phase.timeout_s.is_finite()
                    || phase.timeout_s <= 0.0
                {
                    return bad(format!(
                        "phase {} needs a unique name, finite speed and lean within rider limits, and a positive finite timeout",
                        i + 1
                    ));
                }
            }
            if start + scenario.phases.iter().map(|p| p.timeout_s).sum::<f64>() + PHASE_OBSERVE_S > scenario.seconds {
                return bad(
                    "seconds must include phase_start_s, every phase timeout, and 2 s of final observation".into()
                );
            }
        }
        if scenario.step_off_s.is_some_and(|at| !(scenario.release_s..scenario.seconds).contains(&at)) {
            return bad("step_off_s must fall between release_s and the end of the run".into());
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
        // A rider who comes off the pedals while the wheel stays up must not pass unseen: every
        // scenario without a planned step-off checks it.
        if scenario.step_off_s.is_none() && !scenario.expects.iter().any(|e| matches!(e, Expect::RiderOnPedals { .. }))
        {
            scenario.expects.push(Expect::RiderOnPedals { from_s: None });
        }
        for (i, table) in std::mem::take(&mut scenario.timeline).into_iter().enumerate() {
            let fault =
                timed_fault(table).map_err(|why| ScenarioError::Invalid(format!("timeline entry {}: {why}", i + 1)))?;
            scenario.faults.push(fault);
        }
        if scenario.flat.is_some() && !scenario.terrain.is_flat() {
            return bad("[world] flat rides the plain flat floor: no [terrain] with it".into());
        }
        if let Some(world) = &scenario.world {
            if !scenario.terrain.is_flat() {
                return bad("a scenario has either [terrain] or [world], not both".into());
            }
            if world.route.len() > 256 || world.route.iter().flatten().any(|v| !v.is_finite()) {
                return bad("[world] route takes at most 256 finite [x, y] waypoints".into());
            }
            world.ground().map_err(|why| ScenarioError::Invalid(format!("[world]: {why}")))?;
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
    /// The rider has come off the pedals while the wheel stayed up (contact feet only: both soles
    /// off their pedals for `euc_physics::LOST_FOOTING_S`).
    pub off_pedals: bool,
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
    /// Measured-speed phase results; absent for legacy speed schedules.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub phases: Vec<PhaseResult>,
}

/// Verdict of a measured-speed gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PhaseStatus {
    /// An earlier phase failed, or the run ended before this phase began.
    NotStarted,
    /// The phase is still active (internal progress only).
    Running,
    /// The gate's speed (ground or shown) crossed the threshold while the ride was healthy.
    Reached,
    /// The gate did not complete before a timeout, interruption, or run end.
    TargetUnreachable,
}

/// Sampled measurements for one lean phase. Missing measurements serialize as null.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PhaseResult {
    /// Name from the scenario.
    pub name: String,
    /// Gate verdict, independent of later upright/cutoff checks.
    pub status: PhaseStatus,
    /// Failure reason, if any: timeout, cutoff, fall, halted, `nonfinite_speed`, or `duration_ended`.
    pub reason: Option<String>,
    /// First command time, seconds from power-on.
    pub start_s: Option<f64>,
    /// Gate/failure observation time, seconds from power-on.
    pub end_s: Option<f64>,
    /// Time since the first phase command, seconds.
    pub elapsed_s: Option<f64>,
    /// Signed forward distance change during this phase, metres (not total path length).
    pub distance_m: Option<f64>,
    /// Measured speed when the command begins, km/h.
    pub start_speed_kmh: Option<f64>,
    /// Measured speed when the phase ends, km/h.
    pub end_speed_kmh: Option<f64>,
    /// Highest measured forward speed during this phase, km/h.
    pub max_speed_kmh: Option<f64>,
    /// Largest absolute sampled current among all three motor phases, A.
    pub max_abs_phase_current_a: Option<f64>,
    /// Lowest sampled q current, A.
    pub iq_min_a: Option<f64>,
    /// Highest sampled q current, A.
    pub iq_max_a: Option<f64>,
    /// Lowest sampled battery current, A (negative is regeneration).
    pub battery_current_min_a: Option<f64>,
    /// Highest sampled battery current, A.
    pub battery_current_max_a: Option<f64>,
    /// Lowest sampled bus voltage, V.
    pub battery_voltage_min_v: Option<f64>,
    /// Highest sampled bus voltage, V.
    pub battery_voltage_max_v: Option<f64>,
    #[serde(skip)]
    start_distance_m: Option<f64>,
}

impl PhaseResult {
    fn pending(phase: &Phase) -> Self {
        Self {
            name: phase.name.clone(),
            status: PhaseStatus::NotStarted,
            reason: None,
            start_s: None,
            end_s: None,
            elapsed_s: None,
            distance_m: None,
            start_speed_kmh: None,
            end_speed_kmh: None,
            max_speed_kmh: None,
            max_abs_phase_current_a: None,
            iq_min_a: None,
            iq_max_a: None,
            battery_current_min_a: None,
            battery_current_max_a: None,
            battery_voltage_min_v: None,
            battery_voltage_max_v: None,
            start_distance_m: None,
        }
    }

    fn begin(&mut self, t: f64, speed: f64, distance: f64) {
        self.status = PhaseStatus::Running;
        self.start_s = Some(t);
        self.start_speed_kmh = speed.is_finite().then_some(speed);
        self.start_distance_m = distance.is_finite().then_some(distance);
        extremum(&mut self.max_speed_kmh, speed, f64::max);
    }

    fn measure(&mut self, s: &Sample, signals: &Signals) {
        self.end_s = Some(s.t_s);
        self.elapsed_s = self.start_s.map(|start| s.t_s - start);
        self.end_speed_kmh = s.speed_kmh.is_finite().then_some(s.speed_kmh);
        self.distance_m = self.start_distance_m.and_then(|start| {
            let distance = f64::from(signals.distance_m) - start;
            distance.is_finite().then_some(distance)
        });
        self.measure_extrema(s, signals);
    }

    /// Includes an entry/boundary sample without advancing elapsed time, distance, or gates.
    fn measure_extrema(&mut self, s: &Sample, signals: &Signals) {
        extremum(&mut self.max_speed_kmh, s.speed_kmh, f64::max);
        for current in [signals.phase_a_a, signals.phase_b_a, signals.phase_c_a] {
            extremum(&mut self.max_abs_phase_current_a, f64::from(current).abs(), f64::max);
        }
        extremum(&mut self.iq_min_a, f64::from(signals.iq_a), f64::min);
        extremum(&mut self.iq_max_a, f64::from(signals.iq_a), f64::max);
        extremum(&mut self.battery_current_min_a, f64::from(signals.battery_a), f64::min);
        extremum(&mut self.battery_current_max_a, f64::from(signals.battery_a), f64::max);
        extremum(&mut self.battery_voltage_min_v, f64::from(signals.battery_v), f64::min);
        extremum(&mut self.battery_voltage_max_v, f64::from(signals.battery_v), f64::max);
    }
}

fn extremum(slot: &mut Option<f64>, value: f64, combine: fn(f64, f64) -> f64) {
    if value.is_finite() {
        *slot = Some(slot.map_or(value, |old| combine(old, value)));
    }
}

/// Owns phase progression independently of the CPU and plant. The only output is rider intent.
#[derive(Debug, Clone)]
struct PhaseRun {
    results: Vec<PhaseResult>,
    index: usize,
    previous_speed: f64,
    /// The shown speed at the last observation (NaN before the first frame 00).
    previous_shown: f64,
    /// The wheel has shown a speed at least once.
    shown_seen: bool,
    terminal_s: Option<f64>,
}

impl PhaseRun {
    fn new(phases: &[Phase]) -> Self {
        Self {
            results: phases.iter().map(PhaseResult::pending).collect(),
            index: 0,
            previous_speed: 0.0,
            previous_shown: f64::NAN,
            shown_seen: false,
            terminal_s: None,
        }
    }

    fn command(&mut self, scenario: &Scenario, t: f64, speed: f64, distance: f64) -> Option<Intent> {
        if self.results.is_empty() {
            return None;
        }
        if self.terminal_s.is_some() {
            return Some(if self.index == self.results.len() { Intent::Speed(0.0) } else { Intent::Lean(0.0) });
        }
        if t < scenario.phase_start_s.unwrap_or(scenario.release_s) {
            return None;
        }
        let result = &mut self.results[self.index];
        if result.start_s.is_none() {
            result.begin(t, speed, distance);
            self.previous_speed = speed;
        }
        Some(Intent::Lean(scenario.phases[self.index].lean_deg.to_radians()))
    }

    /// One sample without a shown speed (the tests of ground-speed gates).
    #[cfg(test)]
    fn observe(&mut self, scenario: &Scenario, sample: Sample, signals: &Signals, halted: bool) {
        self.observe_shown(scenario, sample, f64::NAN, signals, halted);
    }

    /// One sample: the ground speed in `sample`, the shown speed in `shown_kmh` (NaN before the
    /// wheel's first frame 00).
    fn observe_shown(&mut self, scenario: &Scenario, sample: Sample, shown_kmh: f64, signals: &Signals, halted: bool) {
        let previous_shown = std::mem::replace(&mut self.previous_shown, shown_kmh);
        self.shown_seen |= shown_kmh.is_finite();
        if self.results.is_empty() || self.terminal_s.is_some() {
            return;
        }
        let result = &mut self.results[self.index];
        // Boot cutoff flags may be normal. Any cut/fall after release, or a halt at any time,
        // makes the first target unreachable without inventing a launch or a braking phase.
        if result.start_s.is_none() {
            let failure = if halted {
                Some("halted")
            } else if sample.t_s >= scenario.release_s && sample.fallen {
                Some("fall")
            } else if sample.t_s >= scenario.release_s && sample.cutoff {
                Some("cutoff")
            } else {
                None
            };
            if let Some(reason) = failure {
                result.status = PhaseStatus::TargetUnreachable;
                result.reason = Some(reason.into());
                result.end_s = Some(sample.t_s);
                self.terminal_s = Some(sample.t_s);
            }
            return;
        }
        result.measure(&sample, signals);
        let phase = &scenario.phases[self.index];
        let elapsed = result.elapsed_s.unwrap_or(0.0);
        let failure = if halted {
            Some("halted")
        } else if sample.fallen {
            Some("fall")
        } else if sample.cutoff {
            Some("cutoff")
        } else if !sample.speed_kmh.is_finite() || !self.previous_speed.is_finite() {
            Some("nonfinite_speed")
        } else if elapsed > phase.timeout_s + 1e-9 {
            Some("timeout")
        } else {
            None
        };
        let reached = failure.is_none()
            && match phase.speed {
                GateSpeed::Ground => phase.crossed(self.previous_speed, sample.speed_kmh),
                // Before the wheel's first frame 00 nothing is shown: no crossing, and no failure.
                GateSpeed::Shown => {
                    previous_shown.is_finite() && shown_kmh.is_finite() && phase.crossed(previous_shown, shown_kmh)
                }
            };
        let failure = failure.or_else(|| (!reached && elapsed + 1e-9 >= phase.timeout_s).then_some("timeout"));
        // A shown-speed gate that never saw the wheel show a speed says so, not just "timeout".
        let failure = failure.map(|reason| {
            if reason == "timeout" && phase.speed == GateSpeed::Shown && !self.shown_seen {
                "no_shown_speed"
            } else {
                reason
            }
        });
        if let Some(reason) = failure {
            result.status = PhaseStatus::TargetUnreachable;
            result.reason = Some(reason.into());
            self.terminal_s = Some(sample.t_s);
        } else if reached {
            result.status = PhaseStatus::Reached;
            self.index += 1;
            if self.index == self.results.len() {
                self.terminal_s = Some(sample.t_s);
            } else {
                let next = &mut self.results[self.index];
                next.begin(sample.t_s, sample.speed_kmh, f64::from(signals.distance_m));
                next.measure_extrema(&sample, signals);
            }
        }
        self.previous_speed = sample.speed_kmh;
    }

    fn finished(&self, t: f64) -> bool {
        self.terminal_s.is_some_and(|end| t + 1e-9 >= end + PHASE_OBSERVE_S)
    }

    fn outcomes(&self, t: f64) -> Vec<PhaseResult> {
        let mut results = self.results.clone();
        if self.terminal_s.is_none()
            && let Some(result) = results.get_mut(self.index)
        {
            result.status = PhaseStatus::TargetUnreachable;
            result.reason = Some("duration_ended".into());
            result.end_s = Some(t);
            result.elapsed_s = result.start_s.map(|start| t - start);
        }
        results
    }
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
            Self::RiderOnPedals { from_s } => {
                let off = after(from_s).find(|s| s.off_pedals);
                CheckResult {
                    what: format!("rider stays on the pedals {}", from_text(from_s)),
                    passed: off.is_none(),
                    measured: off.map_or_else(
                        || "stayed on".to_owned(),
                        |s| format!("both feet off the pedals at {:.2} s, the wheel still up", s.t_s),
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
            Stat::AtCutoff => window(samples, from, by_s).find(|(s, _)| s.cutoff).map_or_else(
                || "the firmware did not cut".to_owned(),
                |(s, i)| match observed.get(i).and_then(|o| source.value(o)) {
                    Some(v) => format!("{v:.2} when the firmware cut at {:.2} s", s.t_s),
                    None => format!("not observed when the firmware cut at {:.2} s", s.t_s),
                },
            ),
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
    phases: PhaseRun,
    /// The next `[world]` route waypoint.
    waypoint: usize,
    /// What the wheel shows: its telemetry frames, decoded from its BLE bytes as they come.
    shown: Shown,
}

/// The speed the wheel shows, from its telemetry frame 00.
#[derive(Debug, Clone)]
struct Shown {
    /// BLE bytes read so far.
    read: usize,
    frames: Assembler,
    /// The last frame 00's speed, km/h; NaN before the first.
    kmh: f64,
}

impl Default for Shown {
    fn default() -> Self {
        Self { read: 0, frames: Assembler::new(), kmh: f64::NAN }
    }
}

impl Shown {
    /// Reads the bytes the firmware sent since the last call; the last frame 00 sets the speed.
    fn update(&mut self, session: &Session) -> f64 {
        let tx = &session.machine().board().ble_tx;
        // A host that takes the bytes away (an app runner) leaves fewer than were read.
        if tx.len() < self.read {
            self.read = 0;
        }
        let bytes: Vec<u8> = tx[self.read..].iter().map(|&(_, byte)| byte).collect();
        self.read = tx.len();
        self.frames.push(&bytes);
        while let Some(segment) = self.frames.next_segment() {
            if let Segment::Frame(frame) = segment
                && let Telemetry::Live(live) = Telemetry::decode(&frame)
            {
                self.kmh = live.speed_kmh();
            }
        }
        self.kmh
    }
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
            phases: PhaseRun::new(&scenario.phases),
            waypoint: 0,
            shown: Shown::default(),
        })
    }

    /// Steers toward the next `[world]` route waypoint, moving on within 1 m of it.
    fn steer(&mut self, scenario: &Scenario, session: &mut Session) {
        let Some(world) = &scenario.world else { return };
        if self.waypoint >= world.route.len() {
            return;
        }
        let Some(physics) = session.machine().plant().live_part().map(crate::plant::Live::physics) else { return };
        let Some(ground) = physics.world_ground() else { return };
        let p = physics.position_m();
        let route: Vec<[f64; 2]> = world.route[self.waypoint..].iter().map(|w| ground.to_physics(*w)).collect();
        let mut reached = 0;
        while route.get(reached).is_some_and(|t| (t[0] - p[0]).hypot(t[1] - p[1]) < 1.0) {
            reached += 1;
        }
        for _ in 0..reached {
            self.waypoint += 1;
            session.note(format!("waypoint {} reached", self.waypoint));
        }
        if let Some(target) = route.get(reached) {
            session.set_heading_target((target[1] - p[1]).atan2(target[0] - p[0]));
        }
    }

    /// The declared duration ended, or the machine halted before completion.
    pub fn finished(&self, scenario: &Scenario, session: &Session) -> bool {
        session.machine().halted().is_some()
            || self.phases.finished(session.time_s())
            || session.time_s() + 0.5 * session.sample_period_s() >= scenario.seconds
    }

    /// Runs one sample period: stand, target speed, pushes, then the machine.
    pub fn sample(&mut self, scenario: &Scenario, session: &mut Session) -> Snapshot {
        let t = session.time_s();
        if t >= scenario.release_s && session.machine().plant().live_part().is_some_and(|l| l.physics().held()) {
            session.set_held(false);
        }
        let target = scenario.target_kmh(t);
        let previous_speed = self.samples.last().map_or(0.0, |s| s.speed_kmh);
        let previous_distance = self.observed.last().map_or(0.0, |o| f64::from(o.signals.distance_m));
        let was_started = self.phases.results.get(self.phases.index).is_some_and(|r| r.start_s.is_some());
        let phase_intent = self.phases.command(scenario, t, previous_speed, previous_distance);
        if !was_started
            && let Some(result) = self.phases.results.get(self.phases.index)
            && result.start_s.is_some()
        {
            session.note(format!(
                "phase {} starts: lean {:.1} deg",
                result.name, scenario.phases[self.phases.index].lean_deg
            ));
        }
        // A note per change of the target: at each ramp's start and end, not every sample on it.
        let ramping = (target - self.last_target).abs() > 1e-9;
        if phase_intent.is_none() && (ramping != self.was_ramping || self.last_target.is_nan()) {
            session.note(format!("target {target:.1} km/h"));
        }
        self.was_ramping = ramping;
        self.last_target = target;
        let intent = phase_intent.unwrap_or(Intent::Speed(target / 3.6));
        session.set_intent(intent);
        if t >= scenario.release_s {
            self.steer(scenario, session);
        }
        if scenario.step_off_s.is_some_and(|at| t >= at) {
            session.step_off();
        }
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
        snapshot.signals.target_speed_kmh = match intent {
            Intent::Speed(v) => (v * 3.6) as f32,
            Intent::Lean(_) | Intent::Accel(_) => f32::NAN,
        };
        let physics = session.machine().plant().live_part().map(crate::plant::Live::physics);
        let state = physics.map(euc_physics::Physics::state);
        let sample = Sample {
            t_s: snapshot.t_s,
            speed_kmh: f64::from(snapshot.signals.speed_kmh),
            pitch_deg: f64::from(snapshot.signals.pitch_deg),
            fallen: state.is_some_and(|s| s.fallen),
            cutoff: session.cutoff().is_some(),
            off_pedals: physics
                .and_then(euc_physics::Physics::left_pedals)
                .is_some_and(|(_, how)| how == euc_physics::Leaving::LostFooting),
        };
        self.samples.push(sample);
        let shown = self.shown.update(session);
        let previous_index = self.phases.index;
        let was_terminal = self.phases.terminal_s.is_some();
        self.phases.observe_shown(scenario, sample, shown, &snapshot.signals, session.machine().halted().is_some());
        if (self.phases.index != previous_index || (!was_terminal && self.phases.terminal_s.is_some()))
            && let Some(result) = self.phases.results.get(previous_index)
        {
            session.note(format!(
                "phase {} {:?}{} at {:.3} s",
                result.name,
                result.status,
                result.reason.as_ref().map_or(String::new(), |reason| format!(" ({reason})")),
                snapshot.t_s
            ));
            if let Some(next) = self.phases.results.get(self.phases.index)
                && next.start_s.is_some()
                && self.phases.terminal_s.is_none()
            {
                session.note(format!(
                    "phase {} starts: lean {:.1} deg",
                    next.name, scenario.phases[self.phases.index].lean_deg
                ));
            }
        }
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
        let phases = self.phases.outcomes(samples.last().map_or(0.0, |s| s.t_s));
        let mut checks: Vec<_> =
            scenario.expects.iter().map(|e| e.judge_observed(samples, &self.observed, scenario.release_s)).collect();
        checks.extend(phases.iter().map(|p| CheckResult {
            what: format!("phase {} reaches measured speed gate", p.name),
            passed: p.status == PhaseStatus::Reached,
            measured: format!(
                "{:?}{}; elapsed {} s; distance {} m",
                p.status,
                p.reason.as_ref().map_or(String::new(), |r| format!(" ({r})")),
                p.elapsed_s.map_or_else(|| "unavailable".into(), |s| format!("{s:.3}")),
                p.distance_m.map_or_else(|| "unavailable".into(), |d| format!("{d:.3}"))
            ),
        }));
        Outcome {
            checks,
            top_speed_kmh: samples.iter().map(|s| s.speed_kmh).fold(0.0, f64::max),
            max_abs_pitch_deg: released.map(|s| s.pitch_deg.abs()).fold(0.0, f64::max),
            distance_m: self.distance_m,
            phases,
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
    fn world_scenarios_parse_and_refuse_terrain_bad_routes_and_another_generator() {
        let with = |table: &str| Scenario::from_toml(&format!("{RIDE}\n{table}"));
        let spawn = with("[world]\nseed = 42\nrevision = 2\n").unwrap();
        let world = spawn.world.as_ref().unwrap();
        assert_eq!(world.spec(), euc_world::WorldSpec::city(42));
        assert_eq!(world.ground().unwrap().start().xy_m, [0.0, 0.0], "the world's spawn");
        let placed = with(
            "[world]\nseed = 42\nrevision = 2\nstart = { x_m = 70.0, y_m = 3.05, heading_deg = 90.0 }\n\
             route = [[70.0, 20.0]]\n",
        )
        .unwrap();
        let ground = placed.world.as_ref().unwrap().ground().unwrap();
        assert_eq!(ground.start().xy_m, [70.0, 3.05]);
        // Heading 90°: the physics frame's +x runs along world +y.
        let ahead = ground.to_world([1.0, 0.0]);
        assert!((ahead[0] - 70.0).abs() < 1e-12 && (ahead[1] - 4.05).abs() < 1e-12, "{ahead:?}");
        let long = format!("[world]\nseed = 1\nrevision = 2\nroute = [{}]\n", ["[1.0, 2.0]"; 257].join(", "));
        for (what, table) in [
            ("terrain as well", "[world]\nseed = 1\nrevision = 2\n[terrain]\nslope_pct = 2.0\n"),
            ("another generator", "[world]\nseed = 1\nrevision = 1\n"),
            ("unknown field", "[world]\nseed = 1\nrevision = 2\nsize = 3\n"),
            ("route not finite", "[world]\nseed = 1\nrevision = 2\nroute = [[0.0, nan]]\n"),
            ("route too long", &long),
            ("density out of range", "[world]\nseed = 1\nrevision = 2\n[world.densities]\npotholes = 99.0\n"),
        ] {
            assert!(with(table).is_err(), "{what} must fail");
        }
        // The plain flat floor on purpose: no world, and the reason kept.
        let flat = with("[world]\nflat = \"a model limit the city exposes\"\n").unwrap();
        assert!(flat.world.is_none() && flat.terrain.is_flat());
        assert_eq!(flat.flat.as_deref(), Some("a model limit the city exposes"));
        assert!(spawn.flat.is_none() && Scenario::from_toml(RIDE).unwrap().flat.is_none());
        for (what, table) in [
            ("no reason", "[world]\nflat = \" \"\n"),
            ("not text", "[world]\nflat = true\n"),
            ("a world as well", "[world]\nflat = \"why\"\nseed = 1\nrevision = 2\n"),
            ("terrain as well", "[world]\nflat = \"why\"\n[terrain]\nslope_pct = 2.0\n"),
        ] {
            assert!(with(table).is_err(), "flat with {what} must fail");
        }
    }

    #[test]
    fn measured_speed_phases_parse_without_changing_static_profiles() {
        let text = format!("{RIDE}\nphase_start_s = 6.0\n");
        // Top-level keys must precede array tables.
        let text =
            text.replace("\nphase_start_s = 6.0\n", "").replacen("[[expect]]", "phase_start_s = 6.0\n[[expect]]", 1);
        let text = format!(
            "{text}\n[[phase]]\nname = \"accelerate\"\nlean_deg = 15.0\nuntil_speed_kmh = 70.0\ncrossing = \"above\"\ntimeout_s = 10.0\n"
        );
        assert!(Scenario::from_toml(&text).is_ok());
        assert!(Scenario::from_toml(RIDE).is_ok());
    }

    fn phase_case() -> Scenario {
        Scenario::from_toml(&format!("{RIDE}\n[[phase]]\nname = \"accelerate\"\nlean_deg = 15.0\nuntil_speed_kmh = 70.0\ncrossing = \"above\"\ntimeout_s = 10.0\n[[phase]]\nname = \"brake\"\nlean_deg = -15.0\nuntil_speed_kmh = 1.0\ncrossing = \"below\"\ntimeout_s = 10.0\n")).unwrap()
    }

    fn phase_signals(speed: f32, distance: f32) -> Signals {
        Signals {
            speed_kmh: speed,
            distance_m: distance,
            phase_a_a: -120.0,
            phase_b_a: 90.0,
            phase_c_a: 30.0,
            iq_a: -100.0,
            battery_a: -20.0,
            battery_v: 130.0,
            ..Signals::default()
        }
    }

    fn phase_sample(t_s: f64, speed_kmh: f64) -> Sample {
        Sample { t_s, speed_kmh, pitch_deg: 0.0, fallen: false, cutoff: false, off_pedals: false }
    }

    #[test]
    fn a_phase_gate_reads_the_speed_the_wheel_shows_when_asked() {
        let gate = |key: &str| {
            Scenario::from_toml(&format!(
                "{RIDE}\n[[phase]]\nname = \"accelerate\"\nlean_deg = 15.0\nuntil_speed_kmh = 70.0\n\
                 crossing = \"above\"\ntimeout_s = 10.0\n{key}"
            ))
        };
        assert_eq!(gate("").unwrap().phases[0].speed, GateSpeed::Ground, "the ground speed by default");
        assert_eq!(gate("speed = \"ground\"\n").unwrap().phases[0].speed, GateSpeed::Ground);
        assert!(gate("speed = \"displayed\"\n").is_err(), "only ground or shown");
        let s = gate("speed = \"shown\"\n").unwrap();
        assert_eq!(s.phases[0].speed, GateSpeed::Shown);
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 0.0, 0.0);
        // Before the wheel's first frame 00 nothing is shown: no crossing and no failure.
        run.observe_shown(&s, phase_sample(6.0, 60.0), f64::NAN, &phase_signals(60.0, 5.0), false);
        run.observe_shown(&s, phase_sample(6.5, 63.0), f64::NAN, &phase_signals(63.0, 6.0), false);
        assert_eq!((run.index, run.terminal_s), (0, None));
        // The wheel shows 1.107 times the ground speed: its 70 km/h comes at 63.2 km/h on the ground.
        run.observe_shown(&s, phase_sample(7.0, 63.0), 69.7, &phase_signals(63.0, 7.0), false);
        assert_eq!(run.index, 0, "shown 69.7 km/h is below the gate");
        run.observe_shown(&s, phase_sample(7.31, 63.3), 70.1, &phase_signals(63.3, 8.0), false);
        assert_eq!(run.index, 1, "shown 70.1 km/h crosses it at 63.3 km/h on the ground");
        assert_eq!(run.results[0].status, PhaseStatus::Reached);
        assert_eq!(run.results[0].end_speed_kmh, Some(63.3), "results keep the ground speed");
        // A wheel that never shows a speed: the gate times out and says why.
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 0.0, 0.0);
        run.observe_shown(&s, phase_sample(6.0, 60.0), f64::NAN, &phase_signals(60.0, 5.0), false);
        run.observe_shown(&s, phase_sample(15.1, 63.0), f64::NAN, &phase_signals(63.0, 9.0), false);
        assert_eq!(run.results[0].status, PhaseStatus::TargetUnreachable);
        assert_eq!(run.results[0].reason.as_deref(), Some("no_shown_speed"));
    }

    #[test]
    fn phases_use_signed_measured_crossings_and_measure_braking_from_gate() {
        let s = phase_case();
        let mut run = PhaseRun::new(&s.phases);
        assert!(run.command(&s, 4.9, 0.0, 0.0).is_none());
        assert_eq!(run.command(&s, 5.0, 0.0, 0.0), Some(Intent::Lean(15_f64.to_radians())));
        run.observe(&s, phase_sample(6.0, -75.0), &phase_signals(-75.0, 5.0), false);
        assert_eq!(run.index, 0);
        run.observe(&s, phase_sample(7.0, 69.0), &phase_signals(69.0, 15.0), false);
        run.observe(&s, phase_sample(7.01, 70.1), &phase_signals(70.1, 15.2), false);
        assert_eq!(run.index, 1);
        assert_eq!(run.command(&s, 7.01, 70.1, 15.2), Some(Intent::Lean(-15_f64.to_radians())));
        run.observe(&s, phase_sample(10.01, 0.9), &phase_signals(0.9, 40.2), false);
        let results = run.outcomes(10.01);
        assert_eq!(results[1].status, PhaseStatus::Reached);
        assert!((results[1].elapsed_s.unwrap() - 3.0).abs() < 1e-9);
        assert!((results[1].distance_m.unwrap() - 25.0).abs() < 1e-5);
        assert_eq!(results[1].max_abs_phase_current_a, Some(120.0));
        assert_eq!(results[1].battery_current_min_a, Some(-20.0));
        assert_eq!(results[1].battery_voltage_max_v, Some(130.0));
        assert!(!run.finished(11.9));
        assert!(run.finished(12.01));
    }

    #[test]
    fn phase_transition_seeds_boundary_extrema_during_deceleration() {
        let s = phase_case();
        for high_boundary in [false, true] {
            let mut run = PhaseRun::new(&s.phases);
            run.command(&s, 5.0, 69.0, 100.0);
            let boundary = Signals {
                phase_a_a: -300.0,
                phase_b_a: 40.0,
                phase_c_a: 260.0,
                iq_a: if high_boundary { 210.0 } else { -210.0 },
                battery_a: if high_boundary { 70.0 } else { -70.0 },
                battery_v: if high_boundary { 144.0 } else { 110.0 },
                ..phase_signals(70.25, 150.0)
            };
            run.observe(&s, phase_sample(6.0, 70.25), &boundary, false);
            let brake = &run.results[1];
            assert_eq!(brake.status, PhaseStatus::Running);
            assert_eq!(brake.start_s, Some(6.0));
            assert_eq!(brake.start_speed_kmh, Some(70.25));
            assert_eq!(brake.max_speed_kmh, brake.start_speed_kmh);
            assert_eq!(brake.max_abs_phase_current_a, Some(300.0));
            assert_eq!(brake.iq_min_a, Some(f64::from(boundary.iq_a)));
            assert_eq!(brake.iq_max_a, Some(f64::from(boundary.iq_a)));
            assert_eq!(brake.battery_current_min_a, Some(f64::from(boundary.battery_a)));
            assert_eq!(brake.battery_current_max_a, Some(f64::from(boundary.battery_a)));
            assert_eq!(brake.battery_voltage_min_v, Some(f64::from(boundary.battery_v)));
            assert_eq!(brake.battery_voltage_max_v, Some(f64::from(boundary.battery_v)));
            // Entry is not a second observation of a completed time/distance interval.
            assert_eq!(brake.end_s, None);
            assert_eq!(brake.end_speed_kmh, None);
            assert_eq!(brake.elapsed_s, None);
            assert_eq!(brake.distance_m, None);
            assert_eq!(run.terminal_s, None);
            assert_eq!(run.command(&s, 6.0, 70.25, 150.0), Some(Intent::Lean(-15_f64.to_radians())));

            let middle = Signals { iq_a: -50.0, battery_a: -20.0, ..phase_signals(35.0, 170.0) };
            run.observe(&s, phase_sample(7.0, 35.0), &middle, false);
            let end = Signals { iq_a: 3.0, battery_a: 1.0, battery_v: 125.0, ..phase_signals(0.5, 180.0) };
            run.observe(&s, phase_sample(8.0, 0.5), &end, false);
            let results = run.outcomes(8.0);
            let brake = &results[1];
            assert_eq!(brake.status, PhaseStatus::Reached);
            assert!(brake.max_speed_kmh.unwrap() >= brake.start_speed_kmh.unwrap());
            assert_eq!(brake.max_speed_kmh, Some(70.25));
            assert_eq!(brake.max_abs_phase_current_a, Some(300.0));
            assert_eq!(brake.iq_min_a, Some(if high_boundary { -50.0 } else { -210.0 }));
            assert_eq!(brake.iq_max_a, Some(if high_boundary { 210.0 } else { 3.0 }));
            assert_eq!(brake.battery_current_min_a, Some(if high_boundary { -20.0 } else { -70.0 }));
            assert_eq!(brake.battery_current_max_a, Some(if high_boundary { 70.0 } else { 1.0 }));
            assert_eq!(brake.battery_voltage_min_v, Some(if high_boundary { 125.0 } else { 110.0 }));
            assert_eq!(brake.battery_voltage_max_v, Some(if high_boundary { 144.0 } else { 130.0 }));
            assert_eq!(brake.elapsed_s, Some(2.0));
            assert_eq!(brake.distance_m, Some(30.0));
            assert_eq!(results[0].distance_m, Some(50.0));
            assert_eq!(run.command(&s, 8.0, 0.5, 180.0), Some(Intent::Speed(0.0)));
            assert!(!run.finished(9.99));
            assert!(run.finished(10.0));
        }
    }

    #[test]
    fn phase_transition_seed_does_not_reuse_the_crossing_for_the_next_gate() {
        let mut s = phase_case();
        s.phases[1].crossing = Crossing::Above;
        s.phases[1].until_speed_kmh = 69.5;
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 69.0, 0.0);
        run.observe(&s, phase_sample(6.0, 70.25), &phase_signals(70.25, 10.0), false);
        assert_eq!(run.index, 1);
        assert_eq!(run.results[1].status, PhaseStatus::Running);
        assert_eq!(run.results[1].max_speed_kmh, Some(70.25));
        assert_eq!(run.terminal_s, None);
        run.observe(&s, phase_sample(7.0, 71.0), &phase_signals(71.0, 20.0), false);
        assert_eq!(run.index, 1, "remaining above the next threshold is not another crossing");
        run.observe(&s, phase_sample(8.0, 68.0), &phase_signals(68.0, 30.0), false);
        run.observe(&s, phase_sample(9.0, 69.5), &phase_signals(69.5, 40.0), false);
        assert_eq!(run.results[1].status, PhaseStatus::Reached);
        assert_eq!(run.results[1].elapsed_s, Some(3.0));
        assert_eq!(run.results[1].distance_m, Some(30.0));
    }

    #[test]
    fn phase_initial_entry_speed_is_included_without_inventing_sensor_values() {
        let mut s = phase_case();
        s.phases.truncate(1);
        s.phases[0].crossing = Crossing::Below;
        s.phases[0].until_speed_kmh = 1.0;
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 70.0, 100.0);
        assert_eq!(run.results[0].max_speed_kmh, Some(70.0));
        assert_eq!(run.results[0].max_abs_phase_current_a, None);
        assert_eq!(run.results[0].battery_voltage_min_v, None);
        run.observe(&s, phase_sample(6.0, 35.0), &phase_signals(35.0, 110.0), false);
        run.observe(&s, phase_sample(7.0, 0.5), &phase_signals(0.5, 120.0), false);
        assert_eq!(run.results[0].status, PhaseStatus::Reached);
        assert_eq!(run.results[0].max_speed_kmh, run.results[0].start_speed_kmh);
        assert_eq!(run.results[0].elapsed_s, Some(2.0));
        assert_eq!(run.results[0].distance_m, Some(20.0));
    }

    #[test]
    fn phase_failures_prevent_a_fake_braking_phase() {
        let s = phase_case();
        for reason in ["cutoff", "fall", "halted", "timeout", "nonfinite_speed"] {
            let mut run = PhaseRun::new(&s.phases);
            run.command(&s, 5.0, 0.0, 0.0);
            let mut sample = phase_sample(if reason == "timeout" { 15.01 } else { 6.0 }, 70.1);
            sample.cutoff = reason == "cutoff";
            sample.fallen = reason == "fall";
            if reason == "nonfinite_speed" {
                sample.speed_kmh = f64::NAN;
            }
            run.observe(&s, sample, &phase_signals(70.1, 10.0), reason == "halted");
            let results = run.outcomes(sample.t_s);
            assert_eq!(results[0].status, PhaseStatus::TargetUnreachable, "{reason}");
            assert_eq!(results[0].reason.as_deref(), Some(reason));
            assert_eq!(results[1].status, PhaseStatus::NotStarted);
            assert_eq!(results[1].elapsed_s, None);
            assert_eq!(results[1].distance_m, None);
            assert_eq!(run.command(&s, sample.t_s, 70.1, 10.0), Some(Intent::Lean(0.0)));
        }
    }

    #[test]
    fn phases_reject_cutoff_or_fall_after_release_before_launch() {
        let mut s = phase_case();
        s.phase_start_s = Some(6.0);
        for cutoff in [false, true] {
            let mut run = PhaseRun::new(&s.phases);
            let mut sample = phase_sample(5.0, 0.0);
            sample.cutoff = cutoff;
            sample.fallen = !cutoff;
            run.observe(&s, sample, &phase_signals(0.0, 0.0), false);
            let results = run.outcomes(5.0);
            assert_eq!(results[0].status, PhaseStatus::TargetUnreachable);
            assert_eq!(results[0].reason.as_deref(), Some(if cutoff { "cutoff" } else { "fall" }));
            assert_eq!(results[0].start_s, None);
            assert_eq!(results[1].status, PhaseStatus::NotStarted);
            assert_eq!(run.command(&s, 5.0, 0.0, 0.0), Some(Intent::Lean(0.0)));
        }
    }

    #[test]
    fn phase_deadline_without_crossing_and_early_end_are_unreachable() {
        let s = phase_case();
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 0.0, 0.0);
        run.observe(&s, phase_sample(15.0, 69.0), &phase_signals(69.0, 20.0), false);
        assert_eq!(run.outcomes(15.0)[0].reason.as_deref(), Some("timeout"));
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 0.0, 0.0);
        assert_eq!(run.outcomes(6.0)[0].reason.as_deref(), Some("duration_ended"));
    }

    #[test]
    fn phase_validation_rejects_unbounded_or_impossible_configuration() {
        let base = format!(
            "{RIDE}\n[[phase]]\nname = \"accelerate\"\nlean_deg = 15.0\nuntil_speed_kmh = 70.0\ncrossing = \"above\"\ntimeout_s = 10.0\n"
        );
        for (from, to) in [
            ("lean_deg = 15.0", "lean_deg = 16.0"),
            ("timeout_s = 10.0", "timeout_s = inf"),
            ("timeout_s = 10.0", "timeout_s = 0.0"),
            ("until_speed_kmh = 70.0", "until_speed_kmh = nan"),
            ("seconds = 30.0", "seconds = 15.0"),
            ("name = \"accelerate\"", "name = \"\""),
        ] {
            assert!(Scenario::from_toml(&base.replace(from, to)).is_err(), "{to}");
        }
    }

    #[test]
    fn published_full_effort_cases_keep_generic_mass_and_initial_soc() {
        for (text, initial_soc) in [
            (include_str!("../../../scenarios/full-effort-70kmh-stop-soc100.toml"), 1.0),
            (include_str!("../../../scenarios/full-effort-70kmh-stop-soc50.toml"), 0.5),
            (include_str!("../../../scenarios/full-effort-70kmh-stop-soc20.toml"), 0.2),
        ] {
            let s = Scenario::from_toml(text).unwrap();
            assert_eq!(s.rider.mass_kg, 80.0);
            assert_eq!(s.phases.len(), 5);
            for (i, (phase, (name, lean, speed))) in s
                .phases
                .iter()
                .zip([
                    ("launch", 30.0, 35.0),
                    ("build", 20.0, 50.0),
                    ("accelerate", 15.0, 70.0),
                    ("brake", -30.0, 15.0),
                    ("settle", -5.0, 1.0),
                ])
                .enumerate()
            {
                assert_eq!(phase.name, name);
                assert!((phase.lean_deg - lean).abs() < f64::EPSILON);
                assert!((phase.until_speed_kmh - speed).abs() < f64::EPSILON);
                assert_eq!(matches!(phase.crossing, Crossing::Above), i < 3);
            }
            assert!((s.rider.control.lean_rate_deg_s - 24.0).abs() < f64::EPSILON);
            assert!((s.rider.control.ankle_max_nm - 400.0).abs() < f64::EPSILON);
            assert!(
                matches!(s.faults[0].fault, Fault::Plant(PlantFault::BatterySoc { soc }) if (soc - initial_soc).abs() < f64::EPSILON)
            );
            assert_eq!(s.faults[0].at_s, 0.0);
        }
    }

    #[test]
    fn phase_crossing_at_deadline_passes_but_remaining_above_does_not() {
        let s = phase_case();
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 69.0, 0.0);
        run.observe(&s, phase_sample(15.0, 70.0), &phase_signals(70.0, 100.0), false);
        assert_eq!(run.results[0].status, PhaseStatus::Reached);
        assert_eq!(run.index, 1);
        // A phase starting beyond its threshold still needs a crossing, not an immediate pass.
        let mut run = PhaseRun::new(&s.phases);
        run.command(&s, 5.0, 71.0, 0.0);
        run.observe(&s, phase_sample(6.0, 72.0), &phase_signals(72.0, 10.0), false);
        assert_eq!(run.index, 0);
        assert_eq!(run.results[0].status, PhaseStatus::Running);
    }

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
    fn every_scenario_without_a_step_off_checks_that_the_rider_stays_on_the_pedals() {
        let implicit = |s: &Scenario| s.expects.iter().filter(|e| matches!(e, Expect::RiderOnPedals { .. })).count();
        assert_eq!(implicit(&Scenario::from_toml(RIDE).unwrap()), 1, "added by default");
        let explicit = format!("{RIDE}\n[[expect]]\ncheck = \"rider_on_pedals\"\nfrom_s = 8.0\n");
        let explicit = Scenario::from_toml(&explicit).unwrap();
        assert_eq!(implicit(&explicit), 1, "not added twice");
        assert!(explicit.expects.contains(&Expect::RiderOnPedals { from_s: Some(8.0) }));
        let stepping = RIDE.replace("release_s = 5.0", "release_s = 5.0\nstep_off_s = 25.0");
        assert_eq!(implicit(&Scenario::from_toml(&stepping).unwrap()), 0, "a planned step-off is not a fall");

        let at =
            |t_s, off_pedals| Sample { t_s, speed_kmh: 10.0, pitch_deg: 0.0, fallen: false, cutoff: false, off_pedals };
        let check = Expect::RiderOnPedals { from_s: None };
        let on = check.judge(&[at(6.0, false), at(7.0, false)], 5.0);
        assert!(on.passed && on.measured == "stayed on", "{on:?}");
        let off = check.judge(&[at(6.0, false), at(17.75, true)], 5.0);
        assert!(!off.passed && off.measured.contains("17.75"), "{off:?}");
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
        let at = |t_s, pitch_deg, cutoff| Sample {
            t_s,
            speed_kmh: 0.0,
            pitch_deg,
            fallen: false,
            cutoff,
            off_pedals: false,
        };
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
        let at = |t_s| Sample { t_s, speed_kmh: 20.0, pitch_deg: 0.0, fallen: false, cutoff: false, off_pedals: false };
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
        let at = |t_s| Sample { t_s, speed_kmh: 0.0, pitch_deg: 0.0, fallen: false, cutoff: false, off_pedals: false };
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
        assert_eq!(measured(record(Some("speed_kmh"), None, Stat::AtCutoff, None)), "the firmware did not cut");
        let pwm = record(None, Some("vq_active"), Stat::FirstAbove, Some(80.0));
        assert_eq!(measured(pwm), "first above 80 at 2.00 s (100.00)");
        assert_eq!(measured(record(None, Some("vq_active"), Stat::FirstBelow, Some(5.0))), "never below 5");
        let both =
            format!("{RIDE}\n[[expect]]\ncheck = \"record\"\nsignal = \"speed_kmh\"\nvar = \"x\"\nstat = \"max\"\n");
        assert!(Scenario::from_toml(&both).unwrap_err().to_string().contains("either signal or var"));
    }

    #[test]
    fn judges_samples() {
        let at = |t_s, speed_kmh, pitch_deg| Sample {
            t_s,
            speed_kmh,
            pitch_deg,
            fallen: false,
            cutoff: false,
            off_pedals: false,
        };
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

//! A/B comparison of two runs of the same scenario (`euc sim compare`).
//!
//! Each run is recorded as a [`Trace`]: the telemetry channels and the named firmware variables at
//! every sample, and the bytes the firmware sent over BLE. Both runs sample at the same simulated
//! instants, so samples align by index. [`compare`] reports, per channel and variable, the largest
//! and RMS difference and the first sample beyond its tolerance; the first divergence of all; and
//! where the BLE streams first differ, down to the frame field.

use std::collections::BTreeMap;

use euc_telemetry::{CHANNELS, Event, Signals};
use serde::{Deserialize, Serialize};

/// One run, as the comparison needs it.
#[derive(Debug, Clone, Default)]
pub struct Trace {
    /// Sample times, seconds.
    pub t: Vec<f64>,
    /// Telemetry at each sample.
    pub signals: Vec<Signals>,
    /// Named firmware variables (stock map or probe names) at each sample.
    pub vars: BTreeMap<String, Vec<f64>>,
    /// What the firmware sent over BLE: the cycle each byte started on the wire, and the byte.
    pub ble: Vec<(u64, u8)>,
    /// Engine cycles per second (for BLE byte times).
    pub hclk: u64,
    /// The run's events.
    pub events: Vec<Event>,
}

impl Trace {
    /// An empty trace for an engine of `hclk` cycles per second.
    pub fn new(hclk: u64) -> Self {
        Self { hclk, ..Self::default() }
    }

    /// Adds one sample: its time, telemetry and the variables read now.
    pub fn push(&mut self, t: f64, signals: Signals, vars: impl IntoIterator<Item = (String, f64)>) {
        let n = self.t.len();
        self.t.push(t);
        self.signals.push(signals);
        for (name, value) in vars {
            let column = self.vars.entry(name).or_insert_with(|| vec![f64::NAN; n]);
            column.resize(n, f64::NAN);
            column.push(value);
        }
        for column in self.vars.values_mut() {
            column.resize(n + 1, f64::NAN);
        }
    }
}

/// Allowed differences (`--tolerance FILE`). Names not listed are reported but not judged, unless
/// the section has a `default`. `[scenario.<name>]` tables adjust them for one scenario.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tolerances {
    /// Largest |A − B| of telemetry channels, by key, in the channel's unit.
    #[serde(default)]
    pub channels: Section,
    /// Largest RMS of A − B of telemetry channels (for signals with fast transients).
    #[serde(default)]
    pub channels_rms: Section,
    /// Largest |A − B| of firmware variables, by name.
    #[serde(default)]
    pub vars: Section,
    /// The BLE streams must carry the same telemetry frames (text between them is not judged).
    #[serde(default)]
    pub ble_identical: bool,
    /// Every check of the scenario that both runs judge must give the same verdict.
    #[serde(default)]
    pub same_checks: bool,
    /// Compare only before this simulated second (after a fall, runs part ways for good).
    pub until_s: Option<f64>,
    /// Adjustments for one scenario, by scenario name.
    #[serde(default)]
    pub scenario: BTreeMap<String, Adjustment>,
}

/// Tolerances for one scenario: listed names replace the file's, the rest stays.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Adjustment {
    /// Why this scenario needs its own tolerances.
    pub note: Option<String>,
    /// As in [`Tolerances::channels`].
    #[serde(default)]
    pub channels: Section,
    /// As in [`Tolerances::channels_rms`].
    #[serde(default)]
    pub channels_rms: Section,
    /// As in [`Tolerances::vars`].
    #[serde(default)]
    pub vars: Section,
    /// As in [`Tolerances::ble_identical`].
    pub ble_identical: Option<bool>,
    /// As in [`Tolerances::same_checks`].
    pub same_checks: Option<bool>,
    /// As in [`Tolerances::until_s`].
    pub until_s: Option<f64>,
}

/// Tolerances of one section (every key but `default` is a name).
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Section {
    /// For every name not listed (absent: not judged).
    pub default: Option<f64>,
    /// By name.
    #[serde(flatten)]
    pub by_name: BTreeMap<String, f64>,
}

impl Section {
    fn of(&self, name: &str) -> Option<f64> {
        self.by_name.get(name).copied().or(self.default)
    }

    fn merged(&self, over: &Self) -> Self {
        let mut by_name = self.by_name.clone();
        by_name.extend(over.by_name.iter().map(|(k, v)| (k.clone(), *v)));
        Self { default: over.default.or(self.default), by_name }
    }

    fn check(&self, what: &str, channels: bool) -> Result<(), String> {
        let named = self.by_name.iter().map(|(k, v)| (k.as_str(), *v));
        let all = named.chain(self.default.map(|d| ("default", d)));
        if let Some((name, value)) = all.into_iter().find(|(_, v)| v.is_nan() || *v < 0.0) {
            return Err(format!("[{what}] {name}: tolerance {value} must not be negative"));
        }
        if channels && let Some(key) = self.by_name.keys().find(|k| !CHANNELS.iter().any(|c| c.key == k.as_str())) {
            return Err(format!("[{what}] {key}: no such telemetry channel"));
        }
        Ok(())
    }
}

impl Tolerances {
    /// Parses a tolerance file.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let tolerances: Self = toml::from_str(text).map_err(|e| e.to_string())?;
        tolerances.channels.check("channels", true)?;
        tolerances.channels_rms.check("channels_rms", true)?;
        tolerances.vars.check("vars", false)?;
        for (name, adjustment) in &tolerances.scenario {
            adjustment.channels.check(&format!("scenario.{name}.channels"), true)?;
            adjustment.channels_rms.check(&format!("scenario.{name}.channels_rms"), true)?;
            adjustment.vars.check(&format!("scenario.{name}.vars"), false)?;
        }
        Ok(tolerances)
    }

    /// The tolerances for scenario `name`: the file's, with that scenario's adjustments.
    #[must_use]
    pub fn for_scenario(&self, name: &str) -> Self {
        let Some(adj) = self.scenario.get(name) else {
            return Self { scenario: BTreeMap::new(), ..self.clone() };
        };
        Self {
            channels: self.channels.merged(&adj.channels),
            channels_rms: self.channels_rms.merged(&adj.channels_rms),
            vars: self.vars.merged(&adj.vars),
            ble_identical: adj.ble_identical.unwrap_or(self.ble_identical),
            same_checks: adj.same_checks.unwrap_or(self.same_checks),
            until_s: adj.until_s.or(self.until_s),
            scenario: BTreeMap::new(),
        }
    }
}

/// How one channel or variable compares.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SeriesDiff {
    /// Channel key or variable name.
    pub name: String,
    /// Unit (empty for variables).
    pub unit: String,
    /// Largest |a − b| (infinite when only one side is a number).
    pub max_abs: f64,
    /// Root mean square of a − b over the samples where both are numbers.
    pub rms: f64,
    /// Time of the largest difference, seconds.
    pub at_max_s: f64,
    /// Values at the largest difference.
    pub a_at_max: f64,
    /// Values at the largest difference.
    pub b_at_max: f64,
    /// The tolerance of the largest difference, if it is judged.
    pub tolerance: Option<f64>,
    /// The tolerance of the RMS, if it is judged.
    pub rms_tolerance: Option<f64>,
    /// First time the difference exceeds the tolerance (or is not zero, without a tolerance).
    pub first_s: Option<f64>,
    /// Within its tolerances (true when it has none).
    pub within: bool,
}

/// The earliest point where the runs differ.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Divergence {
    /// Simulated seconds.
    pub t_s: f64,
    /// `channel`, `var` or `ble`.
    pub kind: String,
    /// What differs: channel key, variable name, or the BLE frame field.
    pub name: String,
    /// The value in run A.
    pub a: String,
    /// The value in run B.
    pub b: String,
}

/// How the BLE streams compare: telemetry frames one by one, and the text between them.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BleDiff {
    /// Bytes sent by A.
    pub bytes_a: usize,
    /// Bytes sent by B.
    pub bytes_b: usize,
    /// Telemetry frames in A.
    pub frames_a: usize,
    /// Telemetry frames in B.
    pub frames_b: usize,
    /// Same frames, in the same order (times may differ).
    pub frames_identical: bool,
    /// The first frame that differs.
    pub first_frame: Option<FrameDiff>,
    /// Largest difference between the start times of matching frames, microseconds.
    pub max_frame_skew_us: f64,
    /// Bytes of text (boot messages, command replies) between the frames, in A.
    pub text_bytes_a: usize,
    /// Text bytes in B.
    pub text_bytes_b: usize,
    /// Same text.
    pub text_identical: bool,
}

/// The first telemetry frame that differs between the runs.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FrameDiff {
    /// Frame number in the stream (0-based).
    pub index: usize,
    /// When it started in A, seconds (absent: A sent fewer frames).
    pub t_a: Option<f64>,
    /// When it started in B, seconds.
    pub t_b: Option<f64>,
    /// The frame type and field of the first differing byte (docs/protocol/begode.md).
    pub field: String,
    /// The frame in A, hex.
    pub a: String,
    /// The frame in B, hex.
    pub b: String,
}

/// The whole comparison.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Comparison {
    /// Samples compared.
    pub samples: usize,
    /// Simulated seconds compared.
    pub duration_s: f64,
    /// Largest difference between the two runs' sample times, microseconds.
    pub sample_skew_us: f64,
    /// The end of the compared window, if the tolerance sets one, seconds.
    pub compared_until_s: Option<f64>,
    /// The earliest difference beyond tolerance (or any difference, where nothing is judged).
    pub first_divergence: Option<Divergence>,
    /// Telemetry channels, in channel id order.
    pub channels: Vec<SeriesDiff>,
    /// Firmware variables both runs name, sorted.
    pub vars: Vec<SeriesDiff>,
    /// Variables only one run names.
    pub vars_unpaired: Vec<String>,
    /// The BLE streams.
    pub ble: BleDiff,
    /// Every judged series is within its tolerance, and the frames are identical if required.
    pub within_tolerance: bool,
}

/// Relative difference below which two values count as equal when no tolerance is given: the
/// telemetry is f32, and the same physics in a different order rounds differently.
const ROUNDING: f64 = 1e-6;

/// Largest difference between the sample times of two runs, seconds. A sample ends at the first
/// quantum boundary after its period, so the two runs' samples sit a few cycles apart.
pub const MAX_SAMPLE_SKEW_S: f64 = 0.001;

/// Compares two traces of the same scenario. `Err` if their samples are not at the same times.
pub fn compare(a: &Trace, b: &Trace, tol: &Tolerances) -> Result<Comparison, String> {
    let until = tol.until_s.unwrap_or(f64::INFINITY);
    let count = a.t.iter().zip(&b.t).take_while(|(x, _)| **x < until).count();
    if let Some(i) = (0..count).find(|&i| (a.t[i] - b.t[i]).abs() > MAX_SAMPLE_SKEW_S) {
        return Err(format!("sample {i} is at {} s in A and {} s in B", a.t[i], b.t[i]));
    }
    let times = &a.t[..count];
    let channels: Vec<SeriesDiff> = CHANNELS
        .iter()
        .map(|spec| {
            let column =
                |trace: &Trace| trace.signals[..count].iter().map(|s| f64::from(s.get(spec.channel))).collect();
            let (ca, cb): (Vec<f64>, Vec<f64>) = (column(a), column(b));
            let limits = (tol.channels.of(spec.key), tol.channels_rms.of(spec.key));
            series(spec.key, spec.unit, times, &ca, &cb, limits)
        })
        .collect();
    let vars: Vec<SeriesDiff> = a
        .vars
        .iter()
        .filter_map(|(name, va)| {
            let vb = b.vars.get(name)?;
            let limits = (tol.vars.of(name), None);
            Some(series(name, "", times, &va[..count.min(va.len())], &vb[..count.min(vb.len())], limits))
        })
        .collect();
    let vars_unpaired =
        a.vars.keys().chain(b.vars.keys()).filter(|k| !(a.vars.contains_key(*k) && b.vars.contains_key(*k)));
    let mut vars_unpaired: Vec<String> = vars_unpaired.cloned().collect();
    vars_unpaired.sort();
    vars_unpaired.dedup();

    let ble = ble_diff(a, b, until);
    let mut first: Option<Divergence> = None;
    let mut consider = |candidate: Divergence| {
        if first.as_ref().is_none_or(|f| candidate.t_s < f.t_s) {
            first = Some(candidate);
        }
    };
    for (kind, list) in [("channel", &channels), ("var", &vars)] {
        for s in list {
            if let Some(t_s) = s.first_s {
                let i = times.iter().position(|&x| x >= t_s).unwrap_or(0);
                let (va, vb) = if kind == "channel" {
                    let spec = CHANNELS.iter().find(|c| c.key == s.name).map(|c| c.channel);
                    spec.map_or((f64::NAN, f64::NAN), |c| {
                        (f64::from(a.signals[i].get(c)), f64::from(b.signals[i].get(c)))
                    })
                } else {
                    (a.vars[&s.name][i], b.vars[&s.name][i])
                };
                consider(Divergence { t_s, kind: kind.into(), name: s.name.clone(), a: number(va), b: number(vb) });
            }
        }
    }
    if let Some(frame) = &ble.first_frame
        && let Some(t_s) = frame.t_a.or(frame.t_b)
    {
        let side = |hex: &str| if hex.is_empty() { "none".to_owned() } else { hex.to_owned() };
        consider(Divergence {
            t_s,
            kind: "ble".into(),
            name: frame.field.clone(),
            a: side(&frame.a),
            b: side(&frame.b),
        });
    }
    let within_tolerance =
        channels.iter().chain(&vars).all(|s| s.within) && (!tol.ble_identical || ble.frames_identical);
    Ok(Comparison {
        samples: count,
        compared_until_s: tol.until_s,
        sample_skew_us: (0..count).map(|i| (a.t[i] - b.t[i]).abs()).fold(0.0, f64::max) * 1e6,
        duration_s: times.last().copied().unwrap_or(0.0),
        first_divergence: first,
        channels,
        vars,
        vars_unpaired,
        ble,
        within_tolerance,
    })
}

/// Checks both outcomes judge (same `what`) whose verdicts differ, as `what: A pass, B fail`.
pub fn check_disagreements(a: &crate::scenario::Outcome, b: &crate::scenario::Outcome) -> Vec<String> {
    a.checks
        .iter()
        .filter_map(|ca| {
            let cb = b.checks.iter().find(|cb| cb.what == ca.what)?;
            let word = |passed: bool| if passed { "pass" } else { "fail" };
            (ca.passed != cb.passed).then(|| {
                format!("{}: A {} ({}), B {} ({})", ca.what, word(ca.passed), ca.measured, word(cb.passed), cb.measured)
            })
        })
        .collect()
}

/// A value with every digit it has (telemetry is f32, so tiny differences show).
fn number(value: f64) -> String {
    format!("{value}")
}

/// Difference of two series at the times `t`.
fn series(name: &str, unit: &str, t: &[f64], a: &[f64], b: &[f64], limits: (Option<f64>, Option<f64>)) -> SeriesDiff {
    let (tolerance, rms_tolerance) = limits;
    let (mut max_abs, mut at, mut sum, mut count, mut first_s) = (0.0_f64, 0, 0.0, 0usize, None);
    for i in 0..t.len().min(a.len()).min(b.len()) {
        let diff = match (a[i].is_nan(), b[i].is_nan()) {
            (true, true) => 0.0,
            (false, false) => (a[i] - b[i]).abs(),
            _ => f64::INFINITY,
        };
        if diff.is_finite() && !(a[i].is_nan() && b[i].is_nan()) {
            sum += diff * diff;
            count += 1;
        }
        if diff > max_abs {
            max_abs = diff;
            at = i;
        }
        // Without a tolerance, a difference counts once it is more than rounding (relative 1e-6).
        let limit = tolerance.unwrap_or_else(|| ROUNDING * a[i].abs().max(b[i].abs()).max(1.0));
        if first_s.is_none() && diff > limit {
            first_s = Some(t[i]);
        }
    }
    let rms = if count > 0 { (sum / count as f64).sqrt() } else { 0.0 };
    SeriesDiff {
        name: name.to_owned(),
        unit: unit.to_owned(),
        max_abs,
        rms,
        at_max_s: t.get(at).copied().unwrap_or(0.0),
        a_at_max: a.get(at).copied().unwrap_or(f64::NAN),
        b_at_max: b.get(at).copied().unwrap_or(f64::NAN),
        tolerance,
        rms_tolerance,
        first_s,
        within: tolerance.is_none_or(|tol| max_abs <= tol) && rms_tolerance.is_none_or(|tol| rms <= tol),
    }
}

fn ble_diff(a: &Trace, b: &Trace, until: f64) -> BleDiff {
    let (mut fa, ta) = split_stream(a);
    let (mut fb, tb) = split_stream(b);
    fa.retain(|f| f.0 < until);
    fb.retain(|f| f.0 < until);
    let first = fa
        .iter()
        .zip(&fb)
        .position(|(x, y)| x.1 != y.1)
        .or_else(|| (fa.len() != fb.len()).then_some(fa.len().min(fb.len())));
    let common = first.unwrap_or(fa.len());
    let skew = fa[..common].iter().zip(&fb[..common]).map(|(x, y)| (x.0 - y.0).abs()).fold(0.0, f64::max);
    let first_frame = first.map(|index| {
        let (frame_a, frame_b) = (fa.get(index), fb.get(index));
        let bytes = |f: Option<&(f64, Vec<u8>)>| f.map_or_else(String::new, |(_, bytes)| crate::bridge::to_hex(bytes));
        let field = match (frame_a, frame_b) {
            (Some((_, x)), Some((_, y))) => field_at(x, x.iter().zip(y).position(|(p, q)| p != q).unwrap_or(0)),
            _ => "missing frame".to_owned(),
        };
        FrameDiff {
            index,
            t_a: frame_a.map(|f| f.0),
            t_b: frame_b.map(|f| f.0),
            field,
            a: bytes(frame_a),
            b: bytes(frame_b),
        }
    });
    BleDiff {
        bytes_a: a.ble.len(),
        bytes_b: b.ble.len(),
        frames_a: fa.len(),
        frames_b: fb.len(),
        frames_identical: first.is_none(),
        first_frame,
        max_frame_skew_us: skew * 1e6,
        text_bytes_a: ta.len(),
        text_bytes_b: tb.len(),
        text_identical: ta == tb,
    }
}

/// The telemetry frames of a BLE stream (start time in seconds, 24 bytes) and the text between them.
fn split_stream(trace: &Trace) -> (Vec<(f64, Vec<u8>)>, Vec<u8>) {
    let bytes: Vec<u8> = trace.ble.iter().map(|&(_, b)| b).collect();
    let (mut frames, mut text, mut i) = (Vec::new(), Vec::new(), 0);
    while i < bytes.len() {
        if i + 24 <= bytes.len() && bytes[i] == 0x55 && bytes[i + 1] == 0xAA && bytes[i + 20..i + 24] == [0x5A; 4] {
            frames.push((trace.ble[i].0 as f64 / trace.hclk as f64, bytes[i..i + 24].to_vec()));
            i += 24;
        } else {
            text.push(bytes[i]);
            i += 1;
        }
    }
    (frames, text)
}

/// The field of byte `offset` of a telemetry frame (docs/protocol/begode.md).
fn field_at(frame: &[u8], offset: usize) -> String {
    let kind = frame[18];
    let name = match (kind, offset) {
        (_, 0 | 1) => "header".to_owned(),
        (_, 18) => "frame type".to_owned(),
        (_, 19) => "sub-index".to_owned(),
        (_, 20..=23) => "footer".to_owned(),
        (0x00, o) => [
            "battery voltage",
            "speed",
            "range",
            "trip",
            "phase current",
            "board temperature",
            "settings echo",
            "beeper volume",
        ][(o - 2) / 2]
            .to_owned(),
        (0x01, o) => [
            "PWM alarm threshold",
            "speed copy",
            "battery voltage",
            "block current",
            "highest BMS temperature",
            "lowest BMS temperature",
            "block word 42",
            "block status",
        ][(o - 2) / 2]
            .to_owned(),
        (0x02 | 0x03, o) => format!("cell {}", 8 * usize::from(frame[19]) + (o - 2) / 2 + 1),
        (0x04, 2..=5) => "odometer".to_owned(),
        (0x04, o) => match o {
            6 | 7 => "modes",
            8 | 9 => "auto power-off countdown",
            10 | 11 => "tiltback speed",
            13 => "W M value",
            14 => "alert flags",
            15 => "light state",
            _ => "constant",
        }
        .to_owned(),
        (0x07, o) => match o {
            2 | 3 => "battery current",
            4 | 5 => "settings echo",
            8 | 9 => "PWM",
            _ => "zero",
        }
        .to_owned(),
        _ => format!("byte {offset}"),
    };
    format!("frame {kind:02x} byte {offset}: {name}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trace(speeds: &[f32], ble: &[u8]) -> Trace {
        // BLE byte i starts at cycle 10 i: 1 ms apart at 10 kHz.
        let mut trace = Trace::new(10_000);
        for (i, &speed) in speeds.iter().enumerate() {
            let signals = Signals { speed_kmh: speed, ..Signals::default() };
            trace.push(i as f64 * 0.01, signals, [("pitch".to_owned(), f64::from(speed) / 10.0)]);
        }
        trace.ble = ble.iter().enumerate().map(|(i, &b)| (i as u64 * 10, b)).collect();
        trace
    }

    fn frame(kind: u8, fill: u8) -> Vec<u8> {
        let mut f = vec![fill; 24];
        f[0..2].copy_from_slice(&[0x55, 0xAA]);
        f[18] = kind;
        f[19] = 0x18;
        f[20..24].copy_from_slice(&[0x5A; 4]);
        f
    }

    #[test]
    fn identical_runs_match_everywhere() {
        let a = trace(&[0.0, 1.0, 2.0], &frame(0, 1));
        let c = compare(&a, &a.clone(), &Tolerances::default()).unwrap();
        assert!(c.first_divergence.is_none() && c.ble.frames_identical && c.within_tolerance);
        assert_eq!(c.channels[0].max_abs, 0.0);
        assert_eq!((c.vars.len(), c.ble.frames_a), (1, 1));
    }

    #[test]
    fn reports_the_first_divergence_and_judges_tolerances() {
        let a = trace(&[0.0, 1.0, 2.0, 3.0], &frame(0, 1));
        let mut ble_b = frame(0, 1);
        ble_b[11] = 9; // frame 00 bytes 10-11: phase current
        let b = trace(&[0.0, 1.0, 2.5, 3.0], &ble_b);
        let tol: Tolerances = Tolerances::from_toml("[channels]\nspeed_kmh = 0.1\n[vars]\ndefault = 1.0").unwrap();
        let c = compare(&a, &b, &tol).unwrap();
        let speed = &c.channels[0];
        assert_eq!((speed.max_abs, speed.at_max_s, speed.first_s, speed.within), (0.5, 0.02, Some(0.02), false));
        assert!(c.vars[0].within, "0.05 within 1.0");
        assert_eq!(c.ble.first_frame.as_ref().map(|f| f.field.as_str()), Some("frame 00 byte 11: phase current"));
        let first = c.first_divergence.unwrap();
        assert_eq!((first.kind.as_str(), first.t_s), ("ble", 0.0), "the frame starts at 0 ms, before 20 ms");
        assert!(!c.within_tolerance);
    }

    #[test]
    fn scenario_adjustments_and_the_window() {
        let file = "until_s = 1.0\n[channels]\nspeed_kmh = 0.1\npitch_deg = 1.0\n\
                    [scenario.fall]\nuntil_s = 0.015\n[scenario.fall.channels]\nspeed_kmh = 5.0\n";
        let tol = Tolerances::from_toml(file).unwrap();
        let fall = tol.for_scenario("fall");
        assert_eq!(
            (fall.channels.of("speed_kmh"), fall.channels.of("pitch_deg"), fall.until_s),
            (Some(5.0), Some(1.0), Some(0.015))
        );
        assert_eq!(tol.for_scenario("other").channels.of("speed_kmh"), Some(0.1));
        let a = trace(&[0.0, 1.0, 2.0, 3.0], &[]);
        let b = trace(&[0.0, 1.0, 9.0, 9.0], &[]);
        let c = compare(&a, &b, &fall).unwrap();
        assert_eq!((c.samples, c.channels[0].max_abs), (2, 0.0), "samples from 0.015 s on are not compared");
        assert!(
            Tolerances::from_toml("[scenario.x.channels]\nnope = 1.0")
                .unwrap_err()
                .contains("no such telemetry channel")
        );
    }

    #[test]
    fn nan_on_one_side_is_a_difference_and_bad_tolerances_are_refused() {
        let a = trace(&[1.0], &[]);
        let b = trace(&[f32::NAN], &[]);
        let c = compare(&a, &b, &Tolerances::default()).unwrap();
        assert!(c.channels[0].max_abs.is_infinite());
        assert!(Tolerances::from_toml("[channels]\nspeed = 1.0").unwrap_err().contains("no such telemetry channel"));
        assert!(Tolerances::from_toml("[channels_rms]\ndefault = -1.0").is_err());
        assert!(Tolerances::from_toml("[vars]\nx = -1.0").is_err());
    }
}

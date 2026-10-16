//! `euc sim compare`: two firmware images ride the same scenario on the same inputs; where do they
//! differ?

use std::fs;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::Context as _;
use euc_profile::Sha256;
use euc_sim::compare::{self, Comparison, SeriesDiff, Tolerances, Trace};
use euc_sim::scenario::{Outcome, Ride, Scenario};
use euc_sim::{Machine, MachineConfig, PlantState, Session, WheelFiles};
use serde::Serialize;

use crate::sim::{board_with, cpu_factory, injection, load_firmware, load_scenario};
use crate::ui::{self, BAD, DIM, GOOD, STRONG, WARN};
use crate::{Context, Verdict};

/// `euc sim compare`.
#[derive(Debug, clap::Args)]
pub(crate) struct CompareArgs {
    /// The scenario both runs ride.
    #[arg(long, value_name = "FILE")]
    scenario: PathBuf,
    /// Run A: `stock`, `compat` or `clean` (our builds in firmware/build/target-*/), or a .bin
    /// file (its .elf next to it names our variables, see profiles/<wheel>/probes.toml).
    #[arg(long, default_value = "stock")]
    a: String,
    /// Run B, as for --a.
    #[arg(long)]
    b: String,
    /// CPU backend for both runs: `own` (every build) or `unicorn` (development builds with the
    /// feature).
    #[arg(long, default_value = "own", value_name = "own|unicorn")]
    cpu: euc_sim::cpu::Backend,
    /// Settings preset both runs boot with (default: the scenario's).
    #[arg(long, value_name = "PRESET")]
    settings: Option<String>,
    /// Allowed differences (TOML with `[channels]`, `[vars]` and `ble_identical`); exceeding them
    /// fails.
    #[arg(long, value_name = "FILE")]
    tolerance: Option<PathBuf>,
    /// Samples per simulated second.
    #[arg(long, default_value_t = 100.0)]
    sample_hz: f64,
    /// Also write the JSON report (what `--json` prints) to this file.
    #[arg(long, value_name = "FILE")]
    out: Option<PathBuf>,
}

/// One run's identity and verdict, in the output.
#[derive(Serialize)]
struct RunOut {
    spec: String,
    label: String,
    sha256: Sha256,
    passed: bool,
    /// Checks of the scenario this run cannot judge: they read a firmware variable its map lacks.
    checks_skipped: Vec<String>,
    #[serde(flatten)]
    outcome: Outcome,
}

/// What `--json` prints.
#[derive(Serialize)]
struct CompareOutput<'a> {
    scenario: &'a str,
    settings: Option<&'a str>,
    a: RunOut,
    b: RunOut,
    tolerance_file: Option<String>,
    /// Why the tolerance file treats this scenario specially, if it does.
    tolerance_note: Option<&'a str>,
    /// Checks both runs judge, with different verdicts.
    check_disagreements: Vec<String>,
    /// The verdict: within tolerance, and the checks agree when the tolerance asks for it.
    ok: bool,
    wall_seconds: f64,
    #[serde(flatten)]
    comparison: &'a Comparison,
}

/// One side of the comparison while it runs.
struct Run {
    spec: String,
    label: String,
    sha256: Sha256,
    /// The scenario, without the checks this firmware's variable map cannot serve.
    scenario: Scenario,
    checks_skipped: Vec<String>,
    session: Session,
    ride: Ride,
    trace: Trace,
}

pub(crate) fn compare(ctx: &Context, args: &CompareArgs) -> anyhow::Result<Verdict> {
    let scenario = load_scenario(&args.scenario)?;
    let file = match &args.tolerance {
        Some(path) => {
            let text = fs::read_to_string(path).with_context(|| format!("cannot read {}", ui::path(path)))?;
            Tolerances::from_toml(&text).map_err(|why| anyhow::anyhow!("{}: {why}", ui::path(path)))?
        }
        None => Tolerances::default(),
    };
    let tolerances = file.for_scenario(&scenario.name);
    let tolerance_note = file.scenario.get(&scenario.name).and_then(|adj| adj.note.as_deref());
    let files = WheelFiles::load(&ctx.profiles(), &scenario.wheel)?;
    let settings = args.settings.as_deref().or(scenario.settings.as_deref());
    let mut a = Run::new(ctx, &files, &scenario, &args.a, settings, args.sample_hz, args.cpu)?;
    let mut b = Run::new(ctx, &files, &scenario, &args.b, settings, args.sample_hz, args.cpu)?;
    let started = Instant::now();
    while !a.ride.finished(&a.scenario, &a.session) {
        a.sample();
        b.sample();
    }
    let wall = started.elapsed().as_secs_f64();
    let comparison = compare::compare(&a.finish(), &b.finish(), &tolerances).map_err(anyhow::Error::msg)?;
    let (a, b) = (a.out(), b.out());
    let check_disagreements = compare::check_disagreements(&a.outcome, &b.outcome);
    let ok = comparison.within_tolerance && (!tolerances.same_checks || check_disagreements.is_empty());
    let output = CompareOutput {
        scenario: &scenario.name,
        settings,
        a,
        b,
        tolerance_file: args.tolerance.as_ref().map(|p| p.display().to_string()),
        tolerance_note,
        check_disagreements,
        ok,
        wall_seconds: (wall * 1000.0).round() / 1000.0,
        comparison: &comparison,
    };
    if let Some(path) = &args.out {
        fs::write(path, serde_json::to_vec_pretty(&output)?)
            .with_context(|| format!("cannot write {}", ui::path(path)))?;
    }
    if ctx.json {
        ui::json_with_profiles(&files.profile.origin, &output)?;
    } else {
        print(&output);
    }
    Ok(if ok { Verdict::Ok } else { Verdict::Refused })
}

impl Run {
    fn new(
        ctx: &Context,
        files: &WheelFiles,
        scenario: &Scenario,
        spec: &str,
        settings: Option<&str>,
        sample_hz: f64,
        cpu: euc_sim::cpu::Backend,
    ) -> anyhow::Result<Self> {
        let cpu = cpu_factory(cpu)?;
        let wheel = &files.profile;
        let fw = load_firmware(ctx, files, spec)?;
        let (image, label, sha256, stock, map) = (fw.image, fw.label, fw.sha256, fw.stock, fw.map);
        let injections = scenario
            .inject
            .iter()
            .map(|inject| injection(&format!("{}@{}", inject.bytes, inject.at_s), 0))
            .collect::<anyhow::Result<Vec<_>>>()?;
        let board = board_with(files, settings)?;
        let config = MachineConfig {
            board: &board,
            stockmap: stock.then_some(&files.stockmap),
            image: &image,
            load_addr: wheel.board.app.base,
            hclk: files.hclk(),
            plant: PlantState::default(),
            injections,
            power_cycles: Vec::new(),
            cpi: 1.25,
            max_quantum: 4000,
            marker_window_s: 4.0,
        };
        let live = files.live_on(scenario.rider, scenario.assist, scenario.terrain.clone())?;
        let machine = Machine::new_live(&config, &euc_sim::registry(), cpu, live)?;
        let mut session = Session::new(machine, map.as_ref(), sample_hz);
        // Checks that read firmware variables need names this firmware's map has; skip the others,
        // so a stock-specific check does not stop the comparison.
        let mut scenario = scenario.clone();
        let names = session.firmware_var_names();
        let mut checks_skipped = Vec::new();
        scenario.expects.retain(|check| match check.firmware_var() {
            Some(var) if !names.iter().any(|n| n == var) => {
                checks_skipped.push(format!("{check:?}"));
                false
            }
            _ => true,
        });
        let ride = Ride::new(&scenario, &mut session)?;
        Ok(Self {
            spec: spec.to_owned(),
            label,
            sha256,
            scenario,
            checks_skipped,
            session,
            ride,
            trace: Trace::new(files.hclk()),
        })
    }

    fn sample(&mut self) {
        let snapshot = self.ride.sample(&self.scenario, &mut self.session);
        let names = self.session.firmware_var_names();
        let vars = names.into_iter().filter_map(|name| Some((name.clone(), self.session.firmware_value(&name)?)));
        let vars: Vec<(String, f64)> = vars.collect();
        self.trace.push(snapshot.t_s, snapshot.signals, vars);
        self.trace.events.extend(self.session.take_events());
    }

    fn finish(&mut self) -> Trace {
        let mut trace = std::mem::take(&mut self.trace);
        trace.ble.clone_from(&self.session.machine().board().ble_tx);
        trace
    }

    fn out(&self) -> RunOut {
        let outcome = self.ride.outcome(&self.scenario);
        RunOut {
            spec: self.spec.clone(),
            label: self.label.clone(),
            sha256: self.sha256,
            passed: outcome.passed() && self.session.machine().halted().is_none(),
            checks_skipped: self.checks_skipped.clone(),
            outcome,
        }
    }
}

/// The channels and variables that differ (or are judged), one row each.
fn print_series(c: &Comparison) {
    let differing: Vec<&SeriesDiff> =
        c.channels.iter().chain(&c.vars).filter(|s| s.max_abs > 0.0 || s.tolerance.is_some()).collect();
    if differing.is_empty() {
        ui::row(
            GOOD,
            "channels",
            format!("all {} channels and {} variables identical", c.channels.len(), c.vars.len()),
        );
    } else {
        ui::detail(format!(
            "{:<20} {:>12} {:>12} {:>9} {:>10} {:>10}",
            "channel / variable", "max |A-B|", "rms", "at s", "first s", "limit"
        ));
        for s in differing {
            let line = format!(
                "{:<20} {:>12.6} {:>12.6} {:>9.2} {:>10} {:>10}",
                s.name,
                s.max_abs,
                s.rms,
                s.at_max_s,
                s.first_s.map_or_else(|| "-".to_owned(), |t| format!("{t:.2}")),
                match (s.tolerance, s.rms_tolerance) {
                    (Some(t), _) => format!("{t}"),
                    (None, Some(t)) => format!("rms {t}"),
                    (None, None) => "-".to_owned(),
                },
            );
            ui::row(if s.within { DIM } else { BAD }, if s.within { "" } else { "OVER" }, line);
        }
    }
}

fn print(output: &CompareOutput<'_>) {
    let c = output.comparison;
    ui::row(
        STRONG,
        "scenario",
        format!("{}, settings {}", ui::safe(output.scenario), output.settings.unwrap_or("blank")),
    );
    for (name, run) in [("A", &output.a), ("B", &output.b)] {
        let verdict = if run.passed { "passes" } else { "fails" };
        let skipped = match run.checks_skipped.len() {
            0 => String::new(),
            n => format!(" ({n} checks skipped: their firmware variables have no name in this build)"),
        };
        ui::row(
            if run.passed { GOOD } else { WARN },
            name,
            format!(
                "{}: {}  {}  {verdict}{skipped}, top {:.2} km/h",
                run.spec,
                ui::safe(&run.label),
                run.sha256.short(),
                run.outcome.top_speed_kmh
            ),
        );
    }
    ui::row(
        STRONG,
        "ran",
        format!(
            "both runs in {:.1} s; compared {:.1} s, {} samples each",
            output.wall_seconds, c.duration_s, c.samples
        ),
    );
    match &c.first_divergence {
        Some(d) => {
            ui::row(WARN, "first", format!("{:.4} s {} {}: A {} B {}", d.t_s, d.kind, ui::safe(&d.name), d.a, d.b));
        }
        None => ui::row(GOOD, "first", "no divergence"),
    }
    print_series(c);
    if !c.vars_unpaired.is_empty() {
        ui::row(DIM, "unpaired", c.vars_unpaired.join(", "));
    }
    let ble = &c.ble;
    let frames = match &ble.first_frame {
        None => format!("{} frames identical, start times within {:.1} µs", ble.frames_a, ble.max_frame_skew_us),
        Some(f) => format!(
            "{} / {} frames; frame {} differs at {}: {}",
            ble.frames_a,
            ble.frames_b,
            f.index,
            f.t_a.or(f.t_b).map_or_else(|| "?".to_owned(), |t| format!("{t:.3} s")),
            f.field
        ),
    };
    ui::row(if ble.frames_identical { GOOD } else { WARN }, "frames", frames);
    let text = if ble.text_identical { "identical" } else { "differs" };
    ui::row(DIM, "text", format!("{text}: {} / {} bytes between frames", ble.text_bytes_a, ble.text_bytes_b));
    for disagreement in &output.check_disagreements {
        ui::row(WARN, "check", ui::safe(disagreement));
    }
    if let Some(note) = output.tolerance_note {
        ui::row(DIM, "note", ui::safe(note));
    }
    if let Some(until) = c.compared_until_s {
        ui::row(DIM, "window", format!("compared up to {until:.2} s"));
    }
    match (output.ok, output.tolerance_file.is_some()) {
        (true, true) => ui::row(GOOD, "RESULT", "within tolerance"),
        (true, false) => ui::row(GOOD, "RESULT", "compared (no tolerance file)"),
        (false, _) if !c.within_tolerance => ui::row(BAD, "RESULT", "exceeds the tolerance"),
        (false, _) => ui::row(BAD, "RESULT", "the runs' checks disagree"),
    }
}

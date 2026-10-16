//! `euc sim run`: the wheel's controller firmware in the simulator (`euc sim serve` and
//! `euc sim bridge-test` live in `serve.rs`).

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context as _, bail};
use euc_fwsource::{FetchOptions, UreqClient};
use euc_profile::{Channel, FirmwareSource, Sha256, WheelProfile};
use euc_sim::cpu::Backend;
use euc_sim::recording::{Header, Recorder};
use euc_sim::scenario::{self, Outcome, Scenario};
use euc_sim::{CpuFactory, Machine, MachineConfig, PlantState, Report, Session, WheelFiles};
use euc_telemetry::{Event, EventKind};
use serde::Serialize;

use crate::ui::{self, BAD, DIM, GOOD, STRONG, WARN};
use crate::{Context, Verdict};

#[derive(Debug, clap::Subcommand)]
pub(crate) enum Command {
    /// Run a wheel's controller firmware in the simulator and report what it did.
    Run(Args),
    /// Run a wheel at real time and bridge its BLE UART to a client over TCP (docs/bridge.md).
    Serve(crate::serve::ServeArgs),
    /// Connect to `euc sim serve`, write a few commands, and check the stream that comes back.
    BridgeTest(crate::serve::BridgeTestArgs),
    /// Ride one scenario with two firmware images on the same inputs and report where they differ.
    Compare(crate::compare::CompareArgs),
}

#[derive(Debug, clap::Args)]
pub(crate) struct Args {
    /// Wheel profile id, e.g. begode-extreme (optional with --scenario, which names its wheel).
    wheel: Option<String>,
    /// Ride a scenario file (scenarios/*.toml) with the motor, battery, body and rider simulated,
    /// and check its expectations.
    #[arg(long, value_name = "FILE")]
    scenario: Option<PathBuf>,
    /// With --scenario: write a recording (header, telemetry batches, events) for the studio.
    #[arg(long, value_name = "FILE", requires = "scenario")]
    record: Option<PathBuf>,
    /// With --scenario: samples per simulated second in the recording and the checks.
    #[arg(long, default_value_t = 100.0, requires = "scenario")]
    sample_hz: f64,
    /// `stock` (the current official build, fetched if it is not cached) or a firmware file.
    #[arg(long, default_value = "stock")]
    firmware: String,
    /// Settings preset the wheel boots with (profiles/<wheel>/settings.toml), or `blank`.
    /// Default: the scenario's, else blank (the reference harness's state).
    #[arg(long, value_name = "PRESET")]
    settings: Option<String>,
    /// Simulated seconds to run (a scenario sets its own).
    #[arg(long, default_value_t = 6.0)]
    seconds: f64,
    /// BLE bytes to send: `V`, `V@2.6`, `hex:5759@4` (without a time: 3.0 s, 4.5 s, ...).
    #[arg(long = "inject", value_name = "BYTES[@S]")]
    injections: Vec<String>,
    /// Switch the battery off and on at this simulated second.
    #[arg(long = "power-cycle-at", value_name = "S")]
    power_cycles: Vec<f64>,
    /// Seconds counted as boot; speed after them is reported as the steady state.
    #[arg(long, default_value_t = 4.0)]
    warmup: f64,
    /// HCLK cycles per instruction.
    #[arg(long, default_value_t = 1.25)]
    cpi: f64,
    /// Most instructions per quantum.
    #[arg(long, default_value_t = 4000)]
    max_quantum: u32,
    /// CPU backend: `own` (our Cortex-M3 core, in every build) or `unicorn` (development builds
    /// with the `unicorn` feature only).
    #[arg(long, default_value = "own", value_name = "own|unicorn")]
    cpu: Backend,
    /// With --cpu unicorn: end every quantum after exactly its instruction budget, as the Python
    /// reference harness and our core do (slower: a CPU callback per instruction). Default: at
    /// the next block boundary.
    #[arg(long)]
    exact_quanta: bool,
    /// Battery voltage.
    #[arg(long, default_value_t = 130.0)]
    vbat: f64,
    /// Board temperature in °C.
    #[arg(long, default_value_t = 25.0)]
    board_temp: f64,
    /// Hall code driven on the three hall inputs.
    #[arg(long, default_value_t = 5)]
    hall: u8,
    /// Write `report.json` and `ble_tx.bin` here.
    #[arg(long, value_name = "DIR")]
    out: Option<PathBuf>,
}

/// The JSON a run prints and writes.
#[derive(Serialize)]
struct RunOutput<'a> {
    wheel: &'a str,
    firmware: FirmwareOut,
    cpu: &'static str,
    wall_seconds: f64,
    realtime_factor: f64,
    steady_state: Steady,
    #[serde(flatten)]
    report: &'a Report,
}

#[derive(Serialize)]
struct FirmwareOut {
    label: String,
    sha256: Sha256,
    bytes: usize,
}

#[derive(Serialize)]
struct Steady {
    sim_seconds: f64,
    wall_seconds: f64,
    realtime_factor: f64,
}

pub(crate) fn run(ctx: &Context, command: Command) -> anyhow::Result<Verdict> {
    let args = match command {
        Command::Run(args) => args,
        Command::Serve(args) => return crate::serve::serve(ctx, &args),
        Command::BridgeTest(args) => return crate::serve::bridge_test(ctx, &args),
        Command::Compare(args) => return crate::compare::compare(ctx, &args),
    };
    let cpu = match args.cpu {
        Backend::Unicorn { .. } => Backend::Unicorn { exact: args.exact_quanta },
        own @ Backend::Own => own,
    };
    let scenario = args.scenario.as_deref().map(load_scenario).transpose()?;
    let wheel_id = match (&args.wheel, &scenario) {
        (Some(wheel), Some(scenario)) if *wheel != scenario.wheel => {
            bail!("the scenario is for {}, not {wheel}", scenario.wheel)
        }
        (Some(wheel), _) => wheel.clone(),
        (None, Some(scenario)) => scenario.wheel.clone(),
        (None, None) => bail!("name a wheel profile, or give a --scenario"),
    };
    let source = ctx.profiles();
    let files = WheelFiles::load(&source, &wheel_id)?;
    let wheel = &files.profile;
    let (image, label) = firmware(ctx, wheel, &args.firmware)?;
    let sha256 = Sha256::digest(&image);
    // Stock-map addresses only mean something in the build they were taken from.
    let stock = wheel.find_pin(&sha256).is_some_and(|pin| pin.label == files.stockmap.build);
    let mut injections =
        args.injections.iter().enumerate().map(|(i, spec)| injection(spec, i)).collect::<anyhow::Result<Vec<_>>>()?;
    if let Some(scenario) = &scenario {
        for inject in &scenario.inject {
            injections.push(injection(&format!("{}@{}", inject.bytes, inject.at_s), 0)?);
        }
    }

    let hclk = files.hclk();
    let preset = args.settings.as_deref().or_else(|| scenario.as_ref().and_then(|s| s.settings.as_deref()));
    let board = board_with(&files, preset)?;
    let config = MachineConfig {
        board: &board,
        stockmap: stock.then_some(&files.stockmap),
        image: &image,
        load_addr: wheel.board.app.base,
        hclk,
        plant: PlantState {
            battery_v: args.vbat,
            board_temp_c: args.board_temp,
            hall: args.hall,
            ..PlantState::default()
        },
        injections: injections.clone(),
        power_cycles: args.power_cycles.clone(),
        cpi: args.cpi,
        max_quantum: args.max_quantum,
        marker_window_s: 4.0,
    };
    let firmware_out = FirmwareOut { label, sha256, bytes: image.len() };
    if let Some(scenario) = scenario {
        let live = files.live_on(scenario.rider, scenario.assist, scenario.terrain.clone())?;
        let machine = Machine::new_live(&config, &euc_sim::registry(), cpu_factory(cpu)?, live)?;
        let session = Session::new(machine, stock.then_some(&files.stockmap), args.sample_hz);
        return ride(ctx, &args, &files, &scenario, session, firmware_out, cpu);
    }
    let mut machine = Machine::new(&config, &euc_sim::registry(), cpu_factory(cpu)?)?;

    let total = (args.seconds * hclk as f64) as u64;
    let warm = total.min((args.warmup * hclk as f64) as u64);
    let started = Instant::now();
    machine.run(warm);
    let steady_start = (Instant::now(), machine.cycles);
    machine.run(total - warm);
    let wall = started.elapsed().as_secs_f64();
    let steady_wall = steady_start.0.elapsed().as_secs_f64();
    let steady_sim = (machine.cycles - steady_start.1) as f64 / hclk as f64;

    let report = Report::new(&machine, &injections);
    let output = RunOutput {
        wheel: &wheel.id,
        firmware: firmware_out,
        cpu: cpu.name(),
        wall_seconds: round(wall),
        realtime_factor: round(report.sim_seconds / wall),
        steady_state: Steady {
            sim_seconds: round(steady_sim),
            wall_seconds: round(steady_wall),
            realtime_factor: if steady_wall > 0.0 { round(steady_sim / steady_wall) } else { 0.0 },
        },
        report: &report,
    };
    if let Some(dir) = &args.out {
        write_outputs(dir, &output, &machine)?;
    }
    if ctx.json {
        ui::json_with_profiles(&wheel.origin, &output)?;
    } else {
        print(wheel, &output);
    }
    Ok(if report.halted.is_some() { Verdict::Refused } else { Verdict::Ok })
}

/// The JSON a scenario run prints.
#[derive(Serialize)]
struct RideOutput<'a> {
    wheel: &'a str,
    firmware: FirmwareOut,
    cpu: &'static str,
    scenario: &'a str,
    sim_seconds: f64,
    wall_seconds: f64,
    realtime_factor: f64,
    passed: bool,
    #[serde(flatten)]
    outcome: &'a Outcome,
    events: &'a [Event],
    halted: Option<String>,
}

pub(crate) fn load_scenario(path: &Path) -> anyhow::Result<Scenario> {
    let text = fs::read_to_string(path).with_context(|| format!("cannot read {}", ui::path(path)))?;
    Scenario::from_toml(&text).with_context(|| format!("scenario {}", ui::path(path)))
}

/// Rides `scenario` and reports its checks.
fn ride(
    ctx: &Context,
    args: &Args,
    files: &WheelFiles,
    scenario: &Scenario,
    mut session: Session,
    firmware: FirmwareOut,
    cpu: Backend,
) -> anyhow::Result<Verdict> {
    let wheel = &files.profile;
    let mut recorder = match &args.record {
        Some(path) => {
            let header = Header {
                wheel: &wheel.id,
                firmware: &firmware.label,
                scenario: Some(&scenario.name),
                sample_hz: args.sample_hz,
                bodies: session.bodies(),
                channels: &[],
            };
            Some(Recorder::create(path, &header).with_context(|| format!("cannot write {}", ui::path(path)))?)
        }
        None => None,
    };
    let mut events = Vec::new();
    let mut write_error = None;
    let started = Instant::now();
    let outcome = scenario::run(scenario, &mut session, |snapshot, session| {
        events.extend(session.take_events());
        if let Some(recorder) = recorder.as_mut()
            && let Err(error) = recorder.push(snapshot)
        {
            write_error.get_or_insert(error);
        }
    })
    .with_context(|| format!("scenario {}", scenario.name))?;
    let wall = started.elapsed().as_secs_f64();
    events.extend(session.take_events());
    if let Some(error) = write_error {
        bail!("recording: {error}");
    }
    if let (Some(recorder), Some(path)) = (recorder, &args.record) {
        recorder.finish(&events).with_context(|| format!("cannot write {}", ui::path(path)))?;
    }
    let sim_seconds = session.time_s();
    let halted = session.machine().halted().map(str::to_owned);
    let passed = outcome.passed() && halted.is_none();
    let output = RideOutput {
        wheel: &wheel.id,
        firmware,
        cpu: cpu.name(),
        scenario: &scenario.name,
        sim_seconds: round(sim_seconds),
        wall_seconds: round(wall),
        realtime_factor: round(sim_seconds / wall),
        passed,
        outcome: &outcome,
        events: &events,
        halted,
    };
    if ctx.json {
        ui::json_with_profiles(&wheel.origin, &output)?;
    } else {
        print_ride(wheel, scenario, args, &output);
    }
    Ok(if passed { Verdict::Ok } else { Verdict::Refused })
}

fn print_ride(wheel: &WheelProfile, scenario: &Scenario, args: &Args, output: &RideOutput<'_>) {
    ui::row(STRONG, "wheel", format!("{} ({}), from {}", wheel.name, wheel.id, wheel.origin));
    ui::row(STRONG, "firmware", format!("{}  {}", output.firmware.label, output.firmware.sha256.short()));
    ui::row(STRONG, "cpu", output.cpu);
    ui::row(STRONG, "scenario", format!("{}: {}", ui::safe(&scenario.name), ui::safe(&scenario.description)));
    ui::row(
        STRONG,
        "ran",
        format!(
            "{:.1} s simulated in {:.1} s: {:.2}x real time",
            output.sim_seconds, output.wall_seconds, output.realtime_factor
        ),
    );
    let o = output.outcome;
    ui::row(
        STRONG,
        "ride",
        format!(
            "top {:.1} km/h, {:.1} m, |pitch| up to {:.1}° after the release",
            o.top_speed_kmh, o.distance_m, o.max_abs_pitch_deg
        ),
    );
    for event in output.events.iter().filter(|e| e.kind != EventKind::Scenario).take(20) {
        let style = if matches!(event.kind, EventKind::Cutoff | EventKind::Fault) { WARN } else { DIM };
        ui::row(style, "event", format!("{:.2} s {}", event.t_s, ui::safe(&event.text)));
    }
    if let Some(path) = &args.record {
        ui::row(STRONG, "recording", ui::path(path));
    }
    for check in &o.checks {
        let (style, word) = if check.passed { (GOOD, "PASS") } else { (BAD, "FAIL") };
        ui::row(style, word, format!("{}: {}", check.what, check.measured));
    }
    if let Some(why) = &output.halted {
        ui::row(BAD, "HALTED", ui::safe(why));
    }
    let passed = o.checks.iter().filter(|c| c.passed).count();
    let summary = format!("{passed} of {} checks", o.checks.len());
    if output.passed {
        ui::row(GOOD, "RESULT", format!("pass, {summary}"));
    } else {
        ui::row(BAD, "RESULT", format!("fail, {summary}"));
    }
}

/// The factory for the `cpu` backend, or why this build cannot run it (Unicorn without the
/// `unicorn` feature).
pub(crate) fn cpu_factory(cpu: Backend) -> anyhow::Result<Box<CpuFactory<'static>>> {
    cpu.factory().map_err(anyhow::Error::msg)
}

fn round(value: f64) -> f64 {
    (value * 1000.0).round() / 1000.0
}

/// The image and its label: the current pinned build from the cache (fetched if missing), or
/// a file.
pub(crate) fn firmware(ctx: &Context, wheel: &WheelProfile, spec: &str) -> anyhow::Result<(Vec<u8>, String)> {
    if spec != "stock" {
        let path = Path::new(spec);
        let bytes = fs::read(path).with_context(|| format!("cannot read {}", ui::path(path)))?;
        let label = path.file_name().map_or_else(|| spec.to_owned(), |n| n.to_string_lossy().into_owned());
        return Ok((bytes, label));
    }
    let current = wheel
        .firmware
        .iter()
        .find_map(|source| match source {
            FirmwareSource::BegodeApi(build) if build.channel == Channel::Current => Some(build),
            _ => None,
        })
        .with_context(|| format!("{} pins no current stock build", wheel.name))?;
    let cache = ctx.cache()?;
    if cache.verified(&current.sha256).is_none() {
        ui::warn(format!("{} is not cached yet; fetching it", current.label));
        let report = euc_fwsource::fetch(wheel, &UreqClient::new(), &cache, FetchOptions::default())?;
        if !report.ok() {
            bail!("could not fetch {}; run `euc fetch {}` for details", current.label, wheel.id);
        }
    }
    let path = cache.verified(&current.sha256).with_context(|| format!("{} is not in the cache", current.label))?;
    Ok((fs::read(&path)?, current.label.clone()))
}

/// The board file with settings preset `preset` seeded (`None` or `blank`: an empty journal).
pub(crate) fn board_with(files: &WheelFiles, preset: Option<&str>) -> anyhow::Result<euc_sim::config::BoardFile> {
    match preset {
        None | Some(BLANK) => Ok(files.board.clone()),
        Some(name) => Ok(files.board_with_settings(name)?),
    }
}

/// The settings "preset" that seeds nothing.
pub(crate) const BLANK: &str = "blank";

/// `V`, `V@2.6` or `hex:5759@4`; without a time the i-th injection goes at 3.0 + 1.5 i seconds.
pub(crate) fn injection(spec: &str, index: usize) -> anyhow::Result<(f64, Vec<u8>)> {
    let (data, at) = match spec.rsplit_once('@') {
        Some((data, at)) => (data, at.parse::<f64>().with_context(|| format!("bad time in \"{spec}\""))?),
        None => (spec, 3.0 + 1.5 * index as f64),
    };
    let bytes = if let Some(hex) = data.strip_prefix("hex:") {
        (0..hex.len())
            .step_by(2)
            .map(|i| hex.get(i..i + 2).and_then(|pair| u8::from_str_radix(pair, 16).ok()))
            .collect::<Option<Vec<u8>>>()
            .with_context(|| format!("bad hex in \"{spec}\""))?
    } else {
        data.chars()
            .map(|c| u8::try_from(u32::from(c)).ok())
            .collect::<Option<Vec<u8>>>()
            .with_context(|| format!("\"{spec}\" has characters above U+00FF; use hex:"))?
    };
    Ok((at, bytes))
}

fn write_outputs(dir: &Path, output: &RunOutput<'_>, machine: &Machine) -> anyhow::Result<()> {
    fs::create_dir_all(dir).with_context(|| format!("cannot create {}", ui::path(dir)))?;
    fs::write(dir.join("report.json"), serde_json::to_vec_pretty(output)?)?;
    let ble: Vec<u8> = machine.board().ble_tx.iter().map(|&(_, b)| b).collect();
    fs::write(dir.join("ble_tx.bin"), ble)?;
    Ok(())
}

fn print(wheel: &WheelProfile, output: &RunOutput<'_>) {
    let report = output.report;
    ui::row(STRONG, "wheel", format!("{} ({}), from {}", wheel.name, wheel.id, wheel.origin));
    ui::row(STRONG, "firmware", format!("{}  {}", output.firmware.label, output.firmware.sha256.short()));
    ui::row(STRONG, "cpu", output.cpu);
    ui::row(
        STRONG,
        "ran",
        format!(
            "{:.3} s simulated in {:.1} s: {:.2}x real time (after warm-up {:.2}x)",
            report.sim_seconds, output.wall_seconds, output.realtime_factor, output.steady_state.realtime_factor
        ),
    );
    let reached: Vec<String> =
        report.markers.iter().filter_map(|m| m.first_ms.map(|t| format!("{} {t:.3} ms", m.name))).collect();
    ui::row(STRONG, "markers", ui::paint(DIM, reached.join(", ")));
    let ble = &report.ble;
    let first = ble.first_frame_ms.map_or_else(|| "never".to_owned(), |t| format!("{t:.2} ms"));
    let types: Vec<String> = ble.frames_by_type.iter().map(|(t, n)| format!("{t}:{n}")).collect();
    ui::row(STRONG, "frames", format!("{} ({}), first at {first}", ble.frames, types.join(" ")));
    for injection in &report.injections {
        let reply: Vec<String> = injection.reply_text.iter().map(|t| ui::quoted(t)).collect();
        let reply = if reply.is_empty() { "no text".to_owned() } else { reply.join(" ") };
        ui::row(STRONG, "inject", format!("{} @ {:.2} s -> {reply}", ui::quoted(&injection.bytes), injection.at_s));
    }
    if report.blockers.is_empty() {
        ui::row(GOOD, "blockers", "none");
    }
    for blocker in &report.blockers {
        let style = if blocker.fatal { BAD } else { WARN };
        ui::row(
            style,
            "blocker",
            format!("{} @ {:.3} ms x{}: {}", blocker.kind, blocker.t_ms, blocker.count, ui::safe(&blocker.text)),
        );
    }
    match &report.halted {
        Some(why) => ui::row(BAD, "HALTED", ui::safe(why)),
        None => ui::row(
            GOOD,
            "OK",
            format!("{} instructions, {} exceptions", report.performance.instructions, report.performance.exceptions),
        ),
    }
}

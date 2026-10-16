//! `euc sim run`: the wheel's controller firmware in the simulator.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context as _, bail};
use euc_fwsource::{FetchOptions, UreqClient};
use euc_profile::{Channel, FirmwareSource, Sha256, WheelProfile};
use euc_sim::config::{BoardFile, StockMap};
use euc_sim::{CpuFactory, Machine, MachineConfig, PlantState, Report};
use serde::Serialize;

use crate::ui::{self, BAD, DIM, GOOD, STRONG, WARN};
use crate::{Context, Verdict};

#[derive(Debug, clap::Subcommand)]
pub(crate) enum Command {
    /// Run a wheel's controller firmware in the simulator and report what it did.
    Run(Args),
}

#[derive(Debug, clap::Args)]
pub(crate) struct Args {
    /// Wheel profile id, e.g. begode-extreme.
    wheel: String,
    /// `stock` (the current official build, fetched if it is not cached) or a firmware file.
    #[arg(long, default_value = "stock")]
    firmware: String,
    /// Simulated seconds to run.
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
    /// End every quantum after exactly its instruction budget, as the Python reference harness
    /// does (slower: a CPU callback per instruction). Default: at the next block boundary.
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
    let Command::Run(args) = command;
    let Some(backend) = BACKEND else {
        bail!(
            "this euc has no CPU backend. The Unicorn backend is GPL-2.0 and for development only; \
             run `cargo run -p euc-cli --features unicorn -- sim run ...`"
        );
    };
    let source = ctx.profiles();
    let wheel = source.load(&args.wheel)?;
    let board: BoardFile = toml::from_str(&source.read(&wheel.id, "board.toml")?).context("board.toml")?;
    let stockmap: StockMap = toml::from_str(&source.read(&wheel.id, "stockmap.toml")?).context("stockmap.toml")?;
    let (image, label) = firmware(ctx, &wheel, &args.firmware)?;
    let sha256 = Sha256::digest(&image);
    // Stock-map addresses only mean something in the build they were taken from.
    let stock = wheel.find_pin(&sha256).is_some_and(|pin| pin.label == stockmap.build);
    let injections =
        args.injections.iter().enumerate().map(|(i, spec)| injection(spec, i)).collect::<anyhow::Result<Vec<_>>>()?;

    let hclk = u64::from(wheel.board.hclk_hz);
    let config = MachineConfig {
        board: &board,
        stockmap: stock.then_some(&stockmap),
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
    let mut machine = Machine::new(&config, &euc_sim::registry(), backend(args.exact_quanta))?;

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
        firmware: FirmwareOut { label, sha256, bytes: image.len() },
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
        print(&wheel, &output);
    }
    Ok(if report.halted.is_some() { Verdict::Refused } else { Verdict::Ok })
}

/// The CPU backend this build has: Unicorn with the `unicorn` feature, none otherwise (our own
/// core arrives in M3).
#[cfg(feature = "unicorn")]
const BACKEND: Option<fn(bool) -> Box<CpuFactory<'static>>> = Some(unicorn);
#[cfg(not(feature = "unicorn"))]
const BACKEND: Option<fn(bool) -> Box<CpuFactory<'static>>> = None;

#[cfg(feature = "unicorn")]
#[allow(unsafe_code)]
fn unicorn(exact: bool) -> Box<CpuFactory<'static>> {
    use euc_unicorn::{Quantum, UnicornCpu};
    let quantum = if exact { Quantum::Exact } else { Quantum::Block };
    Box::new(move |regions| {
        // SAFETY: `Machine::new` calls this with regions whose memory it owns in boxed buffers
        // that never move and outlive the CPU it stores next to them; it touches that memory
        // during a run only from inside the run's bus callbacks.
        let cpu = unsafe { UnicornCpu::with_quantum(regions, quantum) }.map_err(|e| e.to_string())?;
        Ok(Box::new(cpu) as Box<dyn euc_core::Cpu>)
    })
}

fn round(value: f64) -> f64 {
    (value * 1000.0).round() / 1000.0
}

/// The image and its label: the current pinned build from the cache (fetched if missing), or
/// a file.
fn firmware(ctx: &Context, wheel: &WheelProfile, spec: &str) -> anyhow::Result<(Vec<u8>, String)> {
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

/// `V`, `V@2.6` or `hex:5759@4`; without a time the i-th injection goes at 3.0 + 1.5 i seconds.
fn injection(spec: &str, index: usize) -> anyhow::Result<(f64, Vec<u8>)> {
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

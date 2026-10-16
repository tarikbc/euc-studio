//! The stock Begode Extreme image on a machine, for the whole-firmware tests.
//!
//! The image is never in the repo: these helpers read it from the firmware cache (`euc fetch
//! begode-extreme`, or `EUC_CACHE_DIR`) and the tests pass with a printed reason when it is not
//! there.

// Each test binary includes this module and uses a different part of it.
#![allow(dead_code)]

use std::time::Instant;

use euc_fwsource::FirmwareCache;
use euc_profile::{Channel, FirmwareSource, ProfileSource, WheelProfile};
use euc_sim::config::{BoardFile, StockMap};
use euc_sim::{CpuFactory, Machine, MachineConfig, PlantState, Report};

pub const WHEEL: &str = "begode-extreme";

/// Commands the boot sends over BLE, with the simulated second of each.
pub const INJECTIONS: [(f64, &[u8]); 2] = [(2.6, b"V"), (3.6, b"N")];

/// The wheel profile and the cached stock image, or `None` (with the reason printed).
pub fn stock() -> Option<(WheelProfile, Vec<u8>)> {
    let wheel = ProfileSource::Builtin.load(WHEEL).unwrap();
    let current = wheel
        .firmware
        .iter()
        .find_map(|source| match source {
            FirmwareSource::BegodeApi(build) if build.channel == Channel::Current => Some(build.sha256),
            _ => None,
        })
        .unwrap();
    let root = std::env::var_os("EUC_CACHE_DIR").map(Into::into).or_else(FirmwareCache::default_root)?;
    let Some(path) = FirmwareCache::new(root).verified(&current) else {
        eprintln!("skipped: the stock image is not cached; run `euc fetch {WHEEL}`");
        return None;
    };
    Some((wheel, std::fs::read(path).unwrap()))
}

/// A finished run.
pub struct Run {
    pub report: Report,
    /// Wall-clock seconds the run took.
    pub wall: f64,
    /// Instructions the machine counted.
    pub instructions: u64,
}

/// Boots the image on `cpu` and runs it for `seconds` of simulated time, with the stock CLI
/// settings (CPI 1.25, quanta of at most 4000 instructions) and the `V`/`N` injections.
pub fn boot(wheel: &WheelProfile, image: &[u8], seconds: f64, cpu: Box<CpuFactory<'_>>) -> Run {
    let source = ProfileSource::Builtin;
    let board: BoardFile = toml::from_str(&source.read(WHEEL, "board.toml").unwrap()).unwrap();
    let stockmap: StockMap = toml::from_str(&source.read(WHEEL, "stockmap.toml").unwrap()).unwrap();
    let hclk = u64::from(wheel.board.hclk_hz);
    let injections: Vec<(f64, Vec<u8>)> = INJECTIONS.iter().map(|&(at, bytes)| (at, bytes.to_vec())).collect();
    let config = MachineConfig {
        board: &board,
        stockmap: Some(&stockmap),
        image,
        load_addr: wheel.board.app.base,
        hclk,
        plant: PlantState::default(),
        injections: injections.clone(),
        power_cycles: Vec::new(),
        cpi: 1.25,
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    let mut machine = Machine::new(&config, &euc_sim::registry(), cpu).unwrap();
    let started = Instant::now();
    machine.run((seconds * hclk as f64) as u64);
    let wall = started.elapsed().as_secs_f64();
    Run { report: Report::new(&machine, &injections), wall, instructions: machine.instr }
}

/// Simulated seconds from `var`, or `default`.
pub fn seconds(var: &str, default: f64) -> f64 {
    std::env::var(var).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

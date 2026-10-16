//! The stock Begode Extreme firmware boots on the Unicorn backend.
//!
//! The image is never in the repo: these tests read it from the firmware cache (`euc fetch
//! begode-extreme`, or `EUC_CACHE_DIR`) and pass with a printed reason when it is not there.
//! `.github/workflows/stock-boot.yml` fetches it first.

#![allow(unsafe_code)]

use euc_core::Cpu;
use euc_fwsource::FirmwareCache;
use euc_profile::{Channel, FirmwareSource, ProfileSource, WheelProfile};
use euc_sim::config::{BoardFile, StockMap};
use euc_sim::{Machine, MachineConfig, PlantState, Report};
use euc_unicorn::UnicornCpu;

const WHEEL: &str = "begode-extreme";

/// The wheel profile and the cached stock image, or `None` (with the reason printed).
fn stock() -> Option<(WheelProfile, Vec<u8>)> {
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

fn boot(wheel: &WheelProfile, image: &[u8], injections: &[(f64, Vec<u8>)], seconds: f64) -> (Machine, Report) {
    let source = ProfileSource::Builtin;
    let board: BoardFile = toml::from_str(&source.read(WHEEL, "board.toml").unwrap()).unwrap();
    let stockmap: StockMap = toml::from_str(&source.read(WHEEL, "stockmap.toml").unwrap()).unwrap();
    let hclk = u64::from(wheel.board.hclk_hz);
    let config = MachineConfig {
        board: &board,
        stockmap: Some(&stockmap),
        image,
        load_addr: wheel.board.app.base,
        hclk,
        plant: PlantState::default(),
        injections: injections.to_vec(),
        power_cycles: Vec::new(),
        cpi: 1.25,
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    let cpu = Box::new(|regions: &[euc_core::Region]| {
        // SAFETY: `Machine::new` owns the memory behind every region in boxed buffers that never
        // move and outlive the CPU, and touches it during a run only from bus callbacks.
        let cpu = unsafe { UnicornCpu::new(regions) }.map_err(|e| e.to_string())?;
        Ok(Box::new(cpu) as Box<dyn Cpu>)
    });
    let mut machine = Machine::new(&config, &euc_sim::registry(), cpu).unwrap();
    let total = (seconds * hclk as f64) as u64;
    let warm = total.min(4 * hclk);
    machine.run(warm);
    machine.run(total - warm);
    let report = Report::new(&machine, injections);
    (machine, report)
}

fn marker_ms(report: &Report, name: &str) -> f64 {
    report.markers.iter().find(|m| m.name == name).and_then(|m| m.first_ms).unwrap_or(f64::NAN)
}

#[test]
fn boots_answers_v_and_n_and_sends_telemetry() {
    let Some((wheel, image)) = stock() else { return };
    let injections = [(2.6, b"V".to_vec()), (3.6, b"N".to_vec())];
    let (_, report) = boot(&wheel, &image, &injections, 6.0);

    assert_eq!(report.halted, None, "{:#?}", report.blockers);
    assert!(report.blockers.is_empty(), "{:#?}", report.blockers);
    assert!((marker_ms(&report, "main") - 0.069).abs() < 0.005, "main at {} ms", marker_ms(&report, "main"));
    let first = report.ble.first_frame_ms.expect("telemetry frames");
    assert!((first - 2000.0).abs() < 10.0, "first frame at {first} ms");
    assert_eq!(report.injections[0].reply_text, ["GW1825020"]);
    assert_eq!(report.injections[1].reply_text, ["NAME:EXTREME\r\n"]);
    // As in the Python reference run: telemetry pauses for three rounds after each command.
    assert_eq!(report.ble.frames, 40);
}

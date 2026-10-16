//! The stock Begode Extreme firmware on our core alone, as the shipped build runs it: the same
//! boot results the Unicorn backend's `stock_boot` test checks, and the real-time factor over
//! 30 simulated seconds (`EUC_STOCK_SECONDS` changes it).
//!
//! ```text
//! cargo test -p euc-armv7m --release --test stock_boot -- --nocapture
//! ```

#![allow(unsafe_code)]

use std::cell::Cell;
use std::rc::Rc;

use euc_armv7m::Armv7m;
use euc_core::{Bus, CoreRegs, Cpu, Exit, Region};
use euc_sim::CpuFactory;

mod common;

/// Our core, sharing its counters with the test after each run: `(retired, modeled cycles)`.
struct Counted {
    cpu: Armv7m,
    counters: Rc<Cell<(u64, u64)>>,
}

impl Cpu for Counted {
    fn reset(&mut self, sp: u32, reset_vector: u32) {
        self.cpu.reset(sp, reset_vector);
    }

    fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit {
        let exit = self.cpu.run(bus, max_instr);
        self.counters.set((self.cpu.retired(), self.cpu.cycles()));
        exit
    }

    fn regs(&self) -> &CoreRegs {
        self.cpu.regs()
    }

    fn regs_mut(&mut self) -> &mut CoreRegs {
        self.cpu.regs_mut()
    }

    fn retired(&self) -> u64 {
        self.cpu.retired()
    }

    fn watch(&mut self, addrs: &[u32]) {
        self.cpu.watch(addrs);
    }

    fn invalidate(&mut self, addr: u32, len: u32) {
        self.cpu.invalidate(addr, len);
    }
}

fn own_core(counters: Rc<Cell<(u64, u64)>>) -> Box<CpuFactory<'static>> {
    Box::new(move |regions: &[Region]| {
        // SAFETY: `Machine::new` owns the memory behind every region in boxed buffers that never
        // move and outlive the CPU, and touches it during a run only from bus callbacks.
        let cpu = unsafe { Armv7m::new(regions) }.map_err(|e| e.to_string())?;
        Ok(Box::new(Counted { cpu, counters }) as Box<dyn Cpu>)
    })
}

fn marker_ms(report: &euc_sim::Report, name: &str) -> f64 {
    report.markers.iter().find(|m| m.name == name).and_then(|m| m.first_ms).unwrap_or(f64::NAN)
}

/// The assertions of `euc-unicorn/tests/stock_boot.rs`, on our core.
#[test]
fn boots_answers_v_and_n_and_sends_telemetry() {
    let Some((wheel, image)) = common::stock() else { return };
    let report = common::boot(&wheel, &image, 6.0, own_core(Rc::default())).report;

    assert_eq!(report.halted, None, "{:#?}", report.blockers);
    assert!(report.blockers.is_empty(), "{:#?}", report.blockers);
    assert!((marker_ms(&report, "main") - 0.069).abs() < 0.005, "main at {} ms", marker_ms(&report, "main"));
    let first = report.ble.first_frame_ms.expect("telemetry frames");
    assert!((first - 2000.0).abs() < 10.0, "first frame at {first} ms");
    assert_eq!(report.injections[0].reply_text, ["GW1825020"]);
    assert_eq!(report.injections[1].reply_text, ["NAME:EXTREME\r\n"]);
    assert_eq!(report.ble.frames, 40);
}

/// 30 simulated seconds with no blocker, the real-time factor, and the CPI the TRM cycle model
/// gives the firmware (the machine itself still charges a fixed CPI). A release build must run
/// at least 1.5 times faster than real time (the M3 target) on a local machine. A shared CI
/// runner (`CI` set) only has to keep real time; the log keeps the measured factor and MIPS.
/// `EUC_SPEED_FLOOR` overrides the floor in both places.
#[test]
fn runs_faster_than_real_time() {
    let Some((wheel, image)) = common::stock() else { return };
    let seconds = common::seconds("EUC_STOCK_SECONDS", 30.0);
    let counters = Rc::new(Cell::new((0, 0)));
    let run = common::boot(&wheel, &image, seconds, own_core(Rc::clone(&counters)));
    let factor = seconds / run.wall;
    let (retired, cycles) = counters.get();
    eprintln!(
        "stock image on our core: {seconds} s simulated in {:.2} s = {factor:.2}x real time, {retired} instructions \
         ({:.0} MIPS with the engine), {} telemetry frames, modeled CPI {:.3}",
        run.wall,
        retired as f64 / run.wall / 1e6,
        run.report.ble.frames,
        cycles as f64 / retired as f64
    );
    assert_eq!(run.report.halted, None, "{:#?}", run.report.blockers);
    assert!(run.report.blockers.is_empty(), "{:#?}", run.report.blockers);
    if !cfg!(debug_assertions) {
        let default = if std::env::var_os("CI").is_some() { 1.0 } else { 1.5 };
        let floor = std::env::var("EUC_SPEED_FLOOR").ok().and_then(|v| v.parse().ok()).unwrap_or(default);
        assert!(factor >= floor, "{factor:.2}x real time, below the {floor}x target");
    }
}

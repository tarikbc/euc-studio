//! Building a host: the scenario, the wheel's files, the stock image (from the firmware cache,
//! fetched from the vendor's update server if missing), the machine and its physics.

use euc_fwsource::{FetchOptions, FirmwareCache, UreqClient};
use euc_profile::{Channel, FirmwareSource, ProfileSource, Sha256, WheelProfile};
use euc_sim::{Machine, MachineConfig, PlantState, Session, WheelFiles};

use super::cpu;
use super::display::DisplayLayout;
use super::host::{RideHost, StillHost};
use super::lights::LightsLayout;
use super::runner::Built;
use super::scenarios;

/// Snapshots per simulated second; the runner steps in 1/120 s.
const SAMPLE_HZ: f64 = 120.0;

/// Builds the host for scenario `id`. Runs on the sim thread; `progress` names each step.
pub fn build_host(id: &str, progress: &mut dyn FnMut(&str)) -> Result<Built, String> {
    let scenario = scenarios::load(id)?;
    progress("Loading the wheel profile");
    let source = ProfileSource::Builtin;
    let files = WheelFiles::load(&source, &scenario.wheel).map_err(|e| e.to_string())?;
    let wheel = &files.profile;
    // The display replica is optional: a wheel without `display.toml` shows the raw digits.
    // The display and tail-light layouts are optional: without them (or with a file that does not
    // parse) the run goes on and the card shows the raw digits and LEDs.
    let mut problems = Vec::new();
    let display = source
        .read(&wheel.id, "display.toml")
        .ok()
        .and_then(|text| DisplayLayout::parse(&text).map_err(|e| problems.push(e)).ok());
    let lights = source
        .read(&wheel.id, "lights.toml")
        .ok()
        .and_then(|text| LightsLayout::parse(&text).map_err(|e| problems.push(e)).ok());
    let note = (!problems.is_empty()).then(|| problems.join("; "));
    let (image, label) = stock_image(wheel, progress)?;
    let sha256 = Sha256::digest(&image);
    // Stock-map addresses only mean something in the build they were taken from.
    let stock = wheel.find_pin(&sha256).is_some_and(|pin| pin.label == files.stockmap.build);

    // The settings journal the wheel boots with ("blank" or none: an empty journal).
    let board = match scenario.settings.as_deref() {
        None | Some("blank") => files.board.clone(),
        Some(preset) => files.board_with_settings(preset).map_err(|e| e.to_string())?,
    };

    progress("Building the simulated controller");
    let factory = cpu::factory()?;
    let plant = PlantState::default();
    let battery_v = plant.battery_v;
    let config = MachineConfig {
        board: &board,
        stockmap: stock.then_some(&files.stockmap),
        image: &image,
        load_addr: wheel.board.app.base,
        hclk: files.hclk(),
        plant,
        injections: scenario.inject.iter().map(|i| (i.at_s, i.bytes.bytes().collect())).collect(),
        power_cycles: Vec::new(),
        cpi: 1.25,
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    // The ground the scenario rides on, as points for the web view to draw (none when flat).
    let terrain = (!scenario.terrain.is_flat()).then(|| {
        let step = 0.05;
        let end = scenario.terrain.length_m;
        (0..=((end + 5.0) / step) as usize)
            .map(|i| {
                let x = -5.0 + i as f64 * step;
                [x as f32, scenario.terrain.height(x.max(0.0)) as f32]
            })
            .collect::<Vec<_>>()
    });
    match files.live_on(scenario.rider, scenario.assist, scenario.terrain.clone()) {
        Ok(live) => {
            let machine = Machine::new_live(&config, &euc_sim::registry(), factory, live).map_err(|e| e.to_string())?;
            let session = Session::new(machine, stock.then_some(&files.stockmap), SAMPLE_HZ);
            Ok(Built {
                host: Box::new(RideHost::new(session, scenario)?),
                label: "stock firmware".into(),
                build: label,
                note,
                display,
                lights,
                terrain,
            })
        }
        // Without a body model the firmware still runs, on a wheel that stands still.
        Err(e) => {
            let machine = Machine::new(&config, &euc_sim::registry(), factory).map_err(|e| e.to_string())?;
            Ok(Built {
                host: Box::new(StillHost::new(machine, battery_v)),
                label: "stock firmware (no physics)".into(),
                build: label,
                note: Some(format!("The wheel stands still: {e}")).map(|n| match &note {
                    Some(more) => format!("{n}; {more}"),
                    None => n,
                }),
                display,
                lights,
                terrain: None,
            })
        }
    }
}

/// The current pinned stock build, from the cache or fetched into it.
fn stock_image(wheel: &WheelProfile, progress: &mut dyn FnMut(&str)) -> Result<(Vec<u8>, String), String> {
    let current = wheel
        .firmware
        .iter()
        .find_map(|source| match source {
            FirmwareSource::BegodeApi(build) if build.channel == Channel::Current => Some(build),
            _ => None,
        })
        .ok_or_else(|| format!("{} pins no current stock build", wheel.name))?;
    let root = std::env::var_os("EUC_CACHE_DIR")
        .map(Into::into)
        .or_else(FirmwareCache::default_root)
        .ok_or("this system reports no per-user cache folder; set EUC_CACHE_DIR")?;
    let cache = FirmwareCache::new(root);
    if cache.verified(&current.sha256).is_none() {
        progress("Fetching the stock image from Begode's update server");
        let report = euc_fwsource::fetch(wheel, &UreqClient::new(), &cache, FetchOptions::default())
            .map_err(|e| format!("could not fetch {}: {e}", current.label))?;
        if !report.ok() {
            return Err(format!("could not fetch {}; run `euc fetch {}` for details", current.label, wheel.id));
        }
    }
    let path = cache.verified(&current.sha256).ok_or_else(|| format!("{} is not in the cache", current.label))?;
    let image = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok((image, current.label.clone()))
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use euc_telemetry::{BodyRole, Snapshot};

    use super::super::host::Host;
    use super::super::phone::{PhoneDecoder, PhoneFrame};
    use super::*;

    /// The host and what a phone made of its BLE output so far.
    struct Rig<'a> {
        host: &'a mut dyn Host,
        decoder: PhoneDecoder,
        frames: Vec<PhoneFrame>,
    }

    impl<'a> Rig<'a> {
        fn new(host: &'a mut dyn Host) -> Self {
            Self { host, decoder: PhoneDecoder::default(), frames: Vec::new() }
        }

        fn run(&mut self, seconds: f64) {
            let mut ble = Vec::new();
            for _ in 0..(seconds * 120.0).round() as u32 {
                self.host.step(1.0 / 120.0);
                self.host.drain_ble_tx(&mut ble);
                self.decoder.push(&ble, &mut self.frames);
                ble.clear();
            }
        }
    }

    /// Needs the stock image in the firmware cache (`euc fetch begode-extreme`):
    /// `cargo test -- --ignored --nocapture` (our own core; `EUC_CPU=unicorn` with the feature).
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn stock_firmware_boots_and_streams() {
        let mut built = build_host("balance-60s", &mut |_| {}).expect("host");
        let roles: Vec<_> = built.host.bodies().iter().map(|b| b.role).collect();
        assert!(roles.contains(&BodyRole::Rider), "the physics carries the rider: {roles:?}");
        let mut rig = Rig::new(built.host.as_mut());
        let started = Instant::now();
        rig.run(4.0);
        let wall = started.elapsed().as_secs_f64();
        let mut snap = Snapshot::default();
        rig.host.snapshot(&mut snap);
        let mut events = Vec::new();
        rig.host.drain_events(&mut events);
        println!("4 s simulated in {wall:.2} s ({:.2}x), {} frames, events {events:?}", 4.0 / wall, rig.frames.len());
        assert!(rig.frames.iter().any(|f| f.kind == "live"), "the firmware streams live frames");
        assert!(snap.signals.battery_v > 100.0);
        assert_eq!(snap.bodies.len(), roles.len(), "a pose per body");

        // The phone asks for the version, then sounds the horn.
        rig.host.inject_ble(b"V").expect("inject");
        rig.run(1.0);
        let mut boot_tones = Vec::new();
        rig.host.drain_tones(&mut boot_tones);
        rig.host.inject_ble(b"b").expect("inject");
        rig.run(2.0);
        let mut events = Vec::new();
        rig.host.drain_events(&mut events);
        let mut tones = Vec::new();
        rig.host.drain_tones(&mut tones);
        println!("boot tones {boot_tones:?}");
        println!("horn tones {tones:?}");
        let starts = tones.iter().filter(|t| t.hz > 0.0).count();
        assert_eq!(starts, 2, "the horn beeps twice");
        assert!(tones.iter().all(|t| t.hz == 0.0 || (t.hz - 2700.0).abs() < 10.0), "at about 2.7 kHz");
        let outputs = rig.host.outputs().expect("outputs");
        let replies: Vec<_> = rig.frames.iter().filter(|f| f.reply).map(|f| f.summary.clone()).collect();
        println!("replies {replies:?}; beeps {:?}", events.iter().filter(|e| e.text.starts_with("Beep")).count());
        println!("display on {} grid {:02X?}; {} LEDs", outputs.display_on, outputs.display_grid, outputs.leds.len());
        assert!(replies.iter().any(|r| r.starts_with("Version GW")), "V answers with the banner");
        assert!(events.iter().any(|e| e.text.starts_with("Beep")), "b sounds the buzzer");
        assert!(outputs.display_on, "the display is lit");
        assert_eq!(outputs.leds.len(), 32, "the light strip has 32 LEDs");
    }

    /// The whole ride, as the studio steps it: the scenario's checks pass.
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn stock_firmware_rides_to_20_kmh_and_stops() {
        let mut built = build_host("ride-20kmh-stop", &mut |_| {}).expect("host");
        let mut rig = Rig::new(built.host.as_mut());
        let started = Instant::now();
        let mut top = 0.0_f32;
        let mut snap = Snapshot::default();
        for _ in 0..41 {
            rig.run(1.0);
            rig.host.snapshot(&mut snap);
            top = top.max(snap.signals.speed_kmh);
        }
        let wall = started.elapsed().as_secs_f64();
        let host = rig.host;
        let mut events = Vec::new();
        host.drain_events(&mut events);
        for e in &events {
            println!("{:7.2} {:?} {}", e.t_s, e.kind, e.text);
        }
        println!("41 s simulated in {wall:.2} s ({:.2}x), top {top:.1} km/h", 41.0 / wall);
        assert!(host.halted().is_none());
        assert!(top > 19.0, "top speed {top}");
        assert!(events.iter().any(|e| e.text.starts_with("Scenario ride-20kmh-stop passed")), "the checks pass");
    }
}

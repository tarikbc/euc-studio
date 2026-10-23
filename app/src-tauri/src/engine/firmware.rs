//! Building a host: the scenario, the wheel's files, the stock image (from the firmware cache,
//! fetched from the vendor's update server if missing), the machine and its physics.

use euc_fwsource::{FetchOptions, FirmwareCache, UreqClient};
use euc_profile::{Channel, FirmwareSource, ProfileSource, Sha256, WheelProfile};
use euc_sim::scenario::Scenario;
use euc_sim::{Machine, MachineConfig, PlantState, Session, WheelFiles};

use super::display::DisplayLayout;
use super::host::RideHost;
use super::lights::LightsLayout;
use super::runner::{Built, SuspensionInfo};
use super::scenarios;

/// Snapshots per simulated second; the runner steps in 1/120 s.
const SAMPLE_HZ: f64 = 120.0;

/// Builds an interactive Session on one authoritative finite map. No scenario driver controls its rider.
pub(super) fn build_free_host(
    spec: &super::free_ride::FreeRideSpec,
    firmware: &str,
    epoch: u64,
    controls: super::free_ride::FreeRideState,
    progress: &mut dyn FnMut(&str),
) -> Result<Built, String> {
    spec.validate()?;
    let rider = spec.rider()?;
    let (map, world) = match (&spec.map, &spec.world) {
        (Some(map), _) => {
            progress("Generating the physical map");
            let revision =
                u32::try_from(epoch).map_err(|_| "Restart the app before the run epoch exceeds its map limit")?;
            let map = std::sync::Arc::new(map.generate(revision)?);
            controls.begin(epoch, Some(std::sync::Arc::clone(&map)), rider);
            (Some(map), None)
        }
        (None, Some(choice)) => {
            progress("Preparing the world around the spawn");
            let world = super::world::shared(choice.spec())?;
            // A quick reset starts where the wheel was; a new ride at the world's spawn.
            let start = spec.start.map_or_else(
                || {
                    let spawn = world.spawn();
                    euc_physics::WorldStart {
                        xy_m: [spawn.position[0], spawn.position[1]],
                        heading_rad: spawn.heading[1].atan2(spawn.heading[0]),
                    }
                },
                |pose| euc_physics::WorldStart { xy_m: pose.xy_m, heading_rad: pose.heading_rad },
            );
            let ground = euc_physics::WorldGround::with_world(world, start)?;
            controls.begin_world(epoch, ground.clone(), rider);
            (None, Some(ground))
        }
        (None, None) => return Err("Choose either the test pad or a world".into()),
    };
    controls.set_spec(epoch, spec.clone());
    let source = ProfileSource::Builtin;
    let files = WheelFiles::load(&source, &spec.wheel).map_err(|error| error.to_string())?;
    let wheel = &files.profile;
    let board = if spec.settings == "blank" {
        files.board.clone()
    } else {
        files.board_with_settings(&spec.settings).map_err(|error| error.to_string())?
    };
    let display = source.read(&wheel.id, "display.toml").ok().and_then(|text| DisplayLayout::parse(&text).ok());
    let lights = source.read(&wheel.id, "lights.toml").ok().and_then(|text| LightsLayout::parse(&text).ok());
    let suspension = files.body.suspension.as_ref().and_then(|s| {
        let geometry = source.read(&wheel.id, "geometry.toml").ok()?;
        shock_eyes(&geometry).map(|(eye_top_m, eye_bottom_m)| SuspensionInfo {
            linkage_ratio: s.linkage_ratio,
            eye_top_m,
            eye_bottom_m,
        })
    });
    let loaded = super::images::load(&files, firmware, progress)?;
    let live = match (&map, &world) {
        (Some(map), _) => files.live_on_map(rider, Default::default(), map.as_ref().clone()),
        (None, Some(ground)) => files.live_on_world(rider, Default::default(), ground.clone()),
        (None, None) => unreachable!("the ground was chosen above"),
    }
    .map_err(|error| format!("Invalid free ride physical model: {error}"))?;
    let config = MachineConfig {
        board: &board,
        stockmap: loaded.map.as_ref(),
        image: &loaded.image,
        load_addr: wheel.board.app.base,
        hclk: files.hclk(),
        plant: PlantState::default(),
        injections: Vec::new(),
        power_cycles: Vec::new(),
        cpi: euc_sim::DEFAULT_CPI,
        exception_timing: loaded.exception_timing(),
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    progress("Starting firmware with the rider on the stand");
    let machine =
        Machine::new_live(&config, &euc_sim::registry(), loaded.factory()?, live).map_err(|error| error.to_string())?;
    let frame = world.as_ref().map(super::world::WorldFrame::of);
    let session = Session::new(machine, loaded.map.as_ref(), SAMPLE_HZ);
    let mut header = euc_sim::recording::header(
        concat!("EUC Studio ", env!("CARGO_PKG_VERSION")),
        &source,
        &files,
        &session,
        &loaded.label,
        &Sha256::digest(&loaded.image),
    );
    header.cpu = Some(loaded.cpu_name().into());
    header.cpi = loaded.cpi();
    header.settings = Some(spec.settings.clone());
    header.context = controls.recording_context(epoch)?;
    Ok(Built {
        wheel: wheel.id.clone(),
        header: Some(header),
        host: Box::new(super::free_ride::FreeRideHost::new(session, controls, epoch, map)),
        label: format!("{} firmware · free ride", firmware_name(firmware)),
        build: loaded.label,
        note: Some(match &world {
            Some(ground) => format!(
                "Experimental {} rider. Pedal welds and external yaw/roll aids remain active. World seed {}: \
                 3.1 cm ground around the wheel, curbs and steps as boxes",
                rider.model.id(),
                ground.spec().seed
            ),
            None => format!(
                "Experimental {} rider. Pedal welds and external yaw/roll aids remain active. Map resolution is 0.25 m",
                rider.model.id()
            ),
        }),
        display,
        lights,
        terrain: None,
        world: frame,
        suspension,
    })
}

/// Builds the host for scenario `id`. Runs on the sim thread; `progress` names each step.
pub fn build_host(id: &str, firmware: &str, progress: &mut dyn FnMut(&str)) -> Result<Built, String> {
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
    let suspension = files.body.suspension.as_ref().and_then(|s| {
        let geometry = source.read(&wheel.id, "geometry.toml").ok()?;
        shock_eyes(&geometry).map(|(eye_top_m, eye_bottom_m)| SuspensionInfo {
            linkage_ratio: s.linkage_ratio,
            eye_top_m,
            eye_bottom_m,
        })
    });
    let loaded = super::images::load(&files, firmware, progress)?;
    let (image, label, map) = (&loaded.image, loaded.label.clone(), loaded.map.as_ref());

    // The settings journal the wheel boots with ("blank" or none: an empty journal).
    let board = match scenario.settings.as_deref() {
        None | Some("blank") => files.board.clone(),
        Some(preset) => files.board_with_settings(preset).map_err(|e| e.to_string())?,
    };

    progress("Building the simulated controller");
    let factory = loaded.factory()?;
    let plant = PlantState::default();
    let config = MachineConfig {
        board: &board,
        stockmap: map,
        image,
        load_addr: wheel.board.app.base,
        hclk: files.hclk(),
        plant,
        injections: scenario.inject.iter().map(|i| (i.at_s, i.bytes.bytes().collect())).collect(),
        power_cycles: Vec::new(),
        cpi: euc_sim::DEFAULT_CPI,
        exception_timing: loaded.exception_timing(),
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    let terrain = terrain_profile(&scenario);
    let live = files.live_for(&scenario).map_err(|e| format!("Invalid physical model: {e}"))?;
    let machine = Machine::new_live(&config, &euc_sim::registry(), factory, live).map_err(|e| e.to_string())?;
    let session = Session::new(machine, map, SAMPLE_HZ);
    let world = session
        .machine()
        .plant()
        .live_part()
        .and_then(|live| live.physics().setup().world.as_ref().map(super::world::WorldFrame::of));
    let sha256 = Sha256::digest(image);
    let toml = scenarios::text(id);
    let mut header = euc_sim::recording::header(
        concat!("EUC Studio ", env!("CARGO_PKG_VERSION")),
        &source,
        &files,
        &session,
        &label,
        &sha256,
    );
    header.cpu = Some(loaded.cpu_name().into());
    header.cpi = loaded.cpi();
    header.scenario = Some(scenario.name.clone());
    header.scenario_toml = toml;
    header.settings = scenario.settings.clone().filter(|p| p != "blank");
    // The model and aids the session's physics actually runs with.
    header.context = Some(euc_sim::recording::RecordingContext::scenario(&session).to_json());
    Ok(Built {
        wheel: wheel.id.clone(),
        header: Some(header),
        host: Box::new(RideHost::new(session, scenario)?),
        label: format!("{} firmware", firmware_name(firmware)),
        build: label,
        note,
        display,
        lights,
        terrain,
        world,
        suspension,
    })
}

/// The firmware choice as the status bar names it: `ours-custom` without its quirk list.
fn firmware_name(firmware: &str) -> &str {
    if super::images::custom_quirks(firmware).is_some() { super::images::OURS_CUSTOM } else { firmware }
}

/// The ground the scenario rides on, as points for the web view to draw (none when flat): every
/// 5 cm from 5 m behind the start, `[x, z, slope z]` in metres, where `slope z` is the slope alone
/// (no bumps or potholes), so the view can lay its shadows on the slope and mark the potholes.
pub(super) fn terrain_profile(scenario: &Scenario) -> Option<Vec<[f32; 3]>> {
    let terrain = &scenario.terrain;
    if terrain.is_flat() {
        return None;
    }
    let mut slope = terrain.clone();
    slope.features.clear();
    let step = 0.05;
    let samples = ((terrain.length_m + 5.0) / step) as usize;
    Some(
        (0..=samples)
            .map(|i| {
                let x = -5.0 + i as f64 * step;
                [x as f32, terrain.height(x.max(0.0)) as f32, slope.height(x.max(0.0)) as f32]
            })
            .collect(),
    )
}

/// The shock's eyes from `geometry.toml` `[shock]` (`eye_top_m`, `eye_bottom_m`), if it has them.
pub(super) fn shock_eyes(geometry: &str) -> Option<([f64; 3], [f64; 3])> {
    let table: toml::Table = toml::from_str(geometry).ok()?;
    let shock = table.get("shock")?;
    let eye = |key: &str| -> Option<[f64; 3]> {
        let v: Vec<f64> = shock.get(key)?.as_array()?.iter().map(toml::Value::as_float).collect::<Option<_>>()?;
        v.try_into().ok()
    };
    Some((eye("eye_top_m")?, eye("eye_bottom_m")?))
}

/// The current pinned stock build, from the cache or fetched into it.
pub(super) fn stock_image(wheel: &WheelProfile, progress: &mut dyn FnMut(&str)) -> Result<(Vec<u8>, String), String> {
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

    use super::super::cpu;
    use super::super::host::Host;
    use super::super::phone::{PhoneDecoder, PhoneFrame};
    use super::*;

    #[test]
    #[ignore = "requires verified cached stock firmware; creates a physical model without running it"]
    fn scenario_capture_records_the_actual_rider_model_and_external_aids() {
        let files = WheelFiles::load(&ProfileSource::Builtin, "begode-extreme").unwrap();
        let pinned = files
            .profile
            .firmware
            .iter()
            .find_map(|source| match source {
                FirmwareSource::BegodeApi(build) if build.channel == Channel::Current => Some(build),
                _ => None,
            })
            .unwrap();
        let root = std::env::var_os("EUC_CACHE_DIR").map(Into::into).or_else(FirmwareCache::default_root).unwrap();
        assert!(FirmwareCache::new(root).verified(&pinned.sha256).is_some(), "verified cache required; no download");
        let imported = scenarios::import(
            r#"
schema = 1
name = "recording-context-regression"
description = "Verify recorded model metadata"
wheel = "begode-extreme"
seconds = 1.0
release_s = 0.8
speed_kmh = [[0.0, 0.0]]
[rider]
model = "articulated_sagittal_v1"
[assist]
roll_stiffness = 123.0
roll_damping = 45.0
"#
            .into(),
        )
        .unwrap();
        let built = build_host(&imported.id, "stock", &mut |_| {}).unwrap();
        let header = built.header.unwrap();
        let context = euc_sim::recording::RecordingContext::of(&header).unwrap().expect("scenario context");
        assert_eq!(context.rider_model.id(), "articulated_sagittal_v1");
        assert_eq!(context.run_mode, euc_sim::recording::RunMode::Scenario);
        assert_eq!(context.model_aids.roll_stiffness, 123.0);
        assert_eq!(context.model_aids.roll_damping, 45.0);
        assert!(context.height_map.is_none());
    }

    #[test]
    #[ignore = "requires verified cached stock firmware; creates a physical model without running it"]
    fn a_world_scenario_tells_the_view_its_world_and_records_it() {
        let imported =
            scenarios::import(include_str!("../../../../scenarios/world/world-pothole-15kmh.toml").into()).unwrap();
        let built = build_host(&imported.id, "stock", &mut |_| {}).unwrap();
        let frame = built.world.expect("the view draws the scenario's world");
        assert_eq!(frame.spec.seed, 42);
        assert_eq!(frame.start_xy_m, [70.0, 3.05]);
        assert_eq!(frame.start_heading_rad, 0.0);
        assert!(built.terrain.is_none(), "no 1-D strip under a world");
        let context = euc_sim::recording::RecordingContext::of(&built.header.unwrap()).unwrap().unwrap();
        let world = context.world.expect("the recording keeps the world");
        assert_eq!((world.spec, world.start_xy_m), (frame.spec, frame.start_xy_m));
    }

    #[test]
    fn the_terrain_profile_keeps_the_ground_below_zero() {
        // The shipped scenarios ride the city now; synthetic strips as a user's file has them.
        let strip = |terrain: &str| {
            Scenario::from_toml(&format!(
                "schema = 1\nname = \"strip\"\ndescription = \"strip\"\nwheel = \"begode-extreme\"\nseconds = 10.0\n\
                 release_s = 2.5\nspeed_kmh = [[0, 0]]\n{terrain}"
            ))
            .unwrap()
        };
        assert_eq!(terrain_profile(&strip("")), None, "flat: nothing to draw");
        let at = |profile: &[[f32; 3]], x: f32| *profile.iter().find(|p| (p[0] - x).abs() < 0.01).unwrap();
        // 2 % down from 25 m (a 10 m ramp): at 100 m the ground is 1.4 m below the start.
        let down =
            terrain_profile(&strip("[terrain]\nslope_pct = -2.0\nslope_start_m = 25.0\nlength_m = 250.0\n")).unwrap();
        let [_, z, slope] = at(&down, 100.0);
        assert!((z + 1.4).abs() < 1e-4 && z == slope, "{z} {slope}");
        // The pothole is 2 cm below the slope; the slope itself stays at 0 there.
        let hole = terrain_profile(&strip(
            "[terrain]\nlength_m = 120.0\n[[terrain.feature]]\nkind = \"pothole\"\nat_m = 50.0\ndepth_m = 0.02\nlength_m = 0.5\n",
        ))
        .unwrap();
        // A city scenario draws its world instead of a strip.
        assert_eq!(terrain_profile(&scenarios::load("pothole-20kmh").unwrap()), None);
        let [_, z, slope] = at(&hole, 50.25);
        assert!((z + 0.02).abs() < 1e-4 && slope == 0.0, "{z} {slope}");
        assert_eq!(at(&hole, 49.0)[1..], [0.0, 0.0]);
    }

    #[test]
    fn the_extreme_profile_gives_the_shock_eyes() {
        let geometry = ProfileSource::Builtin.read("begode-extreme", "geometry.toml").unwrap();
        let (top, bottom) = shock_eyes(&geometry).expect("the Extreme has a shock");
        assert!(top[2] > bottom[2], "the top eye is above the bottom one");
        assert_eq!(shock_eyes("[seat]\nheight_m = 0.47"), None, "no [shock], no eyes");
    }

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
        let mut built = build_host("balance-60s", "stock", &mut |_| {}).expect("host");
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

    /// Bounded, read-only trace of the stock startup request and its TIM4/GPIOB writes.
    /// Keeps firmware and device behavior unchanged; run explicitly with --ignored --nocapture.
    #[test]
    #[ignore = "needs the cached stock image and traces four simulated seconds"]
    fn stock_startup_buzzer_register_trace() {
        use euc_core::{Bus, BusFault, CoreRegs, Cpu, Exit, Size};
        use std::cell::RefCell;
        use std::rc::Rc;

        #[derive(Debug)]
        struct Access {
            cycle: u32,
            pc: u32,
            address: u32,
            value: u32,
            ticks: u32,
            volume_compare: u32,
        }
        struct TraceBus<'a> {
            bus: &'a mut dyn Bus,
            accesses: &'a RefCell<Vec<Access>>,
            pc: u32,
        }
        impl TraceBus<'_> {
            fn capture(&mut self, address: u32, value: u32) {
                if self.accesses.borrow().len() >= 512 {
                    return;
                }
                let ticks = self.bus.read(0x2000_0292, Size::Half).unwrap();
                let volume_compare = self.bus.read(0x2000_0296, Size::Half).unwrap();
                let cycle = self.bus.read(0xe000_1004, Size::Word).unwrap();
                self.accesses.borrow_mut().push(Access { cycle, pc: self.pc, address, value, ticks, volume_compare });
            }
        }
        impl Bus for TraceBus<'_> {
            fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
                self.bus.read(addr, size)
            }
            fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
                if (0x4000_0800..0x4000_0850).contains(&addr)
                    || (0x4000_2800..0x4000_2828).contains(&addr)
                    || addr == 0x4002_1020
                    || addr == 0x4001_0c04
                    || ((0x4001_0c0c..=0x4001_0c14).contains(&addr) && value & 0x0200_0200 != 0)
                {
                    self.capture(addr, value);
                }
                self.bus.write(addr, size, value)
            }
            fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
                self.bus.fetch(addr)
            }
            fn take_yield(&mut self) -> bool {
                self.bus.take_yield()
            }
            fn exception_return(&mut self, regs: &mut CoreRegs, value: u32) {
                self.bus.exception_return(regs, value);
            }
            fn note_pc(&mut self, pc: u32) {
                self.pc = pc;
                self.bus.note_pc(pc);
            }
            fn reached(&mut self, addr: u32) {
                if [0x0800_ae68, 0x0800_ae70, 0x0800_73ac].contains(&addr) {
                    self.pc = addr;
                    self.capture(addr, 0);
                }
                self.bus.reached(addr);
            }
        }
        struct TraceCpu {
            cpu: Box<dyn Cpu>,
            accesses: Rc<RefCell<Vec<Access>>>,
        }
        impl Cpu for TraceCpu {
            fn reset(&mut self, sp: u32, vector: u32) {
                self.cpu.reset(sp, vector);
            }
            fn run(&mut self, bus: &mut dyn Bus, budget: u32) -> Exit {
                self.cpu.run(&mut TraceBus { bus, accesses: &self.accesses, pc: 0 }, budget)
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
            fn watch(&mut self, addresses: &[u32]) {
                let mut all = addresses.to_vec();
                if !all.is_empty() {
                    all.extend([0x0800_ae68, 0x0800_ae70, 0x0800_73ac]);
                }
                self.cpu.watch(&all);
            }
            fn invalidate(&mut self, addr: u32, len: u32) {
                self.cpu.invalidate(addr, len);
            }
            fn set_ccr(&mut self, ccr: u32) {
                self.cpu.set_ccr(ccr);
            }
        }

        let files = WheelFiles::load(&ProfileSource::Builtin, "begode-extreme").unwrap();
        let loaded = super::super::images::load(&files, "stock", &mut |_| {}).unwrap();
        let board = files.board_with_settings("neutral").unwrap();
        let accesses = Rc::new(RefCell::new(Vec::new()));
        let captured = Rc::clone(&accesses);
        let factory = cpu::factory().unwrap();
        let config = MachineConfig {
            board: &board,
            stockmap: Some(&files.stockmap),
            image: &loaded.image,
            load_addr: files.profile.board.app.base,
            hclk: files.hclk(),
            plant: PlantState::default(),
            injections: Vec::new(),
            power_cycles: Vec::new(),
            cpi: euc_sim::DEFAULT_CPI,
            exception_timing: euc_sim::ExceptionTiming::default(),
            max_quantum: 4000,
            marker_window_s: 4.0,
        };
        let live = files.live_on(Default::default(), Default::default(), Default::default()).unwrap();
        let mut machine = Machine::new_live(
            &config,
            &euc_sim::registry(),
            Box::new(move |regions| {
                Ok(Box::new(TraceCpu { cpu: factory(regions)?, accesses: captured }) as Box<dyn Cpu>)
            }),
            live,
        )
        .unwrap();
        let mut previous = None;
        for _ in 0..4000 {
            assert!(machine.run(files.hclk() / 1000).is_none());
            let half = |addr| u16::from_le_bytes(machine.memory(addr, 2).unwrap().try_into().unwrap());
            let state = (half(0x2000_0292), half(0x2000_0296));
            if previous != Some(state) {
                println!(
                    "t={:.6} ticks={} volume_compare={}",
                    machine.cycles as f64 / files.hclk() as f64,
                    state.0,
                    state.1
                );
                previous = Some(state);
            }
        }
        for access in accesses.borrow().iter() {
            println!(
                "t={:.6} pc={:#010x} address={:#010x} value={} ticks={} volume_compare={}",
                f64::from(access.cycle) / files.hclk() as f64,
                access.pc,
                access.address,
                access.value,
                access.ticks,
                access.volume_compare
            );
        }
        let buzzer = machine.component::<euc_devices::Buzzer>("buzzer").unwrap();
        println!("stock {}: {} buzzer changes, {} onsets", loaded.label, buzzer.changes(), buzzer.beeps());
        assert!(
            accesses.borrow().iter().any(|access| access.pc == 0x0800_ae70
                && access.address == 0x4000_0840
                && access.value == 151
                && access.ticks == 100
                && access.volume_compare == 151),
            "RTC initializes volume before the actual stock startup CCR4 store"
        );
        assert_eq!(buzzer.beeps(), 1, "stock emits one real startup pulse without input");
    }

    #[test]
    #[ignore = "needs cached stock firmware and local compat and clean builds"]
    fn firmware_startup_emits_actual_pwm() {
        let mut observed = Vec::new();
        for firmware in ["stock", "ours-compat", "ours-clean"] {
            let mut built = build_host("balance-60s", firmware, &mut |_| {}).expect("firmware host");
            let mut rig = Rig::new(built.host.as_mut());
            rig.run(3.0);
            let mut tones = Vec::new();
            rig.host.drain_tones(&mut tones);
            println!("{firmware} startup tones {tones:?}");
            assert!(rig.host.halted().is_none());
            observed.push((firmware, tones));
        }
        for (firmware, tones) in &observed {
            assert_eq!(tones.len(), 2, "{firmware}: one startup pulse, without input");
            let (on, off) = (tones[0], tones[1]);
            assert!((on.hz - 72_000_000.0 / 40.0 / 667.0).abs() < 1e-6, "{firmware}: timer carrier");
            assert!((on.duty - 151.0 / 667.0).abs() < 1e-9, "{firmware}: neutral volume 6");
            assert_eq!(off.hz, 0.0, "{firmware}: startup PWM stops");
            assert!((0.19..0.22).contains(&(off.t_s - on.t_s)), "{firmware}: 100 control ticks");
        }
        let stock = &observed[0].1;
        let compat = &observed[1].1;
        for (stock_edge, compat_edge) in stock.iter().zip(compat) {
            assert!(
                (stock_edge.t_s - compat_edge.t_s).abs() < 0.002016,
                "compat startup edge within one control tick of stock"
            );
        }
    }

    /// The poses the 3D view draws (a guard for the drawing: a changed body tree shows here
    /// first): wheel, shell, then the articulated rider's 7 bodies, the upper body ("rider", its
    /// origin at the pelvis) and each leg's thigh, shank and foot; the wheel's centre one rolling
    /// radius above the ground, the shell's origin (the axle at full extension) the sag below it,
    /// the pelvis above the axle and the feet below it, every body drawn from its own geoms.
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn live_poses_stand_on_the_ground() {
        let mut built = build_host("balance-60s", "stock", &mut |_| {}).expect("host");
        let host = built.host.as_mut();
        let bodies = host.bodies();
        let roles: Vec<_> = bodies.iter().map(|b| b.role).collect();
        assert_eq!(roles[..2], [BodyRole::Wheel, BodyRole::Shell]);
        assert_eq!(roles[2..], [BodyRole::Rider; 7]);
        let names: Vec<&str> = bodies[2..].iter().map(|b| b.name.as_str()).collect();
        assert_eq!(
            names,
            ["rider", "left_thigh", "left_shank", "left_foot", "right_thigh", "right_shank", "right_foot"]
        );
        assert!(bodies[2..].iter().all(|b| !b.geoms.is_empty()), "every body has a shape to draw");
        while host.time_s() < 1.0 {
            host.step(1.0 / 120.0);
        }
        let mut snap = Snapshot::default();
        host.snapshot(&mut snap);
        assert_eq!(snap.bodies.len(), 9, "a pose per body");
        let [wheel, shell, pelvis, left_foot] = [0, 1, 2, 5].map(|i| snap.bodies[i].pos[2]);
        println!("at 1 s: wheel z {wheel:.4} m, shell z {shell:.4} m, pelvis z {pelvis:.4} m, foot z {left_foot:.4} m");
        assert!((wheel - 0.225).abs() < 0.005, "the wheel centre rides one rolling radius up: {wheel}");
        assert!((0.0..0.02).contains(&(wheel - shell)), "the shell sits the sag below the axle: {shell}");
        assert!(pelvis > wheel + 0.5 && left_foot < pelvis, "the rider stands above the axle: {pelvis}, {left_foot}");
        // The foot's origin is the ankle: the deck top (12 mm above the axle at full extension in
        // plant.toml) plus the rider's 80 mm ankle height, on the welded pedal.
        let ankle = shell + 0.012 + 0.08;
        assert!((left_foot - ankle).abs() < 0.005, "the left ankle stands on its pedal: {left_foot} against {ankle}");
    }

    /// The whole ride, as the studio steps it: the scenario's checks pass.
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn stock_firmware_rides_to_20_kmh_and_stops() {
        let mut built = build_host("ride-20kmh-stop", "stock", &mut |_| {}).expect("host");
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

    /// The real firmware counts Hall edges, transmits metres over BLE, and writes only tenths
    /// of a kilometre to the TM1640. Check both sides of the first visible display increment.
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn stock_odometer_counts_distance_and_updates_the_display() {
        let files = WheelFiles::load(&ProfileSource::Builtin, "begode-extreme").unwrap();
        let loaded = super::super::images::load(&files, "stock", &mut |_| {}).unwrap();
        let board = files.board_with_settings("neutral").unwrap();
        let scenario = Scenario::from_toml(
            r#"schema = 1
name = "odometer-check"
description = "Ride beyond the first 100 m odometer display step."
wheel = "begode-extreme"
seconds = 50.0
release_s = 2.5
speed_kmh = [[0, 0], [6, 0], [11, 20]]
[[expect]]
check = "upright"
[[expect]]
check = "no_cutoff"
"#,
        )
        .unwrap();
        let config = MachineConfig {
            board: &board,
            stockmap: loaded.map.as_ref(),
            image: &loaded.image,
            load_addr: files.profile.board.app.base,
            hclk: files.hclk(),
            plant: PlantState::default(),
            injections: Vec::new(),
            power_cycles: Vec::new(),
            cpi: euc_sim::DEFAULT_CPI,
            exception_timing: euc_sim::ExceptionTiming::default(),
            max_quantum: 4000,
            marker_window_s: 4.0,
        };
        let live = files.live_on(scenario.rider, scenario.assist, scenario.terrain.clone()).unwrap();
        let machine = Machine::new_live(&config, &euc_sim::registry(), cpu::factory().unwrap(), live).unwrap();
        let session = Session::new(machine, loaded.map.as_ref(), SAMPLE_HZ);
        let mut host = RideHost::new(session, scenario).unwrap();
        let mut rig = Rig::new(&mut host);
        let spec = euc_display::Display::parse(&ProfileSource::Builtin.read("begode-extreme", "display.toml").unwrap())
            .unwrap();
        let mut snap = Snapshot::default();
        let mut readings = Vec::new();
        for _ in 0..50 {
            rig.run(1.0);
            rig.host.snapshot(&mut snap);
            let Some(frame) = rig.frames.iter().rev().find(|f| f.kind == "status") else { continue };
            let metres: u32 = frame
                .fields
                .iter()
                .find(|(key, _)| key == "Odometer")
                .unwrap()
                .1
                .trim_end_matches(" m")
                .parse()
                .unwrap();
            let outputs = rig.host.outputs().unwrap();
            let grid: [u8; 16] = outputs.display_grid.try_into().unwrap();
            readings.push((snap.t_s, snap.signals.distance_m, metres, spec.text(&grid, "odometer")));
        }
        for reading in readings.iter().step_by(5).chain(readings.last()) {
            println!(
                "time {:.1} s, physical {:.1} m, BLE {} m, display {}",
                reading.0, reading.1, reading.2, reading.3
            );
        }
        assert!(rig.host.halted().is_none());
        assert!(readings.windows(2).all(|pair| pair[1].2 >= pair[0].2), "the lifetime counter never falls");
        assert!(
            readings
                .iter()
                .any(|(_, distance, metres, text)| *distance > 20.0 && (20..90).contains(metres) && text == "00000.0")
        );
        let (_, distance, metres, text) = readings.last().unwrap();
        assert!(*distance > 150.0, "the test must ride beyond one display step: {distance}");
        assert!(*metres >= 150, "the firmware must count the ride: {metres}");
        assert_eq!(text, &format!("{:05}.{}", metres / 1000, (metres / 100) % 10));
        assert_ne!(text, "00000.0", "the display must advance after 100 m");
    }

    /// The stock firmware holds 15 km/h on the smooth tire with the torque its rolling resistance
    /// and the air ask for. Over 12.5-15 s of `rider-steps-off-15kmh` the wheel torque averages
    /// 9.4 N·m, 3.8 of it `crr · N · r` at crr 0.014. The ring of 48 capsules the first mesh
    /// replaced took 10.3 N·m at a third of that rolling resistance (docs/models/body.md, "Tire");
    /// a tire that drags again shows here. The torque, not the current, is the check: it does not
    /// depend on the motor model.
    #[test]
    #[ignore = "needs the stock image in the local firmware cache"]
    fn stock_firmware_holds_15_kmh_on_the_smooth_tire() {
        let mut built = build_host("rider-steps-off-15kmh", "stock", &mut |_| {}).expect("host");
        let mut rig = Rig::new(built.host.as_mut());
        rig.run(12.5);
        let mut snap = Snapshot::default();
        let (mut speed, mut iq, mut torque) = (0.0, 0.0, 0.0);
        let samples = 300;
        for _ in 0..samples {
            rig.run(1.0 / 120.0);
            rig.host.snapshot(&mut snap);
            speed += f64::from(snap.signals.speed_kmh);
            iq += f64::from(snap.signals.iq_a);
            torque += f64::from(snap.signals.torque_nm);
        }
        let [speed, iq, torque] = [speed, iq, torque].map(|sum| sum / f64::from(samples));
        println!("12.5-15 s: {speed:.2} km/h on {torque:.2} N·m, iq {iq:.2} A");
        assert!(rig.host.halted().is_none());
        assert!((14.0..15.5).contains(&speed), "speed {speed}");
        assert!(torque < 10.5, "{torque} N·m to hold {speed} km/h: the tire drags");
    }
}

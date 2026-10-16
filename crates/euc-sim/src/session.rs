//! A running simulation with a live plant: what the studio, recordings and scenarios drive.
//!
//! A [`Session`] owns a [`Machine`] built with a live plant. It advances in fixed sample periods
//! and, after each, reports a [`Snapshot`] (signals and body poses) and any [`Event`]s: stand
//! released, cutoffs, falls, firmware resets and blockers.

use euc_physics::{Intent, Leaving};
use euc_telemetry::{BodyInfo, Event, EventKind, Signals, Snapshot};

use crate::config::{ROLE_CUTOFF, StockMap};
use crate::machine::Machine;
use crate::plant::Live;

/// The firmware's tiltback offsets (stock map and probe names), degrees, nose up positive: what
/// the rider feels as tiltback when the firmware's map names them.
const TILTBACK_VARS: [&str; 3] = ["tiltback_speed_deg", "tiltback_battery_deg", "tiltback_pwm_deg"];

/// The firmware's commanded q voltage (stock map and probe name), in Q15 of Vbus/√3. Its app
/// frame (Begode frame 07) reports it as PWM: `-vq / 315` in whole percent, so 31 500 is 100 %.
const PWM_VAR: &str = "vq_active";
/// Q15 counts of the firmware's q voltage per percent of its PWM figure.
const Q15_PER_PWM_PERCENT: f64 = 315.0;

/// A firmware variable the session watches.
#[derive(Debug, Clone)]
struct Watched {
    name: String,
    addr: u32,
    size: u32,
    /// The stock map's `kind`: `u8`, `s16`, `f32`...
    kind: String,
}

/// A simulation with a live plant. See the module docs.
pub struct Session {
    machine: Machine,
    hclk: u64,
    sample_cycles: u64,
    bodies: Vec<BodyInfo>,
    cutoff_vars: Vec<Watched>,
    /// Every RAM variable of the stock map, for checks that read one.
    ram: Vec<Watched>,
    cutoff: Option<String>,
    fallen: bool,
    /// The rider's leaving the pedals is on the timeline.
    rider_off_logged: bool,
    seen_events: usize,
    events: Vec<Event>,
}

impl Session {
    /// A session on `machine`, which must have a live plant, sampling at `sample_hz`. With the
    /// stock map of the running build, the session watches the firmware's cutoff flags.
    ///
    /// # Panics
    ///
    /// If the machine's plant is static.
    pub fn new(machine: Machine, stockmap: Option<&StockMap>, sample_hz: f64) -> Self {
        let hclk = machine.hclk();
        let bodies = machine.plant().live_part().expect("a session needs a live plant").physics().bodies();
        let watched = |v: &crate::config::RamVar| Watched {
            name: v.name.clone(),
            addr: v.addr,
            size: v.size,
            kind: v.kind.clone(),
        };
        let cutoff_vars = stockmap
            .map(|map| map.ram.iter().filter(|v| v.role.as_deref() == Some(ROLE_CUTOFF)).map(watched).collect())
            .unwrap_or_default();
        let ram = stockmap.map(|map| map.ram.iter().map(watched).collect()).unwrap_or_default();
        Self {
            machine,
            hclk,
            sample_cycles: ((hclk as f64 / sample_hz).round() as u64).max(1),
            bodies,
            cutoff_vars,
            ram,
            cutoff: None,
            fallen: false,
            rider_off_logged: false,
            seen_events: 0,
            events: Vec::new(),
        }
    }

    /// The machine.
    pub const fn machine(&self) -> &Machine {
        &self.machine
    }

    /// The machine, mutably: to send BLE bytes while the session runs.
    pub const fn machine_mut(&mut self) -> &mut Machine {
        &mut self.machine
    }

    /// Simulated seconds since power-on.
    pub fn time_s(&self) -> f64 {
        self.machine.cycles as f64 / self.hclk as f64
    }

    /// Seconds between two samples.
    pub fn sample_period_s(&self) -> f64 {
        self.sample_cycles as f64 / self.hclk as f64
    }

    /// The bodies the snapshots carry poses for, in order.
    pub fn bodies(&self) -> &[BodyInfo] {
        &self.bodies
    }

    fn live(&self) -> &Live {
        self.machine.plant().live_part().expect("a session has a live plant")
    }

    fn live_mut(&mut self) -> &mut Live {
        self.machine.plant_mut().live_part_mut().expect("a session has a live plant")
    }

    /// Changes what the rider wants.
    pub fn set_intent(&mut self, intent: Intent) {
        self.live_mut().physics_mut().rider().set_intent(intent);
    }

    /// What the rider wants.
    pub fn intent(&mut self) -> Intent {
        self.live_mut().physics_mut().rider().intent()
    }

    /// Pushes the rider along the heading, newtons, until changed (see `Physics::set_push`).
    pub fn set_push(&mut self, newtons: f64) {
        self.live_mut().physics_mut().set_push(newtons);
    }

    /// Lets the wheel off its stand (or puts it back).
    pub fn set_held(&mut self, held: bool) {
        if self.live().physics().held() != held {
            let text = if held { "stand holds the wheel" } else { "stand released" };
            self.push(EventKind::Scenario, text.to_owned());
        }
        self.live_mut().physics_mut().set_held(held);
    }

    /// The rider steps off the pedals (see `euc_physics::Physics::step_off`).
    pub fn step_off(&mut self) {
        if !self.live().physics().rider_off() {
            self.push(EventKind::Scenario, "rider steps off".to_owned());
            self.live_mut().physics_mut().step_off();
        }
    }

    /// Records a scenario step on the timeline.
    pub fn note(&mut self, text: impl Into<String>) {
        self.push(EventKind::Scenario, text.into());
    }

    /// Records an event of any kind on the timeline, at the current time.
    pub fn log(&mut self, kind: EventKind, text: impl Into<String>) {
        self.push(kind, text.into());
    }

    /// The firmware has a cutoff flag set: its name.
    pub fn cutoff(&self) -> Option<&str> {
        self.cutoff.as_deref()
    }

    /// Every cutoff flag the firmware has set now, by stock-map name.
    pub fn cutoff_flags(&self) -> Vec<String> {
        let set =
            |v: &&Watched| self.machine.memory(v.addr, v.size as usize).is_some_and(|b| b.iter().any(|&x| x != 0));
        self.cutoff_vars.iter().filter(set).map(|v| v.name.clone()).collect()
    }

    /// The stock-map RAM variable `name` now, little-endian, as an unsigned number of up to
    /// 8 bytes; `None` without a stock map or for an unknown name.
    pub fn firmware_var(&self, name: &str) -> Option<u64> {
        let var = self.ram.iter().find(|v| v.name == name)?;
        let bytes = self.machine.memory(var.addr, var.size.min(8) as usize)?;
        Some(bytes.iter().rev().fold(0, |value, &b| value << 8 | u64::from(b)))
    }

    /// Names of the firmware's numeric RAM variables (not text), as its map gives them.
    pub fn firmware_var_names(&self) -> Vec<String> {
        self.ram.iter().filter(|v| v.kind != "ascii").map(|v| v.name.clone()).collect()
    }

    /// The stock-map RAM variable `name` now as a number, read as its `kind` says: `s8`, `s16`,
    /// `s32` signed, `f32` floating point, anything else unsigned.
    pub fn firmware_value(&self, name: &str) -> Option<f64> {
        let raw = self.firmware_var(name)?;
        let var = self.ram.iter().find(|v| v.name == name)?;
        let bits = 8 * var.size.min(8);
        Some(match var.kind.as_str() {
            "f32" => f64::from(f32::from_bits(raw as u32)),
            "s8" | "s16" | "s32" => ((raw << (64 - bits)) as i64 >> (64 - bits)) as f64,
            _ => raw as f64,
        })
    }

    /// Runs one sample period and returns the snapshot at its end.
    pub fn step(&mut self) -> Snapshot {
        self.feel_tiltback();
        let cut = self.cutoff.is_some();
        self.live_mut().physics_mut().set_drive_cut(cut);
        self.machine.run(self.sample_cycles);
        self.watch();
        self.snapshot()
    }

    /// Tells the rider the firmware's tiltback, the sum of its nose-up offsets, when the map names
    /// any of them; without them the rider judges the tiltback by the pedals.
    fn feel_tiltback(&mut self) {
        let offsets: Vec<f64> =
            TILTBACK_VARS.iter().filter_map(|v| self.firmware_value(v)).filter(|d| d.is_finite()).collect();
        let felt = (!offsets.is_empty()).then(|| offsets.iter().map(|d| d.max(0.0)).sum::<f64>().to_radians());
        self.live_mut().physics_mut().rider().set_commanded_tiltback(felt);
    }

    /// The state now.
    pub fn snapshot(&self) -> Snapshot {
        let live = self.live();
        let body = live.physics().state();
        let plant = live.electrical().snapshot();
        let phase = plant.phase_current.map(|i| i as f32);
        // As the frame carries it: C integer division (truncation), then the magnitude.
        let pwm = self.firmware_value(PWM_VAR).map_or(f32::NAN, |vq| (-vq / Q15_PER_PWM_PERCENT).trunc().abs() as f32);
        let full_modulation = plant.bus_v / 3f64.sqrt() * Q15_PER_PWM_PERCENT * 100.0 / 32768.0;
        let modulation = plant.voltage_dq.d.hypot(plant.voltage_dq.q) / full_modulation * 100.0;
        let signals = Signals {
            speed_kmh: (body.speed_mps * 3.6) as f32,
            wheel_rad_s: body.wheel_rad_s as f32,
            pwm_pct: pwm,
            phase_a_a: phase[0],
            phase_b_a: phase[1],
            phase_c_a: phase[2],
            // Forward torque positive, like the torque channel (the motor's own q axis may run the
            // other way).
            iq_a: (plant.current_dq.q * f64::from(live.electrical().params().motor.direction)) as f32,
            torque_nm: plant.wheel_torque as f32,
            battery_v: plant.bus_v as f32,
            battery_a: plant.bus_a as f32,
            soc_pct: (plant.soc * 100.0) as f32,
            pitch_deg: body.pitch_rad.to_degrees() as f32,
            roll_deg: body.roll_rad.to_degrees() as f32,
            board_temp_c: plant.stage_c as f32,
            motor_temp_c: plant.winding_c as f32,
            distance_m: body.distance_m as f32,
            rider_lean_deg: body.rider_lean_rad.to_degrees() as f32,
            suspension_mm: (body.suspension_m * 1000.0) as f32,
            shock_mm: (body.shock_m * 1000.0) as f32,
            modulation_pct: if plant.bus_v > 0.0 { modulation as f32 } else { f32::NAN },
            ..Signals::default()
        };
        Snapshot { t_s: self.time_s(), signals, bodies: live.physics().poses() }
    }

    /// Events since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    fn push(&mut self, kind: EventKind, text: String) {
        self.events.push(Event { t_s: self.time_s(), kind, text });
    }

    /// Turns what changed in the last period into events.
    fn watch(&mut self) {
        let cutoff = self.cutoff_vars.iter().find_map(|v| {
            let bytes = self.machine.memory(v.addr, v.size as usize)?;
            bytes.iter().any(|&b| b != 0).then(|| v.name.clone())
        });
        if cutoff != self.cutoff {
            match &cutoff {
                Some(name) => self.push(EventKind::Cutoff, format!("firmware cut the motor ({name})")),
                None => self.push(EventKind::Cutoff, "firmware drives the motor again".to_owned()),
            }
            self.cutoff = cutoff;
        }
        let fallen = self.live().physics().state().fallen;
        if fallen && !self.fallen {
            self.push(EventKind::Fault, "the wheel fell over".to_owned());
        }
        self.fallen = fallen;
        if let Some((t_s, how)) = self.live().physics().left_pedals()
            && !self.rider_off_logged
        {
            self.rider_off_logged = true;
            let text = match how {
                Leaving::SteppedOff => None,
                Leaving::LetGoAfterCut => Some("rider off: lets go of the pedals (drive cut, the wheel tips)"),
                Leaving::LetGoFalling => Some("rider off: lets go of the pedals (the wheel falls)"),
            };
            if let Some(text) = text {
                self.events.push(Event { t_s, kind: EventKind::Fault, text: text.to_owned() });
            }
        }
        let hclk = self.hclk as f64;
        let new: Vec<Event> = self.machine.sys.diag.events[self.seen_events..]
            .iter()
            .filter(|(_, text)| text.starts_with("BLOCKER") || text.contains("RESET") || text.contains("POWER CYCLE"))
            .map(|(cycle, text)| {
                let kind = if text.starts_with("BLOCKER") { EventKind::Fault } else { EventKind::Firmware };
                Event { t_s: *cycle as f64 / hclk, kind, text: text.clone() }
            })
            .collect();
        self.seen_events = self.machine.sys.diag.events.len();
        self.events.extend(new);
    }
}

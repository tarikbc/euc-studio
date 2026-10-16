//! A running simulation with a live plant: what the studio, recordings and scenarios drive.
//!
//! A [`Session`] owns a [`Machine`] built with a live plant. It advances in fixed sample periods
//! and, after each, reports a [`Snapshot`] (signals and body poses) and any [`Event`]s: stand
//! released, cutoffs, falls, firmware resets and blockers.

use euc_physics::Intent;
use euc_telemetry::{BodyInfo, Event, EventKind, Signals, Snapshot};

use crate::config::{ROLE_CUTOFF, StockMap};
use crate::machine::Machine;
use crate::plant::Live;

/// A firmware variable the session watches.
#[derive(Debug, Clone)]
struct Watched {
    name: String,
    addr: u32,
    size: u32,
}

/// A simulation with a live plant. See the module docs.
pub struct Session {
    machine: Machine,
    hclk: u64,
    sample_cycles: u64,
    bodies: Vec<BodyInfo>,
    cutoff_vars: Vec<Watched>,
    cutoff: Option<String>,
    fallen: bool,
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
        let cutoff_vars = stockmap
            .map(|map| {
                map.ram
                    .iter()
                    .filter(|v| v.role.as_deref() == Some(ROLE_CUTOFF))
                    .map(|v| Watched { name: v.name.clone(), addr: v.addr, size: v.size })
                    .collect()
            })
            .unwrap_or_default();
        Self {
            machine,
            hclk,
            sample_cycles: ((hclk as f64 / sample_hz).round() as u64).max(1),
            bodies,
            cutoff_vars,
            cutoff: None,
            fallen: false,
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

    /// Runs one sample period and returns the snapshot at its end.
    pub fn step(&mut self) -> Snapshot {
        self.machine.run(self.sample_cycles);
        self.watch();
        self.snapshot()
    }

    /// The state now.
    pub fn snapshot(&self) -> Snapshot {
        let live = self.live();
        let body = live.physics().state();
        let plant = live.electrical().snapshot();
        let phase = plant.phase_current.map(|i| i as f32);
        let signals = Signals {
            speed_kmh: (body.speed_mps * 3.6) as f32,
            wheel_rad_s: body.wheel_rad_s as f32,
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

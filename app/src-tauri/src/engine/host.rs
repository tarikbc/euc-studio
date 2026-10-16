//! What the runner drives: a deterministic simulation that steps in simulated time.
//!
//! [`Host`] is the seam between the studio and the engine. [`RideHost`] rides a scenario on an
//! [`euc_sim::Session`]: the firmware on the emulated board, the electrical plant and the MuJoCo
//! body. [`StillHost`] runs the firmware with a still plant, for a wheel without a body model.
//! Both read the board the same way ([`BoardWatch`]): BLE frames, boot milestones, display, LEDs
//! and buzzer.

use euc_devices::{Buzzer, Tm1640, ws2812};
use euc_proto_begode::{Assembler, Segment, Telemetry};
use euc_sim::scenario::{Ride, Scenario};
use euc_sim::{Machine, Report, Session};
use euc_stm32f1::Dma;
use euc_telemetry::{BodyInfo, BodyRole, Event, EventKind, Geom, GeomKind, Outputs, Pose, Signals, Snapshot};
use serde::Serialize;

/// A simulation the runner can step. Nothing in it reads the wall clock or blocks. It is built
/// and used on the sim thread only (CPU backends need not be `Send`).
pub trait Host {
    /// Runs `sim_s` more simulated seconds.
    fn step(&mut self, sim_s: f64);
    /// Simulated seconds since power-on.
    fn time_s(&self) -> f64;
    /// The bodies whose poses snapshots carry, in order.
    fn bodies(&self) -> Vec<BodyInfo>;
    /// The current state.
    fn snapshot(&self, out: &mut Snapshot);
    /// Events since the last call.
    fn drain_events(&mut self, out: &mut Vec<Event>);
    /// BLE bytes the firmware sent since the last call, with their simulated time.
    fn drain_ble_tx(&mut self, out: &mut Vec<(f64, u8)>);
    /// Sends bytes to the firmware's BLE module now.
    fn inject_ble(&mut self, bytes: &[u8]) -> Result<(), String>;
    /// Why the run stopped for good, if it did.
    fn halted(&self) -> Option<String>;
    /// What the board shows and sounds (display, LEDs, buzzer), if the host models it.
    fn outputs(&self) -> Option<Outputs> {
        None
    }
    /// Buzzer changes since the last call, at their exact simulated time.
    fn drain_tones(&mut self, _out: &mut Vec<Tone>) {}
}

/// A buzzer change: from `t_s` on, the buzzer sounds `hz` at `duty` of the period high (`hz` 0:
/// silent). The duty sets the loudness; the firmware's beeper volume is the compare value.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Tone {
    pub t_s: f64,
    pub hz: f64,
    pub duty: f64,
}

/// The LED strip's timer period in ticks (TIM3 ARR 29 + 1) and its DMA channel.
const LED_PERIOD: u16 = 30;
const LED_DMA_CHANNEL: u8 = 2;

/// Boot markers are watched for the machine's first few seconds; reports stop after that (a
/// report walks the whole run log, so it gets slower as the run grows).
const REPORT_UNTIL_S: f64 = 6.0;

/// What the studio reads from the board, whichever plant it runs with.
struct BoardWatch {
    hclk: f64,
    ble_read: usize,
    assembler: Assembler,
    /// The firmware's last Live frame.
    live: Option<euc_proto_begode::Live>,
    first_frame: bool,
    events: Vec<Event>,
    reported_markers: usize,
    next_report_s: f64,
    buzzing: bool,
    /// Buzzer changes read so far, and the new ones.
    buzzer_seen: u64,
    tones: Vec<Tone>,
    halted: Option<String>,
}

impl BoardWatch {
    fn new(machine: &Machine) -> Self {
        Self {
            hclk: machine.hclk() as f64,
            ble_read: 0,
            assembler: Assembler::new(),
            live: None,
            first_frame: false,
            events: Vec::new(),
            reported_markers: 0,
            next_report_s: 0.5,
            buzzing: false,
            buzzer_seen: 0,
            tones: Vec::new(),
            halted: None,
        }
    }

    fn time_s(&self, machine: &Machine) -> f64 {
        machine.cycles as f64 / self.hclk
    }

    /// Turns what changed on the board since the last call into events.
    fn after_step(&mut self, machine: &Machine) {
        let now = self.time_s(machine);
        if self.halted.is_none()
            && let Some(why) = machine.halted()
        {
            self.events.push(Event { t_s: now, kind: EventKind::Fault, text: format!("Halted: {why}") });
            self.halted = Some(why.to_owned());
        }
        if now >= self.next_report_s && self.next_report_s <= REPORT_UNTIL_S {
            self.next_report_s = now + 1.0;
            self.collect_report(machine);
        }
        // Every buzzer change at its exact time; a start is worth a timeline entry (the `b` horn
        // beeps twice).
        let Some(buzzer) = machine.component::<Buzzer>("buzzer") else { return };
        for change in buzzer.changes_since(self.buzzer_seen) {
            let t_s = change.at as f64 / self.hclk;
            let (hz, duty) = change.tone.map_or((0.0, 0.0), |pwm| (pwm.frequency_hz(), pwm.duty()));
            if hz > 0.0 && !self.buzzing {
                let text = format!("Beep, {:.1} kHz", hz / 1000.0);
                self.events.push(Event { t_s, kind: EventKind::Firmware, text });
            }
            self.buzzing = hz > 0.0;
            self.tones.push(Tone { t_s, hz, duty });
        }
        self.buzzer_seen = buzzer.changes();
    }

    /// Turns the boot markers the machine logged into events.
    fn collect_report(&mut self, machine: &Machine) {
        let report = Report::new(machine, &[]);
        let mut reached: Vec<_> = report.markers.iter().filter_map(|m| Some((m.first_ms?, m.name.clone()))).collect();
        reached.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (ms, name) in reached.iter().skip(self.reported_markers) {
            self.events.push(Event { t_s: ms / 1000.0, kind: EventKind::Firmware, text: format!("{name}() reached") });
        }
        self.reported_markers = reached.len();
    }

    fn drain_ble_tx(&mut self, machine: &Machine, out: &mut Vec<(f64, u8)>) {
        let tx = &machine.board().ble_tx;
        let new = &tx[self.ble_read.min(tx.len())..];
        self.ble_read = tx.len();
        let bytes: Vec<u8> = new.iter().map(|&(_, b)| b).collect();
        out.extend(new.iter().map(|&(cycle, b)| (cycle as f64 / self.hclk, b)));
        self.assembler.push(&bytes);
        while let Some(segment) = self.assembler.next_segment() {
            if let Segment::Frame(frame) = segment {
                if !self.first_frame {
                    self.first_frame = true;
                    self.events.push(Event {
                        t_s: self.time_s(machine),
                        kind: EventKind::Firmware,
                        text: "First telemetry frame".into(),
                    });
                }
                if let Telemetry::Live(live) = Telemetry::decode(&frame) {
                    self.live = Some(live);
                }
            }
        }
    }

    fn outputs(machine: &Machine) -> Outputs {
        let display = machine.component::<Tm1640>("display");
        let leds = machine
            .component::<Dma>("DMA1")
            .and_then(|dma| dma.memory(LED_DMA_CHANNEL))
            .and_then(|(addr, count)| machine.memory(addr, count as usize))
            .map(|duties| ws2812::decode(duties.iter().map(|&d| u16::from(d)), LED_PERIOD))
            .unwrap_or_default();
        Outputs {
            display_grid: display.map(|d| d.grid().to_vec()).unwrap_or_default(),
            display_on: display.is_some_and(Tm1640::display_on),
            display_brightness: display.map_or(0, Tm1640::brightness),
            leds: leds.iter().map(|c| [c.r, c.g, c.b]).collect(),
            buzzer_hz: machine
                .component::<Buzzer>("buzzer")
                .and_then(Buzzer::tone)
                .map_or(0, |pwm| pwm.frequency_hz().round() as u32),
        }
    }
}

/// A scenario ridden on a session by `euc_sim::scenario::Ride` (the same driver as `euc sim run`):
/// the rider follows the target speed, the stand lets go at the release time, faults and pushes
/// come on cue, and the checks are judged (and posted as events) when the scenario's time is up.
/// The run goes on after that, holding the last target.
pub struct RideHost {
    session: Session,
    scenario: Scenario,
    ride: Ride,
    watch: BoardWatch,
    bodies: Vec<BodyInfo>,
    judged: bool,
}

impl RideHost {
    /// Rides `scenario` on `session` (built for it, at power-on).
    pub fn new(mut session: Session, scenario: Scenario) -> Result<Self, String> {
        let ride = Ride::new(&scenario, &mut session).map_err(|e| format!("scenario {}: {e}", scenario.name))?;
        Ok(Self {
            watch: BoardWatch::new(session.machine()),
            bodies: session.bodies().to_vec(),
            session,
            scenario,
            ride,
            judged: false,
        })
    }

    fn ride_sample(&mut self) {
        self.ride.sample(&self.scenario, &mut self.session);
        self.watch.after_step(self.session.machine());
        if !self.judged && self.ride.finished(&self.scenario, &self.session) {
            self.judge();
        }
    }

    /// Posts the scenario's checks as events.
    fn judge(&mut self) {
        self.judged = true;
        let outcome = self.ride.outcome(&self.scenario);
        let t_s = self.session.time_s();
        for check in &outcome.checks {
            let (kind, verdict) =
                if check.passed { (EventKind::Scenario, "passed") } else { (EventKind::Fault, "failed") };
            self.watch.events.push(Event {
                t_s,
                kind,
                text: format!("Check {verdict}: {} ({})", check.what, check.measured),
            });
        }
        let passed = outcome.checks.iter().filter(|c| c.passed).count();
        let (kind, verdict) =
            if outcome.passed() { (EventKind::Scenario, "passed") } else { (EventKind::Fault, "failed") };
        self.watch.events.push(Event {
            t_s,
            kind,
            text: format!("Scenario {} {verdict}: {passed} of {} checks", self.scenario.name, outcome.checks.len()),
        });
    }
}

impl Host for RideHost {
    fn step(&mut self, sim_s: f64) {
        let period = self.session.sample_period_s();
        let samples = (sim_s / period).round().max(1.0) as u32;
        for _ in 0..samples {
            if self.watch.halted.is_some() {
                return;
            }
            self.ride_sample();
        }
    }

    fn time_s(&self) -> f64 {
        self.session.time_s()
    }

    fn bodies(&self) -> Vec<BodyInfo> {
        self.bodies.clone()
    }

    fn snapshot(&self, out: &mut Snapshot) {
        *out = self.session.snapshot();
        let s = &mut out.signals;
        s.target_speed_kmh = self.scenario.target_kmh(out.t_s) as f32;
    }

    fn drain_events(&mut self, out: &mut Vec<Event>) {
        // The session writes its notes in lower case ("stand released"); the timeline starts
        // every entry with a capital.
        out.extend(self.session.take_events().into_iter().map(|mut e| {
            if let Some(first) = e.text.get(..1) {
                e.text = first.to_uppercase() + &e.text[1..];
            }
            e
        }));
        out.append(&mut self.watch.events);
    }

    fn drain_ble_tx(&mut self, out: &mut Vec<(f64, u8)>) {
        self.watch.drain_ble_tx(self.session.machine(), out);
    }

    fn drain_tones(&mut self, out: &mut Vec<Tone>) {
        out.append(&mut self.watch.tones);
    }

    fn inject_ble(&mut self, bytes: &[u8]) -> Result<(), String> {
        self.session.machine_mut().inject_ble(bytes);
        Ok(())
    }

    fn halted(&self) -> Option<String> {
        self.watch.halted.clone()
    }

    fn outputs(&self) -> Option<Outputs> {
        Some(BoardWatch::outputs(self.session.machine()))
    }
}

/// Axle height of the Extreme's 22-inch tire, metres.
const AXLE_Z: f32 = 0.28;

/// The machine with a still plant: real firmware, the wheel standing upright, readouts from the
/// firmware's own Live frames.
pub struct StillHost {
    machine: Machine,
    watch: BoardWatch,
    battery_v: f64,
}

impl StillHost {
    /// Wraps a built machine; `battery_v` is the still plant's pack voltage.
    pub fn new(machine: Machine, battery_v: f64) -> Self {
        Self { watch: BoardWatch::new(&machine), machine, battery_v }
    }
}

impl Host for StillHost {
    fn step(&mut self, sim_s: f64) {
        if self.watch.halted.is_some() {
            return;
        }
        self.machine.run((sim_s * self.watch.hclk) as u64);
        self.watch.after_step(&self.machine);
    }

    fn time_s(&self) -> f64 {
        self.watch.time_s(&self.machine)
    }

    fn bodies(&self) -> Vec<BodyInfo> {
        vec![
            BodyInfo { name: "wheel".into(), role: BodyRole::Wheel, geoms: vec![tire()] },
            BodyInfo { name: "shell".into(), role: BodyRole::Shell, geoms: Vec::new() },
        ]
    }

    fn snapshot(&self, out: &mut Snapshot) {
        out.t_s = self.time_s();
        out.signals = Signals {
            battery_v: self.battery_v as f32,
            pitch_deg: 0.0,
            roll_deg: 0.0,
            distance_m: 0.0,
            wheel_rad_s: 0.0,
            ..Signals::default()
        };
        if let Some(live) = &self.watch.live {
            out.signals.speed_kmh = live.speed_kmh() as f32;
            out.signals.battery_v = live.voltage_v() as f32;
            out.signals.iq_a = live.phase_current_a() as f32;
            out.signals.board_temp_c = live.temperature_c() as f32;
        }
        let upright = Pose { pos: [0.0, 0.0, AXLE_Z], quat: [1.0, 0.0, 0.0, 0.0] };
        out.bodies.clear();
        out.bodies.extend([upright, upright]);
    }

    fn drain_events(&mut self, out: &mut Vec<Event>) {
        out.append(&mut self.watch.events);
    }

    fn drain_ble_tx(&mut self, out: &mut Vec<(f64, u8)>) {
        self.watch.drain_ble_tx(&self.machine, out);
    }

    fn drain_tones(&mut self, out: &mut Vec<Tone>) {
        out.append(&mut self.watch.tones);
    }

    fn inject_ble(&mut self, bytes: &[u8]) -> Result<(), String> {
        self.machine.inject_ble(bytes);
        Ok(())
    }

    fn halted(&self) -> Option<String> {
        self.watch.halted.clone()
    }

    fn outputs(&self) -> Option<Outputs> {
        Some(BoardWatch::outputs(&self.machine))
    }
}

/// The tire as a torus approximated by a cylinder about the axle (MJCF y axis).
fn tire() -> Geom {
    // A cylinder's axis is local z; rotate it onto y (90° about x).
    let half = std::f32::consts::FRAC_1_SQRT_2;
    Geom { kind: GeomKind::Cylinder, size: [AXLE_Z, 0.04, 0.0], pos: [0.0; 3], quat: [half, half, 0.0, 0.0] }
}

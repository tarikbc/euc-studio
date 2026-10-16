//! The simulation thread and the stream pump.
//!
//! The sim thread owns the [`Host`] (built inside it, so the CPU backend never crosses a
//! thread), paces it against wall time at the requested real-time factor and drops every
//! snapshot, event and phone frame into an [`Outbox`]. The pump thread empties the outbox about
//! 60 times a second and sends it to the web view. The two meet only in a mutex held for a copy:
//! a slow web view costs dropped samples (counted in the batch header), never a stalled sim.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use euc_telemetry::batch::BatchWriter;
use euc_telemetry::{BodyInfo, Event, EventKind, Outputs, Snapshot};
use serde::Serialize;
use tauri::ipc::{Channel, InvokeResponseBody};

use super::display::DisplayLayout;
use super::host::{Host, Tone};
use super::lights::LightsLayout;
use super::phone::{PhoneDecoder, PhoneFrame};

/// Physics and firmware advance in steps of this many simulated seconds.
const STEP_S: f64 = 1.0 / 120.0;
/// A snapshot every this many steps (60 Hz).
const SNAPSHOT_EVERY: u32 = 2;
/// Samples the outbox keeps when nobody drains it (10 s).
const OUTBOX_SAMPLES: usize = 600;
/// The sim thread yields to commands at least this often.
const SLICE: Duration = Duration::from_millis(8);
/// How often the pump sends.
const PUMP_PERIOD: Duration = Duration::from_millis(16);
/// Behind by more than this, the sim gives up catching up (the measured factor drops instead).
const MAX_DEBT_S: f64 = 0.25;
/// Simulated seconds between two queued phone commands, as two taps would be.
const COMMAND_GAP_S: f64 = 0.3;
/// How long the queue holds after a command that expects a text answer (`V`, `N`). The stock
/// firmware answers about 0.7 to 1.7 s later and drops the answer when another command arrives
/// first (seen in the simulator), so the next command waits for the answer or this timeout.
const ANSWER_TIMEOUT_S: f64 = 3.0;

/// A byte waiting for its simulated send time.
struct Pending {
    at: f64,
    byte: u8,
    /// The command's label, on its first byte (the timeline shows it when that byte goes out).
    label: Option<String>,
    /// On the last byte of a command that expects a text answer.
    answer: bool,
}

/// Builds a host for a scenario, reporting progress (`"Fetching the stock image"`).
pub type HostFactory = Box<dyn Fn(&str, &mut dyn FnMut(&str)) -> Result<Built, String> + Send>;

/// A host and what the status bar says about it.
pub struct Built {
    pub host: Box<dyn Host>,
    /// What runs, for the status bar ("Stock firmware, no physics yet").
    pub label: String,
    /// Firmware build label.
    pub build: String,
    /// A note that stays in the status (for example what is not modelled yet).
    pub note: Option<String>,
    /// The wheel's display replica, if the profile describes it.
    pub display: Option<DisplayLayout>,
    /// The tail light's LED layout, if the profile describes it.
    pub lights: Option<LightsLayout>,
    /// The ground along the path, `[x, z, slope z]` metres, when it is not flat (`terrain_profile`).
    pub terrain: Option<Vec<[f32; 3]>>,
    /// The rear suspension's shock, for drawing it compressed; none on a wheel without one.
    pub suspension: Option<SuspensionInfo>,
}

/// The wheel's rear shock as the 3D view draws it: where its eyes are and how axle travel maps to
/// shock travel. From the profile: `plant.toml` `[body.suspension]` and `geometry.toml` `[shock]`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SuspensionInfo {
    /// Axle travel per unit of shock travel.
    pub linkage_ratio: f64,
    /// The eye that stays with the frame, wheel frame (x forward, y left, z up), metres.
    pub eye_top_m: [f64; 3],
    /// The eye that the linkage moves.
    pub eye_bottom_m: [f64; 3],
}

#[derive(Debug, Clone, Serialize)]
pub struct Status {
    /// `starting`, `running`, `paused`, `halted`, `error`.
    pub state: &'static str,
    pub message: String,
    /// What runs ("Stock firmware, no physics yet").
    pub label: Option<String>,
    pub build: Option<String>,
    pub backend: &'static str,
    pub scenario: String,
    pub t_s: f64,
    /// Measured sim seconds per wall second over the last second.
    pub rtf: f64,
    pub target_rtf: f64,
    pub playing: bool,
}

/// What the pump sends on the JSON channel.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Message {
    Status(Status),
    /// A new run started: forget the old history.
    Reset {
        scenario: String,
    },
    Bodies {
        bodies: Vec<BodyInfo>,
    },
    /// The display replica (sent with the bodies; `null` when the wheel has none).
    Display {
        layout: Option<DisplayLayout>,
    },
    /// The tail light's LED layout (sent with the bodies; `null` when the wheel has none).
    Lights {
        layout: Option<LightsLayout>,
    },
    /// The ground's height along the path (sent with the bodies; `null` on flat ground).
    Terrain {
        profile: Option<Vec<[f32; 3]>>,
    },
    /// The rear shock (sent with the bodies; `null` on a wheel without suspension).
    Suspension {
        info: Option<SuspensionInfo>,
    },
    Events {
        events: Vec<Event>,
    },
    Frames {
        frames: Vec<PhoneFrame>,
    },
    Outputs {
        outputs: Outputs,
    },
    /// Buzzer changes at their simulated times, for the web view to play.
    Buzzer {
        tones: Vec<Tone>,
    },
}

enum Cmd {
    /// Repeat the reset, bodies and status messages for a web view that just attached.
    Announce,
    Play(bool),
    Step,
    Rtf(f64),
    Restart(Option<String>),
    Send {
        bytes: Vec<(f64, u8)>,
        label: String,
        answer: bool,
    },
    Stop,
}

#[derive(Default)]
struct Outbox {
    writer: Option<BatchWriter>,
    messages: Vec<Message>,
}

pub struct Sinks {
    pub stream: Channel<InvokeResponseBody>,
    pub messages: Channel<Message>,
}

/// A running engine: the sim thread and the pump.
pub struct Runner {
    tx: Sender<Cmd>,
    sinks: Arc<Mutex<Option<Sinks>>>,
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Runner {
    pub fn start(factory: HostFactory, scenario: String, backend: &'static str) -> Self {
        let (tx, rx) = mpsc::channel();
        let outbox = Arc::new(Mutex::new(Outbox::default()));
        let sinks: Arc<Mutex<Option<Sinks>>> = Arc::new(Mutex::new(None));
        let sim = {
            let outbox = Arc::clone(&outbox);
            thread::Builder::new()
                .name("euc-sim".into())
                .spawn(move || Sim::new(factory, scenario, backend, outbox).run(&rx))
                .expect("spawn the sim thread")
        };
        let stop = Arc::new(AtomicBool::new(false));
        let pump = {
            let (outbox, sinks, stop) = (Arc::clone(&outbox), Arc::clone(&sinks), Arc::clone(&stop));
            thread::Builder::new()
                .name("euc-pump".into())
                .spawn(move || pump(&outbox, &sinks, &stop))
                .expect("spawn the pump")
        };
        Self { tx, sinks, stop, threads: vec![sim, pump] }
    }

    /// Where the pump sends from now on (a reloaded web view attaches again).
    pub fn attach(&self, sinks: Sinks) {
        *lock(&self.sinks) = Some(sinks);
    }

    /// Asks the sim to describe the current run again (bodies, status) for a new web view.
    pub fn announce(&self) {
        let _ = self.tx.send(Cmd::Announce);
    }

    pub fn play(&self, on: bool) {
        let _ = self.tx.send(Cmd::Play(on));
    }

    pub fn step(&self) {
        let _ = self.tx.send(Cmd::Step);
    }

    pub fn set_rtf(&self, rtf: f64) {
        let _ = self.tx.send(Cmd::Rtf(rtf.clamp(0.05, 8.0)));
    }

    pub fn restart(&self, scenario: Option<String>) {
        let _ = self.tx.send(Cmd::Restart(scenario));
    }

    /// Queues bytes for the firmware's BLE input, each after its gap in simulated seconds;
    /// `answer` holds the queue after them until the firmware answers (see [`ANSWER_TIMEOUT_S`]).
    pub fn send(&self, bytes: Vec<(f64, u8)>, label: String, answer: bool) {
        let _ = self.tx.send(Cmd::Send { bytes, label, answer });
    }
}

impl Drop for Runner {
    fn drop(&mut self) {
        let _ = self.tx.send(Cmd::Stop);
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

fn pump(outbox: &Mutex<Outbox>, sinks: &Mutex<Option<Sinks>>, stop: &AtomicBool) {
    while !stop.load(Ordering::Relaxed) {
        thread::sleep(PUMP_PERIOD);
        let (batch, messages) = {
            let mut out = lock(outbox);
            (out.writer.as_mut().and_then(BatchWriter::take), std::mem::take(&mut out.messages))
        };
        let sinks = lock(sinks);
        let Some(s) = sinks.as_ref() else { continue };
        // Messages that reset or describe bodies go before the samples that depend on them.
        let (early, late): (Vec<_>, Vec<_>) = messages.into_iter().partition(|m| {
            matches!(
                m,
                Message::Reset { .. }
                    | Message::Bodies { .. }
                    | Message::Display { .. }
                    | Message::Lights { .. }
                    | Message::Terrain { .. }
                    | Message::Suspension { .. }
            )
        });
        for m in early {
            let _ = s.messages.send(m);
        }
        if let Some(bytes) = batch {
            let _ = s.stream.send(InvokeResponseBody::Raw(bytes));
        }
        for m in late {
            let _ = s.messages.send(m);
        }
    }
}

struct Sim {
    factory: HostFactory,
    scenario: String,
    backend: &'static str,
    outbox: Arc<Mutex<Outbox>>,
    host: Option<Box<dyn Host>>,
    build: Option<String>,
    label: Option<String>,
    note: Option<String>,
    display: Option<DisplayLayout>,
    lights: Option<LightsLayout>,
    terrain: Option<Vec<[f32; 3]>>,
    suspension: Option<SuspensionInfo>,
    error: Option<String>,
    outputs: Option<Outputs>,
    outputs_at: Option<Instant>,
    playing: bool,
    rtf: f64,
    debt: f64,
    last: Instant,
    steps: u32,
    snapshot: Snapshot,
    events: Vec<Event>,
    ble: Vec<(f64, u8)>,
    tones: Vec<Tone>,
    frames: Vec<PhoneFrame>,
    phone: PhoneDecoder,
    /// Bytes waiting for their simulated send time; a command's first byte carries its label.
    pending: VecDeque<Pending>,
    /// Holding the queue for an answer until this simulated time.
    awaiting: Option<f64>,
    window: VecDeque<(Instant, f64)>,
    last_status: Instant,
}

impl Sim {
    fn new(factory: HostFactory, scenario: String, backend: &'static str, outbox: Arc<Mutex<Outbox>>) -> Self {
        Self {
            factory,
            scenario,
            backend,
            outbox,
            host: None,
            build: None,
            label: None,
            note: None,
            display: None,
            lights: None,
            terrain: None,
            suspension: None,
            error: None,
            outputs: None,
            outputs_at: None,
            playing: true,
            rtf: 1.0,
            debt: 0.0,
            last: Instant::now(),
            steps: 0,
            snapshot: Snapshot::default(),
            events: Vec::new(),
            ble: Vec::new(),
            tones: Vec::new(),
            frames: Vec::new(),
            phone: PhoneDecoder::default(),
            pending: VecDeque::new(),
            awaiting: None,
            window: VecDeque::new(),
            last_status: Instant::now(),
        }
    }

    fn post(&self, message: Message) {
        lock(&self.outbox).messages.push(message);
    }

    fn status(&self, state: &'static str, message: String) -> Status {
        let t_s = self.host.as_ref().map_or(0.0, |h| h.time_s());
        Status {
            state,
            message,
            label: self.label.clone(),
            build: self.build.clone(),
            backend: self.backend,
            scenario: self.scenario.clone(),
            t_s,
            rtf: self.measured_rtf(),
            target_rtf: self.rtf,
            playing: self.playing,
        }
    }

    fn post_status(&mut self) {
        self.last_status = Instant::now();
        let status = match (&self.error, &self.host) {
            (Some(e), _) => self.status("error", e.clone()),
            (None, None) => self.status("starting", "Starting the simulator".into()),
            (None, Some(h)) => match h.halted() {
                Some(why) => self.status("halted", why),
                None if self.playing => self.status("running", self.note.clone().unwrap_or_default()),
                None => self.status("paused", self.note.clone().unwrap_or_default()),
            },
        };
        self.post(Message::Status(status));
    }

    fn measured_rtf(&self) -> f64 {
        match (self.window.front(), self.window.back()) {
            (Some(a), Some(b)) if b.0 > a.0 => {
                let wall = b.0.duration_since(a.0).as_secs_f64();
                if wall > 0.2 { (b.1 - a.1) / wall } else { 0.0 }
            }
            _ => 0.0,
        }
    }

    /// Builds the host for the current scenario, posting progress as status.
    fn build(&mut self) {
        self.host = None;
        self.error = None;
        let outbox = Arc::clone(&self.outbox);
        let (scenario, backend) = (self.scenario.clone(), self.backend);
        let mut progress = |step: &str| {
            lock(&outbox).messages.push(Message::Status(Status {
                state: "starting",
                message: step.to_owned(),
                label: None,
                build: None,
                backend,
                scenario: scenario.clone(),
                t_s: 0.0,
                rtf: 0.0,
                target_rtf: 1.0,
                playing: false,
            }));
        };
        match (self.factory)(&self.scenario, &mut progress) {
            Ok(built) => {
                let bodies = built.host.bodies();
                {
                    let mut out = lock(&self.outbox);
                    out.writer = Some(BatchWriter::new(bodies.len(), OUTBOX_SAMPLES));
                    out.messages.push(Message::Reset { scenario: self.scenario.clone() });
                    out.messages.push(Message::Bodies { bodies });
                    out.messages.push(Message::Display { layout: built.display.clone() });
                    out.messages.push(Message::Lights { layout: built.lights.clone() });
                    out.messages.push(Message::Terrain { profile: built.terrain.clone() });
                    out.messages.push(Message::Suspension { info: built.suspension.clone() });
                }
                self.build = Some(built.build);
                self.label = Some(built.label);
                self.note = built.note;
                self.display = built.display;
                self.lights = built.lights;
                self.terrain = built.terrain;
                self.suspension = built.suspension;
                self.outputs = None;
                self.host = Some(built.host);
                self.phone = PhoneDecoder::default();
                self.pending.clear();
                self.awaiting = None;
                self.window.clear();
                self.debt = 0.0;
                self.steps = 0;
                self.last = Instant::now();
                self.sample();
            }
            Err(e) => self.error = Some(e),
        }
        self.post_status();
    }

    fn run(mut self, rx: &Receiver<Cmd>) {
        self.build();
        loop {
            let running = self.playing && self.host.as_ref().is_some_and(|h| h.halted().is_none());
            let wait = if running { Duration::ZERO } else { Duration::from_millis(50) };
            loop {
                match rx.recv_timeout(wait) {
                    Ok(Cmd::Stop) | Err(RecvTimeoutError::Disconnected) => return,
                    Ok(cmd) => self.handle(cmd),
                    Err(RecvTimeoutError::Timeout) => break,
                }
                if !running {
                    break;
                }
            }
            if running {
                self.advance();
            } else {
                self.last = Instant::now();
            }
            if self.last_status.elapsed() >= Duration::from_millis(250) {
                self.post_status();
            }
        }
    }

    fn handle(&mut self, cmd: Cmd) {
        match cmd {
            Cmd::Announce => {
                if let Some(h) = &self.host {
                    let bodies = h.bodies();
                    self.post(Message::Reset { scenario: self.scenario.clone() });
                    self.post(Message::Bodies { bodies });
                    self.post(Message::Display { layout: self.display.clone() });
                    self.post(Message::Lights { layout: self.lights.clone() });
                    self.post(Message::Terrain { profile: self.terrain.clone() });
                    self.post(Message::Suspension { info: self.suspension.clone() });
                }
                self.outputs = None;
                self.post_status();
            }
            Cmd::Play(on) => {
                if on != self.playing {
                    self.playing = on;
                    self.window.clear();
                    self.post_status();
                }
            }
            Cmd::Step => {
                if !self.playing {
                    for _ in 0..SNAPSHOT_EVERY {
                        self.step_once();
                    }
                    self.post_status();
                }
            }
            Cmd::Rtf(rtf) => {
                self.rtf = rtf;
                self.window.clear();
                self.post_status();
            }
            Cmd::Restart(scenario) => {
                if let Some(s) = scenario {
                    self.scenario = s;
                }
                self.build();
            }
            Cmd::Send { bytes, label, answer } => {
                let now = self.host.as_ref().map_or(0.0, |h| h.time_s());
                let mut at = now.max(self.pending.back().map_or(now, |p| p.at + COMMAND_GAP_S));
                let mut label = Some(label);
                let last = bytes.len().saturating_sub(1);
                for (i, (gap, byte)) in bytes.into_iter().enumerate() {
                    at += gap;
                    self.pending.push_back(Pending { at, byte, label: label.take(), answer: answer && i == last });
                }
            }
            Cmd::Stop => {}
        }
    }

    fn advance(&mut self) {
        let now = Instant::now();
        self.debt += now.duration_since(self.last).as_secs_f64().min(0.1) * self.rtf;
        self.last = now;
        let mut stepped = false;
        while self.debt >= STEP_S {
            self.step_once();
            self.debt -= STEP_S;
            stepped = true;
            if now.elapsed() >= SLICE {
                break;
            }
        }
        if self.debt > MAX_DEBT_S {
            self.debt = 0.0;
        }
        if let Some(h) = &self.host {
            self.window.push_back((Instant::now(), h.time_s()));
            while self.window.len() > 2 && self.window.front().is_some_and(|f| f.0.elapsed() > Duration::from_secs(1)) {
                self.window.pop_front();
            }
        }
        if !stepped {
            // Ahead of the wall clock: sleep until the next step is due.
            let due = (STEP_S - self.debt) / self.rtf;
            thread::sleep(Duration::from_secs_f64(due.clamp(0.0005, 0.004)));
        }
    }

    fn step_once(&mut self) {
        let Some(host) = self.host.as_mut() else { return };
        let t = host.time_s();
        if self.awaiting.is_some_and(|until| t >= until) {
            Self::release(&mut self.pending, &mut self.awaiting, t);
        }
        while self.awaiting.is_none()
            && let Some(p) = self.pending.pop_front_if(|p| p.at <= t)
        {
            if let Some(text) = p.label {
                self.events.push(Event { t_s: t, kind: EventKind::Command, text });
            }
            if let Err(e) = host.inject_ble(&[p.byte]) {
                self.events.push(Event { t_s: t, kind: EventKind::Fault, text: e });
                self.pending.clear();
                break;
            }
            if p.answer {
                self.awaiting = Some(t + ANSWER_TIMEOUT_S);
            }
        }
        host.step(STEP_S);
        self.steps += 1;
        if self.steps.is_multiple_of(SNAPSHOT_EVERY) {
            self.sample();
        }
    }

    /// Ends a hold on the queue at `t`; the commands behind it keep their spacing from now on.
    fn release(pending: &mut VecDeque<Pending>, awaiting: &mut Option<f64>, t: f64) {
        *awaiting = None;
        let late = pending.front().map_or(0.0, |p| (t - p.at).max(0.0));
        for p in pending {
            p.at += late;
        }
    }

    /// Takes a snapshot, events and BLE output and puts them in the outbox.
    fn sample(&mut self) {
        let Some(host) = self.host.as_mut() else { return };
        host.snapshot(&mut self.snapshot);
        host.drain_events(&mut self.events);
        host.drain_ble_tx(&mut self.ble);
        host.drain_tones(&mut self.tones);
        if !self.ble.is_empty() {
            let from = self.frames.len();
            self.phone.push(&self.ble, &mut self.frames);
            self.ble.clear();
            if self.awaiting.is_some() && self.frames[from..].iter().any(|f| f.reply) {
                Self::release(&mut self.pending, &mut self.awaiting, host.time_s());
            }
            for f in &self.frames[from..] {
                if f.reply {
                    self.events.push(Event {
                        t_s: f.t,
                        kind: EventKind::Command,
                        text: format!("Reply: {}", f.summary),
                    });
                }
            }
        }
        let mut out = lock(&self.outbox);
        if let Some(w) = out.writer.as_mut() {
            w.push(&self.snapshot);
        }
        if !self.events.is_empty() {
            out.messages.push(Message::Events { events: std::mem::take(&mut self.events) });
        }
        if !self.frames.is_empty() {
            out.messages.push(Message::Frames { frames: std::mem::take(&mut self.frames) });
        }
        if !self.tones.is_empty() {
            out.messages.push(Message::Buzzer { tones: std::mem::take(&mut self.tones) });
        }
        // Board outputs go out when they change, at most 30 times a second.
        if self.outputs_at.is_none_or(|at| at.elapsed() >= Duration::from_millis(33))
            && let Some(now) = host.outputs()
            && self.outputs.as_ref() != Some(&now)
        {
            self.outputs_at = Some(Instant::now());
            out.messages.push(Message::Outputs { outputs: now.clone() });
            self.outputs = Some(now);
        }
        // A web view that stopped draining must not grow the message list without bound.
        if out.messages.len() > 2000 {
            let excess = out.messages.len() - 2000;
            out.messages.drain(..excess);
        }
    }
}

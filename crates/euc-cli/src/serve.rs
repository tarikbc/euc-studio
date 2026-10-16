//! `euc sim serve` and `euc sim bridge-test`: a wheel at real time with its BLE UART on a TCP
//! socket, and a small client to check it. The protocol is in `docs/bridge.md` and
//! `euc_sim::bridge`.

use std::io::{self, BufRead, BufReader, ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context as _, bail};
use euc_physics::{Assist, Intent, Rider};
use euc_proto_begode::{Assembler, Segment, Telemetry, decode_text};
use euc_sim::bridge::{self, ClientMessage, NeverSend, Notifier, NotifyMode, ServerMessage};
use euc_sim::scenario::{Ride, Scenario};
use euc_sim::{Machine, MachineConfig, PlantState, Session, WheelFiles};
use euc_telemetry::EventKind;

use crate::sim::{BACKEND, firmware, injection, load_scenario};
use crate::ui::{self, BAD, DIM, GOOD, STRONG, WARN};
use crate::{Context, Verdict};

/// `euc sim serve`.
#[derive(Debug, clap::Args)]
pub(crate) struct ServeArgs {
    /// Wheel profile id (optional with --scenario).
    wheel: Option<String>,
    /// Where clients connect: `tcp://HOST:PORT`. Loopback by default; anything else exposes the
    /// simulated wheel to the network.
    #[arg(long, default_value = "tcp://127.0.0.1:7720", value_name = "URL")]
    ble_bridge: String,
    /// Ride this scenario (its rider, aids, BLE commands, faults, speed profile); a client's
    /// `rider` or `push` message takes the rider over.
    #[arg(long, value_name = "FILE")]
    scenario: Option<PathBuf>,
    /// `stock` or a firmware file.
    #[arg(long, default_value = "stock")]
    firmware: String,
    /// Simulated seconds per wall second.
    #[arg(long, default_value_t = 1.0)]
    rate: f64,
    /// How UART bytes become notifications: `split` (the module's 20-byte packets) or `burst`.
    #[arg(long, default_value = "split", value_parser = ["split", "burst"])]
    notify: String,
    /// Without a scenario: when the stand lets the wheel go, simulated seconds.
    #[arg(long, default_value_t = 2.5)]
    release_s: f64,
    /// Stop after this many simulated seconds (default: run until interrupted).
    #[arg(long)]
    seconds: Option<f64>,
}

/// `euc sim bridge-test`.
#[derive(Debug, clap::Args)]
pub(crate) struct BridgeTestArgs {
    /// The server.
    #[arg(long, default_value = "tcp://127.0.0.1:7720", value_name = "URL")]
    url: String,
    /// Wall seconds to listen.
    #[arg(long, default_value_t = 6.0)]
    seconds: f64,
    /// Bytes to write, one at a time, 1 s apart from 1 s on (the wheel takes one command byte at
    /// a time).
    #[arg(long, default_value = "VN")]
    send: String,
}

/// Simulated seconds between two state messages.
const STATE_EVERY_S: f64 = 0.2;
/// Samples per simulated second: the granularity of pacing and rider control.
const SAMPLE_HZ: f64 = 200.0;
/// Wall seconds `bridge-test` keeps trying to connect.
const CONNECT_WAIT_S: u64 = 20;

fn address(url: &str) -> anyhow::Result<&str> {
    url.strip_prefix("tcp://").with_context(|| format!("\"{url}\" is not tcp://HOST:PORT"))
}

/// One connected client: lines in, lines out, never blocking the simulation.
struct Client {
    stream: TcpStream,
    inbox: Vec<u8>,
    outbox: Vec<u8>,
}

impl Client {
    fn send(&mut self, message: &ServerMessage) {
        if let Ok(line) = serde_json::to_vec(message) {
            self.outbox.extend_from_slice(&line);
            self.outbox.push(b'\n');
        }
    }

    /// Writes what the socket takes now. `Err` when the client is gone.
    fn flush(&mut self) -> io::Result<()> {
        while !self.outbox.is_empty() {
            match self.stream.write(&self.outbox) {
                Ok(0) => return Err(ErrorKind::WriteZero.into()),
                Ok(n) => {
                    self.outbox.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Complete lines received so far. `Err` when the client is gone.
    fn lines(&mut self) -> io::Result<Vec<String>> {
        let mut buf = [0u8; 4096];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => return Err(ErrorKind::UnexpectedEof.into()),
                Ok(n) => self.inbox.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        let mut lines = Vec::new();
        while let Some(end) = self.inbox.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.inbox.drain(..=end).collect();
            lines.push(String::from_utf8_lossy(&line).trim().to_owned());
        }
        Ok(lines)
    }
}

/// Who rides: the scenario until a client takes over.
enum Driver {
    Scenario(Box<Scenario>, Ride),
    Manual { release_s: f64 },
}

/// The served wheel and its one client.
struct Server {
    session: Session,
    driver: Driver,
    notifier: Notifier,
    guard: NeverSend,
    client: Option<Client>,
    hello: ServerMessage,
    hclk: f64,
    push_until: f64,
    next_state: f64,
    rate_mark: (Instant, f64),
}

/// `euc sim serve`.
pub(crate) fn serve(ctx: &Context, args: &ServeArgs) -> anyhow::Result<Verdict> {
    if !(args.rate > 0.0 && args.rate.is_finite()) {
        bail!("--rate must be positive");
    }
    let mut server = Server::new(ctx, args)?;
    let addr = address(&args.ble_bridge)?;
    let listener = TcpListener::bind(addr).with_context(|| format!("cannot listen on {addr}"))?;
    listener.set_nonblocking(true)?;
    let local = listener.local_addr()?;
    if !local.ip().is_loopback() {
        ui::warn(format!("listening on {local}: anyone on the network can drive this simulated wheel"));
    }
    ui::row(STRONG, "bridge", format!("tcp://{local}, {} notifications, {}x real time", args.notify, args.rate));

    let started = Instant::now();
    let sim0 = server.session.time_s();
    loop {
        server.accept(&listener)?;
        server.read();
        server.advance_to(sim0 + started.elapsed().as_secs_f64() * args.rate);
        server.publish();
        let t = server.session.time_s();
        if let Some(why) = server.session.machine().halted() {
            ui::row(BAD, "HALTED", ui::safe(why));
            return Ok(Verdict::Refused);
        }
        if args.seconds.is_some_and(|limit| t >= limit) {
            ui::row(GOOD, "done", format!("{t:.1} s simulated"));
            return Ok(Verdict::Ok);
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

impl Server {
    /// Builds the wheel with a live plant, on its stand at power-on.
    fn new(ctx: &Context, args: &ServeArgs) -> anyhow::Result<Self> {
        let Some(backend) = BACKEND else {
            bail!("this euc has no CPU backend; run `cargo run -p euc-cli --features unicorn -- sim serve ...`");
        };
        let scenario = args.scenario.as_deref().map(load_scenario).transpose()?;
        let wheel_id = match (&args.wheel, &scenario) {
            (Some(wheel), _) => wheel.clone(),
            (None, Some(scenario)) => scenario.wheel.clone(),
            (None, None) => bail!("name a wheel profile, or give a --scenario"),
        };
        let files = WheelFiles::load(&ctx.profiles(), &wheel_id)?;
        let wheel = &files.profile;
        let (image, label) = firmware(ctx, wheel, &args.firmware)?;
        let sha256 = euc_profile::Sha256::digest(&image);
        let stock = wheel.find_pin(&sha256).is_some_and(|pin| pin.label == files.stockmap.build);
        let injections = scenario
            .iter()
            .flat_map(|s| &s.inject)
            .map(|inject| injection(&format!("{}@{}", inject.bytes, inject.at_s), 0))
            .collect::<anyhow::Result<Vec<_>>>()?;
        let config = MachineConfig {
            board: &files.board,
            stockmap: stock.then_some(&files.stockmap),
            image: &image,
            load_addr: wheel.board.app.base,
            hclk: files.hclk(),
            plant: PlantState::default(),
            injections,
            power_cycles: Vec::new(),
            cpi: 1.25,
            max_quantum: 4000,
            marker_window_s: 4.0,
        };
        let (rider, assist) = scenario.as_ref().map_or((Rider::default(), Assist::default()), |s| (s.rider, s.assist));
        let machine = Machine::new_live(&config, &euc_sim::registry(), backend(false), files.live(rider, assist)?)?;
        let mut session = Session::new(machine, stock.then_some(&files.stockmap), SAMPLE_HZ);
        let driver = match scenario {
            Some(scenario) => {
                let ride = Ride::new(&scenario, &mut session)?;
                Driver::Scenario(Box::new(scenario), ride)
            }
            None => Driver::Manual { release_s: args.release_s },
        };
        ui::row(STRONG, "wheel", format!("{} ({}), firmware {label}", wheel.name, wheel.id));
        let mode = if args.notify == "burst" { NotifyMode::Burst } else { NotifyMode::Split };
        let hello = ServerMessage::Hello {
            version: bridge::VERSION,
            wheel: wheel.id.clone(),
            firmware: label,
            notify: mode.name().to_owned(),
            rate: args.rate,
            t: 0.0,
        };
        let now = session.time_s();
        Ok(Self {
            session,
            driver,
            notifier: Notifier::new(mode, files.hclk()),
            guard: NeverSend::default(),
            client: None,
            hello,
            hclk: files.hclk() as f64,
            push_until: f64::NAN,
            next_state: 0.0,
            rate_mark: (Instant::now(), now),
        })
    }

    /// Takes a waiting client if none is connected, and greets it.
    fn accept(&mut self, listener: &TcpListener) -> anyhow::Result<()> {
        if self.client.is_some() {
            return Ok(());
        }
        match listener.accept() {
            Ok((stream, peer)) => {
                stream.set_nonblocking(true)?;
                stream.set_nodelay(true)?;
                let t = self.session.time_s();
                ui::row(GOOD, "client", format!("{peer} connected at {t:.2} s"));
                let mut client = Client { stream, inbox: Vec::new(), outbox: Vec::new() };
                let mut hello = self.hello.clone();
                if let ServerMessage::Hello { t: at, .. } = &mut hello {
                    *at = t;
                }
                client.send(&hello);
                self.client = Some(client);
                Ok(())
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => Ok(()),
            Err(e) => Err(e).context("accepting a client"),
        }
    }

    fn drop_client(&mut self) {
        ui::row(DIM, "client", format!("disconnected at {:.2} s", self.session.time_s()));
        self.client = None;
    }

    /// Handles every complete line the client sent.
    fn read(&mut self) {
        let lines = match self.client.as_mut().map(Client::lines) {
            Some(Ok(lines)) => lines,
            Some(Err(_)) => {
                self.drop_client();
                return;
            }
            None => return,
        };
        for line in lines.iter().filter(|l| !l.is_empty()) {
            if let Err(text) = self.handle(line) {
                let t = self.session.time_s();
                if let Some(client) = self.client.as_mut() {
                    client.send(&ServerMessage::Log { t, level: "warn".into(), text });
                }
            }
        }
    }

    /// One client message; `Err` says what was wrong with it.
    fn handle(&mut self, line: &str) -> Result<(), String> {
        let session = &mut self.session;
        match serde_json::from_str::<ClientMessage>(line).map_err(|e| format!("cannot read {line:?}: {e}"))? {
            ClientMessage::Write { hex } => {
                let bytes = bridge::from_hex(&hex).ok_or_else(|| format!("bad hex: {hex}"))?;
                for (command, why) in self.guard.check(&bytes) {
                    let text = format!("never-send command \"{command}\" passed through (on a real wheel it {why})");
                    ui::row(WARN, "never-send", &text);
                    session.log(EventKind::Command, text);
                }
                session.machine_mut().inject_ble(&bytes);
            }
            ClientMessage::Rider { kmh } if kmh.is_finite() => {
                self.driver = Driver::Manual { release_s: 0.0 };
                session.set_intent(Intent::Speed(kmh / 3.6));
                session.note(format!("client: target {kmh:.1} km/h"));
            }
            ClientMessage::Push { newtons, seconds } if newtons.is_finite() && seconds > 0.0 => {
                self.driver = Driver::Manual { release_s: 0.0 };
                session.set_push(newtons);
                session.note(format!("client: push {newtons:.0} N for {seconds:.1} s"));
                self.push_until = session.time_s() + seconds;
            }
            _ => return Err(format!("out of range: {line}")),
        }
        Ok(())
    }

    /// Runs whole sample periods up to simulated time `target`.
    fn advance_to(&mut self, target: f64) {
        let session = &mut self.session;
        while session.time_s() + session.sample_period_s() <= target && session.machine().halted().is_none() {
            let t = session.time_s();
            if t >= self.push_until {
                session.set_push(0.0);
                self.push_until = f64::NAN;
            }
            match &mut self.driver {
                Driver::Scenario(scenario, ride) if !ride.finished(scenario, session) => {
                    ride.sample(scenario, session);
                }
                Driver::Scenario(..) => {
                    session.step();
                }
                Driver::Manual { release_s } => {
                    if t >= *release_s && session.machine().plant().live_part().is_some_and(|l| l.physics().held()) {
                        session.set_held(false);
                    }
                    session.step();
                }
            }
        }
    }

    /// Sends what the firmware said, what happened and the state; drops a client that is gone.
    fn publish(&mut self) {
        let mut out = Vec::new();
        for (cycle, byte) in self.session.machine_mut().take_ble_tx() {
            out.extend(self.notifier.push(cycle, byte));
        }
        out.extend(self.notifier.idle(self.session.machine().cycles));
        let mut messages: Vec<ServerMessage> = out
            .into_iter()
            .map(|(cycle, bytes)| ServerMessage::Notify { t: cycle as f64 / self.hclk, hex: bridge::to_hex(&bytes) })
            .collect();
        for event in self.session.take_events() {
            let warn = matches!(event.kind, EventKind::Cutoff | EventKind::Fault);
            if warn {
                ui::row(WARN, "event", format!("{:.2} s {}", event.t_s, ui::safe(&event.text)));
            }
            let level = if warn { "warn" } else { "info" };
            messages.push(ServerMessage::Log { t: event.t_s, level: level.into(), text: event.text });
        }
        let t = self.session.time_s();
        if t >= self.next_state {
            self.next_state = t + STATE_EVERY_S;
            let s = self.session.snapshot().signals;
            let rate = (t - self.rate_mark.1) / self.rate_mark.0.elapsed().as_secs_f64().max(1e-9);
            self.rate_mark = (Instant::now(), t);
            messages.push(ServerMessage::State {
                t,
                speed_kmh: s.speed_kmh,
                pitch_deg: s.pitch_deg,
                battery_v: s.battery_v,
                battery_a: s.battery_a,
                rate,
            });
        }
        let Some(client) = self.client.as_mut() else { return };
        for message in &messages {
            client.send(message);
        }
        if client.flush().is_err() {
            self.drop_client();
        }
    }
}

/// What `bridge-test` saw.
#[derive(Default)]
struct Seen {
    hello: Option<ServerMessage>,
    notifications: usize,
    sizes: std::collections::BTreeMap<usize, usize>,
    frames: usize,
    texts: Vec<String>,
    frame00: Option<(f64, f64)>,
    state: Option<ServerMessage>,
    logs: usize,
    assembler: Assembler,
}

impl Seen {
    fn take(&mut self, message: ServerMessage) -> anyhow::Result<()> {
        match message {
            ServerMessage::Hello { .. } => self.hello = Some(message),
            ServerMessage::Notify { hex, .. } => {
                let bytes = bridge::from_hex(&hex).context("bad hex from the server")?;
                self.notifications += 1;
                *self.sizes.entry(bytes.len()).or_insert(0) += 1;
                self.assembler.push(&bytes);
                while let Some(segment) = self.assembler.next_segment() {
                    match segment {
                        Segment::Frame(frame) => {
                            self.frames += 1;
                            if let Telemetry::Live(live) = Telemetry::decode(&frame) {
                                self.frame00 = Some((live.speed_kmh(), live.voltage_v()));
                            }
                        }
                        Segment::Text(text) => self.texts.push(decode_text(&text)),
                    }
                }
            }
            ServerMessage::Log { text, level, t } => {
                self.logs += 1;
                ui::row(if level == "warn" { WARN } else { DIM }, "log", format!("{t:.2} s {}", ui::safe(&text)));
            }
            ServerMessage::State { .. } => self.state = Some(message),
        }
        Ok(())
    }

    fn print(&self) -> anyhow::Result<Verdict> {
        let Some(ServerMessage::Hello { version, wheel, firmware, notify, .. }) = &self.hello else {
            bail!("no hello from the server");
        };
        ui::row(STRONG, "server", format!("protocol {version}, {wheel}, firmware {firmware}, {notify} notifications"));
        let sizes: Vec<String> = self.sizes.iter().map(|(len, n)| format!("{len} B x{n}")).collect();
        ui::row(STRONG, "notify", format!("{} ({})", self.notifications, sizes.join(", ")));
        ui::row(STRONG, "frames", self.frames.to_string());
        if let Some((speed, volts)) = self.frame00 {
            ui::row(STRONG, "frame 00", format!("{speed:.1} km/h, {volts:.1} V"));
        }
        let texts: Vec<String> =
            self.texts.iter().filter(|t| !t.trim().is_empty()).map(|t| ui::quoted(t.trim())).collect();
        ui::row(STRONG, "text", if texts.is_empty() { "none".to_owned() } else { texts.join(" ") });
        if let Some(ServerMessage::State { t, speed_kmh, pitch_deg, rate, .. }) = self.state {
            ui::row(
                STRONG,
                "state",
                format!("{t:.1} s, {speed_kmh:.1} km/h, pitch {pitch_deg:.1}°, {rate:.2}x real time"),
            );
        }
        ui::row(DIM, "logs", self.logs.to_string());
        if self.frames == 0 {
            ui::row(BAD, "FAIL", "no telemetry frames came through the bridge");
            return Ok(Verdict::Refused);
        }
        ui::row(GOOD, "OK", "the bridge carries the wheel's BLE stream");
        Ok(Verdict::Ok)
    }
}

/// `euc sim bridge-test`: connects, writes a few command bytes, and checks what comes back.
pub(crate) fn bridge_test(_ctx: &Context, args: &BridgeTestArgs) -> anyhow::Result<Verdict> {
    let addr = address(&args.url)?;
    // The server may still be starting (building the wheel): try for a few seconds.
    let deadline = Instant::now() + Duration::from_secs(CONNECT_WAIT_S);
    let stream = loop {
        match TcpStream::connect(addr) {
            Ok(stream) => break stream,
            Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(200)),
            Err(e) => return Err(e).with_context(|| format!("cannot connect to {addr}")),
        }
    };
    stream.set_read_timeout(Some(Duration::from_millis(100)))?;
    let mut writer = stream.try_clone()?;
    let mut reader = BufReader::new(stream);
    let started = Instant::now();
    let mut to_send: Vec<u8> = args.send.bytes().rev().collect();
    let mut next_send = 1.0;
    let mut seen = Seen::default();
    let mut line = String::new();
    while started.elapsed().as_secs_f64() < args.seconds {
        if started.elapsed().as_secs_f64() >= next_send
            && let Some(byte) = to_send.pop()
        {
            let message = ClientMessage::Write { hex: bridge::to_hex(&[byte]) };
            writeln!(writer, "{}", serde_json::to_string(&message)?)?;
            ui::row(DIM, "write", ui::quoted(&char::from(byte).to_string()));
            next_send += 1.0;
        }
        // A timeout keeps what it read in `line`; a line is handled once it is complete.
        match reader.read_line(&mut line) {
            Ok(0) => bail!("the server closed the connection"),
            Ok(_) if line.ends_with('\n') => {}
            Ok(_) => continue,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => continue,
            Err(e) => return Err(e.into()),
        }
        let text = std::mem::take(&mut line);
        let message = serde_json::from_str::<ServerMessage>(text.trim())
            .with_context(|| format!("not a bridge message: {}", text.trim()))?;
        seen.take(message)?;
    }
    seen.print()
}

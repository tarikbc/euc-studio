//! `euc replay`: a capture played back as a wheel, on this Mac's Bluetooth radio (`--ble`), on
//! a TCP bridge (`--bridge`, `docs/bridge.md`), or both. Writes get the firmware's answers from
//! the brand's responder (`euc_ble_radio::Replayer`); `docs/ble-radio.md` is the guide.

use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context as _, bail};
use euc_ble_radio::{Action, Control, Entry, Link, Replayer, capture};
use euc_profile::ProfileError;
use euc_sim::bridge::{self, ClientMessage, ServerMessage};

use crate::ble::{self, BleArgs, WheelLink};
use crate::tcp::{self, Client};
use crate::ui::{self, DIM, GOOD, STRONG};
use crate::{Context, Verdict};

/// `euc replay`.
#[derive(Debug, clap::Args)]
pub(crate) struct Args {
    /// Capture file: one notification per line, `<seconds> <hex bytes>`; `#` lines are comments
    /// (docs/ble-radio.md has the optional `# euc-capture v1` header).
    file: PathBuf,
    /// Wheel profile, or radio-only brand profile (`leaperkim`), whose BLE module the replay
    /// presents [default: the capture's `wheel` header, else begode-extreme]
    #[arg(long)]
    wheel: Option<String>,
    #[command(flatten)]
    radio: BleArgs,
    /// Serve the replay on a TCP bridge (docs/bridge.md), with or without --ble, e.g.
    /// `tcp://127.0.0.1:7720`; port 0 takes any free port.
    #[arg(long, value_name = "URL")]
    bridge: Option<String>,
    /// Start again after the last line, until quit.
    #[arg(long = "loop")]
    looping: bool,
    /// Capture time to start at, seconds; the first line at or after it plays at once.
    #[arg(long, default_value_t = 0.0, value_name = "SECONDS")]
    from: f64,
    /// Stop after this many seconds.
    #[arg(long, value_name = "SECONDS")]
    seconds: Option<f64>,
}

/// The wheel a capture without a `wheel` header is replayed as.
const DEFAULT_WHEEL: &str = "begode-extreme";

/// The profile a replay presents: wheel `id`, or, when no such wheel exists, a radio-only profile
/// (a brand's BLE module in `profiles/<id>/ble.toml`, with no simulated wheel behind it).
fn profile_id(ctx: &Context, id: &str) -> anyhow::Result<String> {
    match ctx.profiles().load(id) {
        Ok(wheel) => Ok(wheel.id),
        Err(ProfileError::UnknownWheel { .. }) if ble::has_profile(ctx, id) => Ok(id.to_owned()),
        Err(error) => Err(error.into()),
    }
}

/// How often the replay loop runs. A capture's timestamps have 1 ms steps; every line goes out
/// within a few ms of its time.
const POLL: Duration = Duration::from_millis(2);

/// `euc replay`.
pub(crate) fn run(ctx: &Context, args: &Args) -> anyhow::Result<Verdict> {
    if !args.radio.ble && args.bridge.is_none() {
        bail!("say where to replay: --ble (this Mac's Bluetooth) and/or --bridge tcp://HOST:PORT");
    }
    if !(args.from >= 0.0 && args.from.is_finite()) {
        bail!("--from must be 0 or more seconds");
    }
    if args.seconds.is_some_and(|s| !(s > 0.0 && s.is_finite())) {
        bail!("--seconds must be positive");
    }
    let capture = capture::load(&args.file).with_context(|| format!("cannot read {}", ui::path(&args.file)))?;
    let wheel_id = args.wheel.clone().or_else(|| capture.header.wheel.clone()).unwrap_or_else(|| DEFAULT_WHEEL.into());
    let wheel_id = profile_id(ctx, &wheel_id)?;
    let mut bridge = args.bridge.as_deref().map(|url| Bridge::bind(url, &wheel_id, &args.file)).transpose()?;
    let mut link = WheelLink::start(ctx, &wheel_id, &args.radio)?;
    let profile = link.link.profile();
    let mut replayer = Replayer::new(&capture, profile, ble::responder(&profile.protocol), args.from, args.looping)?;
    let tx = profile.tx();
    let count = replayer.len();
    let laps = if args.looping { ", looping" } else { "" };
    let recorded =
        capture.header.name.as_deref().map(|n| format!(", recorded from {}", ui::quoted(n))).unwrap_or_default();
    ui::row(
        STRONG,
        "replay",
        format!("{}: {count} notifications{recorded}, {:.1} s a lap{laps}", ui::path(&args.file), replayer.period()),
    );
    if replayer.skipped_writes() > 0 {
        ui::row(
            DIM,
            "replay",
            format!("{} captured writes left out: the live central writes for itself", replayer.skipped_writes()),
        );
    }

    let started = Instant::now();
    let mut lap = 0;
    loop {
        let now = started.elapsed().as_secs_f64();
        let mut actions = bridge.as_mut().map(|b| b.poll(&mut link.link, now)).unwrap_or_default();
        actions.extend(link.poll());
        for action in actions {
            match action {
                Action::Write { characteristic, bytes } => replayer.write(now, characteristic, &bytes),
                Action::Quit => return Ok(Verdict::Ok),
            }
        }
        for out in replayer.advance(now) {
            // The bridge protocol carries one characteristic: the wheel's main output.
            if link.link.notify(out.characteristic, &out.bytes)
                && out.characteristic == tx
                && let Some(bridge) = &mut bridge
            {
                bridge.send(&ServerMessage::Notify { t: out.at, hex: bridge::to_hex(&out.bytes) });
            }
        }
        let stale = replayer.take_stale();
        if stale > 0 {
            let text = format!("the replay fell behind (was the program paused?): {stale} stale notifications skipped");
            link.link.log().line(&Entry::Error { text: &text });
        }
        if replayer.lap() != lap {
            lap = replayer.lap();
            link.link.log().line(&Entry::Lap { lap: lap + 1 });
        }
        if let Some(bridge) = &mut bridge {
            bridge.flush();
        }
        if replayer.finished() {
            link.link.log().line(&Entry::End);
            ui::row(GOOD, "done", format!("{count} notifications in {now:.1} s"));
            // Let the radio send what it still holds.
            std::thread::sleep(Duration::from_millis(200));
            return Ok(Verdict::Ok);
        }
        if args.seconds.is_some_and(|limit| now >= limit) {
            ui::row(GOOD, "done", format!("{now:.1} s"));
            return Ok(Verdict::Ok);
        }
        std::thread::sleep(POLL);
    }
}

/// The replay's TCP bridge: one client at a time, as `euc sim serve` does.
struct Bridge {
    listener: TcpListener,
    client: Option<Client>,
    hello: ServerMessage,
}

impl Bridge {
    fn bind(url: &str, wheel: &str, file: &Path) -> anyhow::Result<Self> {
        let addr = tcp::address(url)?;
        let listener = TcpListener::bind(addr).with_context(|| format!("cannot listen on {addr}"))?;
        listener.set_nonblocking(true)?;
        let local = listener.local_addr()?;
        if !local.ip().is_loopback() {
            ui::warn(format!("listening on {local}: anyone on the network can drive this replay"));
        }
        ui::row(STRONG, "bridge", format!("tcp://{local}, split notifications"));
        let name = file.file_name().map_or_else(|| "capture".into(), |n| n.to_string_lossy().into_owned());
        let hello = ServerMessage::Hello {
            version: bridge::VERSION,
            wheel: wheel.to_owned(),
            firmware: format!("replay of {}", ui::safe(&name)),
            notify: "split".into(),
            rate: 1.0,
            t: 0.0,
        };
        Ok(Self { listener, client: None, hello })
    }

    /// Takes a waiting client, and handles what the connected one sent: its writes (passed on,
    /// unless the wheel is off) and controls go through the link.
    fn poll(&mut self, link: &mut Link, now: f64) -> Vec<Action> {
        if self.client.is_none()
            && let Ok((stream, peer)) = self.listener.accept()
            && stream.set_nonblocking(true).is_ok()
        {
            let _ = stream.set_nodelay(true);
            ui::row(GOOD, "client", format!("{peer} connected at {now:.2} s"));
            let mut client = Client::new(stream);
            let mut hello = self.hello.clone();
            if let ServerMessage::Hello { t, .. } = &mut hello {
                *t = now;
            }
            client.send(&hello);
            self.client = Some(client);
        }
        let lines = match self.client.as_mut().map(Client::lines) {
            Some(Ok(lines)) => lines,
            Some(Err(_)) => {
                self.drop_client(now);
                return Vec::new();
            }
            None => return Vec::new(),
        };
        let mut actions = Vec::new();
        for line in lines.iter().filter(|l| !l.is_empty()) {
            if let Err(text) = Self::handle(line, link, &mut actions) {
                self.send(&ServerMessage::Log { t: now, level: "warn".into(), text });
            }
        }
        actions
    }

    /// One client message; `Err` says what was wrong with it.
    fn handle(message: &str, link: &mut Link, actions: &mut Vec<Action>) -> Result<(), String> {
        match serde_json::from_str::<ClientMessage>(message).map_err(|e| format!("cannot read {message:?}: {e}"))? {
            ClientMessage::Write { hex } => {
                let bytes = bridge::from_hex(&hex).ok_or_else(|| format!("bad hex: {hex}"))?;
                if !link.tcp_write(&bytes) {
                    return Err("the wheel is off: write ignored".into());
                }
                actions.push(Action::Write { characteristic: link.profile().rx(), bytes });
            }
            ClientMessage::Control { command } => {
                let control = command.parse::<Control>().map_err(|e| e.to_string())?;
                actions.extend(link.control(control));
            }
            ClientMessage::Rider { .. } | ClientMessage::Push { .. } => {
                return Err("a replay has no rider: rider and push do nothing".into());
            }
        }
        Ok(())
    }

    fn send(&mut self, message: &ServerMessage) {
        if let Some(client) = self.client.as_mut() {
            client.send(message);
        }
    }

    fn flush(&mut self) {
        if self.client.as_mut().is_some_and(|c| c.flush().is_err()) {
            self.client = None;
            ui::row(DIM, "client", "disconnected");
        }
    }

    fn drop_client(&mut self, now: f64) {
        ui::row(DIM, "client", format!("disconnected at {now:.2} s"));
        self.client = None;
    }
}

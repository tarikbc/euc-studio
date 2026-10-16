//! The wheel's BLE link for `euc sim serve` and `euc replay`: the wheel's BLE profile, its
//! module model, the brand's protocol, the radio (`--ble`), the test controls on stdin and the
//! link log. The logic lives in `euc_ble_radio`; `docs/ble-radio.md` is the guide.

use std::io::BufRead as _;
use std::sync::mpsc::{self, Receiver};

use anyhow::{Context as _, bail};
use euc_ble_radio::{
    Action, BleProfile, CharId, Control, Entry, Link, Log, Opaque, Protocol, Radio, Responder, STALE_S, UartBridge,
};
use euc_profile::ProfileError;

use crate::Context;
use crate::ui::{self, STRONG, WARN};

/// Radio options of `sim serve` and `replay`.
#[derive(Debug, Clone, clap::Args)]
pub(crate) struct BleArgs {
    /// Present the wheel on this Mac's Bluetooth radio, as a BLE peripheral phone and watch apps
    /// connect to (macOS only).
    #[arg(long)]
    pub(crate) ble: bool,
    /// Name to advertise, e.g. `GotWay_71105` [default: the wheel profile's, `GotWay_EMU`]
    #[arg(long, value_name = "NAME", requires = "ble")]
    pub(crate) name: Option<String>,
    /// Also log every notification sent and when the radio is busy.
    #[arg(long)]
    pub(crate) verbose: bool,
}

/// The profile file that describes a wheel's BLE module.
const BLE_FILE: &str = "ble.toml";

/// The protocol crates the link knows, by the name a `ble.toml` gives: they find the text
/// replies in the wheel's stream. Adding a brand adds a line here (docs/ble-radio.md).
fn protocol(name: &str) -> Box<dyn Protocol> {
    match name {
        "begode" => Box::new(euc_proto_begode::ble::TextFinder::default()),
        _ => Box::new(Opaque),
    }
}

/// How the firmware of a protocol answers writes, for `euc replay`.
pub(crate) fn responder(name: &str) -> Box<dyn Responder> {
    match name {
        "begode" => Box::new(euc_proto_begode::ble::StockResponder::default()),
        _ => Box::new(Opaque),
    }
}

/// Whether this program has a protocol crate for `name`.
fn known_protocol(name: &str) -> bool {
    name == "begode"
}

/// Wheel `id`'s BLE profile, from its `ble.toml`. A wheel without one gets the common HM-10
/// layout, enough for the log and the TCP bridge; it cannot go on the radio.
fn profile(ctx: &Context, id: &str, args: &BleArgs) -> anyhow::Result<BleProfile> {
    match ctx.profiles().read(id, BLE_FILE) {
        Ok(text) => BleProfile::from_toml(&text).with_context(|| format!("wheel profile \"{id}\"")),
        Err(ProfileError::UnknownFile { .. }) if !args.ble => Ok(BleProfile::hm10("EUC_EMU")),
        Err(error) => Err(error).with_context(|| format!("wheel profile \"{id}\" does not describe its BLE module")),
    }
}

/// The link with its radio and the module model, fed by the controls typed on stdin.
pub(crate) struct WheelLink {
    pub(crate) link: Link,
    /// The module model between the simulated controller's UART and the characteristics, when the
    /// wheel's module is a UART bridge.
    module: Option<UartBridge>,
    controls: Receiver<String>,
}

impl WheelLink {
    /// Starts wheel `id`'s link: with `--ble` the radio (advertising follows once Bluetooth is
    /// on), and the stdin reader.
    pub(crate) fn start(ctx: &Context, id: &str, args: &BleArgs) -> anyhow::Result<Self> {
        let mut profile = profile(ctx, id, args)?;
        if let Some(name) = &args.name {
            profile.advertising.name.clone_from(name);
        }
        profile.validate()?;
        if !profile.protocol.is_empty() && !known_protocol(&profile.protocol) {
            ui::warn(format!(
                "no protocol crate for \"{}\": replies are not logged and a replay does not answer writes",
                profile.protocol
            ));
        }
        let radio = if args.ble {
            if Radio::authorization() == "notDetermined" {
                ui::row(WARN, "bluetooth", "macOS may ask to allow Bluetooth for the app this runs in: click Allow");
            }
            let radio = Radio::start(&profile).context("cannot start the BLE radio")?;
            let name = &profile.advertising.name;
            let services = profile.advertising.services.join(", ");
            ui::row(STRONG, "radio", format!("advertising {} ({services}) on this Mac's Bluetooth", ui::quoted(name)));
            Some(radio)
        } else {
            None
        };
        let names: Vec<&str> = Control::ALL.iter().map(|c| c.name()).collect();
        ui::row(STRONG, "controls", format!("type {} and Enter", names.join(", ")));
        let module = UartBridge::new(&profile);
        let protocol = protocol(&profile.protocol);
        Ok(Self { link: Link::new(profile, protocol, radio, Log::new(args.verbose)), module, controls: stdin_lines() })
    }

    /// The module model, for the live simulator: the wheel's BLE side must be a UART bridge
    /// (a wheel whose BLE chip runs firmware of its own needs that node in the simulator).
    pub(crate) fn require_uart_bridge(&self) -> anyhow::Result<()> {
        if self.module.is_none() {
            bail!("this wheel's BLE chip runs firmware of its own; the simulator does not run that node yet");
        }
        Ok(())
    }

    /// The simulated controller's UART output (`(seconds, byte)`, in order), up to `now`: the
    /// module's notifications go through the link. Returns those that went out, for the TCP bridge.
    pub(crate) fn uart(&mut self, bytes: impl IntoIterator<Item = (f64, u8)>, now: f64) -> Vec<(f64, Vec<u8>)> {
        let Some(module) = self.module.as_mut() else { return Vec::new() };
        for (t, byte) in bytes {
            module.uart(t, byte);
        }
        let mut sent = Vec::new();
        let mut stale = 0;
        for notification in module.poll(now) {
            // After a pause the simulator catches up at once; a real module would never send
            // that much old telemetry in one burst.
            if now - notification.at > STALE_S {
                stale += 1;
            } else if self.link.notify(notification.characteristic, &notification.bytes) {
                sent.push((notification.at, notification.bytes));
            }
        }
        if stale > 0 {
            let text =
                format!("the simulator fell behind (was the program paused?): {stale} stale notifications skipped");
            self.link.log().line(&Entry::Error { text: &text });
        }
        sent
    }

    /// What reaches the controller's UART when a central writes `bytes` on `characteristic`.
    pub(crate) fn to_uart<'a>(&self, characteristic: CharId, bytes: &'a [u8]) -> Option<&'a [u8]> {
        self.module.as_ref()?.write(characteristic, bytes)
    }

    /// Writes from the radio, and the controls typed since the last call, in that order.
    pub(crate) fn poll(&mut self) -> Vec<Action> {
        let mut actions = self.link.poll();
        let lines: Vec<String> = self.controls.try_iter().collect();
        for line in lines.iter().filter(|l| !l.trim().is_empty()) {
            match line.parse::<Control>() {
                Ok(control) => actions.extend(self.link.control(control)),
                Err(error) => self.link.log().line(&Entry::Error { text: &error.to_string() }),
            }
        }
        actions
    }
}

/// Lines typed on stdin, read on a thread so the wheel never waits for them. At end of input
/// (`< /dev/null`, a closed pipe) the thread stops and the link keeps running.
fn stdin_lines() -> Receiver<String> {
    let (sender, receiver) = mpsc::channel();
    let reader = move || {
        for line in std::io::stdin().lock().lines() {
            let Ok(line) = line else { break };
            if sender.send(line).is_err() {
                break;
            }
        }
    };
    if let Err(error) = std::thread::Builder::new().name("stdin controls".into()).spawn(reader) {
        ui::warn(format!("controls on stdin are off: {error}"));
    }
    receiver
}

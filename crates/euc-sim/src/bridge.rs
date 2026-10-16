//! The BLE bridge protocol: the wheel's BLE UART as newline-delimited JSON over a byte stream.
//!
//! `euc sim serve` runs a wheel at real time and lets one client (a BLE peripheral emulator such
//! as EUC Remote's `eucctl emulate`, or a test) see what the firmware sends to its BLE module
//! and write what a phone would write. `docs/bridge.md` is the specification; this module holds
//! the message types and the two pieces of logic a server needs: grouping UART bytes into BLE
//! notifications ([`Notifier`]) and spotting commands a phone must never send to a real wheel
//! ([`NeverSend`]).

use serde::{Deserialize, Serialize};

/// Protocol version in the `hello` message.
pub const VERSION: u32 = 1;

/// Largest notification payload of the wheel's BLE module, bytes.
pub const MODULE_MTU: usize = 20;

/// UART silence that ends a notification, seconds: the module packs the byte stream into
/// 20-byte notifications and sends a shorter one once the line has been idle this long. A lone
/// 24-byte frame goes out as 20 + 4; the stock firmware's bursts of five frames (120 bytes, frames
/// about 2.2 ms apart) go out as six notifications of 20 bytes, as in the capture
/// `tests/data/begode-extreme/idle-2026-10-15.hex`.
pub const IDLE_GAP_S: f64 = 0.001;

/// A line the server sends.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    /// First line after connecting.
    Hello {
        /// [`VERSION`].
        version: u32,
        /// Wheel profile id.
        wheel: String,
        /// Firmware label.
        firmware: String,
        /// How notifications are cut: `split` (the module's 20-byte packets) or `burst`.
        notify: String,
        /// Simulated seconds per wall second the server aims for.
        rate: f64,
        /// Simulated seconds since power-on.
        t: f64,
    },
    /// Bytes the firmware sent, as one BLE notification on FFE1.
    Notify {
        /// Simulated time the last byte left the UART, seconds.
        t: f64,
        /// The bytes, lowercase hex.
        hex: String,
    },
    /// Something happened (cutoff, fault, firmware reset, a never-send command, a bad request).
    Log {
        /// Simulated seconds.
        t: f64,
        /// `info` or `warn`.
        level: String,
        /// What happened.
        text: String,
    },
    /// The simulated wheel's state, a few times per second (not from the firmware: the truth).
    State {
        /// Simulated seconds.
        t: f64,
        /// Ground speed, km/h.
        speed_kmh: f32,
        /// Shell pitch, degrees, nose up positive.
        pitch_deg: f32,
        /// Battery voltage.
        battery_v: f32,
        /// Battery current, discharge positive.
        battery_a: f32,
        /// Simulated seconds per wall second over the last report.
        rate: f64,
    },
}

/// A line the client sends.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ClientMessage {
    /// A phone wrote these bytes to FFE1: they go to the firmware's BLE UART input.
    Write {
        /// The bytes, hex.
        hex: String,
    },
    /// The rider should ride at this speed (km/h; 0 = stand still).
    Rider {
        /// Target ground speed, km/h.
        kmh: f64,
    },
    /// Push the rider (newtons forward, negative = backward) for a while.
    Push {
        /// Force, newtons.
        newtons: f64,
        /// How long, seconds.
        seconds: f64,
    },
    /// A test control for the wheel's BLE link (`docs/ble-radio.md`): `silent`, `resume`, `off`,
    /// `on` or `quit`.
    Control {
        /// The control's word.
        command: String,
    },
}

/// How UART bytes become notifications.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotifyMode {
    /// As the wheel's BLE module does: at most [`MODULE_MTU`] bytes, cut at an idle line.
    Split,
    /// One notification per burst (bytes separated by less than [`IDLE_GAP_S`]), any length.
    Burst,
}

impl NotifyMode {
    /// The name in `hello`.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Split => "split",
            Self::Burst => "burst",
        }
    }
}

/// Groups the bytes the firmware sends into BLE notifications.
#[derive(Debug, Clone)]
pub struct Notifier {
    mode: NotifyMode,
    gap: u64,
    pending: Vec<u8>,
    last: u64,
}

impl Notifier {
    /// A notifier for an engine of `hclk` cycles per second.
    pub fn new(mode: NotifyMode, hclk: u64) -> Self {
        Self { mode, gap: (IDLE_GAP_S * hclk as f64) as u64, pending: Vec::new(), last: 0 }
    }

    /// Adds a byte that started on the wire at `cycle`; returns the notifications it completes,
    /// each with the cycle of its last byte.
    pub fn push(&mut self, cycle: u64, byte: u8) -> Vec<(u64, Vec<u8>)> {
        let mut out = Vec::new();
        if !self.pending.is_empty() && cycle.saturating_sub(self.last) > self.gap {
            out.push((self.last, std::mem::take(&mut self.pending)));
        }
        self.pending.push(byte);
        self.last = cycle;
        if self.mode == NotifyMode::Split && self.pending.len() == MODULE_MTU {
            out.push((cycle, std::mem::take(&mut self.pending)));
        }
        out
    }

    /// The pending notification, if the line has been idle long enough at `now`.
    pub fn idle(&mut self, now: u64) -> Option<(u64, Vec<u8>)> {
        (!self.pending.is_empty() && now.saturating_sub(self.last) > self.gap)
            .then(|| (self.last, std::mem::take(&mut self.pending)))
    }
}

/// Spots commands a phone must never send to a real wheel, across writes.
#[derive(Debug, Clone, Default)]
pub struct NeverSend {
    last: Option<u8>,
}

/// Two-byte commands that must never reach a real wheel, and why.
pub const NEVER_SEND: &[(&[u8; 2], &str)] = &[
    (b"+-", "switches the drive mode, saves it to flash and powers the wheel off"),
    (b"cy", "erases the IMU calibration; the wheel recalibrates at the next power-on"),
    (b"!@", "starts a firmware update (OTA)"),
];

impl NeverSend {
    /// Feeds written bytes; returns each never-send command they complete, with why.
    pub fn check(&mut self, bytes: &[u8]) -> Vec<(&'static str, &'static str)> {
        let mut found = Vec::new();
        for &byte in bytes {
            if let Some(prev) = self.last
                && let Some((command, why)) = NEVER_SEND.iter().find(|(c, _)| **c == [prev, byte])
            {
                found.push((std::str::from_utf8(&command[..]).unwrap_or("??"), *why));
            }
            self.last = Some(byte);
        }
        found
    }
}

/// Lowercase hex of `bytes`.
pub fn to_hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    bytes.iter().fold(String::with_capacity(2 * bytes.len()), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// Bytes from hex (either case, no separators).
pub fn from_hex(text: &str) -> Option<Vec<u8>> {
    if !text.len().is_multiple_of(2) {
        return None;
    }
    (0..text.len()).step_by(2).map(|i| u8::from_str_radix(text.get(i..i + 2)?, 16).ok()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const HCLK: u64 = 72_000_000;
    /// One byte time at 115 200 baud, in cycles.
    const BYTE: u64 = 72_000_000 * 10 / 115_200;

    fn frame(notifier: &mut Notifier, start: u64) -> Vec<(u64, Vec<u8>)> {
        let mut out = Vec::new();
        for i in 0..24u8 {
            out.extend(notifier.push(start + u64::from(i) * BYTE, i));
        }
        out
    }

    #[test]
    fn a_frame_goes_out_as_20_plus_4_like_the_module() {
        let mut notifier = Notifier::new(NotifyMode::Split, HCLK);
        let first = frame(&mut notifier, 1000);
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].1, (0..20).collect::<Vec<u8>>());
        assert!(notifier.idle(1000 + 24 * BYTE).is_none(), "the line is not idle yet");
        let (at, rest) = notifier.idle(1000 + 23 * BYTE + 2 * HCLK / 1000).unwrap();
        assert_eq!((at, rest), (1000 + 23 * BYTE, (20..24).collect::<Vec<u8>>()));
        // The next frame after a pause flushes nothing stale.
        assert_eq!(frame(&mut notifier, HCLK).len(), 1);
    }

    #[test]
    fn a_burst_of_frames_packs_into_20_byte_notifications() {
        // The stock firmware starts the frames of a burst 2.22 ms apart; the real module packs
        // them, so five frames make six full notifications.
        let mut notifier = Notifier::new(NotifyMode::Split, HCLK);
        let mut sizes = Vec::new();
        for k in 0..5 {
            sizes.extend(frame(&mut notifier, k * 2220 * HCLK / 1_000_000).into_iter().map(|(_, b)| b.len()));
        }
        sizes.extend(notifier.idle(HCLK).map(|(_, b)| b.len()));
        assert_eq!(sizes, [20; 6]);
    }

    #[test]
    fn burst_mode_keeps_a_frame_whole() {
        let mut notifier = Notifier::new(NotifyMode::Burst, HCLK);
        assert_eq!(frame(&mut notifier, 0).len(), 0);
        let second = frame(&mut notifier, HCLK / 10);
        assert_eq!(second.len(), 1, "the gap before the second frame ends the first");
        assert_eq!(second[0].1.len(), 24);
    }

    #[test]
    fn never_send_commands_are_caught_across_writes() {
        let mut guard = NeverSend::default();
        assert_eq!(guard.check(b"V").len(), 0);
        assert_eq!(guard.check(b"c").len(), 0);
        assert_eq!(guard.check(b"y")[0].0, "cy");
        assert_eq!(guard.check(b"x+-")[0].0, "+-");
        assert_eq!(guard.check(b"!N@").len(), 0);
    }

    #[test]
    fn messages_are_one_json_object_per_line() {
        let hello = ServerMessage::Notify { t: 1.5, hex: to_hex(&[0x55, 0xAA]) };
        assert_eq!(serde_json::to_string(&hello).unwrap(), r#"{"type":"notify","t":1.5,"hex":"55aa"}"#);
        let write: ClientMessage = serde_json::from_str(r#"{"type":"write","hex":"564E"}"#).unwrap();
        assert_eq!(write, ClientMessage::Write { hex: "564E".into() });
        assert_eq!(from_hex("564E"), Some(b"VN".to_vec()));
        assert_eq!(from_hex("5"), None);
        assert!(serde_json::from_str::<ClientMessage>(r#"{"type":"write","bytes":"V"}"#).is_err());
        let control: ClientMessage = serde_json::from_str(r#"{"type":"control","command":"silent"}"#).unwrap();
        assert_eq!(control, ClientMessage::Control { command: "silent".into() });
    }
}

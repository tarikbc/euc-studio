use std::fmt::Write as _;
use std::io::Write as _;
use std::time::Instant;

use crate::Control;

/// Bytes as log text: printable ASCII (space included) as is, every other byte and the
/// backslash as `\xNN`. Anyone in radio range can write to the wheel, so written bytes never
/// reach the terminal raw.
///
/// ```
/// assert_eq!(euc_ble_radio::escape(b"NAME:EXTREME\r\n"), r"NAME:EXTREME\x0d\x0a");
/// ```
pub fn escape(bytes: &[u8]) -> String {
    bytes.iter().fold(String::with_capacity(bytes.len()), |mut out, &byte| {
        if (0x20..0x7F).contains(&byte) && byte != b'\\' {
            out.push(char::from(byte));
        } else {
            let _ = write!(out, "\\x{byte:02x}");
        }
        out
    })
}

/// One line of the link log on stdout: `ble <event> t=<seconds> <fields>`.
///
/// Device tests wait for these lines, so their format is stable: the event is the second word,
/// `t` (wall seconds since the program started) the third, and a field that can hold spaces
/// (`name`, `data`, `text`) comes last. Characteristics (`char`) are UUIDs as the profile writes
/// them. `docs/ble-radio.md` lists them all.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Entry<'a> {
    /// Advertising started: centrals can find and connect to the wheel.
    Ready {
        /// Advertised service UUIDs, comma separated.
        services: &'a str,
        /// Characteristics the wheel's output goes out on, comma separated.
        tx: &'a str,
        /// Characteristics whose writes go into the wheel, comma separated.
        rx: &'a str,
        /// Advertised name.
        name: &'a str,
    },
    /// A central subscribed to a characteristic.
    Subscribe {
        /// The central's identifier (CoreBluetooth's per-Mac UUID for it).
        central: &'a str,
        /// The characteristic's UUID.
        characteristic: &'a str,
        /// Centrals subscribed to anything now.
        centrals: usize,
        /// Its largest notification payload.
        max: usize,
    },
    /// A central unsubscribed (or disconnected).
    Unsubscribe {
        /// The central's identifier.
        central: &'a str,
        /// The characteristic's UUID.
        characteristic: &'a str,
        /// Centrals subscribed to anything now.
        centrals: usize,
    },
    /// Bytes written to the wheel, by a central or a TCP client (`central=tcp`).
    Write {
        /// Who wrote.
        central: &'a str,
        /// The characteristic's UUID.
        characteristic: &'a str,
        /// What.
        bytes: &'a [u8],
    },
    /// The wheel sent text (a reply such as `GW1825020`, or other firmware text).
    Reply {
        /// The characteristic it went out on.
        characteristic: &'a str,
        /// Centrals subscribed when it went out.
        centrals: usize,
        /// The text.
        bytes: &'a [u8],
    },
    /// A test control took effect.
    Control(Control),
    /// The Mac's Bluetooth changed state (`poweredOn`, `poweredOff`, `unauthorized`, ...).
    Bluetooth {
        /// CoreBluetooth's state name.
        state: &'a str,
    },
    /// The replay started its capture again (`--loop`).
    Lap {
        /// Laps started so far, the first one is 1.
        lap: u64,
    },
    /// The replay reached the end of its capture.
    End,
    /// Something failed; `text` says what.
    Error {
        /// What failed.
        text: &'a str,
    },
    /// `--verbose`: one notification went out.
    Notify {
        /// The characteristic's UUID.
        characteristic: &'a str,
        /// Its payload.
        bytes: &'a [u8],
    },
    /// `--verbose`: the radio's transmit queue is full; notifications wait for it.
    Busy {
        /// Notifications waiting.
        queued: usize,
    },
    /// Notifications thrown away because the queue stayed full.
    Dropped {
        /// How many, since the last such line.
        count: usize,
    },
}

impl Entry<'_> {
    /// The line, without its line end.
    pub fn render(&self, t: f64) -> String {
        let fields = match self {
            Self::Ready { services, tx, rx, name } => {
                format!("ready t={t:.3} services={services} tx={tx} rx={rx} name={}", escape(name.as_bytes()))
            }
            Self::Subscribe { central, characteristic, centrals, max } => {
                format!("subscribe t={t:.3} central={central} char={characteristic} centrals={centrals} max={max}")
            }
            Self::Unsubscribe { central, characteristic, centrals } => {
                format!("unsubscribe t={t:.3} central={central} char={characteristic} centrals={centrals}")
            }
            Self::Write { central, characteristic, bytes } => {
                format!(
                    "write t={t:.3} central={central} char={characteristic} len={} data={}",
                    bytes.len(),
                    escape(bytes)
                )
            }
            Self::Reply { characteristic, centrals, bytes } => {
                format!(
                    "reply t={t:.3} char={characteristic} centrals={centrals} len={} text={}",
                    bytes.len(),
                    escape(bytes)
                )
            }
            Self::Control(control) => format!("{control} t={t:.3}"),
            Self::Bluetooth { state } => format!("bluetooth t={t:.3} state={state}"),
            Self::Lap { lap } => format!("lap t={t:.3} lap={lap}"),
            Self::End => format!("end t={t:.3}"),
            Self::Error { text } => format!("error t={t:.3} text={}", escape(text.as_bytes())),
            Self::Notify { characteristic, bytes } => {
                format!("notify t={t:.3} char={characteristic} len={} hex={}", bytes.len(), hex(bytes))
            }
            Self::Busy { queued } => format!("busy t={t:.3} queued={queued}"),
            Self::Dropped { count } => format!("dropped t={t:.3} count={count}"),
        };
        format!("ble {fields}")
    }

    /// Lines only `--verbose` prints.
    pub const fn verbose_only(&self) -> bool {
        matches!(self, Self::Notify { .. } | Self::Busy { .. })
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::with_capacity(2 * bytes.len()), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// Prints [`Entry`] lines on stdout, one write per line, so a reader sees each line as soon as
/// it happens even through a pipe.
#[derive(Debug)]
pub struct Log {
    start: Instant,
    verbose: bool,
}

impl Log {
    /// A log whose `t` counts from now. `verbose` adds the notification lines.
    pub fn new(verbose: bool) -> Self {
        Self { start: Instant::now(), verbose }
    }

    /// Seconds since the log started.
    pub fn elapsed(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    /// Whether `--verbose` lines print.
    pub const fn verbose(&self) -> bool {
        self.verbose
    }

    /// Prints one line (verbose lines only with `verbose`).
    pub fn line(&self, entry: &Entry<'_>) {
        if entry.verbose_only() && !self.verbose {
            return;
        }
        let mut line = entry.render(self.elapsed());
        line.push('\n');
        let mut out = std::io::stdout().lock();
        // A closed stdout must not stop the wheel.
        let _ = out.write_all(line.as_bytes()).and_then(|()| out.flush());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_everything_but_printable_ascii() {
        assert_eq!(escape(b"V"), "V");
        assert_eq!(escape(b"W Y"), "W Y");
        assert_eq!(escape(&[0x00, 0x1B, 0x7F, 0x80, 0xFF]), r"\x00\x1b\x7f\x80\xff");
        assert_eq!(escape(br"a\x41"), r"a\x5cx41", "a backslash cannot fake an escape");
        assert_eq!(escape("é".as_bytes()), r"\xc3\xa9");
    }

    #[test]
    fn lines_have_a_stable_format() {
        let central = "6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D";
        let cases = [
            (
                Entry::Ready { services: "FFE0", tx: "FFE1", rx: "FFE1", name: "GotWay_EMU" },
                "ble ready t=1.250 services=FFE0 tx=FFE1 rx=FFE1 name=GotWay_EMU",
            ),
            (
                Entry::Subscribe { central, characteristic: "FFE1", centrals: 1, max: 182 },
                "ble subscribe t=1.250 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 centrals=1 max=182",
            ),
            (
                Entry::Unsubscribe { central, characteristic: "FFE1", centrals: 0 },
                "ble unsubscribe t=1.250 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 centrals=0",
            ),
            (
                Entry::Write { central, characteristic: "FFE1", bytes: b"N" },
                "ble write t=1.250 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 len=1 data=N",
            ),
            (
                Entry::Write { central: "tcp", characteristic: "FFE1", bytes: &[0x00, b' '] },
                r"ble write t=1.250 central=tcp char=FFE1 len=2 data=\x00 ",
            ),
            (
                Entry::Reply { characteristic: "FFE1", centrals: 1, bytes: b"GW1825020" },
                "ble reply t=1.250 char=FFE1 centrals=1 len=9 text=GW1825020",
            ),
            (
                Entry::Reply { characteristic: "FFE1", centrals: 0, bytes: b"NAME:EXTREME\r\n" },
                r"ble reply t=1.250 char=FFE1 centrals=0 len=14 text=NAME:EXTREME\x0d\x0a",
            ),
            (Entry::Control(Control::Silent), "ble silent t=1.250"),
            (Entry::Control(Control::On), "ble on t=1.250"),
            (Entry::Bluetooth { state: "poweredOn" }, "ble bluetooth t=1.250 state=poweredOn"),
            (Entry::Lap { lap: 2 }, "ble lap t=1.250 lap=2"),
            (Entry::End, "ble end t=1.250"),
            (Entry::Error { text: "no\nway" }, r"ble error t=1.250 text=no\x0away"),
            (
                Entry::Notify { characteristic: "FFE1", bytes: &[0x55, 0xAA] },
                "ble notify t=1.250 char=FFE1 len=2 hex=55aa",
            ),
            (Entry::Busy { queued: 3 }, "ble busy t=1.250 queued=3"),
            (Entry::Dropped { count: 7 }, "ble dropped t=1.250 count=7"),
        ];
        for (entry, line) in cases {
            assert_eq!(entry.render(1.25), line);
        }
        assert!(Entry::Notify { characteristic: "FFE1", bytes: &[] }.verbose_only());
        assert!(!Entry::Write { central, characteristic: "FFE1", bytes: b"V" }.verbose_only());
    }
}

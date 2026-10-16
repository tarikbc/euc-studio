//! The Begode (Gotway) BLE protocol: what a Begode wheel sends to its phone app and what it
//! accepts back.
//!
//! The wheel's controller talks to a BLE module over a plain UART (115200 8N1). The module
//! bridges that byte stream to GATT service `FFE0`, characteristic `FFE1`: notifications carry
//! the controller's output, writes carry commands. Nothing in the stream is encrypted or
//! checksummed.
//!
//! - Outbound: 24-byte telemetry [`Frame`]s, split into notifications of up to 20 bytes, with text
//!   replies in between. [`Assembler`] puts the stream back together; [`Telemetry`] decodes frames;
//!   [`Reply`] interprets text.
//! - Inbound: one-byte commands and short sequences, see [`Command`] and its [`Safety`] classes.
//!
//! The field map and the command table come from the Begode Extreme build GW1825020 and are
//! checked against a capture from a real wheel (`tests/data/begode-extreme/`). The protocol page
//! `docs/protocol/begode.md` describes the same facts in prose, with the same names. Other Begode
//! models share the frame format; their fields may differ.
//!
//! ```
//! use euc_proto_begode::{Assembler, Segment, Telemetry};
//!
//! let notifications: [&[u8]; 2] = [
//!     &[0x55, 0xAA, 0x19, 0x55, 0, 0, 0, 0x75, 0, 0x06, 0, 0xBE, 0xF0, 0xFA, 0x14, 0xA9, 0, 0x09, 0, 0x18],
//!     &[0x5A, 0x5A, 0x5A, 0x5A],
//! ];
//! let mut assembler = Assembler::new();
//! for notification in notifications {
//!     assembler.push(notification);
//! }
//! let Some(Segment::Frame(frame)) = assembler.next_segment() else { panic!("one frame") };
//! let Telemetry::Live(live) = Telemetry::decode(&frame) else { panic!("type 00") };
//! assert_eq!(format!("{:.2} V", live.voltage_v()), "129.70 V");
//! ```

mod assembler;
mod command;
mod frame;
mod settings;
mod telemetry;
mod text;

pub use assembler::{Assembler, MAX_TEXT, Segment, Stats};
pub use command::{Command, Info, RangeError, SAFE_BYTE_GAP, Safety, Step, TICK};
pub use frame::{FOOTER, FRAME_LEN, Frame, FrameError, HEADER};
pub use settings::{AlarmMode, DriveMode, LightMode, PedalMode, RollCutoff, Units};
pub use telemetry::{Alerts, Bms, Cells, Live, LiveSettings, Modes, Pack, Power, Status, Telemetry};
pub use text::{AlarmReply, Reply, decode_text};

//! A wheel's BLE link on a real radio, for apps that should not know they talk to a simulator.
//!
//! The crate is brand neutral: it deals in advertisements, GATT tables, notifications and writes
//! per characteristic, and leaves the meaning of the bytes to the brand's protocol crate.
//!
//! - [`BleProfile`]: what a wheel's BLE module advertises and serves, and how it sends, from the
//!   wheel profile's `ble.toml`.
//! - [`UartBridge`]: the module model of wheels whose controller talks to an HM-10 style
//!   transparent bridge over a UART (Begode). A wheel whose BLE chip runs firmware of its own
//!   ([`Module::Node`]) hands notifications to the [`Link`] directly.
//! - [`Link`]: the link between the wheel (or a replay) and the radio: test [`Control`]s (silent,
//!   off), the stable log lines device tests wait for ([`Entry`]), text replies.
//! - [`Radio`]: a CoreBluetooth peripheral that serves any [`BleProfile`] (macOS, `radio`
//!   feature).
//! - [`capture`] and [`Replayer`]: a capture file played back as the wheel.
//! - [`Protocol`] and [`Responder`]: what a protocol crate implements (`euc_proto_begode::ble`
//!   for Begode) so the link finds text replies and a replay answers writes like the firmware.
//!
//! `docs/ble-radio.md` is the user guide; its "Adding a brand" section walks the extension points.

mod control;
mod link;
mod log;
mod module;
// Only the CoreBluetooth radio sends through it; its tests run in every build.
#[cfg_attr(not(all(feature = "radio", target_os = "macos")), allow(dead_code))]
mod outbox;
mod profile;
mod protocol;
mod radio;
mod replay;

pub mod capture;

pub use control::{Control, UnknownControl};
pub use link::{Action, Link};
pub use log::{Entry, Log, escape};
pub use module::{Notification, UartBridge};
pub use profile::{
    Advertising, BleProfile, CharId, Characteristic, MAX_NAME, Module, ProfileError, Properties, SCHEMA, Service,
};
pub use protocol::{Group, Opaque, Protocol, Responder};
pub use radio::{Radio, RadioError, RadioEvent};
pub use replay::{GROUP_GAP_S, GROUP_PERIOD_S, ReplayError, Replayer, STALE_S};

/// Cuts `bytes` into notifications of at most `max` bytes, as a module does with a long reply.
///
/// ```
/// let parts: Vec<&[u8]> = euc_ble_radio::split(&[0; 24], 20).collect();
/// assert_eq!(parts.iter().map(|p| p.len()).collect::<Vec<_>>(), [20, 4]);
/// ```
pub fn split(bytes: &[u8], max: usize) -> impl Iterator<Item = &[u8]> {
    bytes.chunks(max.max(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_keeps_short_notifications_whole() {
        assert_eq!(split(b"GW1825020", 20).collect::<Vec<_>>(), [b"GW1825020"]);
        assert_eq!(split(b"", 20).count(), 0);
        let sizes: Vec<usize> = split(&[0x55; 45], 20).map(<[u8]>::len).collect();
        assert_eq!(sizes, [20, 20, 5]);
        assert_eq!(split(b"ab", 0).count(), 2, "a zero size sends one byte per notification");
    }
}

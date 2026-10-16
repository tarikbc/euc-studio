use std::sync::mpsc::Receiver;

use crate::{BleProfile, CharId, ProfileError};

#[cfg(all(feature = "radio", target_os = "macos"))]
mod macos;
#[cfg(all(feature = "radio", target_os = "macos"))]
use macos as imp;

/// Something the radio saw. [`Link`](crate::Link) logs every one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RadioEvent {
    /// The Mac's Bluetooth changed state; CoreBluetooth's name for it (`poweredOn`, `unauthorized`, ...).
    Bluetooth(&'static str),
    /// Advertising started.
    Advertising,
    /// A central subscribed to a characteristic.
    Subscribed {
        /// Its identifier.
        central: String,
        /// The characteristic.
        characteristic: CharId,
        /// Its largest notification payload.
        max: usize,
    },
    /// A central unsubscribed or disconnected.
    Unsubscribed {
        /// Its identifier.
        central: String,
        /// The characteristic.
        characteristic: CharId,
    },
    /// A central wrote these bytes on a characteristic; a write request is already answered
    /// with success.
    Write {
        /// Its identifier.
        central: String,
        /// The characteristic.
        characteristic: CharId,
        /// The bytes.
        bytes: Vec<u8>,
    },
    /// The transmit queue is full: notifications wait until the radio is ready again.
    Busy {
        /// Notifications waiting.
        queued: usize,
    },
    /// Notifications thrown away because too many were waiting.
    Dropped {
        /// How many.
        count: usize,
    },
    /// Something failed.
    Error(String),
}

/// A radio that cannot start.
#[derive(Debug, thiserror::Error)]
pub enum RadioError {
    /// Not macOS, or a build without the `radio` feature.
    #[error(
        "no BLE radio here: it needs macOS (other systems are not supported yet) and the `radio` feature; \
         use a TCP bridge and a BLE peripheral emulator instead"
    )]
    Unsupported,
    /// The BLE profile is not valid.
    #[error(transparent)]
    Profile(#[from] ProfileError),
    /// macOS does not let this program use Bluetooth.
    #[error(
        "macOS does not allow Bluetooth for this program: open System Settings > Privacy & Security > Bluetooth, \
         turn on the app you run `euc` from (your terminal), then run it again"
    )]
    Denied,
}

/// The wheel's BLE module on the Mac's radio: a peripheral that advertises and serves a
/// [`BleProfile`], passes every write on as a [`RadioEvent::Write`] and sends notifications to the
/// centrals subscribed to their characteristic, waiting when the radio is busy.
pub struct Radio {
    inner: imp::Radio,
    events: Receiver<RadioEvent>,
}

impl Radio {
    /// Starts the peripheral. Advertising begins once Bluetooth is on ([`RadioEvent::Advertising`]);
    /// the first run may make macOS ask the user to allow Bluetooth.
    pub fn start(profile: &BleProfile) -> Result<Self, RadioError> {
        profile.validate()?;
        let (inner, events) = imp::Radio::start(profile)?;
        Ok(Self { inner, events })
    }

    /// Sends one notification on `characteristic` to every central subscribed to it (nothing
    /// when none is). A payload larger than a central's limit goes out in pieces.
    pub fn notify(&self, characteristic: CharId, bytes: &[u8]) {
        self.inner.notify(characteristic, bytes);
    }

    /// Switches the wheel's module on (serve and advertise) or off (remove the service, stop
    /// advertising).
    pub fn set_on(&self, on: bool) {
        self.inner.set_on(on);
    }

    /// Events since the last call, in order.
    pub fn events(&self) -> Vec<RadioEvent> {
        self.events.try_iter().collect()
    }

    /// Whether macOS lets this program use Bluetooth: `allowed`, `notDetermined` (macOS asks
    /// the user when the radio starts), `denied` or `restricted`; `unsupported` on other OSes.
    pub fn authorization() -> &'static str {
        imp::authorization()
    }
}

impl std::fmt::Debug for Radio {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Radio")
    }
}

#[cfg(not(all(feature = "radio", target_os = "macos")))]
mod imp {
    use std::sync::mpsc::Receiver;

    use super::{RadioError, RadioEvent};
    use crate::{BleProfile, CharId};

    /// No radio in this build: never made.
    pub(super) enum Radio {}

    pub(super) const fn authorization() -> &'static str {
        "unsupported"
    }

    impl Radio {
        pub(super) fn start(_profile: &BleProfile) -> Result<(Self, Receiver<RadioEvent>), RadioError> {
            Err(RadioError::Unsupported)
        }

        pub(super) fn notify(&self, _characteristic: CharId, _bytes: &[u8]) {
            match *self {}
        }

        pub(super) fn set_on(&self, _on: bool) {
            match *self {}
        }
    }
}

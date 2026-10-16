//! What a brand's protocol tells the link and the replay. The radio and the link never look
//! inside the bytes; a protocol crate does (`euc_proto_begode::ble` for Begode), and the program
//! picks it by the profile's `protocol` name.

/// Where the text is in a wheel's output, so the link can log replies (`ble reply` lines).
pub trait Protocol {
    /// Takes one notification on `characteristic` (its UUID as the profile writes it) at `now`,
    /// seconds; returns the text segments it completes.
    fn texts(&mut self, now: f64, characteristic: &str, bytes: &[u8]) -> Vec<Vec<u8>>;

    /// Text held back until the stream went quiet (a reply without a line end), or nothing.
    /// Called often.
    fn quiet(&mut self, now: f64) -> Option<Vec<u8>>;
}

/// How a wheel's firmware answers writes, so a replayed capture answers like the wheel.
///
/// A capture plays what the wheel sent on its own; the responder decides, at each telemetry group
/// time of the capture, what the wheel sends instead after a central wrote something.
pub trait Responder {
    /// Whether a captured notification starts a telemetry group: a moment the firmware decides
    /// what to send. When no line does, the replay takes a pause of more than
    /// [`GROUP_GAP_S`](crate::GROUP_GAP_S) as the start of a group.
    fn starts_group(&self, bytes: &[u8]) -> bool;

    /// A central wrote these bytes (to an `rx` characteristic). Returns what the wheel sends at
    /// once, if anything.
    fn write(&mut self, bytes: &[u8]) -> Vec<Vec<u8>>;

    /// A group time: what the wheel sends.
    fn group_time(&mut self) -> Group;
}

/// What the wheel sends at a group time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Group {
    /// The captured group.
    Send,
    /// Nothing.
    Skip,
    /// These bytes instead of the group (a reply), on the default `tx` characteristic.
    Reply(Vec<u8>),
}

/// A protocol nothing is known about: no text, groups by pauses, writes change nothing.
#[derive(Debug, Clone, Copy, Default)]
pub struct Opaque;

impl Protocol for Opaque {
    fn texts(&mut self, _now: f64, _characteristic: &str, _bytes: &[u8]) -> Vec<Vec<u8>> {
        Vec::new()
    }

    fn quiet(&mut self, _now: f64) -> Option<Vec<u8>> {
        None
    }
}

impl Responder for Opaque {
    fn starts_group(&self, _bytes: &[u8]) -> bool {
        false
    }

    fn write(&mut self, _bytes: &[u8]) -> Vec<Vec<u8>> {
        Vec::new()
    }

    fn group_time(&mut self) -> Group {
        Group::Send
    }
}

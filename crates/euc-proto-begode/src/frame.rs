use std::fmt;

use thiserror::Error;

/// Length of one telemetry frame in bytes.
pub const FRAME_LEN: usize = 24;

/// The two bytes every frame starts with.
pub const HEADER: [u8; 2] = [0x55, 0xAA];

/// The four bytes every frame ends with.
pub const FOOTER: [u8; 4] = [0x5A; 4];

/// Frame byte that holds the frame type.
const KIND_AT: usize = 18;

/// Frame byte that holds the sub-index (block, page, or a constant per type).
const SUB_AT: usize = 19;

/// One telemetry frame as it travels on the wire.
///
/// ```text
/// byte  0  1 | 2 ............ 17 | 18   19  | 20 21 22 23
///       55 AA| 16 payload bytes   | type sub | 5A 5A 5A 5A
/// ```
///
/// Payload fields are big-endian. There is no length field and no checksum: the header, the
/// fixed length and the footer are the only framing. A `Frame` always holds a valid header and
/// footer; the payload, type and sub-index can be anything.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Frame([u8; FRAME_LEN]);

/// Why a byte slice is not a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum FrameError {
    /// The slice is not 24 bytes long.
    #[error("a frame is {FRAME_LEN} bytes, got {0}")]
    Length(usize),
    /// The slice does not start with `55 AA`.
    #[error("a frame starts with 55 AA")]
    Header,
    /// The slice does not end with `5A 5A 5A 5A`.
    #[error("a frame ends with 5A 5A 5A 5A")]
    Footer,
}

impl Frame {
    /// A frame of type `kind` with sub-index `sub` and an all-zero payload.
    pub fn new(kind: u8, sub: u8) -> Self {
        let mut bytes = [0; FRAME_LEN];
        bytes[..2].copy_from_slice(&HEADER);
        bytes[KIND_AT] = kind;
        bytes[SUB_AT] = sub;
        bytes[FRAME_LEN - 4..].copy_from_slice(&FOOTER);
        Self(bytes)
    }

    /// Checks the length, header and footer of `bytes`.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, FrameError> {
        let bytes: [u8; FRAME_LEN] = bytes.try_into().map_err(|_| FrameError::Length(bytes.len()))?;
        if bytes[..2] != HEADER {
            return Err(FrameError::Header);
        }
        if bytes[FRAME_LEN - 4..] != FOOTER {
            return Err(FrameError::Footer);
        }
        Ok(Self(bytes))
    }

    /// Wraps bytes whose header and footer the caller has already checked.
    pub(crate) fn from_checked(bytes: [u8; FRAME_LEN]) -> Self {
        debug_assert!(bytes[..2] == HEADER && bytes[FRAME_LEN - 4..] == FOOTER, "unchecked frame bytes");
        Self(bytes)
    }

    /// The 24 bytes as sent.
    pub const fn as_bytes(&self) -> &[u8; FRAME_LEN] {
        &self.0
    }

    /// The frame type (byte 18).
    pub const fn kind(&self) -> u8 {
        self.0[KIND_AT]
    }

    /// The sub-index (byte 19).
    pub const fn sub(&self) -> u8 {
        self.0[SUB_AT]
    }

    /// The byte at frame offset `at` (2..=17 for payload bytes).
    pub const fn u8_at(&self, at: usize) -> u8 {
        self.0[at]
    }

    /// The big-endian `u16` at frame bytes `at..at + 2`.
    pub const fn u16_at(&self, at: usize) -> u16 {
        u16::from_be_bytes([self.0[at], self.0[at + 1]])
    }

    /// The big-endian `i16` at frame bytes `at..at + 2`.
    pub const fn i16_at(&self, at: usize) -> i16 {
        i16::from_be_bytes([self.0[at], self.0[at + 1]])
    }

    /// The big-endian `u32` at frame bytes `at..at + 4`.
    pub const fn u32_at(&self, at: usize) -> u32 {
        u32::from_be_bytes([self.0[at], self.0[at + 1], self.0[at + 2], self.0[at + 3]])
    }

    /// Sets the payload byte at frame offset `at` (2..=17).
    ///
    /// # Panics
    ///
    /// If `at` is outside the payload: the header, type, sub-index and footer are not payload.
    pub fn set_u8(&mut self, at: usize, value: u8) -> &mut Self {
        assert!((2..KIND_AT).contains(&at), "frame byte {at} is not payload");
        self.0[at] = value;
        self
    }

    /// Sets the big-endian `u16` at frame bytes `at..at + 2`.
    ///
    /// # Panics
    ///
    /// If the field leaves the payload (bytes 2..=17).
    pub fn set_u16(&mut self, at: usize, value: u16) -> &mut Self {
        let [hi, lo] = value.to_be_bytes();
        self.set_u8(at, hi).set_u8(at + 1, lo)
    }

    /// Sets the big-endian `i16` at frame bytes `at..at + 2`.
    ///
    /// # Panics
    ///
    /// If the field leaves the payload (bytes 2..=17).
    pub fn set_i16(&mut self, at: usize, value: i16) -> &mut Self {
        self.set_u16(at, value as u16)
    }

    /// Sets the big-endian `u32` at frame bytes `at..at + 4`.
    ///
    /// # Panics
    ///
    /// If the field leaves the payload (bytes 2..=17).
    pub fn set_u32(&mut self, at: usize, value: u32) -> &mut Self {
        self.set_u16(at, (value >> 16) as u16).set_u16(at + 2, value as u16)
    }
}

impl fmt::Debug for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Frame({self})")
    }
}

/// Space-separated lowercase hex, the format of captures and of the studio's hex log.
impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, byte) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIVE: [u8; FRAME_LEN] = [
        0x55, 0xAA, 0x19, 0x55, 0x00, 0x00, 0x00, 0x75, 0x00, 0x06, 0x00, 0xBE, 0xF0, 0xFA, 0x14, 0xA9, 0x00, 0x09,
        0x00, 0x18, 0x5A, 0x5A, 0x5A, 0x5A,
    ];

    #[test]
    fn reads_fields_big_endian() {
        let frame = Frame::from_bytes(&LIVE).unwrap();
        assert_eq!(frame.kind(), 0x00);
        assert_eq!(frame.sub(), 0x18);
        assert_eq!(frame.u16_at(2), 0x1955);
        assert_eq!(frame.i16_at(12), -3846);
        assert_eq!(frame.u32_at(2), 0x1955_0000);
        assert_eq!(frame.u8_at(17), 0x09);
    }

    #[test]
    fn new_frames_carry_header_type_and_footer() {
        let mut frame = Frame::new(0x04, 0x18);
        frame.set_u32(2, 1_468_670).set_u16(16, 0x0012).set_i16(10, -2);
        let bytes = frame.as_bytes();
        assert_eq!(bytes[..2], HEADER);
        assert_eq!(bytes[20..], FOOTER);
        assert_eq!((frame.kind(), frame.sub()), (0x04, 0x18));
        assert_eq!(frame.u32_at(2), 1_468_670);
        assert_eq!(frame.i16_at(10), -2);
        assert_eq!(Frame::from_bytes(bytes), Ok(frame));
    }

    #[test]
    fn rejects_bad_framing() {
        assert_eq!(Frame::from_bytes(&LIVE[..23]), Err(FrameError::Length(23)));
        let mut bad = LIVE;
        bad[1] = 0xAB;
        assert_eq!(Frame::from_bytes(&bad), Err(FrameError::Header));
        let mut bad = LIVE;
        bad[23] = 0;
        assert_eq!(Frame::from_bytes(&bad), Err(FrameError::Footer));
    }

    #[test]
    #[should_panic(expected = "frame byte 18 is not payload")]
    fn setters_stay_inside_the_payload() {
        Frame::new(0, 0).set_u16(17, 1);
    }

    #[test]
    fn displays_as_capture_hex() {
        let frame = Frame::from_bytes(&LIVE).unwrap();
        assert_eq!(frame.to_string(), "55 aa 19 55 00 00 00 75 00 06 00 be f0 fa 14 a9 00 09 00 18 5a 5a 5a 5a");
    }
}

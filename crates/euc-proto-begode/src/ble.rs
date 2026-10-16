//! The Begode protocol on the BLE link (`euc_ble_radio`, feature `ble`): the text in the wheel's
//! stream, for the link log, and the stock firmware's answers to writes, for a replay.
//!
//! The stock firmware (`docs/protocol/begode.md`, Commands) sends one telemetry group per group
//! time, always starting with frame `00`, and handles a received byte like this:
//!
//! - One command slot, overwritten by every received byte. `V`, `N` (and `k`) wait there for a
//!   group time; any other byte is used within a tick and leaves the slot empty.
//! - Every received byte pauses telemetry: the next two group times send nothing, the third one
//!   sends the group, or, when `V` or `N` waits in the slot, the reply instead of the group.

use euc_ble_radio::{Group, Protocol, Responder};

use crate::{Assembler, FRAME_LEN, HEADER, Segment};

/// Quiet time after which text from the wheel counts as complete, seconds. `V`'s answer has no
/// line end, and the next group comes 0.3 s later.
const TEXT_QUIET_S: f64 = 0.1;

/// Group times a received byte affects: the first two send nothing, the last one sends again.
const HOLDOFF_GROUPS: u8 = 3;

/// Offset of the frame type in a frame.
const FRAME_TYPE: usize = 18;

/// Finds the text in the wheel's notifications: replies such as `GW1825020`, and whatever else
/// the firmware prints between frames.
#[derive(Debug, Default)]
pub struct TextFinder {
    assembler: Assembler,
    /// The bytes the assembler holds: pushed, not handed out in a segment yet.
    held: Vec<u8>,
    /// When the last bytes came.
    last: f64,
}

impl Protocol for TextFinder {
    fn texts(&mut self, now: f64, _characteristic: &str, bytes: &[u8]) -> Vec<Vec<u8>> {
        self.assembler.push(bytes);
        self.held.extend_from_slice(bytes);
        self.last = now;
        let mut texts = Vec::new();
        while let Some(segment) = self.assembler.next_segment() {
            match segment {
                Segment::Frame(_) => drop(self.held.drain(..FRAME_LEN)),
                Segment::Text(text) => {
                    self.held.drain(..text.len());
                    texts.push(text);
                }
            }
        }
        texts
    }

    /// Text held back once the stream has been quiet for a while (`V`'s answer has no line
    /// end). Not while a frame may be under way: a capture's notifications of one group can be
    /// over 100 ms apart.
    fn quiet(&mut self, now: f64) -> Option<Vec<u8>> {
        if self.held.is_empty() || now - self.last < TEXT_QUIET_S || self.frame_may_follow() {
            return None;
        }
        self.held.clear();
        match self.assembler.finish()? {
            Segment::Text(text) => Some(text),
            Segment::Frame(_) => None,
        }
    }
}

impl TextFinder {
    /// Whether the held bytes contain a frame header, or end with its first byte.
    fn frame_may_follow(&self) -> bool {
        self.held.windows(2).any(|pair| pair == HEADER) || self.held.last() == Some(&HEADER[0])
    }
}

/// The firmware's answers to `V` and `N`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Replies {
    /// Answer to `V`: the firmware banner.
    pub version: Vec<u8>,
    /// Answer to `N`: the model name.
    pub name: Vec<u8>,
}

impl Default for Replies {
    /// The Begode Extreme's stock answers (build GW1825020).
    fn default() -> Self {
        Self { version: b"GW1825020".to_vec(), name: b"NAME:EXTREME\r\n".to_vec() }
    }
}

/// The stock firmware's handling of writes: one command slot and the telemetry pause.
#[derive(Debug, Clone, Default)]
pub struct StockResponder {
    /// `V`, `N` or `k` waiting for a group time.
    slot: Option<u8>,
    holdoff: u8,
    replies: Replies,
}

impl StockResponder {
    /// Answers `V` and `N` with these texts instead of the Begode Extreme's.
    #[must_use]
    pub fn with_replies(mut self, replies: Replies) -> Self {
        self.replies = replies;
        self
    }
}

impl Responder for StockResponder {
    /// Every group starts with frame `00`; in the stream of 20-byte notifications it always
    /// starts a notification.
    fn starts_group(&self, bytes: &[u8]) -> bool {
        bytes.starts_with(&HEADER) && bytes.get(FRAME_TYPE) == Some(&0x00)
    }

    /// Bytes arrive one at a time, in order; nothing is answered at once.
    fn write(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        for &byte in bytes {
            self.slot = matches!(byte, b'V' | b'N' | b'k').then_some(byte);
            self.holdoff = HOLDOFF_GROUPS;
        }
        Vec::new()
    }

    fn group_time(&mut self) -> Group {
        if self.holdoff > 0 {
            self.holdoff -= 1;
            if self.holdoff > 0 {
                return Group::Skip;
            }
        }
        match self.slot.take() {
            Some(b'V') => Group::Reply(self.replies.version.clone()),
            Some(b'N') => Group::Reply(self.replies.name.clone()),
            // A debug request uses up the group time too; its text dump is not replayed.
            Some(_) => Group::Skip,
            None => Group::Send,
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_ble_radio::capture::{self, Capture, Direction, Line};
    use euc_ble_radio::{BleProfile, CharId, Notification, Replayer};

    use super::*;

    const CAPTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/data/begode-extreme/idle-2026-10-15.hex");

    fn frame(kind: u8) -> Vec<u8> {
        let mut frame = vec![0x55, 0xAA];
        frame.extend([0; 16]);
        frame.extend([kind, 0x18, 0x5A, 0x5A, 0x5A, 0x5A]);
        frame
    }

    fn texts(finder: &mut TextFinder, now: f64, bytes: &[u8]) -> Vec<Vec<u8>> {
        finder.texts(now, "FFE1", bytes)
    }

    #[test]
    fn text_between_frames_is_found_with_or_without_a_line_end() {
        let mut finder = TextFinder::default();
        let f = frame(0);
        assert_eq!(texts(&mut finder, 0.0, &f[..20]), Vec::<Vec<u8>>::new());
        assert_eq!(texts(&mut finder, 0.0, &f[20..]), Vec::<Vec<u8>>::new());
        assert_eq!(finder.quiet(0.5), None, "nothing waits after a whole frame");
        // `N`'s answer ends with a line feed: complete at once.
        assert_eq!(texts(&mut finder, 1.0, b"NAME:EXTREME\r\n"), [b"NAME:EXTREME\r\n".to_vec()]);
        // `V`'s answer has none: complete when the stream goes quiet, or at the next frame.
        assert_eq!(texts(&mut finder, 2.0, b"GW1825020"), Vec::<Vec<u8>>::new());
        assert_eq!(finder.quiet(2.05), None);
        assert_eq!(finder.quiet(2.1), Some(b"GW1825020".to_vec()));
        assert_eq!(texts(&mut finder, 3.0, b"GW1825020"), Vec::<Vec<u8>>::new());
        assert_eq!(texts(&mut finder, 3.03, &f[..20]), Vec::<Vec<u8>>::new(), "a frame may still follow");
        assert_eq!(texts(&mut finder, 3.03, &f[20..]), [b"GW1825020".to_vec()]);
    }

    #[test]
    fn a_frame_whose_parts_come_far_apart_is_not_text() {
        let mut finder = TextFinder::default();
        let f = frame(4);
        assert_eq!(texts(&mut finder, 0.0, &f[..8]), Vec::<Vec<u8>>::new());
        assert_eq!(finder.quiet(0.5), None, "the rest of the frame may still come");
        assert_eq!(texts(&mut finder, 0.5, &f[8..]), Vec::<Vec<u8>>::new());
        assert_eq!(finder.quiet(1.0), None);
    }

    #[test]
    fn the_wheel_capture_holds_no_text() {
        // A group's notifications are up to 119 ms apart in it: no frame part may pass for text.
        let mut finder = TextFinder::default();
        let mut found = Vec::new();
        for line in capture::load(std::path::Path::new(CAPTURE)).unwrap().lines {
            found.extend(finder.quiet(line.t));
            found.extend(texts(&mut finder, line.t, &line.bytes));
        }
        found.extend(finder.quiet(1e9));
        assert_eq!(found, Vec::<Vec<u8>>::new());
    }

    /// Groups of two notifications (a frame 00 as 20 + 4 bytes) every 0.3 s from t = 1; byte 2
    /// numbers the group.
    fn groups(count: u8) -> Capture {
        let lines = (0..count)
            .flat_map(|g| {
                let t = 1.0 + f64::from(g) * 0.3;
                let mut f = frame(0);
                f[2] = g;
                [(t, f[..20].to_vec()), (t + 0.01, f[20..].to_vec())]
            })
            .enumerate()
            .map(|(i, (t, bytes))| Line { number: i + 1, t, direction: Direction::Notify, characteristic: None, bytes })
            .collect();
        Capture { header: capture::Header::default(), lines }
    }

    fn replay_of(capture: &Capture) -> Replayer {
        let profile = BleProfile::from_toml(include_str!("../../../profiles/begode-extreme/ble.toml")).unwrap();
        Replayer::new(capture, &profile, Box::new(StockResponder::default()), 0.0, false).unwrap()
    }

    /// The groups (by number) and replies sent by `until`, polling every 10 ms.
    fn played(replayer: &mut Replayer, writes: &[(f64, &[u8])], until: f64) -> Vec<String> {
        let mut seen = Vec::new();
        let mut writes = writes.iter().peekable();
        for step in 0..=(until * 100.0).round() as u32 {
            let now = f64::from(step) / 100.0;
            for out in replayer.advance(now) {
                match out.bytes.len() {
                    20 => seen.push(format!("g{}", out.bytes[2])),
                    4 => {}
                    _ => seen.push(String::from_utf8_lossy(&out.bytes).into_owned()),
                }
            }
            while let Some((_, bytes)) = writes.next_if(|(at, _)| *at <= now) {
                replayer.write(now, CharId(0), bytes);
            }
        }
        seen
    }

    #[test]
    fn v_is_answered_at_the_third_group_time_instead_of_a_group() {
        // Groups at 0, 0.3, 0.6, ...; V at 0.35: 0.6 and 0.9 send nothing, 1.2 sends the banner.
        let mut replayer = replay_of(&groups(8));
        assert_eq!(played(&mut replayer, &[(0.35, b"V")], 2.2), ["g0", "g1", "GW1825020", "g5", "g6", "g7"]);
    }

    #[test]
    fn n_gets_the_name_with_its_line_end() {
        let mut replayer = replay_of(&groups(6));
        assert_eq!(played(&mut replayer, &[(0.05, b"N")], 1.6), ["g0", "NAME:EXTREME\r\n", "g4", "g5"]);
    }

    #[test]
    fn a_later_byte_overwrites_a_waiting_request_and_restarts_the_pause() {
        // N at 0.05 waits for 0.9; V at 0.4 replaces it and pauses 0.6 and 0.9 again: V at 1.2.
        let mut replayer = replay_of(&groups(8));
        assert_eq!(played(&mut replayer, &[(0.05, b"N"), (0.4, b"V")], 2.2), ["g0", "GW1825020", "g5", "g6", "g7"]);
        // Any other byte empties the slot: the third group time sends the group.
        let mut replayer = replay_of(&groups(8));
        assert_eq!(played(&mut replayer, &[(0.05, b"V"), (0.4, b"b")], 2.2), ["g0", "g4", "g5", "g6", "g7"]);
    }

    #[test]
    fn every_byte_pauses_telemetry_for_two_group_times() {
        let mut replayer = replay_of(&groups(6));
        assert_eq!(played(&mut replayer, &[(0.05, b"Q")], 1.6), ["g0", "g3", "g4", "g5"]);
        // Bytes together count as one pause; the last one decides what waits.
        let mut replayer = replay_of(&groups(6));
        assert_eq!(played(&mut replayer, &[(0.05, b"VN")], 1.6), ["g0", "NAME:EXTREME\r\n", "g4", "g5"]);
        // k uses up the group time without a reply.
        let mut replayer = replay_of(&groups(6));
        assert_eq!(played(&mut replayer, &[(0.05, b"k")], 1.6), ["g0", "g4", "g5"]);
    }

    #[test]
    fn a_long_reply_goes_out_in_module_sized_pieces() {
        let replies = Replies { version: b"GW1825020-a-much-longer-banner".to_vec(), ..Replies::default() };
        let profile = BleProfile::from_toml(include_str!("../../../profiles/begode-extreme/ble.toml")).unwrap();
        let responder = StockResponder::default().with_replies(replies);
        let mut replayer = Replayer::new(&groups(4), &profile, Box::new(responder), 0.0, false).unwrap();
        replayer.advance(0.05);
        replayer.write(0.05, CharId(0), b"V");
        let reply: Vec<usize> = replayer.advance(0.95).iter().map(|n: &Notification| n.bytes.len()).collect();
        assert_eq!(reply, [20, 10], "the banner replaces the group at 0.9");
    }

    #[test]
    fn the_wheel_capture_replays_in_76_groups_of_about_310_ms() {
        let capture = capture::load(std::path::Path::new(CAPTURE)).unwrap();
        let mut replayer = replay_of(&capture);
        assert!((0.30..0.32).contains(&(replayer.period() / 76.0)), "{}", replayer.period());
        assert_eq!(replayer.advance(1e9).len(), 456);
    }
}

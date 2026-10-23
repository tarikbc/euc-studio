use std::collections::VecDeque;

use crate::frame::{FOOTER, FRAME_LEN, Frame, HEADER};

/// Longest text segment the assembler holds before it hands the text out anyway.
///
/// Text normally ends at a line feed or at the next frame. A wheel in its debug-dump mode prints
/// for as long as it runs and sends no frames, so text needs an upper bound too.
pub const MAX_TEXT: usize = 256;

/// A piece of the byte stream: one frame, or bytes between frames.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Segment {
    /// A complete telemetry frame.
    Frame(Frame),
    /// Bytes that are not part of a frame: replies such as `GW1825020` or `NAME:EXTREME\r\n`,
    /// boot and debug text, or line noise. See [`Reply`](crate::Reply) to interpret them.
    Text(Vec<u8>),
}

/// Turns the wheel's byte stream back into frames and text.
///
/// The stream arrives in BLE notifications of up to 20 bytes, so frames are split across
/// notifications (a 24-byte frame usually arrives as 20 + 4 bytes), and text replies can sit
/// between frames. Push every notification in order; take segments out as they complete.
///
/// A frame is recognised by its header `55 AA` and its footer `5A 5A 5A 5A` 20 bytes later.
/// Anything else is text. A `55` that turns out not to start a frame becomes text too, so the
/// assembler resynchronises on the next real frame after garbage or a lost notification.
///
/// ```
/// use euc_proto_begode::{Assembler, Segment};
///
/// let frame = [0x55, 0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x04, 0x18, 0x5A, 0x5A, 0x5A, 0x5A];
/// let mut assembler = Assembler::new();
/// assembler.push(b"GW1825020");
/// assembler.push(&frame[..20]);
/// assert_eq!(assembler.next_segment(), None, "the frame is not complete yet");
/// assembler.push(&frame[20..]);
/// assert_eq!(assembler.next_segment(), Some(Segment::Text(b"GW1825020".to_vec())));
/// let Some(Segment::Frame(frame)) = assembler.next_segment() else { panic!("expected a frame") };
/// assert_eq!(frame.kind(), 0x04);
/// ```
#[derive(Debug, Clone, Default)]
pub struct Assembler {
    /// Bytes pushed but not yet classified.
    pending: VecDeque<u8>,
    /// Bytes classified as text, not handed out yet.
    text: Vec<u8>,
    stats: Stats,
}

/// What an [`Assembler`] has seen so far.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Complete frames handed out.
    pub frames: u64,
    /// Bytes classified as text (replies, debug output, garbage).
    pub text_bytes: u64,
}

impl Assembler {
    /// An assembler with nothing buffered.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends received bytes (one BLE notification, or any other chunk of the stream).
    pub fn push(&mut self, bytes: &[u8]) {
        self.pending.extend(bytes);
    }

    /// The next complete segment, or `None` until more bytes arrive.
    ///
    /// Text is held back until a line feed, the next frame, or [`MAX_TEXT`] bytes, so a reply
    /// split across notifications comes out in one piece.
    pub fn next_segment(&mut self) -> Option<Segment> {
        loop {
            let first = *self.pending.front()?;
            if first == HEADER[0] {
                match self.frame_at_front() {
                    Candidate::Frame => {
                        if !self.text.is_empty() {
                            return Some(self.take_text());
                        }
                        let mut bytes = [0; FRAME_LEN];
                        for (slot, byte) in bytes.iter_mut().zip(self.pending.drain(..FRAME_LEN)) {
                            *slot = byte;
                        }
                        self.stats.frames += 1;
                        return Some(Segment::Frame(Frame::from_checked(bytes)));
                    }
                    Candidate::NeedMore => return None,
                    Candidate::NotAFrame => {}
                }
            }
            self.pending.pop_front();
            self.text.push(first);
            self.stats.text_bytes += 1;
            if first == b'\n' || self.text.len() >= MAX_TEXT {
                return Some(self.take_text());
            }
        }
    }

    /// Ends the stream: everything still buffered comes out as one text segment, if any.
    ///
    /// Call it when the link closes, so a trailing reply with no line feed is not lost.
    pub fn finish(&mut self) -> Option<Segment> {
        self.stats.text_bytes += self.pending.len() as u64;
        self.text.extend(self.pending.drain(..));
        (!self.text.is_empty()).then(|| self.take_text())
    }

    /// Counters since the assembler was created.
    pub const fn stats(&self) -> Stats {
        self.stats
    }

    fn take_text(&mut self) -> Segment {
        Segment::Text(std::mem::take(&mut self.text))
    }

    /// Whether the `55` at the front starts a frame, judged on the bytes that have arrived.
    fn frame_at_front(&self) -> Candidate {
        let footer_at = FRAME_LEN - FOOTER.len();
        let mismatch = self.pending.iter().enumerate().take(FRAME_LEN).any(|(i, &byte)| match i {
            1 => byte != HEADER[1],
            i if i >= footer_at => byte != FOOTER[i - footer_at],
            _ => false,
        });
        if mismatch {
            Candidate::NotAFrame
        } else if self.pending.len() < FRAME_LEN {
            Candidate::NeedMore
        } else {
            Candidate::Frame
        }
    }
}

impl Iterator for Assembler {
    type Item = Segment;

    fn next(&mut self) -> Option<Segment> {
        self.next_segment()
    }
}

enum Candidate {
    Frame,
    NeedMore,
    NotAFrame,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(kind: u8, fill: u8) -> Vec<u8> {
        let mut bytes = vec![fill; FRAME_LEN];
        bytes[..2].copy_from_slice(&HEADER);
        bytes[18] = kind;
        bytes[20..].copy_from_slice(&FOOTER);
        bytes
    }

    fn kinds(segments: &[Segment]) -> Vec<String> {
        segments
            .iter()
            .map(|s| match s {
                Segment::Frame(f) => format!("F{:02}", f.kind()),
                Segment::Text(t) => format!("T:{}", String::from_utf8_lossy(t)),
            })
            .collect()
    }

    #[test]
    fn splits_frames_and_text_in_order() {
        let mut stream = frame(0, 1);
        stream.extend(b"NAME:EXTREME\r\n");
        stream.extend(frame(1, 2));
        stream.extend(b"GW1825020");
        stream.extend(frame(4, 3));
        let mut assembler = Assembler::new();
        assembler.push(&stream);
        let segments: Vec<_> = assembler.by_ref().collect();
        assert_eq!(kinds(&segments), ["F00", "T:NAME:EXTREME\r\n", "F01", "T:GW1825020", "F04"]);
        assert_eq!(assembler.stats(), Stats { frames: 3, text_bytes: 23 });
    }

    #[test]
    fn reassembles_any_split() {
        let mut stream = Vec::new();
        for kind in [0, 1, 2, 4, 7] {
            stream.extend(frame(kind, 0x5A));
        }
        for chunk in 1..=stream.len() {
            let mut assembler = Assembler::new();
            let mut segments = Vec::new();
            for piece in stream.chunks(chunk) {
                assembler.push(piece);
                segments.extend(assembler.by_ref());
            }
            assert_eq!(kinds(&segments), ["F00", "F01", "F02", "F04", "F07"], "chunk size {chunk}");
        }
    }

    #[test]
    fn resyncs_after_garbage_and_a_truncated_frame() {
        let mut stream = vec![0x55, 0x55, 0xAA, 0x00];
        stream.extend(&frame(7, 0)[..15]);
        stream.extend(frame(3, 0));
        let mut assembler = Assembler::new();
        assembler.push(&stream);
        let segments: Vec<_> = assembler.by_ref().collect();
        assert_eq!(segments.len(), 2);
        assert!(matches!(&segments[0], Segment::Text(t) if t.len() == 4 + 15));
        assert!(matches!(&segments[1], Segment::Frame(f) if f.kind() == 3));
    }

    #[test]
    fn rejects_a_header_early_on_a_wrong_footer_byte() {
        let mut assembler = Assembler::new();
        let mut almost = frame(0, 0);
        almost[20] = 0;
        assembler.push(&almost[..21]);
        assert_eq!(assembler.next_segment(), None, "text waits for a line feed or a frame");
        assert_eq!(assembler.stats().text_bytes, 21, "the bytes are already known to be text");
    }

    #[test]
    fn text_ends_at_line_feeds_and_at_the_cap() {
        let mut assembler = Assembler::new();
        assembler.push(b" AZ = 8192\r\n AY =    0\r\n");
        assert_eq!(assembler.next_segment(), Some(Segment::Text(b" AZ = 8192\r\n".to_vec())));
        assert_eq!(assembler.next_segment(), Some(Segment::Text(b" AY =    0\r\n".to_vec())));
        assembler.push(&[b'x'; MAX_TEXT + 1]);
        assert_eq!(assembler.next_segment(), Some(Segment::Text(vec![b'x'; MAX_TEXT])));
        assert_eq!(assembler.next_segment(), None);
        assert_eq!(assembler.finish(), Some(Segment::Text(b"x".to_vec())));
        assert_eq!(assembler.finish(), None);
    }

    #[test]
    fn finish_flushes_an_incomplete_frame_as_text() {
        let mut assembler = Assembler::new();
        assembler.push(&frame(0, 0)[..10]);
        assert_eq!(assembler.next_segment(), None);
        assert!(matches!(assembler.finish(), Some(Segment::Text(t)) if t.len() == 10));
        assert_eq!(assembler.stats().text_bytes, 10);
    }
}

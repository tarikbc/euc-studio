//! A capture played back as a wheel: its notifications at their own pace, and the firmware's
//! answers to what a central writes.
//!
//! The wheel sends telemetry in groups; at each group time its firmware decides what to send. The
//! protocol's [`Responder`] finds the group times in the capture and makes those decisions after
//! writes (for Begode: the stock firmware's command slot and telemetry pause, so `V` and `N` get
//! their answers at the third group time). Write lines in the capture are what the recording
//! central wrote; a replay skips them, since the live central writes for itself.

use crate::capture::{Capture, Direction};
use crate::{BleProfile, CharId, Group, Notification, Responder};

/// In a capture whose protocol marks no group starts, a pause longer than this between two lines
/// starts a new group, seconds. (A central receives a group's notifications over up to about
/// 120 ms, and groups come about 300 ms apart.)
pub const GROUP_GAP_S: f64 = 0.1;

/// Lap period of a replay with fewer than two groups, seconds (the Begode group period).
pub const GROUP_PERIOD_S: f64 = 0.304;

/// A capture that cannot be replayed.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ReplayError {
    /// No notification at or after the start time.
    #[error("nothing to replay from {from} s on (the capture's last notification is at {end} s)")]
    Empty {
        /// The requested start.
        from: f64,
        /// The last notification's time.
        end: f64,
    },
    /// A line names a characteristic the wheel's profile does not have.
    #[error("line {line}: characteristic {uuid} is not in the wheel's BLE profile")]
    UnknownCharacteristic {
        /// Line number.
        line: usize,
        /// As written.
        uuid: String,
    },
}

/// One notification of the capture, ready to play.
#[derive(Debug, Clone)]
struct Entry {
    t: f64,
    characteristic: CharId,
    bytes: Vec<u8>,
    /// Its time is a group time.
    starts_group: bool,
}

/// Plays a capture by its timestamps and answers writes like the wheel's firmware.
///
/// Time is the caller's: [`advance`](Self::advance) takes the seconds since the replay started
/// and returns what is due by then, so tests run without waiting.
pub struct Replayer {
    entries: Vec<Entry>,
    /// Capture time of the first entry played.
    origin: f64,
    /// Seconds from the first entry of one lap to the first entry of the next.
    period: f64,
    looping: bool,
    next: usize,
    lap: u64,
    /// The decision for the group being played: send its lines or not.
    sending: bool,
    responder: Box<dyn Responder>,
    tx: CharId,
    rx: Vec<CharId>,
    chunk: usize,
    /// Answers sent at once to writes, out at the next advance.
    immediate: Vec<Notification>,
    /// Write lines of the capture left out.
    skipped_writes: usize,
}

impl std::fmt::Debug for Replayer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Replayer").field("entries", &self.entries.len()).field("lap", &self.lap).finish_non_exhaustive()
    }
}

impl Replayer {
    /// Replays the notifications of `capture` on `profile`'s characteristics, from its first one
    /// at or after `from` seconds (that one plays at once), once or forever (`looping`). A lap
    /// starts one group period after the last group of the one before.
    pub fn new(
        capture: &Capture,
        profile: &BleProfile,
        responder: Box<dyn Responder>,
        from: f64,
        looping: bool,
    ) -> Result<Self, ReplayError> {
        let mut entries = Vec::new();
        let mut skipped_writes = 0;
        let mut end = 0.0;
        for line in &capture.lines {
            if line.direction == Direction::Write {
                skipped_writes += 1;
                continue;
            }
            let characteristic = match &line.characteristic {
                None => profile.tx(),
                Some(uuid) => profile
                    .find(uuid)
                    .ok_or_else(|| ReplayError::UnknownCharacteristic { line: line.number, uuid: uuid.clone() })?,
            };
            end = line.t;
            if line.t >= from {
                let starts_group = responder.starts_group(&line.bytes);
                entries.push(Entry { t: line.t, characteristic, bytes: line.bytes.clone(), starts_group });
            }
        }
        let Some(origin) = entries.first().map(|e| e.t) else {
            return Err(ReplayError::Empty { from, end });
        };
        if !entries.iter().any(|e| e.starts_group) {
            for i in 0..entries.len() {
                entries[i].starts_group = i == 0 || entries[i].t - entries[i - 1].t > GROUP_GAP_S;
            }
        }
        let group_times: Vec<f64> = entries.iter().filter(|e| e.starts_group).map(|e| e.t).collect();
        let mut gaps: Vec<f64> = group_times.windows(2).map(|w| w[1] - w[0]).collect();
        gaps.sort_by(f64::total_cmp);
        let gap = gaps.get(gaps.len() / 2).copied().unwrap_or(GROUP_PERIOD_S);
        // The next lap's first group comes one (median) group period after the last group.
        let period = group_times.last().copied().unwrap_or(origin) - origin + gap;
        Ok(Self {
            entries,
            origin,
            period,
            looping,
            next: 0,
            lap: 0,
            sending: true,
            responder,
            tx: profile.tx(),
            rx: profile.characteristics().filter(|(_, _, c)| c.rx).map(|(id, _, _)| id).collect(),
            chunk: profile.chunk(),
            immediate: Vec::new(),
            skipped_writes,
        })
    }

    /// One lap, seconds.
    pub const fn period(&self) -> f64 {
        self.period
    }

    /// Notifications played per lap.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether there is nothing to play (never: [`new`](Self::new) refuses that).
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Write lines of the capture that the replay leaves out.
    pub const fn skipped_writes(&self) -> usize {
        self.skipped_writes
    }

    /// Laps started so far beyond the first (0 during the first).
    pub const fn lap(&self) -> u64 {
        self.lap
    }

    /// Whether everything has been played (never with looping).
    pub fn finished(&self) -> bool {
        !self.looping && self.next == self.entries.len() && self.immediate.is_empty()
    }

    /// When entry `index` of lap `lap` is due, seconds since the start.
    fn due(&self, index: usize, lap: u64) -> f64 {
        self.entries[index].t - self.origin + lap as f64 * self.period
    }

    /// Everything due by `now` (seconds since the replay started), in order.
    pub fn advance(&mut self, now: f64) -> Vec<Notification> {
        let mut out = std::mem::take(&mut self.immediate);
        loop {
            let (next, lap) = if self.next < self.entries.len() {
                (self.next, self.lap)
            } else if self.looping {
                (0, self.lap + 1)
            } else {
                break;
            };
            let at = self.due(next, lap);
            if at > now {
                break;
            }
            (self.next, self.lap) = (next, lap);
            if self.entries[next].starts_group {
                self.sending = match self.responder.group_time() {
                    Group::Send => true,
                    Group::Skip => false,
                    Group::Reply(bytes) => {
                        out.extend(self.pieces(at, &bytes));
                        false
                    }
                };
            }
            if self.sending {
                let entry = &self.entries[next];
                out.push(Notification { at, characteristic: entry.characteristic, bytes: entry.bytes.clone() });
            }
            self.next += 1;
        }
        out
    }

    /// A central wrote `bytes` on `characteristic` at `now`. Writes on an `rx` characteristic go
    /// to the responder; its immediate answers come out of the next [`advance`](Self::advance).
    pub fn write(&mut self, now: f64, characteristic: CharId, bytes: &[u8]) {
        if !self.rx.contains(&characteristic) {
            return;
        }
        for answer in self.responder.write(bytes) {
            let pieces = self.pieces(now, &answer);
            self.immediate.extend(pieces);
        }
    }

    /// `bytes` as notifications on the `tx` characteristic, cut to the module's size.
    fn pieces(&self, at: f64, bytes: &[u8]) -> Vec<Notification> {
        crate::split(bytes, self.chunk)
            .map(|part| Notification { at, characteristic: self.tx, bytes: part.to_vec() })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{self, Direction, Line};

    /// Answers every write with `ok` at the second group time after it, and marks groups by a
    /// first byte of 0xA0 or more.
    #[derive(Default)]
    struct Echo {
        wait: u8,
    }

    impl Responder for Echo {
        fn starts_group(&self, bytes: &[u8]) -> bool {
            bytes[0] >= 0xA0
        }

        fn write(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
            self.wait = 2;
            if bytes == b"now" { vec![b"at once".to_vec()] } else { Vec::new() }
        }

        fn group_time(&mut self) -> Group {
            match self.wait {
                0 => Group::Send,
                1 => {
                    self.wait = 0;
                    Group::Reply(b"ok".to_vec())
                }
                _ => {
                    self.wait -= 1;
                    Group::Skip
                }
            }
        }
    }

    /// Groups of two lines every 0.3 s from t = 1: `A0+g` (20 bytes) then 4 bytes 10 ms later.
    fn capture(groups: u8) -> Capture {
        let lines = (0..groups)
            .flat_map(|g| {
                let t = 1.0 + f64::from(g) * 0.3;
                [(t, vec![0xA0 + g; 20]), (t + 0.01, vec![0x5A; 4])]
            })
            .enumerate()
            .map(|(i, (t, bytes))| Line { number: i + 1, t, direction: Direction::Notify, characteristic: None, bytes })
            .collect();
        Capture { header: capture::Header::default(), lines }
    }

    fn start(capture: &Capture, from: f64, looping: bool) -> Replayer {
        Replayer::new(capture, &BleProfile::hm10("T"), Box::new(Echo::default()), from, looping).unwrap()
    }

    /// What went out by `until`, polling every 10 ms: `g<n>` for a group's first line, text otherwise.
    fn played(replayer: &mut Replayer, writes: &[(f64, &[u8])], until: f64) -> Vec<String> {
        let mut seen = Vec::new();
        let mut writes = writes.iter().peekable();
        for step in 0..=(until * 100.0).round() as u32 {
            let now = f64::from(step) / 100.0;
            for out in replayer.advance(now) {
                match out.bytes.len() {
                    20 => seen.push(format!("g{}", out.bytes[0] - 0xA0)),
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
    fn plays_lines_at_their_times_from_the_first_one() {
        let mut replayer = start(&capture(3), 0.0, false);
        let first = replayer.advance(0.0);
        assert_eq!(first, [Notification { at: 0.0, characteristic: CharId(0), bytes: vec![0xA0; 20] }]);
        assert_eq!(replayer.advance(0.009), Vec::<Notification>::new());
        assert_eq!(replayer.advance(0.011).len(), 1);
        let rest = replayer.advance(10.0);
        assert_eq!(rest.len(), 4);
        assert!((rest[0].at - 0.3).abs() < 1e-9 && (rest[3].at - 0.61).abs() < 1e-9);
        assert!(replayer.finished());
    }

    #[test]
    fn the_responder_decides_at_group_times() {
        let mut replayer = start(&capture(6), 0.0, false);
        // A write at 0.35: the group at 0.6 is skipped, the one at 0.9 replaced by the reply.
        assert_eq!(played(&mut replayer, &[(0.35, b"x")], 1.6), ["g0", "g1", "ok", "g4", "g5"]);
        let mut replayer = start(&capture(4), 0.0, false);
        assert_eq!(played(&mut replayer, &[(0.05, b"now")], 0.06), ["g0", "at once"]);
    }

    #[test]
    fn writes_on_other_characteristics_are_not_the_wheels() {
        let mut replayer = start(&capture(4), 0.0, false);
        replayer.advance(0.05);
        replayer.write(0.05, CharId(1), b"now");
        assert_eq!(replayer.advance(1.0).len(), 6, "nothing changed");
    }

    #[test]
    fn without_group_marks_pauses_make_the_groups() {
        let mut flat = capture(3);
        for line in &mut flat.lines {
            line.bytes[0] = 0x10;
        }
        let replayer = Replayer::new(&flat, &BleProfile::hm10("T"), Box::new(crate::Opaque), 0.0, false).unwrap();
        assert_eq!(replayer.entries.iter().filter(|e| e.starts_group).count(), 3);
    }

    #[test]
    fn loops_one_group_period_after_the_last_group() {
        let mut replayer = start(&capture(3), 0.0, true);
        assert!((replayer.period() - 0.9).abs() < 1e-9, "0.6 to the last group, plus 0.3");
        assert_eq!(replayer.advance(0.89).len(), 6);
        assert_eq!(replayer.lap(), 0);
        let next = replayer.advance(0.901);
        assert_eq!((next.len(), replayer.lap()), (1, 1));
        assert!(!replayer.finished());
    }

    #[test]
    fn starts_from_a_given_time_and_skips_captured_writes() {
        let mut with_write = capture(4);
        with_write.lines.insert(
            2,
            Line { number: 99, t: 1.1, direction: Direction::Write, characteristic: None, bytes: b"V".to_vec() },
        );
        let mut replayer = start(&with_write, 1.5, false);
        assert_eq!(replayer.skipped_writes(), 1);
        assert_eq!(replayer.advance(0.0)[0].bytes, vec![0xA2; 20], "the first group at or after 1.5 s plays at once");
        let empty = Replayer::new(&capture(4), &BleProfile::hm10("T"), Box::new(Echo::default()), 9.0, false);
        assert!(matches!(empty, Err(ReplayError::Empty { .. })));
    }

    #[test]
    fn lines_play_on_the_characteristic_they_name() {
        let text = "0.0 notify:FFE1 a0 01\n0.1 notify:2A00 a1 02\n";
        let error = Replayer::new(
            &capture::parse(text).unwrap(),
            &BleProfile::hm10("T"),
            Box::new(Echo::default()),
            0.0,
            false,
        );
        assert_eq!(error.unwrap_err(), ReplayError::UnknownCharacteristic { line: 2, uuid: "2A00".into() });
        let text = "0.0 notify:0000ffe1-0000-1000-8000-00805f9b34fb a0 01\n";
        let mut replayer = Replayer::new(
            &capture::parse(text).unwrap(),
            &BleProfile::hm10("T"),
            Box::new(Echo::default()),
            0.0,
            false,
        )
        .unwrap();
        assert_eq!(replayer.advance(0.0)[0].characteristic, CharId(0));
    }
}

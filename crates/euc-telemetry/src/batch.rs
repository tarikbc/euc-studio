//! Snapshots packed into one binary message: columnar, little-endian, zero-copy to read.
//!
//! ```text
//! offset  size        field
//!      0  4           magic "EUCT"
//!      4  u16         version (1)
//!      6  u16         flags (bit 0: poses present)
//!      8  u32         n: samples
//!     12  u32         c: channels (the producer's CHANNELS length)
//!     16  u32         b: bodies
//!     20  u32         dropped: samples discarded before this batch because nobody read them
//!     24  u32         seq: batch number, from 0
//!     28  u32         reserved (0)
//!     32  f64[n]      sim time of each sample, seconds
//!         f32[c][n]   one column per channel, in channel id order
//!         f32[n][b][7] poses: for each sample, each body's x y z qw qx qy qz
//! ```
//!
//! The header is 32 bytes and every column starts on an 8-byte boundary, so a reader can view
//! the columns as `Float64Array` and `Float32Array` without copying.

use std::collections::VecDeque;
use std::fmt;

use crate::model::{Pose, Snapshot};
use crate::schema::{CHANNEL_COUNT, CHANNELS};

/// First four bytes of every batch.
pub const MAGIC: [u8; 4] = *b"EUCT";
/// Format version this crate writes and reads.
pub const VERSION: u16 = 1;
/// Header size in bytes.
pub const HEADER_BYTES: usize = 32;
/// Floats per pose: position then quaternion.
pub const POSE_FLOATS: usize = 7;
/// Flag: the batch carries poses.
pub const FLAG_POSES: u16 = 1;

/// Collects snapshots between two sends and encodes them as one batch.
///
/// It holds at most `capacity` samples. When a reader falls behind, the oldest samples are
/// dropped and counted, so the producer never waits and never grows without bound.
#[derive(Debug)]
pub struct BatchWriter {
    bodies: usize,
    capacity: usize,
    seq: u32,
    dropped: u32,
    times: VecDeque<f64>,
    signals: VecDeque<[f32; CHANNEL_COUNT]>,
    poses: VecDeque<[f32; POSE_FLOATS]>,
}

impl BatchWriter {
    /// A writer for snapshots with `bodies` poses, holding at most `capacity` samples.
    pub fn new(bodies: usize, capacity: usize) -> Self {
        let capacity = capacity.max(1);
        Self {
            bodies,
            capacity,
            seq: 0,
            dropped: 0,
            times: VecDeque::with_capacity(capacity),
            signals: VecDeque::with_capacity(capacity),
            poses: VecDeque::with_capacity(capacity * bodies),
        }
    }

    /// Samples waiting.
    pub fn len(&self) -> usize {
        self.times.len()
    }

    /// No samples waiting.
    pub fn is_empty(&self) -> bool {
        self.times.is_empty()
    }

    /// Adds a snapshot. Missing poses read as [`Pose::IDENTITY`]; extra ones are ignored.
    pub fn push(&mut self, snapshot: &Snapshot) {
        if self.times.len() == self.capacity {
            self.times.pop_front();
            self.signals.pop_front();
            self.poses.drain(..self.bodies);
            self.dropped = self.dropped.saturating_add(1);
        }
        self.times.push_back(snapshot.t_s);
        let mut row = [0.0; CHANNEL_COUNT];
        for (value, spec) in row.iter_mut().zip(CHANNELS.iter()) {
            *value = snapshot.signals.get(spec.channel);
        }
        self.signals.push_back(row);
        for i in 0..self.bodies {
            self.poses.push_back(snapshot.bodies.get(i).copied().unwrap_or(Pose::IDENTITY).to_array());
        }
    }

    /// Encodes the waiting samples as one batch and starts the next. `None` when empty.
    pub fn take(&mut self) -> Option<Vec<u8>> {
        if self.times.is_empty() {
            return None;
        }
        let n = self.times.len();
        let size = HEADER_BYTES + 8 * n + 4 * n * CHANNEL_COUNT + 4 * n * self.bodies * POSE_FLOATS;
        let mut out = Vec::with_capacity(size);
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&VERSION.to_le_bytes());
        out.extend_from_slice(&(if self.bodies > 0 { FLAG_POSES } else { 0 }).to_le_bytes());
        for word in [n, CHANNEL_COUNT, self.bodies] {
            out.extend_from_slice(&(word as u32).to_le_bytes());
        }
        out.extend_from_slice(&self.dropped.to_le_bytes());
        out.extend_from_slice(&self.seq.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes());
        for t in &self.times {
            out.extend_from_slice(&t.to_le_bytes());
        }
        for c in 0..CHANNEL_COUNT {
            for row in &self.signals {
                out.extend_from_slice(&row[c].to_le_bytes());
            }
        }
        for pose in &self.poses {
            for value in pose {
                out.extend_from_slice(&value.to_le_bytes());
            }
        }
        debug_assert_eq!(out.len(), size);
        self.times.clear();
        self.signals.clear();
        self.poses.clear();
        self.dropped = 0;
        self.seq = self.seq.wrapping_add(1);
        Some(out)
    }
}

/// A decoded batch (for tests, recorders and Rust consumers; the studio reads batches in place).
#[derive(Debug, Clone, PartialEq)]
pub struct Batch {
    /// Batch number.
    pub seq: u32,
    /// Samples dropped before this batch.
    pub dropped: u32,
    /// Sim time per sample.
    pub t: Vec<f64>,
    /// One column per channel.
    pub channels: Vec<Vec<f32>>,
    /// Bodies per sample.
    pub bodies: usize,
    /// `t.len() * bodies * 7` floats.
    pub poses: Vec<f32>,
}

impl Batch {
    /// Pose of `body` in sample `i`.
    pub fn pose(&self, i: usize, body: usize) -> Option<Pose> {
        let at = (i * self.bodies + body) * POSE_FLOATS;
        let p = self.poses.get(at..at + POSE_FLOATS)?;
        Some(Pose { pos: [p[0], p[1], p[2]], quat: [p[3], p[4], p[5], p[6]] })
    }
}

/// Why bytes are not a batch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than its header or its declared columns.
    Truncated,
    /// Wrong magic.
    Magic,
    /// A version this crate does not read.
    Version(u16),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated => f.write_str("batch is truncated"),
            Self::Magic => f.write_str("not a telemetry batch (bad magic)"),
            Self::Version(v) => write!(f, "telemetry batch version {v} is not supported"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// Reads a batch written by [`BatchWriter::take`].
pub fn decode(bytes: &[u8]) -> Result<Batch, DecodeError> {
    let word = |at: usize| -> Result<u32, DecodeError> {
        bytes.get(at..at + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).ok_or(DecodeError::Truncated)
    };
    let header = bytes.get(..HEADER_BYTES).ok_or(DecodeError::Truncated)?;
    if header[..4] != MAGIC {
        return Err(DecodeError::Magic);
    }
    let version = u16::from_le_bytes([header[4], header[5]]);
    if version != VERSION {
        return Err(DecodeError::Version(version));
    }
    let (n, c, b) = (word(8)? as usize, word(12)? as usize, word(16)? as usize);
    let (dropped, seq) = (word(20)?, word(24)?);
    let floats = |from: usize, count: usize| -> Result<Vec<f32>, DecodeError> {
        let raw = bytes.get(from..from + 4 * count).ok_or(DecodeError::Truncated)?;
        Ok(raw.as_chunks::<4>().0.iter().map(|q| f32::from_le_bytes(*q)).collect())
    };
    let raw_t = bytes.get(HEADER_BYTES..HEADER_BYTES + 8 * n).ok_or(DecodeError::Truncated)?;
    let t = raw_t.as_chunks::<8>().0.iter().map(|q| f64::from_le_bytes(*q)).collect();
    let columns_at = HEADER_BYTES + 8 * n;
    let channels = (0..c).map(|i| floats(columns_at + 4 * n * i, n)).collect::<Result<_, _>>()?;
    let poses = floats(columns_at + 4 * n * c, n * b * POSE_FLOATS)?;
    Ok(Batch { seq, dropped, t, channels, bodies: b, poses })
}

/// Path of the cross-language fixture, relative to the repository root. The studio's tests decode
/// it, so a layout change that is not mirrored in TypeScript fails there.
pub const FIXTURE_PATH: &str = "app/src/engine/fixtures/batch-v1.bin";

/// A small deterministic batch: 3 samples, 2 bodies, sample `i` at `t = i / 120` with
/// `speed_kmh = i + 0.5`, `battery_v = 130`, the rest unmodelled, body 0 at `x = i`.
pub fn fixture() -> Vec<u8> {
    let mut writer = BatchWriter::new(2, 8);
    for i in 0..3u8 {
        let signals = crate::Signals { speed_kmh: f32::from(i) + 0.5, battery_v: 130.0, ..crate::Signals::default() };
        let bodies = vec![Pose { pos: [f32::from(i), 0.0, 0.28], quat: [1.0, 0.0, 0.0, 0.0] }, Pose::IDENTITY];
        writer.push(&Snapshot { t_s: f64::from(i) / 120.0, signals, bodies });
    }
    writer.take().unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Signals;

    #[test]
    fn committed_fixture_is_current() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").join(FIXTURE_PATH);
        let committed = std::fs::read(&path).unwrap_or_default();
        assert!(
            committed == fixture(),
            "{} is stale: run `cargo run -p euc-telemetry --example gen_ts`",
            path.display()
        );
    }
    use crate::schema::Channel;

    fn snapshot(t: f64, speed: f32) -> Snapshot {
        let signals = Signals { speed_kmh: speed, battery_v: 130.0, ..Signals::default() };
        Snapshot {
            t_s: t,
            signals,
            bodies: vec![Pose { pos: [t as f32, 0.0, 0.28], quat: [1.0, 0.0, 0.0, 0.0] }, Pose::IDENTITY],
        }
    }

    #[test]
    fn round_trip() {
        let mut w = BatchWriter::new(2, 16);
        assert!(w.take().is_none(), "nothing to send");
        for i in 0..3 {
            w.push(&snapshot(f64::from(i) / 120.0, i as f32));
        }
        let bytes = w.take().expect("a batch");
        assert_eq!(bytes.len(), HEADER_BYTES + 8 * 3 + 4 * 3 * CHANNEL_COUNT + 4 * 3 * 2 * POSE_FLOATS);
        let batch = decode(&bytes).unwrap();
        assert_eq!((batch.seq, batch.dropped, batch.t.len(), batch.bodies), (0, 0, 3, 2));
        assert_eq!(batch.channels[Channel::SpeedKmh as usize], vec![0.0, 1.0, 2.0]);
        assert_eq!(batch.channels[Channel::BatteryV as usize], vec![130.0; 3]);
        assert!(batch.channels[Channel::PwmPct as usize].iter().all(|v| v.is_nan()));
        assert_eq!(batch.pose(2, 0).unwrap().pos, [2.0 / 120.0, 0.0, 0.28]);
        assert_eq!(batch.pose(1, 1), Some(Pose::IDENTITY));
        assert!(w.is_empty());
        w.push(&snapshot(1.0, 9.0));
        assert_eq!(decode(&w.take().unwrap()).unwrap().seq, 1, "batches are numbered");
    }

    #[test]
    fn columns_are_eight_byte_aligned() {
        let mut w = BatchWriter::new(1, 4);
        w.push(&snapshot(0.0, 0.0));
        let bytes = w.take().unwrap();
        assert_eq!(HEADER_BYTES % 8, 0);
        assert_eq!((HEADER_BYTES + 8) % 4, 0);
        assert_eq!(&bytes[..4], b"EUCT");
    }

    #[test]
    fn a_full_writer_drops_the_oldest_and_counts_them() {
        let mut w = BatchWriter::new(1, 2);
        for i in 0..5 {
            w.push(&snapshot(f64::from(i), i as f32));
        }
        let batch = decode(&w.take().unwrap()).unwrap();
        assert_eq!(batch.t, vec![3.0, 4.0]);
        assert_eq!(batch.dropped, 3);
        assert_eq!(batch.pose(1, 0).unwrap().pos[0], 4.0);
    }

    #[test]
    fn rejects_garbage() {
        assert_eq!(decode(b"short"), Err(DecodeError::Truncated));
        assert_eq!(decode(&[0u8; 32]), Err(DecodeError::Magic));
        let mut bad = vec![0u8; 32];
        bad[..4].copy_from_slice(b"EUCT");
        bad[4] = 9;
        assert_eq!(decode(&bad), Err(DecodeError::Version(9)));
        let mut w = BatchWriter::new(0, 1);
        w.push(&snapshot(0.0, 0.0));
        let bytes = w.take().unwrap();
        assert_eq!(decode(&bytes[..bytes.len() - 1]), Err(DecodeError::Truncated));
    }
}

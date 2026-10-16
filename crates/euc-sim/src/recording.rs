//! Recordings: a run's snapshots and events in one file the studio can replay.
//!
//! ```text
//! offset  size  field
//!      0  4     magic "EUCR"
//!      4  u32   version (1)
//!      8  u32   h: header length
//!     12  h     header, JSON: {"wheel", "firmware", "scenario", "sample_hz", "bodies", "channels"}
//!         then records until the end of the file, each:
//!         u8    kind: 1 = batch (euc-telemetry's "EUCT" format), 2 = events (JSON array)
//!         u32   n: length
//!         n     bytes
//! ```
//!
//! All integers little-endian. Batches hold consecutive samples in time order; the events
//! record comes last.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use euc_telemetry::batch::BatchWriter;
use euc_telemetry::{BodyInfo, CHANNELS, ChannelSpec, Event, Snapshot};
use serde::Serialize;

/// First four bytes of a recording.
pub const MAGIC: [u8; 4] = *b"EUCR";
/// Format version this module writes.
pub const VERSION: u32 = 1;
/// Record kind: a telemetry batch.
pub const RECORD_BATCH: u8 = 1;
/// Record kind: the run's events as a JSON array.
pub const RECORD_EVENTS: u8 = 2;

/// Samples per batch record (one second at 100 Hz).
const SAMPLES_PER_BATCH: usize = 100;

/// What a recording is of.
#[derive(Debug, Clone, Serialize)]
pub struct Header<'a> {
    /// Wheel profile id.
    pub wheel: &'a str,
    /// Firmware label.
    pub firmware: &'a str,
    /// Scenario name, if one ran.
    pub scenario: Option<&'a str>,
    /// Samples per second.
    pub sample_hz: f64,
    /// The bodies, in pose order.
    pub bodies: &'a [BodyInfo],
    /// The channel schema, in batch column order.
    pub channels: &'a [ChannelSpec],
}

/// Writes a recording as the run goes.
pub struct Recorder {
    out: BufWriter<File>,
    batch: BatchWriter,
}

impl Recorder {
    /// Creates `path` and writes the header (the channel list is filled in).
    pub fn create(path: &Path, header: &Header<'_>) -> io::Result<Self> {
        let mut out = BufWriter::new(File::create(path)?);
        let header = Header { channels: &CHANNELS, ..header.clone() };
        let json = serde_json::to_vec(&header)?;
        out.write_all(&MAGIC)?;
        out.write_all(&VERSION.to_le_bytes())?;
        out.write_all(&len32(json.len())?.to_le_bytes())?;
        out.write_all(&json)?;
        Ok(Self { out, batch: BatchWriter::new(header.bodies.len(), SAMPLES_PER_BATCH) })
    }

    /// Adds a sample.
    pub fn push(&mut self, snapshot: &Snapshot) -> io::Result<()> {
        self.batch.push(snapshot);
        if self.batch.len() >= SAMPLES_PER_BATCH {
            self.flush_batch()?;
        }
        Ok(())
    }

    /// Writes what is left and the events, and closes the file.
    pub fn finish(mut self, events: &[Event]) -> io::Result<()> {
        self.flush_batch()?;
        self.record(RECORD_EVENTS, &serde_json::to_vec(events)?)?;
        self.out.flush()
    }

    fn flush_batch(&mut self) -> io::Result<()> {
        match self.batch.take() {
            Some(bytes) => self.record(RECORD_BATCH, &bytes),
            None => Ok(()),
        }
    }

    fn record(&mut self, kind: u8, bytes: &[u8]) -> io::Result<()> {
        self.out.write_all(&[kind])?;
        self.out.write_all(&len32(bytes.len())?.to_le_bytes())?;
        self.out.write_all(bytes)
    }
}

fn len32(n: usize) -> io::Result<u32> {
    u32::try_from(n).map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "record over 4 GiB"))
}

#[cfg(test)]
mod tests {
    use euc_telemetry::batch::decode;
    use euc_telemetry::{EventKind, Signals};

    use super::*;

    #[test]
    fn writes_header_batches_and_events() {
        let dir = std::env::temp_dir().join(format!("euc-recording-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("run.eucr");
        let header =
            Header { wheel: "w", firmware: "f", scenario: Some("s"), sample_hz: 100.0, bodies: &[], channels: &[] };
        let mut recorder = Recorder::create(&path, &header).unwrap();
        for i in 0..150 {
            recorder
                .push(&Snapshot { t_s: f64::from(i) / 100.0, signals: Signals::default(), bodies: vec![] })
                .unwrap();
        }
        let event = Event { t_s: 1.0, kind: EventKind::Scenario, text: "x".into() };
        recorder.finish(&[event]).unwrap();

        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(&bytes[..4], b"EUCR");
        let h = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
        let json: serde_json::Value = serde_json::from_slice(&bytes[12..12 + h]).unwrap();
        assert_eq!(json["channels"].as_array().unwrap().len(), CHANNELS.len());
        let mut at = 12 + h;
        let mut kinds = Vec::new();
        let mut samples = 0;
        while at < bytes.len() {
            let n = u32::from_le_bytes(bytes[at + 1..at + 5].try_into().unwrap()) as usize;
            let body = &bytes[at + 5..at + 5 + n];
            if bytes[at] == RECORD_BATCH {
                samples += decode(body).unwrap().t.len();
            }
            kinds.push(bytes[at]);
            at += 5 + n;
        }
        assert_eq!((kinds, samples), (vec![RECORD_BATCH, RECORD_BATCH, RECORD_EVENTS], 150));
        std::fs::remove_dir_all(dir).unwrap();
    }
}

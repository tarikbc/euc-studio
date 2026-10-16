//! `report.json`: what a run did, in the shape of the reference harness's report.

use std::collections::BTreeMap;

use euc_proto_begode::{FOOTER, FRAME_LEN, HEADER, decode_text};
use serde::Serialize;

use crate::machine::{Machine, exception_name};

/// Text bursts separated by more than this much silence are reported separately.
const TEXT_GAP_S: f64 = 0.02;
/// An injection's reply window when no later injection closes it.
const REPLY_WINDOW_S: f64 = 2.0;

/// The run report.
#[derive(Debug, Clone, Serialize)]
pub struct Report {
    /// Simulated seconds.
    pub sim_seconds: f64,
    /// Why the run stopped for good, if it did.
    pub halted: Option<String>,
    /// First time execution reached each marker.
    pub markers: Vec<Marker>,
    /// Timeline events, NVIC enables excluded.
    pub milestones: Vec<Event>,
    /// NVIC enables.
    pub nvic_enables: Vec<Event>,
    /// Exceptions taken, by name.
    pub exceptions: BTreeMap<String, Count>,
    /// What the controller sent over BLE.
    pub ble: Ble,
    /// Each injected command and what came back before the next one.
    pub injections: Vec<Injection>,
    /// Problems the run hit.
    pub blockers: Vec<BlockerOut>,
    /// Register traffic per component, busiest first.
    pub peripherals: Vec<Traffic>,
    /// Where poll loops were fast-forwarded.
    pub poll_fast_forward: Vec<PollSkip>,
    /// Counters.
    pub performance: Performance,
}

/// A marker and when it was first reached.
#[derive(Debug, Clone, Serialize)]
pub struct Marker {
    /// Name from the stock map.
    pub name: String,
    /// Address, hex.
    pub addr: String,
    /// Simulated milliseconds, `None` if never reached.
    pub first_ms: Option<f64>,
}

/// A timeline event.
#[derive(Debug, Clone, Serialize)]
pub struct Event {
    /// Simulated milliseconds.
    pub t_ms: f64,
    /// What happened.
    pub what: String,
}

/// An exception count.
#[derive(Debug, Clone, Serialize)]
pub struct Count {
    /// Times taken.
    pub count: u64,
    /// Per simulated second.
    pub per_sim_s: f64,
}

/// The BLE output stream.
#[derive(Debug, Clone, Serialize)]
pub struct Ble {
    /// Bytes sent.
    pub tx_bytes: usize,
    /// Telemetry frames.
    pub frames: usize,
    /// Frames by type (`"00"`).
    pub frames_by_type: BTreeMap<String, usize>,
    /// When the first frame started, simulated milliseconds.
    pub first_frame_ms: Option<f64>,
    /// Start time of every frame, simulated milliseconds.
    pub frame_times_ms: Vec<f64>,
    /// Text between frames (debug log, command replies), split at silences.
    pub text_segments: Vec<Text>,
    /// The first frames.
    pub first_frames: Vec<FrameOut>,
    /// The last frames.
    pub last_frames: Vec<FrameOut>,
}

/// A text burst.
#[derive(Debug, Clone, Serialize)]
pub struct Text {
    /// Start, simulated milliseconds.
    pub t_ms: f64,
    /// Decoded text (GBK, as the firmware writes it).
    pub text: String,
}

/// One frame.
#[derive(Debug, Clone, Serialize)]
pub struct FrameOut {
    /// Start, simulated milliseconds.
    pub t_ms: f64,
    /// The 24 bytes.
    pub hex: String,
}

/// An injected command and its answer.
#[derive(Debug, Clone, Serialize)]
pub struct Injection {
    /// The bytes sent.
    pub bytes: String,
    /// When, simulated seconds.
    pub at_s: f64,
    /// Seconds of output attributed to it.
    pub window_s: f64,
    /// Text that came back.
    pub reply_text: Vec<String>,
    /// Frames sent in the window.
    pub frames_in_window: usize,
}

/// A blocker.
#[derive(Debug, Clone, Serialize)]
pub struct BlockerOut {
    /// Kind.
    pub kind: String,
    /// What happened.
    pub text: String,
    /// Occurrences.
    pub count: u64,
    /// The run stopped on it.
    pub fatal: bool,
    /// First occurrence, simulated milliseconds.
    pub t_ms: f64,
}

/// Register traffic of one component.
#[derive(Debug, Clone, Serialize)]
pub struct Traffic {
    /// Component name.
    pub name: String,
    /// Reads.
    pub reads: u64,
    /// Writes.
    pub writes: u64,
    /// Busiest registers (`"SR R"`).
    pub top_registers: Vec<(String, u64)>,
}

/// A fast-forwarded poll site.
#[derive(Debug, Clone, Serialize)]
pub struct PollSkip {
    /// PC of the loop, hex.
    pub pc: String,
    /// Register polled.
    pub register: String,
    /// Simulated milliseconds skipped.
    pub skipped_ms: f64,
}

/// Counters.
#[derive(Debug, Clone, Serialize)]
pub struct Performance {
    /// Instructions retired in full quanta.
    pub instructions: u64,
    /// Quanta.
    pub quanta: u64,
    /// Quanta cut short.
    pub early_quanta: u64,
    /// MMIO accesses.
    pub mmio_accesses: u64,
    /// Exceptions taken.
    pub exceptions: u64,
    /// Simulated milliseconds skipped by poll fast-forward.
    pub poll_fast_forward_ms: f64,
    /// Lowest exception frame, hex.
    pub deepest_exception_frame: Option<String>,
    /// Resets after power-on.
    pub resets: u32,
}

impl Report {
    /// Builds the report; `injections` are the `(seconds, bytes)` the run was given.
    pub fn new(machine: &Machine, injections: &[(f64, Vec<u8>)]) -> Self {
        let sys = &machine.sys;
        let hz = machine.hclk() as f64;
        let ms = |cycle: u64| round(cycle as f64 / hz * 1000.0, 3);
        let event = |&(cycle, ref what): &(u64, String)| Event { t_ms: ms(cycle), what: what.clone() };
        let (nvic, rest): (Vec<_>, Vec<_>) =
            sys.diag.events.iter().partition(|(_, what)| what.starts_with("NVIC enable"));

        let mut markers: Vec<Marker> = sys
            .marker_names()
            .map(|(addr, name)| Marker {
                name: name.clone(),
                addr: format!("0x{addr:08X}"),
                first_ms: sys.diag.reached.iter().find(|&&(a, _)| a == addr).map(|&(_, cycle)| ms(cycle)),
            })
            .collect();
        markers.sort_by(|a, b| a.first_ms.unwrap_or(f64::MAX).total_cmp(&b.first_ms.unwrap_or(f64::MAX)));

        let sim_s = machine.cycles as f64 / hz;
        let exceptions = machine
            .exceptions_taken()
            .iter()
            .enumerate()
            .filter(|&(_, &count)| count > 0)
            .map(|(exc, &count)| {
                let per_sim_s = if sim_s > 0.0 { round(count as f64 / sim_s, 1) } else { 0.0 };
                (exception_name(exc as u16, &sys.irq_names), Count { count, per_sim_s })
            })
            .collect();

        let stream = &machine.board().ble_tx;
        let ble = ble(stream, hz);
        let injections = injections_out(stream, injections, hz);

        let blockers = sys
            .diag
            .blockers
            .iter()
            .map(|b| BlockerOut {
                kind: b.kind.clone(),
                text: b.text.clone(),
                count: b.count,
                fatal: b.fatal,
                t_ms: ms(b.cycle),
            })
            .collect();

        let mut skips: Vec<PollSkip> = sys
            .diag
            .poll_skips
            .iter()
            .map(|(&(pc, _), (cycles, register))| PollSkip {
                pc: format!("0x{pc:08X}"),
                register: register.clone(),
                skipped_ms: ms(*cycles),
            })
            .collect();
        skips.sort_by(|a, b| b.skipped_ms.total_cmp(&a.skipped_ms));
        skips.truncate(15);
        let skipped: u64 = sys.diag.poll_skips.values().map(|(cycles, _)| cycles).sum();

        Self {
            sim_seconds: round(sim_s, 4),
            halted: sys.diag.halted.clone(),
            markers,
            milestones: rest.iter().map(|e| event(e)).take(200).collect(),
            nvic_enables: nvic.iter().map(|e| event(e)).collect(),
            exceptions,
            ble,
            injections,
            blockers,
            peripherals: traffic(machine),
            poll_fast_forward: skips,
            performance: Performance {
                instructions: machine.instr,
                quanta: machine.quanta,
                early_quanta: machine.early_quanta,
                mmio_accesses: sys.diag.mmio_count,
                exceptions: machine.exceptions_taken().iter().sum(),
                poll_fast_forward_ms: round(skipped as f64 / hz * 1000.0, 1),
                deepest_exception_frame: sys.diag.min_frame.map(|frame| format!("0x{frame:08X}")),
                resets: machine.resets,
            },
        }
    }
}

fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// A piece of the BLE stream: a frame or text, with its byte offset.
enum Piece<'a> {
    Frame(usize, &'a [u8]),
    Text(usize, &'a [u8]),
}

/// Splits the stream into frames (`55 AA … 5A 5A 5A 5A`) and the text between them.
fn split(data: &[u8]) -> Vec<Piece<'_>> {
    let mut pieces = Vec::new();
    let (mut i, mut start) = (0, 0);
    while i < data.len() {
        let frame = data.get(i..i + FRAME_LEN).filter(|f| f[..2] == HEADER && f[FRAME_LEN - 4..] == FOOTER);
        if let Some(frame) = frame {
            if start < i {
                pieces.push(Piece::Text(start, &data[start..i]));
            }
            pieces.push(Piece::Frame(i, frame));
            i += FRAME_LEN;
            start = i;
        } else {
            i += 1;
        }
    }
    if start < data.len() {
        pieces.push(Piece::Text(start, &data[start..]));
    }
    pieces
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

fn ble(stream: &[(u64, u8)], hz: f64) -> Ble {
    let bytes: Vec<u8> = stream.iter().map(|&(_, b)| b).collect();
    let ms = |offset: usize| round(stream[offset].0 as f64 / hz * 1000.0, 2);
    let gap = (TEXT_GAP_S * hz) as u64;
    let mut frames = Vec::new();
    let mut texts = Vec::new();
    for piece in split(&bytes) {
        match piece {
            Piece::Frame(offset, frame) => frames.push((offset, frame)),
            Piece::Text(offset, text) => {
                let mut burst = 0;
                for k in 1..=text.len() {
                    if k == text.len() || stream[offset + k].0 - stream[offset + k - 1].0 > gap {
                        texts.push(Text { t_ms: ms(offset + burst), text: decode_text(&text[burst..k]) });
                        burst = k;
                    }
                }
            }
        }
    }
    let mut by_type = BTreeMap::new();
    for (_, frame) in &frames {
        *by_type.entry(format!("{:02X}", frame[18])).or_default() += 1;
    }
    let out = |&(offset, frame): &(usize, &[u8])| FrameOut { t_ms: ms(offset), hex: hex(frame) };
    Ble {
        tx_bytes: bytes.len(),
        frames: frames.len(),
        frames_by_type: by_type,
        first_frame_ms: frames.first().map(|&(offset, _)| ms(offset)),
        frame_times_ms: frames.iter().map(|&(offset, _)| ms(offset)).collect(),
        text_segments: texts.into_iter().take(60).collect(),
        first_frames: frames.iter().take(12).map(out).collect(),
        last_frames: frames.iter().rev().take(8).rev().map(out).collect(),
    }
}

fn injections_out(stream: &[(u64, u8)], injections: &[(f64, Vec<u8>)], hz: f64) -> Vec<Injection> {
    let mut starts: Vec<f64> = injections.iter().map(|&(at, _)| at).collect();
    starts.sort_by(f64::total_cmp);
    injections
        .iter()
        .map(|(at, bytes)| {
            let until =
                starts.iter().copied().find(|&t| t > *at).unwrap_or(at + REPLY_WINDOW_S).min(at + REPLY_WINDOW_S);
            let window: Vec<u8> = stream
                .iter()
                .filter(|&&(cycle, _)| cycle as f64 >= at * hz && (cycle as f64) < until * hz)
                .map(|&(_, b)| b)
                .collect();
            let pieces = split(&window);
            Injection {
                bytes: String::from_utf8_lossy(bytes).into_owned(),
                at_s: *at,
                window_s: round(until - at, 2),
                reply_text: pieces
                    .iter()
                    .filter_map(|p| match p {
                        Piece::Text(_, text) => Some(decode_text(text)),
                        Piece::Frame(..) => None,
                    })
                    .collect(),
                frames_in_window: pieces.iter().filter(|p| matches!(p, Piece::Frame(..))).count(),
            }
        })
        .collect()
}

fn traffic(machine: &Machine) -> Vec<Traffic> {
    let sys = &machine.sys;
    let mut per: BTreeMap<String, (u64, u64, BTreeMap<String, u64>)> = BTreeMap::new();
    for (&(id, off, write), &count) in &sys.diag.accesses {
        let (name, reg) = sys.register_label(id, off);
        let entry = per.entry(name).or_default();
        if write {
            entry.1 += count;
        } else {
            entry.0 += count;
        }
        *entry.2.entry(format!("{reg} {}", if write { "W" } else { "R" })).or_default() += count;
    }
    let mut out: Vec<Traffic> = per
        .into_iter()
        .map(|(name, (reads, writes, regs))| {
            let mut top: Vec<(String, u64)> = regs.into_iter().collect();
            top.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
            top.truncate(12);
            Traffic { name, reads, writes, top_registers: top }
        })
        .collect();
    out.sort_by(|a, b| (b.reads + b.writes).cmp(&(a.reads + a.writes)).then_with(|| a.name.cmp(&b.name)));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_frames_from_text() {
        let mut data = b"hi".to_vec();
        let mut frame = vec![0x55, 0xAA];
        frame.extend([0; 16]);
        frame.extend([0x04, 0x00, 0x5A, 0x5A, 0x5A, 0x5A]);
        data.extend(&frame);
        data.extend(b"GW1");
        let pieces = split(&data);
        assert!(matches!(pieces[..], [Piece::Text(0, b"hi"), Piece::Frame(2, _), Piece::Text(26, b"GW1")]));
    }

    #[test]
    fn text_bursts_split_at_silence_and_replies_attach_to_injections() {
        let hz = 72_000_000.0;
        let at = |s: f64| (s * hz) as u64;
        let mut stream: Vec<(u64, u8)> =
            b"GW1".iter().enumerate().map(|(i, &b)| (at(2.7) + i as u64 * 6250, b)).collect();
        stream.extend(b"NAME".iter().enumerate().map(|(i, &b)| (at(3.7) + i as u64 * 6250, b)));
        let report = ble(&stream, hz);
        let texts: Vec<_> = report.text_segments.iter().map(|t| (t.t_ms, t.text.as_str())).collect();
        assert_eq!(texts, [(2700.0, "GW1"), (3700.0, "NAME")]);
        let replies = injections_out(&stream, &[(2.6, b"V".to_vec()), (3.6, b"N".to_vec())], hz);
        assert_eq!((replies[0].reply_text.clone(), replies[0].window_s), (vec!["GW1".to_owned()], 1.0));
        assert_eq!(replies[1].reply_text, ["NAME"]);
    }
}

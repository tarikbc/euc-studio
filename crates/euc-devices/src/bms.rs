//! The Begode smart BMS as the controller sees it: a 5-byte poll on a UART, answered with four
//! 44-byte blocks.

use std::time::Duration;

use euc_core::{Build, ConfigError, Ctx, PackReading, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::fault::{DeviceFault, Schedule};
use crate::rng::Rng;

/// The controller's poll, sent on USART3 (9600 8N1) about every 3 s. Its meaning is UNKNOWN; the
/// BMS answers any occurrence of it.
pub const REQUEST: [u8; 5] = [0x1C, 0x03, 0x40, 0x31, 0xFE];

/// Bytes in one block of the reply.
pub const BLOCK_LEN: usize = 44;

/// Blocks in one reply: two packs of two 16-cell boards.
pub const BLOCKS: usize = 4;

/// Bytes in one reply.
pub const REPLY_LEN: usize = BLOCK_LEN * BLOCKS;

/// Status word of a healthy block, as seen on a real wheel.
pub const STATUS_OK: u16 = 0x7D01;

/// Cells in series per pack (two packs in parallel).
const SERIES: f64 = 32.0;

/// Packs: blocks `2k` and `2k + 1` are pack `k`.
const PACKS: usize = BLOCKS / 2;

const PORT_RX: u8 = 0;
const PORT_TX: u8 = 1;

/// What one BMS board reports. Fields are big-endian on the wire, in this order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Block {
    /// Bytes 0-5: three temperature sensors, °C.
    pub temps_c: [i16; 3],
    /// Bytes 6-37: sixteen cell voltages, mV.
    pub cells_mv: [u16; 16],
    /// Bytes 38-39: status word; see [`STATUS_OK`].
    pub status: u16,
    /// Bytes 40-41: current, raw. The controller converts it with `raw × 90 / 32.769` to mA
    /// (unit INFERRED); see [`Block::with_current`].
    pub current: i16,
    /// Bytes 42-43: meaning UNKNOWN; 651 on a real wheel.
    pub word42: u16,
}

impl Block {
    /// A healthy block with every cell at `cell_mv`, sensors at `temps_c`, no current.
    pub const fn uniform(cell_mv: u16, temps_c: [i16; 3]) -> Self {
        Self { temps_c, cells_mv: [cell_mv; 16], status: STATUS_OK, current: 0, word42: 651 }
    }

    /// The block with its current set from amperes, by the inverse of the controller's scaling.
    #[must_use]
    pub fn with_current(self, amps: f64) -> Self {
        let raw = (amps * 1000.0 * 32.769 / 90.0).round().clamp(f64::from(i16::MIN), f64::from(i16::MAX));
        Self { current: raw as i16, ..self }
    }

    fn encode(&self, out: &mut Vec<u8>) {
        let words = self.temps_c.iter().map(|&t| t as u16).chain(self.cells_mv).chain([
            self.status,
            self.current as u16,
            self.word42,
        ]);
        for word in words {
            out.extend(word.to_be_bytes());
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// Seconds from the request's last byte to the first reply byte.
    #[serde(default = "default_delay")]
    delay_s: f64,
    /// The three temperature sensors of every block, °C.
    #[serde(default = "default_temps")]
    temps: [i16; 3],
    /// Whether the BMS is wired and answering.
    #[serde(default = "default_connected")]
    connected: bool,
}

const fn default_delay() -> f64 {
    0.005
}

const fn default_temps() -> [i16; 3] {
    [23, 22, 23]
}

const fn default_connected() -> bool {
    true
}

/// The smart BMS on the controller's BMS UART.
///
/// Feed it every byte the controller transmits with [`receive`](Self::receive); when the last five
/// bytes are [`REQUEST`] it answers with [`reply`](Self::reply), [`REPLY_LEN`] bytes the board
/// delivers to the controller's receiver [`reply_delay`](Self::reply_delay) later, paced at the
/// line rate. A disconnected BMS never answers.
///
/// Reply layout per block, parsed by the controller with no header, checksum or length check:
/// three temperatures, sixteen cells, status, current, one unknown word (all big-endian `u16`).
/// Blocks 0 and 1 are pack A, blocks 2 and 3 pack B.
///
/// As a board component (kind `smart-bms`): bytes arrive on `"rx"` and the reply leaves on
/// `"tx"`, with the plant's packs at the moment of the request ([`set_packs`](Self::set_packs)), or
/// every cell at the plant's battery voltage when the plant does not model packs. Parameters
/// `delay_s` (0.005), `temps` (`[23, 22, 23]`, the maintainer's wheel) and `connected` (true).
#[derive(Debug, Clone)]
pub struct SmartBms {
    /// The four boards' values.
    pub blocks: [Block; BLOCKS],
    /// Time from the end of the request to the first reply byte.
    pub reply_delay: Duration,
    /// Whether the BMS is wired and answering.
    pub connected: bool,
    window: [u8; 5],
    requests: u64,
    faults: Schedule,
}

impl SmartBms {
    /// A BMS reporting a balanced 2 × 32S pack at `pack_v` volts with sensors at `temps_c`, 5 ms
    /// reply delay.
    pub fn new(pack_v: f64, temps_c: [i16; 3]) -> Self {
        let mut bms = Self {
            blocks: [Block::uniform(0, temps_c); BLOCKS],
            reply_delay: Duration::from_millis(5),
            connected: true,
            window: [0; 5],
            requests: 0,
            faults: Schedule::default(),
        };
        bms.set_pack_voltage(pack_v);
        bms
    }

    /// Builds the `smart-bms` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let reply_delay = Duration::try_from_secs_f64(config.delay_s)
            .map_err(|_| build.error(format!("delay_s {} is not a duration", config.delay_s)))?;
        let mut bms = Self::new(0.0, config.temps);
        bms.reply_delay = reply_delay;
        bms.connected = config.connected;
        Ok(Box::new(bms))
    }

    /// Sets every cell to `pack_v / 32` (two packs in parallel, 32 cells in series each), rounded
    /// to the nearest millivolt with ties to even, as the reference harness does.
    pub fn set_pack_voltage(&mut self, pack_v: f64) {
        let cell_mv = (pack_v / SERIES * 1000.0).round_ties_even().clamp(0.0, f64::from(u16::MAX)) as u16;
        for block in &mut self.blocks {
            block.cells_mv = [cell_mv; 16];
        }
    }

    /// Sets the blocks from what each pack's boards measure: blocks `2k` and `2k + 1` are pack `k`,
    /// with its cells 0-15 and 16-31 and its temperature sensors 0-2 and 3-5; both report the pack
    /// current (discharge positive: the sign the real boards use is UNKNOWN). Packs, cells and
    /// sensors that `packs` lacks keep their values.
    pub fn set_packs(&mut self, packs: &[PackReading]) {
        for (k, pack) in packs.iter().enumerate().take(PACKS) {
            for half in 0..2 {
                let block = &mut self.blocks[2 * k + half];
                for (slot, &mv) in block.cells_mv.iter_mut().zip(pack.cells_mv.iter().skip(16 * half)) {
                    *slot = mv;
                }
                for (slot, &t) in block.temps_c.iter_mut().zip(pack.temps_c.iter().skip(3 * half)) {
                    *slot = t.round_ties_even().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
                }
                *block = block.with_current(pack.current_a);
            }
        }
    }

    /// One byte from the controller. True when it completes a request that the BMS answers.
    pub fn receive(&mut self, byte: u8) -> bool {
        self.window.rotate_left(1);
        self.window[4] = byte;
        if self.window != REQUEST {
            return false;
        }
        self.window = [0; 5];
        self.requests += 1;
        self.connected
    }

    /// The reply to a request.
    pub fn reply(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(REPLY_LEN);
        for block in &self.blocks {
            block.encode(&mut out);
        }
        out
    }

    /// Requests seen since creation, answered or not.
    pub const fn requests(&self) -> u64 {
        self.requests
    }

    /// Adds a BMS fault (`bms_silent`, `bms_garbled`) for the cycles `from..until` (`None`: to
    /// the end of the run). As a board component the BMS applies it to every poll it answers
    /// inside the window (the cycle of the poll's last byte).
    pub fn add_fault(&mut self, from: u64, until: Option<u64>, fault: DeviceFault) -> Result<(), String> {
        self.faults.add("smart-bms", from, until, fault)
    }

    /// The reply to a poll at `now` with the active faults: `None` if the BMS stays silent.
    fn reply_at(&self, now: u64) -> Option<Vec<u8>> {
        let mut reply = self.reply();
        let mut rng = Rng::new(now ^ 0x424D_5300);
        for fault in self.faults.active(now) {
            match fault {
                DeviceFault::BmsSilent => return None,
                DeviceFault::BmsGarbled { byte_error_rate, missing_bytes } => {
                    for byte in &mut reply {
                        if rng.unit() <= byte_error_rate {
                            *byte = rng.next_u64() as u8;
                        }
                    }
                    reply.truncate(reply.len().saturating_sub(usize::from(missing_bytes)));
                }
                DeviceFault::ImuNoise { .. } | DeviceFault::ImuBias { .. } | DeviceFault::ImuStuck => {}
            }
        }
        Some(reply)
    }
}

impl Peripheral for SmartBms {
    fn reset(&mut self, kind: ResetKind, _ctx: &mut Ctx<'_>) {
        // The BMS has its own supply; only a power cycle of the wheel restarts it.
        if kind == ResetKind::Power {
            self.window = [0; 5];
        }
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let (PORT_RX, Signal::Byte { value, at }) = (port, signal) else { return };
        if !self.receive(value as u8) {
            return;
        }
        self.set_pack_voltage(ctx.plant().battery_v());
        let packs: Vec<PackReading> = (0..PACKS).map_while(|k| ctx.plant().pack(k)).collect();
        self.set_packs(&packs);
        let Some(reply) = self.reply_at(at) else { return };
        let delay = self.reply_delay.as_nanos() * u128::from(ctx.hclk()) / 1_000_000_000;
        let start = at + u64::try_from(delay).unwrap_or(u64::MAX);
        for byte in reply {
            ctx.emit(PORT_TX, Signal::Byte { value: u16::from(byte), at: start });
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "rx" => Some(PORT_RX),
            "tx" => Some(PORT_TX),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    fn be16(bytes: &[u8], at: usize) -> u16 {
        u16::from_be_bytes([bytes[at], bytes[at + 1]])
    }

    #[test]
    fn answers_the_poll_with_four_blocks() {
        let mut bms = SmartBms::new(130.0, [23; 3]);
        bms.blocks[2].temps_c = [21, -5, 30];
        bms.blocks[3] = bms.blocks[3].with_current(-1.5);
        for byte in [0x00, 0x1C, 0x03, 0x40, 0x31] {
            assert!(!bms.receive(byte));
        }
        assert!(bms.receive(0xFE));
        let reply = bms.reply();
        assert_eq!(reply.len(), 176);
        assert_eq!(bms.requests(), 1);
        // Block 0: temperatures, then 16 cells of 130 V / 32 = 4062.5 mV, rounded to even.
        assert_eq!([be16(&reply, 0), be16(&reply, 2), be16(&reply, 4)], [23, 23, 23]);
        assert!((0..16).all(|k| be16(&reply, 6 + 2 * k) == 4062));
        assert_eq!((be16(&reply, 38), be16(&reply, 40), be16(&reply, 42)), (STATUS_OK, 0, 651));
        // Block 2 temperatures are signed.
        assert_eq!(be16(&reply, 2 * BLOCK_LEN + 2) as i16, -5);
        // Block 3 current: -1.5 A -> -546 raw, and back through the controller's formula.
        let raw = be16(&reply, 3 * BLOCK_LEN + 40) as i16;
        assert_eq!(raw, -546);
        assert!((f64::from(raw) * 90.0 / 32.769 / 1000.0 + 1.5).abs() < 0.002);
    }

    #[test]
    fn a_disconnected_bms_counts_requests_but_stays_silent() {
        let mut bms = SmartBms::new(120.0, [20; 3]);
        bms.connected = false;
        let answered = REQUEST.iter().chain(&REQUEST).filter(|&&b| bms.receive(b)).count();
        assert_eq!(answered, 0);
        assert_eq!(bms.requests(), 2);
    }

    #[test]
    fn packs_fill_their_two_blocks() {
        let mut bms = SmartBms::new(130.0, [23; 3]);
        let mut cells: Vec<u16> = (0..32).map(|c| 4000 + c).collect();
        cells[30] = 3500;
        let pack = PackReading { cells_mv: cells, temps_c: vec![24.4, 24.6, 25.0, 31.5, 30.0, 29.0], current_a: 12.5 };
        bms.set_packs(&[pack]);
        let [a0, a1, b0, _] = bms.blocks;
        assert_eq!((a0.cells_mv[0], a0.cells_mv[15], a1.cells_mv[0], a1.cells_mv[14]), (4000, 4015, 4016, 3500));
        assert_eq!((a0.temps_c, a1.temps_c), ([24, 25, 25], [32, 30, 29]), "rounded half to even");
        assert_eq!(a0.current, a1.current, "both boards of a pack see its current");
        assert!((f64::from(a1.current) * 90.0 / 32.769 / 1000.0 - 12.5).abs() < 0.003);
        assert_eq!(b0, Block::uniform(4062, [23; 3]), "pack B not given: unchanged");
    }

    #[test]
    fn faults_silence_or_garble_the_reply() {
        let mut bms = SmartBms::new(130.0, [23; 3]);
        bms.add_fault(100, Some(200), DeviceFault::BmsSilent).unwrap();
        bms.add_fault(300, None, DeviceFault::BmsGarbled { byte_error_rate: 0.2, missing_bytes: 6 }).unwrap();
        assert!(bms.add_fault(0, None, DeviceFault::ImuStuck).is_err());
        assert_eq!(bms.reply_at(50), Some(bms.reply()));
        assert_eq!(bms.reply_at(150), None, "silent");
        let garbled = bms.reply_at(300).unwrap();
        assert_eq!(garbled.len(), REPLY_LEN - 6);
        let wrong = garbled.iter().zip(bms.reply()).filter(|(a, b)| **a != *b).count();
        assert!((15..60).contains(&wrong), "about 20 % of 170 bytes: {wrong}");
        assert_eq!(garbled, bms.reply_at(300).unwrap(), "deterministic");
    }

    #[test]
    fn requests_split_by_other_bytes_do_not_count() {
        let mut bms = SmartBms::new(120.0, [20; 3]);
        assert!(![0x1C, 0x03, 0x40, 0x00, 0x31, 0xFE].iter().any(|&b| bms.receive(b)));
        assert_eq!(bms.requests(), 0);
    }

    #[test]
    fn as_a_component_it_replies_on_tx_after_the_delay_with_plant_cells() {
        let mut host = TestHost::new(72_000_000);
        host.plant.battery_v = 129.7;
        let mut bms = SmartBms::new(0.0, [23, 22, 23]);
        for (i, byte) in REQUEST.into_iter().enumerate() {
            let at = 1_000 + 75_000 * i as u64;
            bms.input(bms.port("rx").unwrap(), Signal::Byte { value: u16::from(byte), at }, &mut host.ctx());
        }
        assert_eq!(host.emitted.len(), REPLY_LEN);
        let start = 1_000 + 75_000 * 4 + 360_000;
        assert!(
            host.emitted
                .iter()
                .all(|&(port, signal)| port == PORT_TX && matches!(signal, Signal::Byte { at, .. } if at == start))
        );
        let bytes: Vec<u8> = host
            .emitted
            .iter()
            .map(|(_, s)| if let Signal::Byte { value, .. } = s { *value as u8 } else { 0 })
            .collect();
        assert_eq!(be16(&bytes, 6), 4053, "129.7 V / 32");
        assert_eq!([be16(&bytes, 0), be16(&bytes, 2)], [23, 22]);
    }
}

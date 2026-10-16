//! The simulator's own components: the board logic around the controller and the DWT.

use std::collections::HashMap;

use euc_core::{Ctx, Peripheral, Registry, ResetKind, Scheduler, Signal};

/// Port numbers of [`BoardLogic`].
mod port {
    pub(super) const BLE_RX: u8 = 0;
    pub(super) const BLE_TX: u8 = 1;
    pub(super) const LATCH: u8 = 2;
    pub(super) const HALL_A: u8 = 3;
    pub(super) const HALL_B: u8 = 4;
    pub(super) const HALL_C: u8 = 5;
    pub(super) const BUTTON: u8 = 6;
}

/// Something the board does at a set time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// A phone sends these bytes over BLE.
    Inject(Vec<u8>),
    /// The rider lets go of the power button.
    ReleaseButton,
    /// The battery is switched off and on.
    PowerCycle,
}

/// Everything around the controller that the firmware talks to through pins and the BLE UART:
/// the phone (capturing what the controller sends, injecting commands), the power button and
/// latch, and the hall sensors' static levels. Timed actions run in `(cycle, order)` order, like
/// the reference harness's timeline, which is why this component is the last one on the board.
pub struct BoardLogic {
    hall: u8,
    button_active_low: bool,
    release_after: u64,
    button_pressed: bool,
    latch_time: Option<u64>,
    actions: Scheduler<Action>,
    /// Bytes the controller sent over BLE, with the cycle each started on the wire.
    pub ble_tx: Vec<(u64, u8)>,
}

const TIMELINE: u32 = 0;

impl BoardLogic {
    /// Board logic with static hall code `hall`; the rider lets go of the button
    /// `release_after` cycles after the firmware sets the power latch.
    pub fn new(hall: u8, button_active_low: bool, release_after: u64) -> Self {
        Self {
            hall,
            button_active_low,
            release_after,
            button_pressed: true,
            latch_time: None,
            actions: Scheduler::new(),
            ble_tx: Vec::new(),
        }
    }

    /// Schedules `action` at cycle `at` (before the run starts).
    pub fn at(&mut self, at: u64, action: Action) {
        self.actions.schedule(at, action);
    }

    fn arm(&mut self, ctx: &mut Ctx<'_>) {
        match self.actions.next_at() {
            Some(at) => ctx.schedule(at, TIMELINE),
            None => ctx.cancel(TIMELINE),
        }
    }

    /// Drives the hall and button pins from the plant.
    fn apply_inputs(&self, ctx: &mut Ctx<'_>) {
        for (bit, out) in [(0, port::HALL_A), (1, port::HALL_B), (2, port::HALL_C)] {
            ctx.emit(out, Signal::Level(self.hall >> bit & 1 == 1));
        }
        ctx.emit(port::BUTTON, Signal::Level(self.button_pressed != self.button_active_low));
    }
}

impl Peripheral for BoardLogic {
    fn reset(&mut self, kind: ResetKind, ctx: &mut Ctx<'_>) {
        if kind == ResetKind::Power {
            self.latch_time = None;
            self.button_pressed = true;
            self.apply_inputs(ctx);
        }
        self.arm(ctx);
    }

    fn on_event(&mut self, _tag: u32, ctx: &mut Ctx<'_>) {
        while let Some((_, action)) = self.actions.pop_due(ctx.now()) {
            match action {
                Action::Inject(bytes) => {
                    ctx.log(format!("BLE RX inject {:?}", String::from_utf8_lossy(&bytes)));
                    for byte in bytes {
                        ctx.emit(port::BLE_TX, Signal::Byte { value: u16::from(byte), at: ctx.now() });
                    }
                }
                Action::ReleaseButton => {
                    ctx.log("power button released");
                    self.button_pressed = false;
                    self.apply_inputs(ctx);
                }
                Action::PowerCycle => ctx.request_reset(ResetKind::Power, "power cycle"),
            }
        }
        self.arm(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        match (port, signal) {
            (port::BLE_RX, Signal::Byte { value, at }) => self.ble_tx.push((at, value as u8)),
            (port::LATCH, Signal::Level(high)) => {
                let off = if high { "" } else { " (firmware powers the wheel off)" };
                ctx.log(format!("power latch -> {}{off}", u8::from(high)));
                if high && self.latch_time.is_none() {
                    self.latch_time = Some(ctx.now());
                    if self.button_pressed {
                        self.actions.schedule(ctx.now() + self.release_after, Action::ReleaseButton);
                        self.arm(ctx);
                    }
                }
            }
            _ => {}
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        Some(match name {
            "ble_rx" => port::BLE_RX,
            "ble_tx" => port::BLE_TX,
            "latch" => port::LATCH,
            "hall_a" => port::HALL_A,
            "hall_b" => port::HALL_B,
            "hall_c" => port::HALL_C,
            "button" => port::BUTTON,
            _ => return None,
        })
    }
}

/// The Cortex-M3 data watchpoint and trace unit: `CYCCNT` counts HCLK; other registers read
/// back what was written.
#[derive(Default)]
pub struct Dwt {
    regs: HashMap<u32, u32>,
}

impl Peripheral for Dwt {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        self.regs.clear();
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        match off {
            0x004 => ctx.now() as u32,
            _ => self.regs.get(&off).copied().unwrap_or(0),
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, _ctx: &mut Ctx<'_>) {
        let reg = self.regs.entry(off).or_default();
        *reg = (*reg & !mask) | (value & mask);
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        match off {
            0x000 => Some("CTRL"),
            0x004 => Some("CYCCNT"),
            _ => None,
        }
    }
}

/// Registers the simulator's own kinds (`cortex-m3.dwt`).
pub fn register(registry: &mut Registry) {
    registry.register("cortex-m3.dwt", |_| Ok(Box::new(Dwt::default())));
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    #[test]
    fn power_on_drives_the_plant_pins() {
        let mut host = TestHost::new(72_000_000);
        let mut board = BoardLogic::new(5, true, 7_200_000);
        board.reset(ResetKind::Power, &mut host.ctx());
        let levels: Vec<_> = host.emitted.iter().map(|&(port, signal)| (port, signal)).collect();
        assert_eq!(
            levels,
            [
                (port::HALL_A, Signal::Level(true)),
                (port::HALL_B, Signal::Level(false)),
                (port::HALL_C, Signal::Level(true)),
                (port::BUTTON, Signal::Level(false)),
            ],
            "hall code 5, button held (active low)"
        );
    }

    #[test]
    fn latch_schedules_the_button_release() {
        let mut host = TestHost::new(72_000_000);
        let mut board = BoardLogic::new(5, true, 7_200_000);
        board.reset(ResetKind::Power, &mut host.ctx());
        host.now = 1_000;
        board.input(port::LATCH, Signal::Level(true), &mut host.ctx());
        assert_eq!(host.scheduled[&TIMELINE], 7_201_000);
        host.emitted.clear();
        host.now = 7_201_000;
        board.on_event(TIMELINE, &mut host.ctx());
        assert_eq!(host.emitted.last(), Some(&(port::BUTTON, Signal::Level(true))), "released: high");
        assert!(!host.scheduled.contains_key(&TIMELINE));
    }

    #[test]
    fn injections_go_out_on_ble_tx_and_tx_is_captured() {
        let mut host = TestHost::new(72_000_000);
        let mut board = BoardLogic::new(5, true, 7_200_000);
        board.at(500, Action::Inject(b"VN".to_vec()));
        board.reset(ResetKind::Power, &mut host.ctx());
        host.emitted.clear();
        host.now = 503;
        board.on_event(TIMELINE, &mut host.ctx());
        assert_eq!(
            host.emitted,
            [
                (port::BLE_TX, Signal::Byte { value: 0x56, at: 503 }),
                (port::BLE_TX, Signal::Byte { value: 0x4E, at: 503 })
            ]
        );
        board.input(port::BLE_RX, Signal::Byte { value: 0x55, at: 900 }, &mut host.ctx());
        assert_eq!(board.ble_tx, [(900, 0x55)]);
    }
}

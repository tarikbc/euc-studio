//! Universal synchronous/asynchronous receiver-transmitter, asynchronous mode (RM0008 §27).

use std::collections::VecDeque;

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::rcc::{Apb, Rcc};
use crate::util::{Rate, clear_w0, merge};

const SR: u32 = 0x00;
const DR: u32 = 0x04;
const BRR: u32 = 0x08;
const CR1: u32 = 0x0C;
const CR2: u32 = 0x10;
const CR3: u32 = 0x14;
const GTPR: u32 = 0x18;

// SR
const PE: u32 = 1 << 0;
const FE: u32 = 1 << 1;
const NE: u32 = 1 << 2;
const ORE: u32 = 1 << 3;
const IDLE: u32 = 1 << 4;
const RXNE: u32 = 1 << 5;
const TC: u32 = 1 << 6;
const TXE: u32 = 1 << 7;
/// Flags cleared by writing 0: CTS, LBD, TC, RXNE.
const SR_W0: u32 = 0x360;
/// Flags cleared by reading SR, then DR.
const SR_READ_CLEARED: u32 = PE | FE | NE | ORE | IDLE;
// CR1
const SBK: u32 = 1 << 0;
const RE: u32 = 1 << 2;
const TE: u32 = 1 << 3;
const IDLEIE: u32 = 1 << 4;
const RXNEIE: u32 = 1 << 5;
const TCIE: u32 = 1 << 6;
const TXEIE: u32 = 1 << 7;
const PEIE: u32 = 1 << 8;
const M: u32 = 1 << 12;
const UE: u32 = 1 << 13;
// CR3
const EIE: u32 = 1 << 0;
const DMAR: u32 = 1 << 6;
const DMAT: u32 = 1 << 7;

const TAG_TX: u32 = 0;
const TAG_RX: u32 = 1;
const TAG_IDLE: u32 = 2;

const PORT_TX: u8 = 0;
const PORT_RX: u8 = 1;

/// DMA request lines carry the number of the port with the same name (`"rx"`, `"tx"`).
const DMA_RX: u8 = PORT_RX;
const DMA_TX: u8 = PORT_TX;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// The RCC instance that gives the bus clock.
    clock: String,
    /// APB2 for USART1, APB1 for the others.
    bus: Apb,
}

/// A USART in asynchronous mode.
///
/// **Transmit**: a write to `DR` moves into the shift register at once when it is idle (`TXE`
/// stays 1) and goes out on `"tx"` as a [`Signal::Byte`] stamped with the cycle its start bit
/// begins; otherwise it waits in `DR` (`TXE` = 0) until the shift register frees. `TC` sets when the
/// last frame ends. Frame time = (start + 8 or 9 data bits + `STOP` bits) × `BRR` / PCLK, from the
/// configured clocks. With `TE` or `UE` off the byte waits in `DR` and the write is reported.
///
/// **Receive**: bytes arriving on `"rx"` are queued and complete one frame after they start
/// (back-to-back bytes are paced at the line rate), so a device can hand over a whole reply at
/// once. A completed byte sets `RXNE`, or `ORE` if the previous byte is still unread (the new byte
/// is lost); `IDLE` follows one frame after the last byte. With `UE` or `RE` off the byte is
/// dropped and reported.
///
/// Flag clearing follows the hardware sequences (`TC`/`RXNE` by writing 0 or by the data access,
/// `IDLE`/`ORE`/`NE`/`FE`/`PE` by reading `SR` then `DR`). Interrupt role `irq`; DMA requests
/// `"rx"` (`DMAR`) and `"tx"` (`DMAT`).
///
/// Not modeled: parity generation and errors, break, synchronous, LIN, smartcard, infrared, hardware
/// flow control, multiprocessor wake-up.
#[derive(Debug, Clone)]
pub struct Usart {
    rcc: DeviceId,
    bus: Apb,
    irq: Option<(u16, bool)>,
    sr: u32,
    rdr: u16,
    brr: u32,
    cr1: u32,
    cr2: u32,
    cr3: u32,
    gtpr: u32,
    /// Byte waiting in `DR` for the shift register.
    tdr: Option<u16>,
    /// Cycle the frame in the shift register ends, while one is being sent.
    shift_end: Option<u64>,
    /// Bytes on their way in: value and the cycle their reception completes.
    rx_queue: VecDeque<(u16, u64)>,
    /// Cycle the last queued byte completes.
    rx_end: u64,
    sr_read: bool,
    sent: u64,
    received: u64,
}

impl Usart {
    /// A USART after reset on `bus`, clocked by `rcc`, on interrupt `irq`.
    pub fn new(rcc: DeviceId, bus: Apb, irq: Option<u16>) -> Self {
        Self {
            rcc,
            bus,
            irq: irq.map(|irq| (irq, false)),
            sr: TXE | TC,
            rdr: 0,
            brr: 0,
            cr1: 0,
            cr2: 0,
            cr3: 0,
            gtpr: 0,
            tdr: None,
            shift_end: None,
            rx_queue: VecDeque::new(),
            rx_end: 0,
            sr_read: false,
            sent: 0,
            received: 0,
        }
    }

    /// Builds the `stm32f1.usart` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        Ok(Box::new(Self::new(build.id_of(&config.clock)?, config.bus, build.irq("irq").ok())))
    }

    /// Bytes sent and received since reset.
    pub const fn traffic(&self) -> (u64, u64) {
        (self.sent, self.received)
    }

    /// Engine cycles one frame takes at the current settings.
    fn frame_cycles(&self, ctx: &mut Ctx<'_>) -> u64 {
        let hclk = ctx.hclk();
        let pclk = ctx.peer::<Rcc>(self.rcc).map_or(hclk, |rcc| u64::from(rcc.clocks().pclk(self.bus)));
        let data = if self.cr1 & M != 0 { 9 } else { 8 };
        let stop_halves = [2, 1, 4, 3][(self.cr2 >> 12 & 3) as usize];
        let halves = 2 * (1 + data) + stop_halves;
        Rate::new(hclk, 2 * pclk).cycles(halves * u64::from(self.brr.max(16))).max(1)
    }

    fn data_mask(&self) -> u16 {
        if self.cr1 & M != 0 { 0x1FF } else { 0xFF }
    }

    fn transmitting(&self) -> bool {
        self.cr1 & (UE | TE) == UE | TE
    }

    /// Moves `value` into the shift register at `at` and puts it on the wire.
    fn shift_out(&mut self, value: u16, at: u64, ctx: &mut Ctx<'_>) {
        let end = at + self.frame_cycles(ctx);
        self.shift_end = Some(end);
        self.sent += 1;
        ctx.emit(PORT_TX, Signal::Byte { value, at });
        ctx.schedule(end, TAG_TX);
        self.sr |= TXE;
        self.sr &= !TC;
        if self.cr3 & DMAT != 0 {
            ctx.dma_request(DMA_TX);
        }
    }

    /// The shift register finished its frame.
    fn frame_sent(&mut self, ctx: &mut Ctx<'_>) {
        let Some(end) = self.shift_end.take() else { return };
        match self.tdr.take() {
            Some(value) if self.transmitting() => self.shift_out(value, end, ctx),
            pending => {
                self.tdr = pending;
                if pending.is_none() {
                    self.sr |= TC;
                }
            }
        }
    }

    fn write_dr(&mut self, value: u16, ctx: &mut Ctx<'_>) {
        self.sr &= !TC;
        self.sr_read = false;
        if !self.transmitting() {
            ctx.blocker("usart-tx-disabled", format!("DR write 0x{value:02X} with UE or TE off (held in DR)"), false);
        }
        if self.transmitting() && self.shift_end.is_none() {
            self.shift_out(value, ctx.now(), ctx);
        } else {
            if self.sr & TXE == 0 {
                ctx.blocker("usart-tx-overwrite", format!("DR write 0x{value:02X} while TXE = 0 (byte lost)"), false);
            }
            self.tdr = Some(value);
            self.sr &= !TXE;
        }
    }

    /// A byte completed reception.
    fn byte_received(&mut self, ctx: &mut Ctx<'_>) {
        let Some((value, _)) = self.rx_queue.pop_front() else { return };
        if self.cr1 & (UE | RE) != UE | RE {
            ctx.blocker("usart-rx-disabled", format!("byte 0x{value:02X} arrived with UE or RE off (dropped)"), false);
        } else if self.sr & RXNE != 0 {
            self.sr |= ORE;
        } else {
            self.rdr = value & self.data_mask();
            self.sr |= RXNE;
            self.received += 1;
            if self.cr3 & DMAR != 0 {
                ctx.dma_request(DMA_RX);
            }
        }
        if let Some(&(_, end)) = self.rx_queue.front() {
            ctx.schedule(end, TAG_RX);
        } else {
            let idle = ctx.now() + self.frame_cycles(ctx);
            ctx.schedule(idle, TAG_IDLE);
        }
    }

    fn update_irq(&mut self, ctx: &mut Ctx<'_>) {
        let Some((irq, level)) = &mut self.irq else { return };
        let (sr, cr1) = (self.sr, self.cr1);
        let on = |flags: u32, enable: u32| sr & flags != 0 && cr1 & enable != 0;
        let high = on(TXE, TXEIE)
            || on(TC, TCIE)
            || on(IDLE, IDLEIE)
            || on(RXNE | ORE, RXNEIE)
            || on(PE, PEIE)
            || self.cr3 & (EIE | DMAR) == EIE | DMAR && sr & (FE | NE | ORE) != 0;
        if high != *level {
            *level = high;
            ctx.set_irq(*irq, high);
        }
    }
}

impl Peripheral for Usart {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new(self.rcc, self.bus, self.irq.map(|(irq, _)| irq));
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        let value = match off {
            SR => {
                self.sr_read = true;
                self.sr
            }
            DR => {
                let mut clear = RXNE;
                if self.sr_read {
                    clear |= SR_READ_CLEARED;
                }
                self.sr &= !clear;
                self.sr_read = false;
                u32::from(self.rdr)
            }
            BRR => self.brr,
            CR1 => self.cr1,
            CR2 => self.cr2,
            CR3 => self.cr3,
            GTPR => self.gtpr,
            _ => 0,
        };
        if off == DR {
            self.update_irq(ctx);
        }
        value
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        match off {
            SR => self.sr = clear_w0(self.sr, value, mask, SR_W0),
            DR => self.write_dr((value & mask) as u16 & self.data_mask(), ctx),
            BRR => self.brr = merge(self.brr, value, mask) & 0xFFFF,
            CR1 => {
                // SBK is cleared by hardware after the break; breaks are not modeled.
                self.cr1 = merge(self.cr1, value, mask) & 0x3FFF & !SBK;
                if self.transmitting()
                    && self.shift_end.is_none()
                    && let Some(pending) = self.tdr.take()
                {
                    self.shift_out(pending, ctx.now(), ctx);
                }
            }
            CR2 => self.cr2 = merge(self.cr2, value, mask) & 0x7F7F,
            CR3 => {
                let old = self.cr3;
                self.cr3 = merge(self.cr3, value, mask) & 0x7FF;
                if old & DMAT == 0 && self.cr3 & DMAT != 0 && self.sr & TXE != 0 {
                    ctx.dma_request(DMA_TX);
                }
            }
            GTPR => self.gtpr = merge(self.gtpr, value, mask) & 0xFFFF,
            _ => return,
        }
        self.update_irq(ctx);
    }

    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        match tag {
            TAG_TX => self.frame_sent(ctx),
            TAG_RX => self.byte_received(ctx),
            TAG_IDLE if self.rx_queue.is_empty() => self.sr |= IDLE,
            _ => {}
        }
        self.update_irq(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let (PORT_RX, Signal::Byte { value, at }) = (port, signal) else { return };
        let start = at.max(self.rx_end).max(ctx.now());
        self.rx_end = start + self.frame_cycles(ctx);
        self.rx_queue.push_back((value, self.rx_end));
        if self.rx_queue.len() == 1 {
            ctx.cancel(TAG_IDLE);
            ctx.schedule(self.rx_end, TAG_RX);
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "tx" => Some(PORT_TX),
            "rx" => Some(PORT_RX),
            _ => None,
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            SR => "SR",
            DR => "DR",
            BRR => "BRR",
            CR1 => "CR1",
            CR2 => "CR2",
            CR3 => "CR3",
            GTPR => "GTPR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const RCC: DeviceId = 1;

    /// USART1 at 115200 (BRR 0x271 on the 72 MHz APB2), interrupt 37, as on the Begode Extreme.
    fn usart1() -> (TestHost, Usart) {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        for (off, value) in [(0x00, 0x0101_0083), (0x04, 0x001D_8402)] {
            rcc.write(off, value, u32::MAX, &mut host.ctx());
        }
        host.peers.insert(RCC, Box::new(rcc));
        let mut usart = Usart::new(RCC, Apb::Apb2, Some(37));
        usart.write(BRR, 0x271, u32::MAX, &mut host.ctx());
        usart.write(CR1, UE | TE | RE, u32::MAX, &mut host.ctx());
        (host, usart)
    }

    /// 10 bits at 72 MHz / 625 = 6250 cycles per frame.
    const FRAME: u64 = 6250;

    fn fire(usart: &mut Usart, host: &mut TestHost, tag: u32) -> u64 {
        let at = host.scheduled.remove(&tag).expect("event scheduled");
        host.now = at;
        usart.on_event(tag, &mut host.ctx());
        at
    }

    #[test]
    fn transmit_paces_bytes_at_the_baud_rate() {
        let (mut host, mut usart) = usart1();
        host.now = 100;
        usart.write(DR, u32::from(b'G'), 0xFF, &mut host.ctx());
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & (TXE | TC), TXE, "shifting, TDR free");
        usart.write(DR, u32::from(b'W'), 0xFF, &mut host.ctx());
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & TXE, 0, "second byte waits in DR");
        assert_eq!(host.emitted, [(PORT_TX, Signal::Byte { value: 0x47, at: 100 })]);
        assert_eq!(fire(&mut usart, &mut host, TAG_TX), 100 + FRAME);
        assert_eq!(host.emitted[1], (PORT_TX, Signal::Byte { value: 0x57, at: 100 + FRAME }));
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & (TXE | TC), TXE);
        fire(&mut usart, &mut host, TAG_TX);
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & (TXE | TC), TXE | TC, "all sent");
        assert_eq!(usart.traffic(), (2, 0));
    }

    #[test]
    fn txe_and_tc_interrupts() {
        let (mut host, mut usart) = usart1();
        usart.write(CR1, UE | TE | RE | TXEIE, u32::MAX, &mut host.ctx());
        assert!(host.irq(37), "TXE is 1 after reset");
        usart.write(DR, 1, 0xFF, &mut host.ctx());
        usart.write(DR, 2, 0xFF, &mut host.ctx());
        assert!(!host.irq(37));
        usart.write(CR1, UE | TE | RE | TCIE, u32::MAX, &mut host.ctx());
        fire(&mut usart, &mut host, TAG_TX);
        assert!(!host.irq(37));
        fire(&mut usart, &mut host, TAG_TX);
        assert!(host.irq(37), "TC");
        usart.write(SR, !TC, u32::MAX, &mut host.ctx());
        assert!(!host.irq(37));
    }

    #[test]
    fn received_bytes_are_paced_and_overrun_when_unread() {
        let (mut host, mut usart) = usart1();
        usart.write(CR1, UE | TE | RE | RXNEIE, u32::MAX, &mut host.ctx());
        for &b in b"Vk" {
            usart.input(PORT_RX, Signal::Byte { value: u16::from(b), at: 1000 }, &mut host.ctx());
        }
        assert_eq!(fire(&mut usart, &mut host, TAG_RX), 1000 + FRAME);
        assert!(host.irq(37));
        assert_eq!(usart.read(DR, u32::MAX, &mut host.ctx()), u32::from(b'V'));
        assert!(!host.irq(37), "reading DR clears RXNE");
        assert_eq!(fire(&mut usart, &mut host, TAG_RX), 1000 + 2 * FRAME);
        usart.input(PORT_RX, Signal::Byte { value: 0x21, at: 0 }, &mut host.ctx());
        fire(&mut usart, &mut host, TAG_RX);
        let sr = usart.read(SR, u32::MAX, &mut host.ctx());
        assert_eq!(sr & (RXNE | ORE), RXNE | ORE, "'k' unread when '!' completed");
        assert_eq!(usart.read(DR, u32::MAX, &mut host.ctx()), u32::from(b'k'));
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & ORE, 0, "SR then DR clears ORE");
        assert_eq!(fire(&mut usart, &mut host, TAG_IDLE), 1000 + 4 * FRAME);
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & IDLE, IDLE);
    }

    #[test]
    fn receive_dma_request_per_byte() {
        let (mut host, mut usart) = usart1();
        usart.write(CR3, DMAR, u32::MAX, &mut host.ctx());
        usart.input(PORT_RX, Signal::Byte { value: 0x1C, at: 0 }, &mut host.ctx());
        fire(&mut usart, &mut host, TAG_RX);
        assert_eq!(host.dma_requests, [DMA_RX]);
    }

    #[test]
    fn disabled_receiver_drops_and_disabled_transmitter_holds() {
        let (mut host, mut usart) = usart1();
        usart.write(CR1, UE, u32::MAX, &mut host.ctx());
        usart.input(PORT_RX, Signal::Byte { value: 0x56, at: 0 }, &mut host.ctx());
        fire(&mut usart, &mut host, TAG_RX);
        assert_eq!(usart.read(SR, u32::MAX, &mut host.ctx()) & RXNE, 0);
        usart.write(DR, 0x41, 0xFF, &mut host.ctx());
        assert_eq!(host.emitted, []);
        let kinds: Vec<_> = host.blockers.iter().map(|b| b.0).collect();
        assert_eq!(kinds, ["usart-rx-disabled", "usart-tx-disabled"]);
        usart.write(CR1, UE | TE, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted.len(), 1, "enabling TE sends the held byte");
    }

    #[test]
    fn frame_time_follows_brr_word_length_and_stop_bits() {
        let (mut host, mut usart) = usart1();
        // USART3 on APB1 (36 MHz) at 9600: BRR 0xEA6 = 3750 → 10 bits = 75000 cycles of 72 MHz.
        let mut usart3 = Usart::new(RCC, Apb::Apb1, None);
        usart3.write(BRR, 0xEA6, u32::MAX, &mut host.ctx());
        assert_eq!(usart3.frame_cycles(&mut host.ctx()), 75_000);
        usart.write(CR1, UE | TE | M, u32::MAX, &mut host.ctx());
        usart.write(CR2, 2 << 12, u32::MAX, &mut host.ctx());
        assert_eq!(usart.frame_cycles(&mut host.ctx()), 12 * 625, "start + 9 data + 2 stop bits");
        usart.write(CR2, 1 << 12, u32::MAX, &mut host.ctx());
        assert_eq!(usart.frame_cycles(&mut host.ctx()), (21 * 625_u64).div_ceil(2), "half a stop bit");
    }
}

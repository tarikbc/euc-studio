//! Serial peripheral interface, master mode (RM0008 §25).

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind, Signal};

use crate::util::{NoParams, clear_w0, merge};

const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const SR: u32 = 0x08;
const DR: u32 = 0x0C;
const CRCPR: u32 = 0x10;
const RXCRCR: u32 = 0x14;
const TXCRCR: u32 = 0x18;
const I2SCFGR: u32 = 0x1C;
const I2SPR: u32 = 0x20;

// CR1
const MSTR: u32 = 1 << 2;
const SPE: u32 = 1 << 6;
const DFF: u32 = 1 << 11;
// CR2
const RXDMAEN: u32 = 1 << 0;
const TXDMAEN: u32 = 1 << 1;
const ERRIE: u32 = 1 << 5;
const RXNEIE: u32 = 1 << 6;
const TXEIE: u32 = 1 << 7;
// SR
const RXNE: u32 = 1 << 0;
const TXE: u32 = 1 << 1;
const CRCERR: u32 = 1 << 4;
const MODF: u32 = 1 << 5;
const OVR: u32 = 1 << 6;

const PORT_MOSI: u8 = 0;
const PORT_MISO: u8 = 1;
/// DMA requests `"rx"` and `"tx"`.
const DMA_RX: u8 = 2;
const DMA_TX: u8 = 3;

/// An SPI port in master mode, with the device on the other side reached through wires.
///
/// A write to `DR` with `SPE` and `MSTR` set exchanges one frame at once: the data goes out on
/// `"mosi"` as a [`Signal::Byte`], `RXNE` sets, and `DR` reads back what the device answers on
/// `"miso"` before the CPU's next instruction, or all ones when nothing answers (MISO pulled up,
/// as with an absent IMU). A frame received while `RXNE` is still set raises `OVR`, which clears
/// by reading `DR` then `SR`. `TXE` stays 1 and `BSY` 0: transfers take no simulated time.
/// Written with the port off, the data waits in `DR` (`TXE` = 0) until `SPE` is set, and the write
/// is reported. Interrupt role `irq`; DMA requests `"rx"` (`RXDMAEN`) and `"tx"` (`TXDMAEN`).
///
/// Chip select is a GPIO pin of the board, not part of this model.
///
/// Not modeled: transfer time (`BR`), slave mode, NSS management, CRC calculation (the CRC
/// registers read back), I2S.
#[derive(Debug, Clone)]
pub struct Spi {
    irq: Option<(u16, bool)>,
    cr1: u32,
    cr2: u32,
    sr: u32,
    rdr: u16,
    /// Frame written while the port was off.
    tdr: Option<u16>,
    /// The last frame went out and its answer may still arrive on `"miso"`.
    awaiting: bool,
    crcpr: u32,
    i2scfgr: u32,
    i2spr: u32,
    sr_read_after_dr: bool,
    frames: u64,
}

impl Spi {
    /// An SPI port after reset, on interrupt `irq`.
    pub fn new(irq: Option<u16>) -> Self {
        Self {
            irq: irq.map(|irq| (irq, false)),
            cr1: 0,
            cr2: 0,
            sr: TXE,
            rdr: 0,
            tdr: None,
            awaiting: false,
            crcpr: 7,
            i2scfgr: 0,
            i2spr: 2,
            sr_read_after_dr: false,
            frames: 0,
        }
    }

    /// Builds the `stm32f1.spi` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new(build.irq("irq").ok())))
    }

    /// Frames exchanged since reset.
    pub const fn frames(&self) -> u64 {
        self.frames
    }

    fn mask(&self) -> u16 {
        if self.cr1 & DFF != 0 { 0xFFFF } else { 0xFF }
    }

    fn enabled(&self) -> bool {
        self.cr1 & (SPE | MSTR) == SPE | MSTR
    }

    fn exchange(&mut self, value: u16, ctx: &mut Ctx<'_>) {
        if self.sr & RXNE != 0 {
            self.sr |= OVR;
        }
        self.rdr = self.mask();
        self.sr |= RXNE | TXE;
        self.awaiting = true;
        self.frames += 1;
        ctx.emit(PORT_MOSI, Signal::Byte { value, at: ctx.now() });
        if self.cr2 & RXDMAEN != 0 {
            ctx.dma_request(DMA_RX);
        }
        if self.cr2 & TXDMAEN != 0 {
            ctx.dma_request(DMA_TX);
        }
    }

    fn update_irq(&mut self, ctx: &mut Ctx<'_>) {
        let Some((irq, level)) = &mut self.irq else { return };
        let (sr, cr2) = (self.sr, self.cr2);
        let high = sr & TXE != 0 && cr2 & TXEIE != 0
            || sr & RXNE != 0 && cr2 & RXNEIE != 0
            || sr & (OVR | MODF | CRCERR) != 0 && cr2 & ERRIE != 0;
        if high != *level {
            *level = high;
            ctx.set_irq(*irq, high);
        }
    }
}

impl Peripheral for Spi {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new(self.irq.map(|(irq, _)| irq));
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        let value = match off {
            CR1 => self.cr1,
            CR2 => self.cr2,
            SR => {
                let sr = self.sr;
                if self.sr_read_after_dr {
                    self.sr &= !OVR;
                    self.sr_read_after_dr = false;
                }
                sr
            }
            DR => {
                self.sr &= !RXNE;
                self.awaiting = false;
                self.sr_read_after_dr = self.sr & OVR != 0;
                u32::from(self.rdr)
            }
            CRCPR => self.crcpr,
            I2SCFGR => self.i2scfgr,
            I2SPR => self.i2spr,
            _ => 0,
        };
        self.update_irq(ctx);
        value
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        match off {
            CR1 => {
                self.cr1 = merge(self.cr1, value, mask) & 0xFFFF;
                if self.enabled()
                    && let Some(pending) = self.tdr.take()
                {
                    self.exchange(pending, ctx);
                }
            }
            CR2 => self.cr2 = merge(self.cr2, value, mask) & 0xE7,
            SR => self.sr = clear_w0(self.sr, value, mask, CRCERR),
            DR => {
                let data = (value & mask) as u16 & self.mask();
                if self.enabled() {
                    self.exchange(data, ctx);
                } else {
                    ctx.blocker(
                        "spi-disabled",
                        format!("DR write 0x{data:02X} with SPE or MSTR off (held in DR)"),
                        false,
                    );
                    self.tdr = Some(data);
                    self.sr &= !TXE;
                }
            }
            CRCPR => self.crcpr = merge(self.crcpr, value, mask) & 0xFFFF,
            I2SCFGR => self.i2scfgr = merge(self.i2scfgr, value, mask) & 0xFBF,
            I2SPR => self.i2spr = merge(self.i2spr, value, mask) & 0x3FF,
            _ => return,
        }
        self.update_irq(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        if let (PORT_MISO, Signal::Byte { value, .. }) = (port, signal)
            && self.awaiting
        {
            self.rdr = value & self.mask();
            self.awaiting = false;
            self.update_irq(ctx);
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "mosi" => Some(PORT_MOSI),
            "miso" => Some(PORT_MISO),
            "rx" => Some(DMA_RX),
            "tx" => Some(DMA_TX),
            _ => None,
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            CR1 => "CR1",
            CR2 => "CR2",
            SR => "SR",
            DR => "DR",
            CRCPR => "CRCPR",
            RXCRCR => "RXCRCR",
            TXCRCR => "TXCRCR",
            I2SCFGR => "I2SCFGR",
            I2SPR => "I2SPR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    /// SPI3 as the Begode Extreme sets it up: master, mode 3, 8-bit, enabled.
    fn spi3() -> (TestHost, Spi) {
        let mut host = TestHost::new(72_000_000);
        let mut spi = Spi::new(Some(51));
        spi.write(CR1, SPE | MSTR | 0x3 | 3 << 3 | 0x300, u32::MAX, &mut host.ctx());
        (host, spi)
    }

    #[test]
    fn exchange_with_a_device_that_answers() {
        let (mut host, mut spi) = spi3();
        host.now = 77;
        spi.write(DR, 0xF5, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted, [(PORT_MOSI, Signal::Byte { value: 0xF5, at: 77 })]);
        spi.input(PORT_MISO, Signal::Byte { value: 0x70, at: 77 }, &mut host.ctx());
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & (TXE | RXNE), TXE | RXNE);
        assert_eq!(spi.read(DR, u32::MAX, &mut host.ctx()), 0x70);
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & RXNE, 0);
        spi.input(PORT_MISO, Signal::Byte { value: 0x12, at: 80 }, &mut host.ctx());
        assert_eq!(spi.read(DR, u32::MAX, &mut host.ctx()), 0x70, "late answers are ignored");
        assert_eq!(spi.frames(), 1);
    }

    #[test]
    fn nobody_answering_reads_all_ones() {
        let (mut host, mut spi) = spi3();
        spi.write(DR, 0xF5, u32::MAX, &mut host.ctx());
        assert_eq!(spi.read(DR, u32::MAX, &mut host.ctx()), 0xFF);
        spi.write(CR1, SPE | MSTR | DFF, u32::MAX, &mut host.ctx());
        spi.write(DR, 0x1234, u32::MAX, &mut host.ctx());
        assert_eq!(spi.read(DR, u32::MAX, &mut host.ctx()), 0xFFFF);
    }

    #[test]
    fn overrun_and_its_clearing_sequence() {
        let (mut host, mut spi) = spi3();
        spi.write(CR2, ERRIE, u32::MAX, &mut host.ctx());
        spi.write(DR, 1, u32::MAX, &mut host.ctx());
        spi.write(DR, 2, u32::MAX, &mut host.ctx());
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & OVR, OVR);
        assert!(host.irq(51));
        spi.read(DR, u32::MAX, &mut host.ctx());
        spi.read(SR, u32::MAX, &mut host.ctx());
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & OVR, 0);
        assert!(!host.irq(51));
    }

    #[test]
    fn writes_while_disabled_wait_for_spe() {
        let mut host = TestHost::new(72_000_000);
        let mut spi = Spi::new(None);
        spi.write(CR1, MSTR, u32::MAX, &mut host.ctx());
        spi.write(DR, 0x75 | 0x80, u32::MAX, &mut host.ctx());
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & TXE, 0);
        assert_eq!(host.blockers[0].0, "spi-disabled");
        spi.write(CR1, MSTR | SPE, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted.len(), 1);
        assert_eq!(spi.read(SR, u32::MAX, &mut host.ctx()) & (TXE | RXNE), TXE | RXNE);
    }

    #[test]
    fn interrupts_and_dma_requests() {
        let (mut host, mut spi) = spi3();
        spi.write(CR2, TXEIE, u32::MAX, &mut host.ctx());
        assert!(host.irq(51), "TXE is set when idle");
        spi.write(CR2, RXNEIE | RXDMAEN, u32::MAX, &mut host.ctx());
        assert!(!host.irq(51));
        spi.write(DR, 0, u32::MAX, &mut host.ctx());
        assert!(host.irq(51));
        assert_eq!(host.dma_requests, [DMA_RX]);
        assert_eq!((spi.reg_name(0x20), spi.port("miso")), (Some("I2SPR"), Some(PORT_MISO)));
    }
}

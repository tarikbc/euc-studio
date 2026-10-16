//! DMA controller (RM0008 §13): 7 channels on DMA1, 5 on DMA2.

use std::collections::BTreeMap;

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind, Size};
use serde::Deserialize;

const ISR: u32 = 0x00;
const IFCR: u32 = 0x04;
/// First channel register block and the size of each.
const CHANNEL_BASE: u32 = 0x08;
const CHANNEL_STRIDE: u32 = 0x14;

const CCR_EN: u32 = 1 << 0;
/// TCIE, HTIE, TEIE: the interrupt enables, aligned with TCIF, HTIF, TEIF in `ISR`.
const CCR_IE: u32 = 0xE;
const CCR_DIR: u32 = 1 << 4;
const CCR_CIRC: u32 = 1 << 5;
const CCR_PINC: u32 = 1 << 6;
const CCR_MINC: u32 = 1 << 7;
const CCR_MEM2MEM: u32 = 1 << 14;

const ISR_GIF: u32 = 1 << 0;
const ISR_TCIF: u32 = 1 << 1;
const ISR_HTIF: u32 = 1 << 2;
const ISR_TEIF: u32 = 1 << 3;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// Channels: 7 on DMA1, 5 on DMA2.
    channels: u8,
}

#[derive(Debug, Clone, Copy, Default)]
struct Channel {
    ccr: u32,
    /// `CNDTR` as written: the reload count.
    ndtr: u32,
    par: u32,
    mar: u32,
    /// Current peripheral and memory addresses and the transfers left.
    cp: u32,
    cm: u32,
    cnt: u32,
}

/// The DMA controller: one transfer per peripheral request, as in the reference harness.
///
/// A request moves one unit between the peripheral (`CPAR`) and memory (`CMAR`) in the
/// configured direction and sizes, steps the addresses, counts `CNDTR` down, raises half- and
/// full-transfer flags and reloads in circular mode. Memory-to-memory channels run to the end
/// as soon as they are enabled. A transfer that hits a bus fault sets `TEIF` and disables the
/// channel.
///
/// Not modeled: channel priorities and arbitration (requests are served as they come), transfer
/// timing (a unit moves at the request instant).
#[derive(Debug, Clone)]
pub struct Dma {
    channels: Vec<Channel>,
    isr: u32,
    /// IRQ line of each channel (DMA2 channels 4 and 5 share one).
    irqs: Vec<Option<u16>>,
    levels: BTreeMap<u16, bool>,
    transfers: Vec<u64>,
}

impl Dma {
    /// A controller with `irqs.len()` channels; `irqs[i]` is channel `i + 1`'s line.
    pub fn new(irqs: Vec<Option<u16>>) -> Self {
        let n = irqs.len();
        Self { channels: vec![Channel::default(); n], isr: 0, irqs, levels: BTreeMap::new(), transfers: vec![0; n] }
    }

    /// Builds the `stm32f1.dma` kind (`channels`, IRQ roles `ch1`..`chN`).
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        if !(1..=7).contains(&config.channels) {
            return Err(build.error(format!("{} channels; a DMA controller has 1 to 7", config.channels)));
        }
        let irqs = (1..=config.channels).map(|ch| build.irq(&format!("ch{ch}")).ok()).collect();
        Ok(Box::new(Self::new(irqs)))
    }

    /// Units moved on channel `channel` (1-based) since power-on.
    pub fn transfers(&self, channel: u8) -> u64 {
        self.transfers.get(usize::from(channel).wrapping_sub(1)).copied().unwrap_or(0)
    }

    /// `(CMAR, CNDTR)` of channel `channel` (1-based) as programmed: the buffer a peripheral
    /// streams from, e.g. the LED strip's duty bytes behind TIM3.
    pub fn memory(&self, channel: u8) -> Option<(u32, u32)> {
        self.channels.get(usize::from(channel).checked_sub(1)?).map(|c| (c.mar, c.ndtr))
    }

    /// Moves one unit on channel index `i`. False if the channel is off or done.
    fn request(&mut self, i: usize, ctx: &mut Ctx<'_>) -> bool {
        let c = self.channels[i];
        if c.ccr & CCR_EN == 0 || c.cnt == 0 {
            return false;
        }
        let psize = size((c.ccr >> 8) & 3);
        let msize = size((c.ccr >> 10) & 3);
        let moved = if c.ccr & CCR_DIR != 0 {
            ctx.bus_read(c.cm, msize).and_then(|data| ctx.bus_write(c.cp, psize, data & psize.mask()))
        } else {
            ctx.bus_read(c.cp, psize).and_then(|data| ctx.bus_write(c.cm, msize, data & msize.mask()))
        };
        let shift = 4 * i as u32;
        if moved.is_err() {
            self.isr |= (ISR_GIF | ISR_TEIF) << shift;
            self.channels[i].ccr &= !CCR_EN;
            return false;
        }
        let c = &mut self.channels[i];
        if c.ccr & CCR_PINC != 0 {
            c.cp = c.cp.wrapping_add(psize.bytes());
        }
        if c.ccr & CCR_MINC != 0 {
            c.cm = c.cm.wrapping_add(msize.bytes());
        }
        c.cnt -= 1;
        self.transfers[i] += 1;
        if c.cnt == c.ndtr / 2 {
            self.isr |= (ISR_GIF | ISR_HTIF) << shift;
        }
        if c.cnt == 0 {
            self.isr |= (ISR_GIF | ISR_TCIF) << shift;
            if c.ccr & CCR_CIRC != 0 {
                (c.cp, c.cm, c.cnt) = (c.par, c.mar, c.ndtr);
            }
        }
        true
    }

    /// Drives each IRQ line from its channels' enabled flags.
    fn update_irqs(&mut self, ctx: &mut Ctx<'_>) {
        let mut wanted: BTreeMap<u16, bool> = BTreeMap::new();
        for (i, (channel, irq)) in self.channels.iter().zip(&self.irqs).enumerate() {
            if let Some(irq) = *irq {
                let high = (self.isr >> (4 * i)) & channel.ccr & CCR_IE != 0;
                *wanted.entry(irq).or_default() |= high;
            }
        }
        for (irq, high) in wanted {
            if self.levels.insert(irq, high) != Some(high) {
                ctx.set_irq(irq, high);
            }
        }
    }

    fn channel_of(off: u32) -> Option<(usize, u32)> {
        let rel = off.checked_sub(CHANNEL_BASE)?;
        Some(((rel / CHANNEL_STRIDE) as usize, rel % CHANNEL_STRIDE))
    }
}

/// Transfer size from a `PSIZE`/`MSIZE` field (the reserved value 3 acts as a word).
const fn size(field: u32) -> Size {
    match field {
        0 => Size::Byte,
        1 => Size::Half,
        _ => Size::Word,
    }
}

impl Peripheral for Dma {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        let transfers = std::mem::take(&mut self.transfers);
        *self = Self::new(self.irqs.clone());
        self.transfers = transfers;
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            ISR => self.isr,
            IFCR => 0,
            _ => match Self::channel_of(off) {
                Some((i, reg)) if i < self.channels.len() => {
                    let c = &self.channels[i];
                    match reg {
                        0x0 => c.ccr,
                        0x4 if c.ccr & CCR_EN != 0 => c.cnt,
                        0x4 => c.ndtr,
                        0x8 => c.par,
                        0xC => c.mar,
                        _ => 0,
                    }
                }
                _ => 0,
            },
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let v = value & mask;
        match off {
            ISR => {}
            IFCR => {
                let mut clear = v;
                for i in 0..self.channels.len() {
                    if v & (1 << (4 * i)) != 0 {
                        clear |= 0xF << (4 * i);
                    }
                }
                self.isr &= !clear;
            }
            _ => {
                let Some((i, reg)) = Self::channel_of(off).filter(|&(i, _)| i < self.channels.len()) else { return };
                let c = &mut self.channels[i];
                match reg {
                    0x0 => {
                        let old = c.ccr;
                        c.ccr = (old & !mask) | v;
                        if c.ccr & CCR_EN != 0 && old & CCR_EN == 0 {
                            (c.cp, c.cm, c.cnt) = (c.par, c.mar, c.ndtr);
                            if c.ccr & CCR_MEM2MEM != 0 {
                                while self.request(i, ctx) {}
                            }
                        }
                    }
                    0x4 => {
                        c.ndtr = v & 0xFFFF;
                        if c.ccr & CCR_EN == 0 {
                            c.cnt = c.ndtr;
                        }
                    }
                    0x8 => c.par = v,
                    0xC => c.mar = v,
                    _ => {}
                }
            }
        }
        self.update_irqs(ctx);
    }

    fn dma_request(&mut self, channel: u8, ctx: &mut Ctx<'_>) -> bool {
        let Some(i) = usize::from(channel).checked_sub(1).filter(|&i| i < self.channels.len()) else { return false };
        let moved = self.request(i, ctx);
        self.update_irqs(ctx);
        moved
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        const NAMES: [[&str; 4]; 7] = [
            ["CCR1", "CNDTR1", "CPAR1", "CMAR1"],
            ["CCR2", "CNDTR2", "CPAR2", "CMAR2"],
            ["CCR3", "CNDTR3", "CPAR3", "CMAR3"],
            ["CCR4", "CNDTR4", "CPAR4", "CMAR4"],
            ["CCR5", "CNDTR5", "CPAR5", "CMAR5"],
            ["CCR6", "CNDTR6", "CPAR6", "CMAR6"],
            ["CCR7", "CNDTR7", "CPAR7", "CMAR7"],
        ];
        match off {
            ISR => Some("ISR"),
            IFCR => Some("IFCR"),
            _ => {
                let (i, reg) = Self::channel_of(off)?;
                NAMES.get(i).filter(|_| i < self.channels.len())?.get((reg / 4) as usize).copied()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const RAM: u32 = 0x2000_079C;
    const DR: u32 = 0x4000_4804;

    fn ch3(ccr: u32, ndtr: u32, host: &mut TestHost) -> Dma {
        let mut dma = Dma::new(vec![Some(11), Some(12), Some(13)]);
        let block = CHANNEL_BASE + 2 * CHANNEL_STRIDE;
        dma.write(block + 0x4, ndtr, u32::MAX, &mut host.ctx());
        dma.write(block + 0x8, DR, u32::MAX, &mut host.ctx());
        dma.write(block + 0xC, RAM, u32::MAX, &mut host.ctx());
        dma.write(block, ccr, u32::MAX, &mut host.ctx());
        dma
    }

    #[test]
    fn peripheral_to_memory_with_flags_and_interrupt() {
        let mut host = TestHost::new(72_000_000);
        host.memory.insert(DR, 0x42);
        let mut dma = ch3(CCR_EN | CCR_MINC | 0x2, 4, &mut host); // TCIE
        for _ in 0..2 {
            assert!(dma.dma_request(3, &mut host.ctx()));
        }
        assert_eq!(dma.read(0x08 + 2 * 0x14 + 4, u32::MAX, &mut host.ctx()), 2, "CNDTR counts down");
        assert_eq!(dma.read(ISR, u32::MAX, &mut host.ctx()) >> 8 & 0xF, ISR_GIF | ISR_HTIF);
        assert!(!host.irq(13), "half transfer interrupt not enabled");
        for _ in 0..2 {
            dma.dma_request(3, &mut host.ctx());
        }
        assert!(!dma.dma_request(3, &mut host.ctx()), "done");
        assert_eq!((host.memory[&(RAM + 3)], dma.transfers(3)), (0x42, 4));
        assert!(host.irq(13));
        dma.write(IFCR, 1 << 8, u32::MAX, &mut host.ctx()); // CGIF3 clears all its flags
        assert_eq!(dma.read(ISR, u32::MAX, &mut host.ctx()), 0);
        assert!(!host.irq(13));
    }

    #[test]
    fn circular_mode_reloads() {
        let mut host = TestHost::new(72_000_000);
        let mut dma = ch3(CCR_EN | CCR_CIRC | CCR_MINC, 2, &mut host);
        for _ in 0..5 {
            assert!(dma.dma_request(3, &mut host.ctx()));
        }
        assert_eq!(dma.read(0x08 + 2 * 0x14 + 4, u32::MAX, &mut host.ctx()), 1);
    }

    #[test]
    fn memory_to_memory_runs_on_enable() {
        let mut host = TestHost::new(72_000_000);
        for i in 0..4 {
            host.memory.insert(0x2000_0000 + i, i as u8 + 1);
        }
        let mut dma = Dma::new(vec![None]);
        dma.write(0x0C, 4, u32::MAX, &mut host.ctx());
        dma.write(0x10, 0x2000_0000, u32::MAX, &mut host.ctx());
        dma.write(0x14, 0x2000_0100, u32::MAX, &mut host.ctx());
        dma.write(0x08, CCR_EN | CCR_MEM2MEM | CCR_PINC | CCR_MINC, u32::MAX, &mut host.ctx());
        assert_eq!((0..4).map(|i| host.memory[&(0x2000_0100 + i)]).collect::<Vec<_>>(), [1, 2, 3, 4]);
        assert_eq!(dma.reg_name(0x08), Some("CCR1"));
        assert_eq!(dma.reg_name(0x08 + 0x14), None, "only one channel");
    }
}

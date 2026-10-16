//! 12-bit ADC with regular and injected groups, dual mode (RM0008 §11).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::rcc::Rcc;
use crate::util::{Rate, clear_w0, merge};

const SR: u32 = 0x00;
const CR1: u32 = 0x04;
const CR2: u32 = 0x08;
const SMPR1: u32 = 0x0C;
const SMPR2: u32 = 0x10;
const JOFR1: u32 = 0x14;
const JOFR4: u32 = 0x20;
const HTR: u32 = 0x24;
const LTR: u32 = 0x28;
const SQR1: u32 = 0x2C;
const SQR3: u32 = 0x34;
const JSQR: u32 = 0x38;
const JDR1: u32 = 0x3C;
const JDR4: u32 = 0x48;
const DR: u32 = 0x4C;

// SR
const AWD: u32 = 1 << 0;
const EOC: u32 = 1 << 1;
const JEOC: u32 = 1 << 2;
const JSTRT: u32 = 1 << 3;
const STRT: u32 = 1 << 4;
// CR1
const EOCIE: u32 = 1 << 5;
const AWDIE: u32 = 1 << 6;
const JEOCIE: u32 = 1 << 7;
const SCAN: u32 = 1 << 8;
const AWDSGL: u32 = 1 << 9;
const JAUTO: u32 = 1 << 10;
const JAWDEN: u32 = 1 << 22;
const AWDEN: u32 = 1 << 23;
// CR2
const ADON: u32 = 1 << 0;
const CONT: u32 = 1 << 1;
const CAL: u32 = 1 << 2;
const RSTCAL: u32 = 1 << 3;
const DMA: u32 = 1 << 8;
const ALIGN: u32 = 1 << 11;
const JEXTTRIG: u32 = 1 << 15;
const EXTTRIG: u32 = 1 << 20;
const JSWSTART: u32 = 1 << 21;
const SWSTART: u32 = 1 << 22;
/// Bits that start something and clear themselves.
const SELF_CLEARING: u32 = CAL | RSTCAL | JSWSTART | SWSTART;

/// Trigger selection value meaning "software start".
const SOFTWARE: u32 = 7;
/// ADC clock cycles a calibration takes.
const CALIBRATION_CYCLES: u64 = 83;

const TAG_INJECTED: u32 = 0;
const TAG_REGULAR: u32 = 1;
const TAG_CALIBRATION: u32 = 2;

/// Ports: dual-mode triggers between master and slave (in and out), timer triggers (in).
const PORT_DUAL_INJECTED: u8 = 0;
const PORT_DUAL_REGULAR: u8 = 1;
const PORT_JEXT: u8 = 2;
const PORT_EXT: u8 = 9;
/// DMA request of regular conversions (`"regular"`).
const DMA_LINE: u8 = 16;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// ADC number (1, 2, 3): which plant inputs it converts.
    index: u8,
    /// The RCC instance that gives ADCCLK.
    clock: String,
    /// When conversions end.
    #[serde(default)]
    conversion: Conversion,
}

/// When an ADC conversion or calibration ends (`conversion` in `board.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Conversion {
    /// At once, as in the Python reference harness: the default, so that a run reproduces the
    /// reference byte for byte. A regular start converts `SQ1` only; in continuous mode the next
    /// conversion happens when `DR` is read.
    #[default]
    Instant,
    /// After the real time: `sample time + 12.5` ADC clock cycles per channel at the ADCCLK the RCC
    /// configures (21 µs for the Begode Extreme's three 71.5-cycle injected channels at 12 MHz),
    /// whole scan sequences, and 83 ADC cycles for a calibration. For studies where the instant of
    /// the end-of-conversion interrupt matters (FOC timing, M2 on).
    Timed,
}

/// One ADC.
///
/// **Conversions**: the plant is sampled when a group starts; the data registers, `EOC`/`JEOC`,
/// the interrupt and the DMA request follow when the conversion ends, at once or after the real
/// conversion time (see [`Conversion`]). `RSTCAL`, `SWSTART` and `JSWSTART` clear at once; `CAL`
/// clears when the calibration ends.
///
/// **Injected group**: `JL`/`JSQR` ranks into `JDR1..JDR4`, `JOFRx` subtracted, right- or
/// left-aligned with sign extension, `JEOC` and its interrupt, `JAUTO` after a regular group.
/// Started by `JSWSTART` (`JEXTSEL` = 111) or a timer pulse on `"jext0"`..`"jext6"`, both only
/// with `JEXTTRIG` set.
///
/// **Regular group**: `L`/`SQRx`, single or scan, continuous, `DR`, `EOC` and its interrupt, DMA
/// request `"regular"` per conversion. Started by `SWSTART`, a timer pulse on `"ext0"`..`"ext6"`, or
/// `ADON` written again while on.
///
/// **Dual mode** (`DUALMOD` of the master, ADC1): in injected or regular simultaneous mode the
/// master's start goes out as a pulse on `"dual_inj"` / `"dual_reg"`, which the board wires to the
/// slave's input of the same name; the slave converts its own sequence at the same instant if its
/// external trigger is enabled (`JEXTTRIG`/`EXTTRIG`; RM0008 asks for that, and for a software
/// trigger selection that keeps the slave from starting on its own). The two share interrupt 18
/// on a wired-OR line.
///
/// Also modeled: the analog watchdog (`AWD` on any or one channel, `HTR`/`LTR`).
///
/// Not modeled: discontinuous modes, the interleaved and alternate-trigger dual modes, ADC2's
/// regular data in ADC1 `DR[31:16]`, power-up stabilisation time, per-rank sampling instants.
#[derive(Debug, Clone)]
pub struct Adc {
    index: u8,
    rcc: DeviceId,
    irq: Option<(u16, bool)>,
    sr: u32,
    cr1: u32,
    cr2: u32,
    smpr: [u32; 2],
    jofr: [u32; 4],
    htr: u32,
    ltr: u32,
    sqr: [u32; 3],
    jsqr: u32,
    jdr: [u32; 4],
    dr: u32,
    /// Injected results sampled at the start, written at the end of the conversion.
    injected: Option<[u32; 4]>,
    /// Regular group in progress: the sampled channels, and how many are done.
    regular: Option<(Vec<u32>, usize)>,
    conversion: Conversion,
    conversions: u64,
}

impl Adc {
    /// ADC number `index` (1-based) after reset, clocked by `rcc`, on interrupt `irq`.
    pub fn new(index: u8, rcc: DeviceId, irq: Option<u16>, conversion: Conversion) -> Self {
        Self {
            index,
            rcc,
            irq: irq.map(|irq| (irq, false)),
            sr: 0,
            cr1: 0,
            cr2: 0,
            smpr: [0; 2],
            jofr: [0; 4],
            htr: 0xFFF,
            ltr: 0,
            sqr: [0; 3],
            jsqr: 0,
            jdr: [0; 4],
            dr: 0,
            injected: None,
            regular: None,
            conversion,
            conversions: 0,
        }
    }

    /// Builds the `stm32f1.adc` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        if !(1..=3).contains(&config.index) {
            return Err(build.error(format!("index {} is not an ADC number 1..=3", config.index)));
        }
        let rcc = build.id_of(&config.clock)?;
        Ok(Box::new(Self::new(config.index, rcc, build.irq("irq").ok(), config.conversion)))
    }

    /// Channel conversions completed since reset.
    pub const fn conversions(&self) -> u64 {
        self.conversions
    }

    /// `JDRx` as the firmware reads it (`x` = 1..=4).
    pub const fn jdr(&self, x: usize) -> u32 {
        self.jdr[x - 1]
    }

    /// The injected sequence: channels in conversion order (`JL + 1` of them).
    fn injected_channels(&self) -> Vec<u32> {
        let len = (self.jsqr >> 20 & 3) + 1;
        (4 - len..4).map(|slot| self.jsqr >> (5 * slot) & 0x1F).collect()
    }

    /// The regular sequence: `SQ1..` (`L + 1` channels in scan mode, `SQ1` alone otherwise).
    fn regular_channels(&self) -> Vec<u32> {
        let len = if self.cr1 & SCAN != 0 { (self.sqr[0] >> 20 & 0xF) + 1 } else { 1 };
        (0..len).map(|rank| self.sqr[2 - (rank / 6) as usize] >> (5 * (rank % 6)) & 0x1F).collect()
    }

    /// Sample time of `channel` in half ADC cycles, plus the 12.5-cycle conversion.
    fn half_cycles(&self, channel: u32) -> u64 {
        const SAMPLE: [u64; 8] = [3, 15, 27, 57, 83, 111, 143, 479];
        let (reg, field) = if channel >= 10 { (0, channel - 10) } else { (1, channel) };
        SAMPLE[(self.smpr[reg] >> (3 * field) & 7) as usize] + 25
    }

    /// ADCCLK from the RCC (HCLK / 6 if there is no RCC peer).
    fn adcclk(&self, ctx: &mut Ctx<'_>) -> u64 {
        let fallback = ctx.hclk() / 6;
        ctx.peer::<Rcc>(self.rcc).map_or(fallback, |rcc| u64::from(rcc.clocks().adcclk))
    }

    /// Engine cycles that `channels` take to convert.
    fn duration(&self, channels: &[u32], ctx: &mut Ctx<'_>) -> u64 {
        let half_cycles: u64 = channels.iter().map(|&ch| self.half_cycles(ch)).sum();
        Rate::new(ctx.hclk(), 2 * self.adcclk(ctx)).cycles(half_cycles).max(1)
    }

    fn sample(&self, channel: u32, ctx: &Ctx<'_>) -> u32 {
        u32::from(ctx.plant().adc_raw(self.index, channel as u8) & 0xFFF)
    }

    fn start_injected(&mut self, ctx: &mut Ctx<'_>) {
        if self.cr2 & ADON == 0 || self.injected.is_some() {
            return;
        }
        let channels = self.injected_channels();
        let mut raw = [0; 4];
        for (slot, &ch) in raw.iter_mut().zip(&channels) {
            *slot = self.sample(ch, ctx);
        }
        self.injected = Some(raw);
        self.sr |= JSTRT;
        match self.conversion {
            Conversion::Instant => self.finish_injected(),
            Conversion::Timed => {
                let at = ctx.now() + self.duration(&channels, ctx);
                ctx.schedule(at, TAG_INJECTED);
            }
        }
        if matches!(self.cr1 >> 16 & 0xF, 0b0001 | 0b0101) {
            ctx.emit(PORT_DUAL_INJECTED, Signal::Pulse);
        }
    }

    fn start_regular(&mut self, ctx: &mut Ctx<'_>) {
        if self.cr2 & ADON == 0 || self.regular.is_some() {
            return;
        }
        let channels = self.regular_channels();
        self.sr |= STRT;
        match self.conversion {
            Conversion::Instant => self.convert_regular_now(channels[0], ctx),
            Conversion::Timed => {
                let samples: Vec<u32> = channels.iter().map(|&ch| self.sample(ch, ctx)).collect();
                let at = ctx.now() + self.duration(&channels[..1], ctx);
                self.regular = Some((samples, 0));
                ctx.schedule(at, TAG_REGULAR);
            }
        }
        if matches!(self.cr1 >> 16 & 0xF, 0b0001 | 0b0010 | 0b0110) {
            ctx.emit(PORT_DUAL_REGULAR, Signal::Pulse);
        }
    }

    fn finish_injected(&mut self) {
        let Some(raw) = self.injected.take() else { return };
        let channels = self.injected_channels();
        for (k, &ch) in channels.iter().enumerate() {
            let value = raw[k] as i32 - (self.jofr[k] & 0xFFF) as i32;
            self.watchdog(ch, raw[k], JAWDEN);
            let aligned = if self.cr2 & ALIGN != 0 { value << 3 } else { value };
            self.jdr[k] = aligned as u32 & 0xFFFF;
        }
        self.conversions += channels.len() as u64;
        self.sr |= JEOC | EOC;
    }

    /// An instant regular conversion of `channel`, ending the group.
    fn convert_regular_now(&mut self, channel: u32, ctx: &mut Ctx<'_>) {
        let raw = self.sample(channel, ctx);
        self.store_regular(channel, raw, ctx);
        self.sr |= EOC;
        if self.cr1 & JAUTO != 0 {
            self.start_injected(ctx);
        }
    }

    /// One regular result into `DR`, with its watchdog check and DMA request.
    fn store_regular(&mut self, channel: u32, raw: u32, ctx: &mut Ctx<'_>) {
        self.watchdog(channel, raw, AWDEN);
        self.dr = if self.cr2 & ALIGN != 0 { raw << 4 } else { raw };
        self.conversions += 1;
        if self.cr2 & DMA != 0 {
            ctx.dma_request(DMA_LINE);
        }
    }

    fn finish_regular_conversion(&mut self, ctx: &mut Ctx<'_>) {
        let Some((samples, done)) = self.regular.take() else { return };
        let channels = self.regular_channels();
        self.store_regular(channels.get(done).copied().unwrap_or(0), samples[done], ctx);
        let done = done + 1;
        if done < samples.len() {
            let at = ctx.now() + self.duration(&channels[done..=done], ctx);
            ctx.schedule(at, TAG_REGULAR);
            self.regular = Some((samples, done));
            return;
        }
        // End of the group.
        self.sr |= EOC;
        if self.cr1 & JAUTO != 0 {
            self.start_injected(ctx);
        }
        if self.cr2 & CONT != 0 {
            self.start_regular(ctx);
        }
    }

    /// Analog watchdog: flags `raw` outside `[LTR, HTR]` when `enable` (AWDEN or JAWDEN) is set and
    /// the channel is watched.
    fn watchdog(&mut self, channel: u32, raw: u32, enable: u32) {
        let watched = self.cr1 & AWDSGL == 0 || self.cr1 & 0x1F == channel;
        if self.cr1 & enable != 0 && watched && !(self.ltr..=self.htr).contains(&raw) {
            self.sr |= AWD;
        }
    }

    fn update_irq(&mut self, ctx: &mut Ctx<'_>) {
        let Some((irq, level)) = &mut self.irq else { return };
        let high = self.sr & EOC != 0 && self.cr1 & EOCIE != 0
            || self.sr & JEOC != 0 && self.cr1 & JEOCIE != 0
            || self.sr & AWD != 0 && self.cr1 & AWDIE != 0;
        if high != *level {
            *level = high;
            ctx.set_irq(*irq, high);
        }
    }

    fn write_cr2(&mut self, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let old = self.cr2;
        let new = merge(old, value, mask) & 0x00FE_F90F;
        self.cr2 = new & !SELF_CLEARING;
        if new & CAL != 0 && new & ADON != 0 && self.conversion == Conversion::Timed {
            self.cr2 |= CAL;
            let at = ctx.now() + Rate::new(ctx.hclk(), self.adcclk(ctx)).cycles(CALIBRATION_CYCLES);
            ctx.schedule(at, TAG_CALIBRATION);
        }
        if new & JSWSTART != 0 && new & JEXTTRIG != 0 && new >> 12 & 7 == SOFTWARE {
            self.start_injected(ctx);
        }
        let swstart = new & SWSTART != 0 && new & EXTTRIG != 0 && new >> 17 & 7 == SOFTWARE;
        // Writing ADON = 1 again starts a regular conversion, unless another bit changes with it.
        let adon_again = old & ADON != 0 && mask & ADON != 0 && value & ADON != 0 && (old ^ new) & !SELF_CLEARING == 0;
        if swstart || adon_again && new & SELF_CLEARING == 0 {
            self.start_regular(ctx);
        }
    }
}

impl Peripheral for Adc {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new(self.index, self.rcc, self.irq.map(|(irq, _)| irq), self.conversion);
    }

    fn read(&mut self, off: u32, _mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        match off {
            SR => self.sr,
            CR1 => self.cr1,
            CR2 => self.cr2,
            SMPR1 | SMPR2 => self.smpr[((off - SMPR1) / 4) as usize],
            JOFR1..=JOFR4 => self.jofr[((off - JOFR1) / 4) as usize],
            HTR => self.htr,
            LTR => self.ltr,
            SQR1..=SQR3 => self.sqr[((off - SQR1) / 4) as usize],
            JSQR => self.jsqr,
            JDR1..=JDR4 => self.jdr[((off - JDR1) / 4) as usize],
            DR => {
                // Reading the regular data clears EOC; with instant conversions in continuous mode
                // the next one is ready at once.
                let value = self.dr;
                self.sr &= !EOC;
                if self.conversion == Conversion::Instant && self.cr2 & (ADON | CONT) == ADON | CONT {
                    let first = self.regular_channels()[0];
                    self.convert_regular_now(first, ctx);
                }
                self.update_irq(ctx);
                value
            }
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let store = |old: u32, bits: u32| merge(old, value, mask) & bits;
        match off {
            SR => self.sr = clear_w0(self.sr, value, mask, AWD | EOC | JEOC | JSTRT | STRT),
            CR1 => self.cr1 = store(self.cr1, 0x00CF_FFFF),
            CR2 => self.write_cr2(value, mask, ctx),
            SMPR1 => self.smpr[0] = store(self.smpr[0], 0x00FF_FFFF),
            SMPR2 => self.smpr[1] = store(self.smpr[1], 0x3FFF_FFFF),
            JOFR1..=JOFR4 => {
                let k = ((off - JOFR1) / 4) as usize;
                self.jofr[k] = store(self.jofr[k], 0xFFF);
            }
            HTR => self.htr = store(self.htr, 0xFFF),
            LTR => self.ltr = store(self.ltr, 0xFFF),
            SQR1..=SQR3 => {
                let k = ((off - SQR1) / 4) as usize;
                self.sqr[k] = store(self.sqr[k], if k == 0 { 0x00FF_FFFF } else { 0x3FFF_FFFF });
            }
            JSQR => self.jsqr = store(self.jsqr, 0x003F_FFFF),
            _ => return,
        }
        self.update_irq(ctx);
    }

    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        match tag {
            TAG_INJECTED => self.finish_injected(),
            TAG_REGULAR => self.finish_regular_conversion(ctx),
            TAG_CALIBRATION => self.cr2 &= !CAL,
            _ => {}
        }
        self.update_irq(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        if signal != Signal::Pulse {
            return;
        }
        let jext = self.cr2 & JEXTTRIG != 0;
        let ext = self.cr2 & EXTTRIG != 0;
        let jextsel = self.cr2 >> 12 & 7;
        let extsel = self.cr2 >> 17 & 7;
        match port {
            // The master's trigger starts the slave whatever the slave's own trigger selection;
            // its external trigger only has to be enabled.
            PORT_DUAL_INJECTED if jext => self.start_injected(ctx),
            PORT_DUAL_REGULAR if ext => self.start_regular(ctx),
            p if (PORT_JEXT..PORT_EXT).contains(&p) && jext && u32::from(p - PORT_JEXT) == jextsel => {
                self.start_injected(ctx);
            }
            p if (PORT_EXT..PORT_EXT + 7).contains(&p) && ext && u32::from(p - PORT_EXT) == extsel => {
                self.start_regular(ctx);
            }
            _ => {}
        }
        self.update_irq(ctx);
    }

    fn port(&self, name: &str) -> Option<u8> {
        let numbered = |prefix: &str, first: u8| -> Option<u8> {
            let n: u8 = name.strip_prefix(prefix)?.parse().ok().filter(|&n| n < 7)?;
            Some(first + n)
        };
        match name {
            "dual_inj" => Some(PORT_DUAL_INJECTED),
            "dual_reg" => Some(PORT_DUAL_REGULAR),
            "regular" => Some(DMA_LINE),
            _ => numbered("jext", PORT_JEXT).or_else(|| numbered("ext", PORT_EXT)),
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            SR => "SR",
            CR1 => "CR1",
            CR2 => "CR2",
            SMPR1 => "SMPR1",
            SMPR2 => "SMPR2",
            0x14 => "JOFR1",
            0x18 => "JOFR2",
            0x1C => "JOFR3",
            JOFR4 => "JOFR4",
            HTR => "HTR",
            LTR => "LTR",
            SQR1 => "SQR1",
            0x30 => "SQR2",
            SQR3 => "SQR3",
            JSQR => "JSQR",
            0x3C => "JDR1",
            0x40 => "JDR2",
            0x44 => "JDR3",
            JDR4 => "JDR4",
            DR => "DR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const RCC: DeviceId = 1;

    /// A host whose RCC runs the Begode Extreme's clocks: ADCCLK = 72 MHz / 6 = 12 MHz.
    fn host() -> TestHost {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        for (off, value) in [(0x00, 0x0101_0083), (0x04, 0x001D_8402)] {
            rcc.write(off, value, u32::MAX, &mut host.ctx());
        }
        host.peers.insert(RCC, Box::new(rcc));
        host
    }

    fn write(adc: &mut Adc, host: &mut TestHost, off: u32, value: u32) {
        adc.write(off, value, u32::MAX, &mut host.ctx());
    }

    fn read(adc: &mut Adc, host: &mut TestHost, off: u32) -> u32 {
        adc.read(off, u32::MAX, &mut host.ctx())
    }

    /// Fires the pending event `tag` at its time.
    fn fire(adc: &mut Adc, host: &mut TestHost, tag: u32) -> u64 {
        let at = host.scheduled.remove(&tag).expect("event scheduled");
        host.now = at;
        adc.on_event(tag, &mut host.ctx());
        at
    }

    /// ADC1 as the Begode Extreme sets it up: injected simultaneous dual mode, left-aligned,
    /// JL = 3 conversions (ch4, ch0, ch11) at 71.5 cycles, software trigger, JEOC interrupt.
    fn extreme_adc1(host: &mut TestHost) -> Adc {
        let mut adc = Adc::new(1, RCC, Some(18), Conversion::Timed);
        write(&mut adc, host, CR1, 0b0101 << 16 | SCAN | JEOCIE);
        write(&mut adc, host, SMPR1, 0b110 << 3);
        write(&mut adc, host, SMPR2, 0b110 << 12 | 0b110);
        write(&mut adc, host, JSQR, 2 << 20 | 11 << 15 | 4 << 5);
        write(&mut adc, host, CR2, ADON | ALIGN | SOFTWARE << 12 | JEXTTRIG | SOFTWARE << 17);
        adc
    }

    #[test]
    fn calibration_takes_83_adc_cycles() {
        let mut host = host();
        let mut adc = Adc::new(1, RCC, None, Conversion::Timed);
        write(&mut adc, &mut host, CR2, ADON);
        write(&mut adc, &mut host, CR2, ADON | RSTCAL);
        assert_eq!(read(&mut adc, &mut host, CR2), ADON, "RSTCAL clears at once");
        write(&mut adc, &mut host, CR2, ADON | CAL);
        assert_eq!(read(&mut adc, &mut host, CR2), ADON | CAL);
        assert_eq!(fire(&mut adc, &mut host, TAG_CALIBRATION), 83 * 6);
        assert_eq!(read(&mut adc, &mut host, CR2), ADON);
    }

    #[test]
    fn injected_group_converts_in_21_us_and_starts_the_slave() {
        let mut host = host();
        host.plant.adc.extend([((1, 4), 2050), ((1, 0), 3087), ((1, 11), 1234)]);
        let mut adc = extreme_adc1(&mut host);
        host.now = 1000;
        let cr2 = read(&mut adc, &mut host, CR2);
        write(&mut adc, &mut host, CR2, cr2 | JSWSTART);
        assert_eq!(host.emitted, [(PORT_DUAL_INJECTED, Signal::Pulse)]);
        assert_eq!(read(&mut adc, &mut host, CR2) & JSWSTART, 0, "JSWSTART clears when the conversion starts");
        assert_eq!(read(&mut adc, &mut host, SR), JSTRT);
        // Three channels of 71.5 + 12.5 ADC cycles at 12 MHz = 21 µs = 1512 HCLK cycles.
        assert_eq!(fire(&mut adc, &mut host, TAG_INJECTED), 1000 + 1512);
        assert_eq!([adc.jdr(1), adc.jdr(2), adc.jdr(3)], [2050 << 3, 3087 << 3, 1234 << 3]);
        assert_eq!(read(&mut adc, &mut host, SR), JSTRT | JEOC | EOC);
        assert!(host.irq(18));
        write(&mut adc, &mut host, SR, !JEOC);
        assert!(!host.irq(18), "the ISR clears JEOC");
        assert_eq!(adc.conversions(), 3);
    }

    #[test]
    fn slave_converts_its_own_sequence_on_the_dual_pulse() {
        let mut host = host();
        host.plant.adc.extend([((2, 5), 2040), ((2, 10), 300)]);
        let mut slave = Adc::new(2, RCC, Some(18), Conversion::Timed);
        write(&mut slave, &mut host, SMPR1, 0b110);
        write(&mut slave, &mut host, SMPR2, 0b110 << 15);
        write(&mut slave, &mut host, JSQR, 1 << 20 | 10 << 15 | 5 << 10);
        write(&mut slave, &mut host, CR2, ADON | ALIGN | SOFTWARE << 12);
        slave.input(PORT_DUAL_INJECTED, Signal::Pulse, &mut host.ctx());
        assert!(host.scheduled.is_empty(), "the slave needs JEXTTRIG too");
        // JEXTSEL = 000 (not software): the dual start ignores the slave's own selection.
        write(&mut slave, &mut host, CR2, ADON | ALIGN | JEXTTRIG);
        slave.input(PORT_DUAL_INJECTED, Signal::Pulse, &mut host.ctx());
        assert_eq!(fire(&mut slave, &mut host, TAG_INJECTED), 1008, "two channels");
        assert_eq!([slave.jdr(1), slave.jdr(2)], [2040 << 3, 300 << 3]);
        assert!(!host.irq(18), "JEOCIE is off on the slave");
        assert_eq!(host.emitted, [], "a slave does not trigger anyone");
    }

    #[test]
    fn offsets_make_signed_right_aligned_data() {
        let mut host = host();
        host.plant.adc.insert((1, 3), 50);
        let mut adc = Adc::new(1, RCC, None, Conversion::Timed);
        write(&mut adc, &mut host, JSQR, 3 << 15);
        write(&mut adc, &mut host, JOFR1, 100);
        write(&mut adc, &mut host, CR2, ADON | SOFTWARE << 12 | JEXTTRIG | JSWSTART);
        fire(&mut adc, &mut host, TAG_INJECTED);
        assert_eq!(adc.jdr(1), 0xFFCE, "50 - 100 = -50, sign-extended");
        write(&mut adc, &mut host, CR2, ADON | ALIGN | SOFTWARE << 12 | JEXTTRIG | JSWSTART);
        fire(&mut adc, &mut host, TAG_INJECTED);
        assert_eq!(adc.jdr(1), (-50_i32 << 3) as u32 & 0xFFFF);
    }

    #[test]
    fn regular_scan_with_dma_and_end_of_group() {
        let mut host = host();
        host.plant.adc.extend([((1, 3), 111), ((1, 7), 222)]);
        let mut adc = Adc::new(1, RCC, Some(18), Conversion::Timed);
        write(&mut adc, &mut host, CR1, SCAN | EOCIE);
        write(&mut adc, &mut host, SQR1, 1 << 20);
        write(&mut adc, &mut host, SQR3, 7 << 5 | 3);
        write(&mut adc, &mut host, CR2, ADON | DMA | SOFTWARE << 17 | EXTTRIG | SWSTART);
        assert_eq!(fire(&mut adc, &mut host, TAG_REGULAR), 84, "1.5 + 12.5 ADC cycles");
        assert_eq!((read(&mut adc, &mut host, DR), host.dma_requests.clone()), (111, vec![DMA_LINE]));
        assert!(!host.irq(18), "EOC waits for the end of the group");
        fire(&mut adc, &mut host, TAG_REGULAR);
        assert!(host.irq(18));
        assert_eq!(read(&mut adc, &mut host, DR), 222);
        assert_eq!(read(&mut adc, &mut host, SR) & EOC, 0, "reading DR clears EOC");
        assert!(!host.scheduled.contains_key(&TAG_REGULAR), "single (not continuous) group");
    }

    #[test]
    fn adon_written_again_starts_a_regular_conversion() {
        let mut host = host();
        let mut adc = Adc::new(1, RCC, None, Conversion::Timed);
        write(&mut adc, &mut host, CR2, ADON);
        assert!(host.scheduled.is_empty());
        write(&mut adc, &mut host, CR2, ADON);
        assert!(host.scheduled.contains_key(&TAG_REGULAR));
        fire(&mut adc, &mut host, TAG_REGULAR);
        assert_eq!(read(&mut adc, &mut host, DR), 2048);
        write(&mut adc, &mut host, CR2, ADON | CONT);
        assert!(!host.scheduled.contains_key(&TAG_REGULAR), "another bit changed with ADON");
    }

    #[test]
    fn continuous_mode_restarts_and_the_watchdog_watches() {
        let mut host = host();
        let mut adc = Adc::new(1, RCC, Some(18), Conversion::Timed);
        write(&mut adc, &mut host, HTR, 1000);
        write(&mut adc, &mut host, CR1, AWDEN | AWDSGL | AWDIE);
        write(&mut adc, &mut host, CR2, ADON | CONT | SOFTWARE << 17 | EXTTRIG | SWSTART);
        fire(&mut adc, &mut host, TAG_REGULAR);
        assert!(host.scheduled.contains_key(&TAG_REGULAR), "continuous");
        assert_eq!(read(&mut adc, &mut host, SR) & AWD, AWD, "2048 is above HTR on channel 0");
        assert!(host.irq(18));
    }

    #[test]
    fn timer_triggers_select_by_jextsel() {
        let mut host = host();
        let mut adc = Adc::new(1, RCC, None, Conversion::Timed);
        write(&mut adc, &mut host, CR2, ADON | JEXTTRIG);
        adc.input(adc.port("jext1").unwrap(), Signal::Pulse, &mut host.ctx());
        assert!(host.scheduled.is_empty(), "JEXTSEL = 000 listens to jext0 only");
        adc.input(adc.port("jext0").unwrap(), Signal::Pulse, &mut host.ctx());
        assert!(host.scheduled.contains_key(&TAG_INJECTED));
        assert_eq!((adc.port("ext6"), adc.port("ext7"), adc.port("dual_reg")), (Some(PORT_EXT + 6), None, Some(1)));
    }

    #[test]
    fn instant_conversions_end_at_once_like_the_reference() {
        let mut host = host();
        host.plant.adc.extend([((1, 4), 2050), ((1, 0), 3087), ((1, 11), 1234)]);
        let mut adc = Adc::new(1, RCC, Some(18), Conversion::Instant);
        write(&mut adc, &mut host, CR1, 0b0101 << 16 | SCAN | JEOCIE);
        write(&mut adc, &mut host, JSQR, 2 << 20 | 11 << 15 | 4 << 5);
        write(&mut adc, &mut host, CR2, ADON | ALIGN | SOFTWARE << 12 | JEXTTRIG);
        write(&mut adc, &mut host, CR2, ADON | ALIGN | SOFTWARE << 12 | JEXTTRIG | CAL);
        assert_eq!(read(&mut adc, &mut host, CR2) & CAL, 0, "calibration ends at once");
        write(&mut adc, &mut host, CR2, ADON | ALIGN | SOFTWARE << 12 | JEXTTRIG | JSWSTART);
        assert_eq!(read(&mut adc, &mut host, SR), JSTRT | JEOC | EOC);
        assert_eq!([adc.jdr(1), adc.jdr(2), adc.jdr(3)], [2050 << 3, 3087 << 3, 1234 << 3]);
        assert!(host.irq(18));
        assert!(host.scheduled.is_empty(), "nothing waits");
        assert_eq!(host.emitted, [(PORT_DUAL_INJECTED, Signal::Pulse)]);
    }

    #[test]
    fn instant_continuous_mode_converts_again_on_each_read() {
        let mut host = host();
        host.plant.adc.insert((1, 3), 777);
        let mut adc = Adc::new(1, RCC, None, Conversion::Instant);
        write(&mut adc, &mut host, SQR3, 3);
        write(&mut adc, &mut host, CR2, ADON | CONT | SOFTWARE << 17 | EXTTRIG | SWSTART);
        assert_eq!(read(&mut adc, &mut host, SR) & (EOC | STRT), EOC | STRT);
        assert_eq!(read(&mut adc, &mut host, DR), 777);
        assert_eq!(read(&mut adc, &mut host, SR) & EOC, EOC, "the next result is ready");
        assert_eq!(adc.conversions(), 2);
    }

    #[test]
    fn conversion_mode_comes_from_board_toml() {
        let config: Config = toml::from_str("index = 1\nclock = \"RCC\"").unwrap();
        assert_eq!(config.conversion, Conversion::Instant);
        let config: Config = toml::from_str("index = 2\nclock = \"RCC\"\nconversion = \"timed\"").unwrap();
        assert_eq!(config.conversion, Conversion::Timed);
    }
}

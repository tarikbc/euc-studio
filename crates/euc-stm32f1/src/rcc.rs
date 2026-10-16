//! Reset and clock control (RM0008 §7.3, low-, medium-, high- and XL-density register map).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind};
use serde::Deserialize;

use crate::pwr::Pwr;
use crate::util::merge;

/// Internal RC oscillator.
pub const HSI_HZ: u32 = 8_000_000;
/// Internal low-speed RC oscillator (typical).
pub const LSI_HZ: u32 = 40_000;
/// Low-speed crystal.
pub const LSE_HZ: u32 = 32_768;

const CR: u32 = 0x00;
const CFGR: u32 = 0x04;
const CIR: u32 = 0x08;
const APB2RSTR: u32 = 0x0C;
const APB1RSTR: u32 = 0x10;
const AHBENR: u32 = 0x14;
const APB2ENR: u32 = 0x18;
const APB1ENR: u32 = 0x1C;
const BDCR: u32 = 0x20;
const CSR: u32 = 0x24;

// CR
const HSION: u32 = 1 << 0;
const HSIRDY: u32 = 1 << 1;
const HSEON: u32 = 1 << 16;
const HSERDY: u32 = 1 << 17;
const PLLON: u32 = 1 << 24;
const PLLRDY: u32 = 1 << 25;
// BDCR
const LSEON: u32 = 1 << 0;
const LSERDY: u32 = 1 << 1;
const RTCSEL: u32 = 3 << 8;
const RTCEN: u32 = 1 << 15;
const BDRST: u32 = 1 << 16;
// CSR
const LSION: u32 = 1 << 0;
const LSIRDY: u32 = 1 << 1;
const RMVF: u32 = 1 << 24;
const PINRSTF: u32 = 1 << 26;
const PORRSTF: u32 = 1 << 27;
const RESET_FLAGS: u32 = 0xFC00_0000;
// CIR: ready flags in bits 0-4 (LSI, LSE, HSI, HSE, PLL), enables in 8-12, clears in 16-20, CSS 7/23.
const CIR_FLAGS: u32 = 0x9F;
const CIR_ENABLES: u32 = 0x1F00;

/// One of the two peripheral buses; it sets a peripheral's clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Apb {
    /// APB1 (36 MHz maximum): TIM2-TIM7, USART2-UART5, SPI2/SPI3, I2C, CAN, PWR, BKP.
    Apb1,
    /// APB2: TIM1, TIM8, USART1, SPI1, ADC, GPIO, AFIO.
    Apb2,
}

/// The clock frequencies an RCC configuration produces, in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clocks {
    /// System clock (`SW`: HSI, HSE or PLL).
    pub sysclk: u32,
    /// AHB clock: CPU, memory, DMA.
    pub hclk: u32,
    /// APB1 clock (36 MHz maximum).
    pub pclk1: u32,
    /// APB2 clock.
    pub pclk2: u32,
    /// Clock of the timers on APB1 (TIM2-7): `pclk1`, doubled when the APB1 prescaler is not 1.
    pub tim_apb1: u32,
    /// Clock of the timers on APB2 (TIM1, TIM8): `pclk2`, doubled when the APB2 prescaler is not 1.
    pub tim_apb2: u32,
    /// ADC clock: `pclk2` divided by `ADCPRE`.
    pub adcclk: u32,
    /// RTC clock when `RTCEN` is set and the selected source runs.
    pub rtcclk: Option<u32>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// Crystal frequency on `OSC_IN`/`OSC_OUT`.
    #[serde(default = "default_hse")]
    hse_hz: u32,
    /// The PWR instance whose `DBP` bit guards `BDCR` writes.
    pwr: Option<String>,
}

const fn default_hse() -> u32 {
    8_000_000
}

/// Reset and clock control.
///
/// Oscillators and the PLL are ready the moment they are switched on (`HSERDY`, `PLLRDY`,
/// `LSERDY`, `LSIRDY` follow their enable bits, `SWS` follows `SW`), so firmware waiting for
/// them proceeds at once. [`clocks`](Self::clocks) gives the resulting frequencies to the
/// timers, UARTs, ADCs and the RTC, which read it as a peer.
///
/// `BDCR` is in the backup domain: it survives a system reset, and while a PWR instance is
/// configured, writes need `PWR_CR.DBP`. Once `RTCSEL` is set it stays until `BDRST`.
///
/// Not modeled: peripheral resets through `APBxRSTR` (the bits read back), the clock security
/// system, MCO, start-up times.
#[derive(Debug, Clone)]
pub struct Rcc {
    hse_hz: u32,
    pwr: Option<DeviceId>,
    regs: [u32; 10],
}

impl Clocks {
    /// The peripheral clock of `bus`.
    pub const fn pclk(&self, bus: Apb) -> u32 {
        match bus {
            Apb::Apb1 => self.pclk1,
            Apb::Apb2 => self.pclk2,
        }
    }

    /// The timer clock of `bus`.
    pub const fn timer(&self, bus: Apb) -> u32 {
        match bus {
            Apb::Apb1 => self.tim_apb1,
            Apb::Apb2 => self.tim_apb2,
        }
    }
}

impl Rcc {
    /// An RCC fed by an `hse_hz` crystal, in its power-on state.
    pub fn new(hse_hz: u32, pwr: Option<DeviceId>) -> Self {
        let mut rcc = Self { hse_hz, pwr, regs: [0; 10] };
        rcc.reset_registers(ResetKind::Power);
        rcc
    }

    /// Builds the `stm32f1.rcc` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let pwr = config.pwr.as_deref().map(|name| build.id_of(name)).transpose()?;
        Ok(Box::new(Self::new(config.hse_hz, pwr)))
    }

    /// The frequencies of the current configuration.
    pub fn clocks(&self) -> Clocks {
        let cfgr = self.reg(CFGR);
        let sysclk = match cfgr & 3 {
            1 => self.hse_hz,
            2 => self.pll_hz(cfgr),
            _ => HSI_HZ,
        };
        let hclk = sysclk >> ahb_shift(cfgr >> 4 & 0xF);
        let ppre1 = cfgr >> 8 & 7;
        let ppre2 = cfgr >> 11 & 7;
        let pclk1 = hclk >> apb_shift(ppre1);
        let pclk2 = hclk >> apb_shift(ppre2);
        let timer = |pclk: u32, ppre: u32| if ppre < 4 { pclk } else { pclk * 2 };
        let adcclk = pclk2 / (2 * ((cfgr >> 14 & 3) + 1));
        Clocks {
            sysclk,
            hclk,
            pclk1,
            pclk2,
            tim_apb1: timer(pclk1, ppre1),
            tim_apb2: timer(pclk2, ppre2),
            adcclk,
            rtcclk: self.rtc_hz(),
        }
    }

    fn pll_hz(&self, cfgr: u32) -> u32 {
        let input = if cfgr & 1 << 16 == 0 { HSI_HZ / 2 } else { self.hse_hz >> (cfgr >> 17 & 1) };
        input * ((cfgr >> 18 & 0xF) + 2).min(16)
    }

    fn rtc_hz(&self) -> Option<u32> {
        let bdcr = self.reg(BDCR);
        if bdcr & RTCEN == 0 {
            return None;
        }
        match bdcr >> 8 & 3 {
            1 if bdcr & LSEON != 0 => Some(LSE_HZ),
            2 if self.reg(CSR) & LSION != 0 => Some(LSI_HZ),
            3 if self.reg(CR) & HSEON != 0 => Some(self.hse_hz / 128),
            _ => None,
        }
    }

    fn reg(&self, off: u32) -> u32 {
        self.regs[(off / 4) as usize]
    }

    fn set(&mut self, off: u32, value: u32) {
        self.regs[(off / 4) as usize] = value;
    }

    fn reset_registers(&mut self, kind: ResetKind) {
        let bdcr = self.reg(BDCR);
        let csr = self.reg(CSR);
        self.regs = [0; 10];
        self.set(CR, HSION | HSIRDY | 0x80);
        self.set(AHBENR, 0x14);
        match kind {
            ResetKind::Power => self.set(CSR, PINRSTF | PORRSTF),
            ResetKind::System => {
                self.set(BDCR, bdcr);
                self.set(CSR, csr & RESET_FLAGS | PINRSTF);
            }
        }
    }

    /// Ready flags for CIR: LSI, LSE, HSI, HSE, PLL in bits 0-4.
    fn ready_bits(&self) -> u32 {
        let cr = self.reg(CR);
        u32::from(self.reg(CSR) & LSIRDY != 0)
            | u32::from(self.reg(BDCR) & LSERDY != 0) << 1
            | u32::from(cr & HSIRDY != 0) << 2
            | u32::from(cr & HSERDY != 0) << 3
            | u32::from(cr & PLLRDY != 0) << 4
    }

    fn backup_writable(&self, ctx: &mut Ctx<'_>) -> bool {
        self.pwr.is_none_or(|id| ctx.peer::<Pwr>(id).is_some_and(|pwr| pwr.backup_access()))
    }
}

/// `HPRE` → right shift of SYSCLK.
const fn ahb_shift(hpre: u32) -> u32 {
    match hpre {
        0..=7 => 0,
        8..=11 => hpre - 7,
        _ => hpre - 6,
    }
}

/// `PPREx` → right shift of HCLK.
const fn apb_shift(ppre: u32) -> u32 {
    if ppre < 4 { 0 } else { ppre - 3 }
}

impl Peripheral for Rcc {
    fn reset(&mut self, kind: ResetKind, _ctx: &mut Ctx<'_>) {
        self.reset_registers(kind);
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            CR..=CSR => self.reg(off),
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let old = self.read(off, mask, ctx);
        let new = merge(old, value, mask);
        match off {
            CR => {
                // Each ready flag follows its enable at once; HSITRIM is writable, HSICAL is not.
                let on = new & (HSION | HSEON | PLLON | 1 << 18 | 1 << 19 | 0xF8);
                let ready = (on & HSION) << 1 | (on & HSEON) << 1 | (on & PLLON) << 1;
                let before = self.ready_bits();
                self.set(CR, on | ready | old & 0xFF00);
                self.latch_ready_flags(before);
            }
            CFGR => {
                // SWS mirrors SW; SW = 3 is not allowed and selects HSI.
                let sw = new & 3;
                let sws = if sw == 3 { 0 } else { sw };
                self.set(CFGR, new & !0xC | sws << 2);
            }
            CIR => {
                let flags = old & CIR_FLAGS & !(new >> 16);
                self.set(CIR, flags | new & CIR_ENABLES);
            }
            BDCR => {
                if !self.backup_writable(ctx) {
                    ctx.blocker(
                        "backup-locked",
                        format!("RCC_BDCR write 0x{new:X} without PWR_CR.DBP (ignored)"),
                        false,
                    );
                    return;
                }
                if new & BDRST != 0 {
                    self.set(BDCR, BDRST);
                    return;
                }
                // RTCSEL is write-once until a backup domain reset.
                let rtcsel = if old & RTCSEL == 0 { new & RTCSEL } else { old & RTCSEL };
                let before = self.ready_bits();
                self.set(BDCR, new & (LSEON | 1 << 2 | RTCEN) | (new & LSEON) << 1 | rtcsel);
                self.latch_ready_flags(before);
            }
            CSR => {
                let flags = if new & RMVF != 0 { 0 } else { old & RESET_FLAGS };
                let before = self.ready_bits();
                self.set(CSR, flags | new & LSION | (new & LSION) << 1);
                self.latch_ready_flags(before);
            }
            APB2RSTR | APB1RSTR | AHBENR | APB2ENR | APB1ENR => self.set(off, new),
            _ => {}
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            CR => "CR",
            CFGR => "CFGR",
            CIR => "CIR",
            APB2RSTR => "APB2RSTR",
            APB1RSTR => "APB1RSTR",
            AHBENR => "AHBENR",
            APB2ENR => "APB2ENR",
            APB1ENR => "APB1ENR",
            BDCR => "BDCR",
            CSR => "CSR",
            _ => return None,
        })
    }
}

impl Rcc {
    /// Sets the CIR ready flag of every clock that just became ready.
    fn latch_ready_flags(&mut self, before: u32) {
        let rose = self.ready_bits() & !before;
        let cir = self.reg(CIR);
        self.set(CIR, cir | rose);
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    fn write(rcc: &mut Rcc, host: &mut TestHost, off: u32, value: u32) {
        rcc.write(off, value, u32::MAX, &mut host.ctx());
    }

    fn read(rcc: &mut Rcc, host: &mut TestHost, off: u32) -> u32 {
        rcc.read(off, u32::MAX, &mut host.ctx())
    }

    #[test]
    fn power_on_state_runs_on_hsi() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        assert_eq!(read(&mut rcc, &mut host, CR), 0x83);
        assert_eq!(read(&mut rcc, &mut host, AHBENR), 0x14);
        assert_eq!(read(&mut rcc, &mut host, CSR), 0x0C00_0000);
        let clocks = rcc.clocks();
        assert_eq!(
            (clocks.sysclk, clocks.hclk, clocks.pclk1, clocks.adcclk),
            (8_000_000, 8_000_000, 8_000_000, 4_000_000)
        );
        assert_eq!(clocks.rtcclk, None);
    }

    /// The CMSIS `SetSysClockTo72` sequence the Begode Extreme runs.
    #[test]
    fn set_sys_clock_to_72() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        write(&mut rcc, &mut host, CR, 0x0001_0083);
        assert_ne!(read(&mut rcc, &mut host, CR) & HSERDY, 0, "HSE ready at once");
        write(&mut rcc, &mut host, CFGR, 0x0000_0400); // HPRE /1, PPRE2 /1, PPRE1 /2
        write(&mut rcc, &mut host, CFGR, 0x001D_0400); // PLL = HSE x9
        write(&mut rcc, &mut host, CR, 0x0101_0083);
        assert_ne!(read(&mut rcc, &mut host, CR) & PLLRDY, 0);
        write(&mut rcc, &mut host, CFGR, 0x001D_0402); // SW = PLL
        assert_eq!(read(&mut rcc, &mut host, CFGR) & 0xC, 0x8, "SWS = PLL");
        write(&mut rcc, &mut host, CFGR, 0x001D_8402); // ADCPRE /6
        let clocks = rcc.clocks();
        assert_eq!(
            (clocks.sysclk, clocks.hclk, clocks.pclk1, clocks.pclk2),
            (72_000_000, 72_000_000, 36_000_000, 72_000_000)
        );
        assert_eq!((clocks.tim_apb1, clocks.tim_apb2, clocks.adcclk), (72_000_000, 72_000_000, 12_000_000));
        assert_eq!(read(&mut rcc, &mut host, CIR) & 0x18, 0x18, "HSE and PLL ready flags latched");
        write(&mut rcc, &mut host, CIR, 0x009F_0000);
        assert_eq!(read(&mut rcc, &mut host, CIR), 0, "CIR clear bits");
    }

    #[test]
    fn prescalers() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        // SW = HSE, HPRE /2 (1000), PPRE1 /4 (101), PPRE2 /16 (111), ADCPRE /8 (11).
        write(&mut rcc, &mut host, CR, HSEON);
        write(&mut rcc, &mut host, CFGR, 0xFD81); // 11 111 101 1000 00 01
        let clocks = rcc.clocks();
        assert_eq!((clocks.sysclk, clocks.hclk), (8_000_000, 4_000_000));
        assert_eq!((clocks.pclk1, clocks.tim_apb1), (1_000_000, 2_000_000));
        assert_eq!((clocks.pclk2, clocks.tim_apb2, clocks.adcclk), (250_000, 500_000, 31_250));
        write(&mut rcc, &mut host, CFGR, 0b1111 << 4);
        assert_eq!(rcc.clocks().hclk, 8_000_000 / 512);
        // PLL from HSI/2 x16 (PLLMUL 1111 also means x16).
        write(&mut rcc, &mut host, CFGR, 0x003C_0002);
        assert_eq!(rcc.clocks().sysclk, 64_000_000);
    }

    #[test]
    fn rtc_clock_is_hse_div_128_when_enabled() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        write(&mut rcc, &mut host, CR, HSEON);
        write(&mut rcc, &mut host, BDCR, 0x300);
        assert_eq!(rcc.clocks().rtcclk, None, "RTCEN still off");
        write(&mut rcc, &mut host, BDCR, 0x8300);
        assert_eq!(rcc.clocks().rtcclk, Some(62_500));
        write(&mut rcc, &mut host, BDCR, 0x8100);
        assert_eq!(read(&mut rcc, &mut host, BDCR) & RTCSEL, 0x300, "RTCSEL is write-once");
        write(&mut rcc, &mut host, BDCR, BDRST);
        write(&mut rcc, &mut host, BDCR, 0);
        write(&mut rcc, &mut host, BDCR, 0x8101);
        assert_eq!(read(&mut rcc, &mut host, BDCR), 0x8103, "LSE ready, new source after BDRST");
        assert_eq!(rcc.clocks().rtcclk, Some(LSE_HZ));
    }

    #[test]
    fn backup_domain_and_reset_flags_across_resets() {
        let mut host = TestHost::new(72_000_000);
        let mut rcc = Rcc::new(8_000_000, None);
        write(&mut rcc, &mut host, BDCR, 0x8300);
        write(&mut rcc, &mut host, CSR, RMVF | LSION);
        assert_eq!(read(&mut rcc, &mut host, CSR), LSION | LSIRDY);
        write(&mut rcc, &mut host, APB2ENR, 0x2000);
        rcc.reset(ResetKind::System, &mut host.ctx());
        assert_eq!(read(&mut rcc, &mut host, BDCR), 0x8300, "backup domain kept");
        assert_eq!(read(&mut rcc, &mut host, CSR), PINRSTF);
        assert_eq!(read(&mut rcc, &mut host, APB2ENR), 0);
        rcc.reset(ResetKind::Power, &mut host.ctx());
        assert_eq!(read(&mut rcc, &mut host, BDCR), 0);
        assert_eq!(read(&mut rcc, &mut host, CSR), PINRSTF | PORRSTF);
    }

    #[test]
    fn bdcr_needs_backup_access_when_pwr_is_wired() {
        let mut host = TestHost::new(72_000_000);
        host.peers.insert(7, Box::new(Pwr::new()));
        let mut rcc = Rcc::new(8_000_000, Some(7));
        write(&mut rcc, &mut host, BDCR, 0x8300);
        assert_eq!(read(&mut rcc, &mut host, BDCR), 0);
        assert_eq!(host.blockers[0].0, "backup-locked");
        let mut pwr = Pwr::new();
        pwr.write(0, crate::pwr::DBP, u32::MAX, &mut TestHost::new(72_000_000).ctx());
        host.peers.insert(7, Box::new(pwr));
        write(&mut rcc, &mut host, BDCR, 0x8300);
        assert_eq!(read(&mut rcc, &mut host, BDCR), 0x8300);
    }
}

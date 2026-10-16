//! General-purpose I/O port (RM0008 §9.2).

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind, Signal};

use crate::util::{NoParams, merge};

const CRL: u32 = 0x00;
const CRH: u32 = 0x04;
const IDR: u32 = 0x08;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;
const BRR: u32 = 0x14;
const LCKR: u32 = 0x18;

/// Pins per port.
pub const PINS: u8 = 16;

/// First output port that carries input edges to the EXTI (`"exti0"`..`"exti15"`).
const EXTI_PORT: u8 = PINS;

/// How one pin is configured (`MODE` and `CNF` bits of `CRL`/`CRH`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
    /// Analog input: the digital input path is off and reads 0.
    Analog,
    /// Floating input.
    Floating,
    /// Input with the pull-up (`ODR` bit 1) or pull-down (`ODR` bit 0) resistor.
    Pull,
    /// General-purpose push-pull output.
    PushPull,
    /// General-purpose open-drain output.
    OpenDrain,
    /// Alternate-function output (push-pull or open-drain): a peripheral drives the pin.
    Alternate,
}

impl PinMode {
    const fn from_bits(bits: u32) -> Self {
        let (mode, cnf) = (bits & 3, bits >> 2 & 3);
        match (mode, cnf) {
            (0, 0) => Self::Analog,
            (0, 2) => Self::Pull,
            (0, _) => Self::Floating,
            (_, 0) => Self::PushPull,
            (_, 1) => Self::OpenDrain,
            _ => Self::Alternate,
        }
    }
}

/// One GPIO port: 16 pins, their configuration, input and output data registers.
///
/// Wiring: port `"0"`..`"15"` is the pin itself. A wire into a pin carries [`Signal::Level`] or
/// [`Signal::Released`] from the outside (a button, a hall sensor, a chip's output). A pin
/// configured as a general-purpose output puts its level on the wire whenever it changes, and
/// [`Signal::Released`] when it stops driving (it turns into an input, or an open-drain output
/// goes high). Every change of an `IDR` bit also goes out on `"exti0"`..`"exti15"` for the EXTI.
///
/// `IDR` reads, per pin: an output's own level (an open-drain output that is released reads the
/// outside level, or 1 for a pull-up resistor on the board); an input's outside level; with
/// nothing driving it, the pull resistor's level, or 0 when floating; 0 in analog mode.
/// Alternate-function pins read their `ODR` bit (the peripheral's level is not modeled).
///
/// Not modeled: the configuration lock (`LCKR` reads back), output speed.
#[derive(Debug, Clone)]
pub struct Gpio {
    cr: u64,
    odr: u16,
    lckr: u32,
    /// Level each pin gets from the outside, if something drives it.
    outside: [Option<bool>; PINS as usize],
    /// What the port drives onto each pin, as last announced on its wire.
    driven: [Option<bool>; PINS as usize],
    idr: u16,
}

impl Default for Gpio {
    fn default() -> Self {
        Self::new()
    }
}

impl Gpio {
    /// A port after reset: every pin a floating input.
    pub fn new() -> Self {
        Self {
            cr: 0x4444_4444_4444_4444,
            odr: 0,
            lckr: 0,
            outside: [None; PINS as usize],
            driven: [None; PINS as usize],
            idr: 0,
        }
    }

    /// Builds the `stm32f1.gpio` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }

    /// The configuration of `pin`.
    pub const fn mode(&self, pin: u8) -> PinMode {
        PinMode::from_bits((self.cr >> (4 * pin) & 0xF) as u32)
    }

    /// The input data register as the firmware reads it.
    pub const fn idr(&self) -> u16 {
        self.idr
    }

    /// The output data register.
    pub const fn odr(&self) -> u16 {
        self.odr
    }

    /// What the port drives onto `pin`: `Some(level)` for an output, `None` when it lets go.
    const fn drive_of(&self, pin: u8) -> Option<bool> {
        let high = self.odr >> pin & 1 == 1;
        match self.mode(pin) {
            PinMode::PushPull => Some(high),
            PinMode::OpenDrain if !high => Some(false),
            _ => None,
        }
    }

    fn compute_idr(&self) -> u16 {
        (0..PINS).fold(0, |idr, pin| {
            let odr = self.odr >> pin & 1 == 1;
            let outside = self.outside[usize::from(pin)];
            let level = match self.mode(pin) {
                PinMode::Analog => false,
                PinMode::Floating => outside.unwrap_or(false),
                PinMode::Pull => outside.unwrap_or(odr),
                PinMode::PushPull | PinMode::Alternate => odr,
                PinMode::OpenDrain => odr && outside.unwrap_or(true),
            };
            idr | u16::from(level) << pin
        })
    }

    /// Re-evaluates outputs and inputs after any change, announcing what changed.
    fn update(&mut self, ctx: &mut Ctx<'_>) {
        for pin in 0..PINS {
            let drive = self.drive_of(pin);
            if drive != self.driven[usize::from(pin)] {
                self.driven[usize::from(pin)] = drive;
                ctx.emit(pin, drive.map_or(Signal::Released, Signal::Level));
            }
        }
        let idr = self.compute_idr();
        let mut changed = idr ^ self.idr;
        self.idr = idr;
        while changed != 0 {
            let pin = changed.trailing_zeros() as u8;
            changed &= changed - 1;
            ctx.emit(EXTI_PORT + pin, Signal::Level(idr >> pin & 1 == 1));
        }
    }
}

impl Peripheral for Gpio {
    fn reset(&mut self, _kind: ResetKind, ctx: &mut Ctx<'_>) {
        let outside = self.outside;
        *self = Self { outside, driven: self.driven, idr: self.idr, ..Self::new() };
        self.update(ctx);
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            CRL => self.cr as u32,
            CRH => (self.cr >> 32) as u32,
            IDR => u32::from(self.idr),
            ODR => u32::from(self.odr),
            LCKR => self.lckr,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let value = value & mask;
        match off {
            CRL => self.cr = self.cr & !u64::from(mask) | u64::from(value),
            CRH => self.cr = self.cr & !(u64::from(mask) << 32) | u64::from(value) << 32,
            ODR => self.odr = merge(u32::from(self.odr), value, mask) as u16,
            // Set bits win over reset bits written in the same access.
            BSRR => self.odr = self.odr & !(value >> 16) as u16 | value as u16,
            BRR => self.odr &= !(value as u16),
            LCKR => self.lckr = merge(self.lckr, value, mask) & 0x1_FFFF,
            _ => return,
        }
        self.update(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let Some(slot) = self.outside.get_mut(usize::from(port)) else { return };
        *slot = match signal {
            Signal::Level(level) => Some(level),
            Signal::Released => None,
            _ => return,
        };
        self.update(ctx);
    }

    fn port(&self, name: &str) -> Option<u8> {
        let (offset, number) = name.strip_prefix("exti").map_or((0, name), |n| (EXTI_PORT, n));
        let pin: u8 = number.parse().ok().filter(|&pin| pin < PINS)?;
        Some(offset + pin)
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            CRL => "CRL",
            CRH => "CRH",
            IDR => "IDR",
            ODR => "ODR",
            BSRR => "BSRR",
            BRR => "BRR",
            LCKR => "LCKR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    fn configure(gpio: &mut Gpio, host: &mut TestHost, pin: u8, bits: u32) {
        let (off, shift) = if pin < 8 { (CRL, 4 * pin) } else { (CRH, 4 * (pin - 8)) };
        let old = gpio.read(off, u32::MAX, &mut host.ctx());
        gpio.write(off, old & !(0xF << shift) | bits << shift, u32::MAX, &mut host.ctx());
    }

    #[test]
    fn reset_state_and_port_names() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        assert_eq!(gpio.read(CRL, u32::MAX, &mut host.ctx()), 0x4444_4444);
        assert_eq!(gpio.read(CRH, u32::MAX, &mut host.ctx()), 0x4444_4444);
        assert_eq!(gpio.mode(3), PinMode::Floating);
        assert_eq!((gpio.port("0"), gpio.port("15"), gpio.port("exti7")), (Some(0), Some(15), Some(23)));
        assert_eq!((gpio.port("16"), gpio.port("x")), (None, None));
    }

    #[test]
    fn outputs_announce_levels_and_bsrr_set_wins() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        configure(&mut gpio, &mut host, 14, 0x3); // PB14 power latch: push-pull 50 MHz
        assert_eq!(host.emitted, [(14, Signal::Level(false))]);
        host.emitted.clear();
        gpio.write(BSRR, 1 << 14, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted, [(14, Signal::Level(true)), (EXTI_PORT + 14, Signal::Level(true))]);
        assert_eq!(gpio.read(IDR, u32::MAX, &mut host.ctx()), 1 << 14, "an output reads its own level");
        host.emitted.clear();
        gpio.write(BSRR, 1 << 30 | 1 << 14, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted, [], "set and reset together: set wins, nothing changes");
        gpio.write(BRR, 1 << 14, u32::MAX, &mut host.ctx());
        assert_eq!(gpio.odr(), 0);
        assert_eq!(gpio.read(BSRR, u32::MAX, &mut host.ctx()), 0, "BSRR and BRR are write-only");
        host.emitted.clear();
        configure(&mut gpio, &mut host, 14, 0x4);
        assert_eq!(host.emitted, [(14, Signal::Released)], "an input stops driving");
    }

    #[test]
    fn inputs_follow_the_outside_and_the_pull_resistors() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        // PB15 button: input with pull-down (CNF 10, ODR 0); PB7 hall: floating.
        configure(&mut gpio, &mut host, 15, 0x8);
        assert_eq!(gpio.idr() >> 15 & 1, 0);
        gpio.write(ODR, 1 << 15, u32::MAX, &mut host.ctx());
        assert_eq!(gpio.idr() >> 15 & 1, 1, "pull-up when ODR = 1");
        gpio.input(15, Signal::Level(false), &mut host.ctx());
        assert_eq!(gpio.idr() >> 15 & 1, 0, "the button pulls it low");
        gpio.input(15, Signal::Released, &mut host.ctx());
        assert_eq!(gpio.idr() >> 15 & 1, 1);
        host.emitted.clear();
        gpio.input(7, Signal::Level(true), &mut host.ctx());
        assert_eq!(host.emitted, [(EXTI_PORT + 7, Signal::Level(true))]);
        configure(&mut gpio, &mut host, 7, 0x0);
        assert_eq!(gpio.idr() >> 7 & 1, 0, "analog inputs read 0");
    }

    #[test]
    fn open_drain_releases_high_and_reads_the_bus() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        configure(&mut gpio, &mut host, 2, 0x7); // open-drain output
        assert_eq!(host.emitted[0], (2, Signal::Level(false)));
        gpio.write(BSRR, 1 << 2, u32::MAX, &mut host.ctx());
        assert_eq!(host.emitted.last(), Some(&(EXTI_PORT + 2, Signal::Level(true))));
        assert!(host.emitted.contains(&(2, Signal::Released)));
        gpio.input(2, Signal::Level(false), &mut host.ctx());
        assert_eq!(gpio.idr() >> 2 & 1, 0, "another device holds the line low");
    }

    #[test]
    fn byte_writes_to_odr_and_crh() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        gpio.write(CRH, 0x0300_0000, 0xFF00_0000, &mut host.ctx()); // PB14 output, PB15 analog
        assert_eq!(gpio.read(CRH, u32::MAX, &mut host.ctx()), 0x0344_4444);
        gpio.write(ODR, 0x4000, 0xFF00, &mut host.ctx());
        assert_eq!(gpio.odr(), 0x4000);
    }

    #[test]
    fn reset_keeps_outside_levels() {
        let mut host = TestHost::new(72_000_000);
        let mut gpio = Gpio::new();
        gpio.input(5, Signal::Level(true), &mut host.ctx());
        configure(&mut gpio, &mut host, 6, 0x3);
        gpio.write(BSRR, 1 << 6, u32::MAX, &mut host.ctx());
        host.emitted.clear();
        gpio.reset(ResetKind::System, &mut host.ctx());
        assert_eq!(gpio.idr(), 1 << 5);
        assert_eq!(host.emitted, [(6, Signal::Released), (EXTI_PORT + 6, Signal::Level(false))]);
    }
}

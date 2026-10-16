//! External interrupt/event controller (RM0008 §10.3).

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind, Signal};
use serde::Deserialize;

use crate::afio::Afio;
use crate::util::merge;

const IMR: u32 = 0x00;
const EMR: u32 = 0x04;
const RTSR: u32 = 0x08;
const FTSR: u32 = 0x0C;
const SWIER: u32 = 0x10;
const PR: u32 = 0x14;

/// Lines 0-18: 16 GPIO lines, PVD, RTC alarm, USB wake-up.
const LINES: u32 = 0x7_FFFF;

/// GPIO ports the AFIO multiplexer can select (A..G).
const GPIO_PORTS: u8 = 7;

/// First input port of the internal lines (16, 17, 18).
const INTERNAL_PORT: u8 = GPIO_PORTS * 16;

/// Interrupt roles in `board.toml` and the lines each one covers.
const ROLES: [(&str, u32); 10] = [
    ("exti0", 1 << 0),
    ("exti1", 1 << 1),
    ("exti2", 1 << 2),
    ("exti3", 1 << 3),
    ("exti4", 1 << 4),
    ("exti9_5", 0x3E0),
    ("exti15_10", 0xFC00),
    ("pvd", 1 << 16),
    ("rtc_alarm", 1 << 17),
    ("usb_wakeup", 1 << 18),
];

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// The AFIO instance that selects the GPIO port of lines 0-15.
    afio: String,
}

/// EXTI: edge detectors on 19 lines that pend interrupts.
///
/// Inputs: `"PA0"`..`"PG15"` take the `"exti<n>"` edge outputs of the GPIO ports; line `n` uses
/// the one that AFIO selects for it. `"pvd"`, `"rtc_alarm"` and `"usb_wakeup"` are lines 16-18.
/// A [`Signal::Level`] is an edge to that level; [`Signal::Pulse`] is a rising edge.
///
/// An edge enabled in `RTSR`/`FTSR` sets its `PR` bit; `PR & IMR` drives the interrupt roles
/// `exti0`..`exti4`, `exti9_5`, `exti15_10`, `pvd`, `rtc_alarm`, `usb_wakeup` (each optional).
/// `SWIER` pends an unmasked line by software; writing 1 to `PR` clears it and its `SWIER` bit.
///
/// Not modeled: event outputs (`EMR` reads back; nothing waits for events).
#[derive(Debug, Clone)]
pub struct Exti {
    afio: DeviceId,
    /// Interrupt line and the EXTI lines behind it, with the level last driven.
    irqs: Vec<(u16, u32, bool)>,
    imr: u32,
    emr: u32,
    rtsr: u32,
    ftsr: u32,
    swier: u32,
    pr: u32,
}

impl Exti {
    /// An EXTI after reset, reading its line multiplexer from `afio`, with `irqs` = (interrupt,
    /// EXTI line mask) pairs.
    pub fn new(afio: DeviceId, irqs: Vec<(u16, u32)>) -> Self {
        let irqs = irqs.into_iter().map(|(irq, lines)| (irq, lines, false)).collect();
        Self { afio, irqs, imr: 0, emr: 0, rtsr: 0, ftsr: 0, swier: 0, pr: 0 }
    }

    /// Builds the `stm32f1.exti` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: Config = build.params()?;
        let irqs = ROLES.iter().filter_map(|&(role, lines)| build.irq(role).ok().map(|irq| (irq, lines))).collect();
        Ok(Box::new(Self::new(build.id_of(&config.afio)?, irqs)))
    }

    /// Pending lines.
    pub const fn pending(&self) -> u32 {
        self.pr
    }

    fn update_irqs(&mut self, ctx: &mut Ctx<'_>) {
        let active = self.pr & self.imr;
        for (irq, lines, level) in &mut self.irqs {
            let high = active & *lines != 0;
            if high != *level {
                *level = high;
                ctx.set_irq(*irq, high);
            }
        }
    }

    fn edge(&mut self, line: u32, rising: bool, ctx: &mut Ctx<'_>) {
        let bit = 1 << line;
        let enabled = if rising { self.rtsr } else { self.ftsr };
        if enabled & bit != 0 {
            self.pr |= bit;
            self.update_irqs(ctx);
        }
    }
}

impl Peripheral for Exti {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        // The engine has dropped the interrupt lines; nothing is pending after reset.
        let irqs = self.irqs.iter().map(|&(irq, lines, _)| (irq, lines)).collect();
        *self = Self::new(self.afio, irqs);
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            IMR => self.imr,
            EMR => self.emr,
            RTSR => self.rtsr,
            FTSR => self.ftsr,
            SWIER => self.swier,
            PR => self.pr,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let set = value & mask & LINES;
        match off {
            IMR => self.imr = merge(self.imr, value, mask) & LINES,
            EMR => self.emr = merge(self.emr, value, mask) & LINES,
            RTSR => self.rtsr = merge(self.rtsr, value, mask) & LINES,
            FTSR => self.ftsr = merge(self.ftsr, value, mask) & LINES,
            SWIER => {
                // A 0 -> 1 transition pends the line if it is unmasked.
                self.pr |= set & !self.swier & self.imr;
                self.swier |= set;
            }
            PR => {
                self.pr &= !set;
                self.swier &= !set;
            }
            _ => return,
        }
        self.update_irqs(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let rising = match signal {
            Signal::Level(level) => level,
            Signal::Pulse => true,
            _ => return,
        };
        if port < INTERNAL_PORT {
            let (gpio, pin) = (port / 16, port % 16);
            if ctx.peer::<Afio>(self.afio).is_some_and(|afio| afio.exti_source(pin) == gpio) {
                self.edge(u32::from(pin), rising, ctx);
            }
        } else if port < INTERNAL_PORT + 3 {
            self.edge(u32::from(port - INTERNAL_PORT) + 16, rising, ctx);
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "pvd" => Some(INTERNAL_PORT),
            "rtc_alarm" => Some(INTERNAL_PORT + 1),
            "usb_wakeup" => Some(INTERNAL_PORT + 2),
            _ => {
                let rest = name.strip_prefix('P')?;
                let gpio = rest.as_bytes().first()?.checked_sub(b'A').filter(|&g| g < GPIO_PORTS)?;
                let pin: u8 = rest[1..].parse().ok().filter(|&pin| pin < 16)?;
                Some(gpio * 16 + pin)
            }
        }
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            IMR => "IMR",
            EMR => "EMR",
            RTSR => "RTSR",
            FTSR => "FTSR",
            SWIER => "SWIER",
            PR => "PR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const AFIO: DeviceId = 9;

    fn setup() -> (TestHost, Exti) {
        let mut host = TestHost::new(72_000_000);
        let mut afio = Afio::new();
        afio.write(0x08, 0x0010, u32::MAX, &mut host.ctx()); // line 1 on port B
        host.peers.insert(AFIO, Box::new(afio));
        let exti = Exti::new(AFIO, vec![(6, 1 << 0), (7, 1 << 1), (40, 0xFC00), (41, 1 << 17)]);
        (host, exti)
    }

    #[test]
    fn port_names() {
        let (_, exti) = setup();
        assert_eq!((exti.port("PA0"), exti.port("PB1"), exti.port("PG15")), (Some(0), Some(17), Some(111)));
        assert_eq!((exti.port("rtc_alarm"), exti.port("PH0"), exti.port("PA16")), (Some(113), None, None));
    }

    #[test]
    fn edges_on_the_selected_port_pend_unmasked_lines() {
        let (mut host, mut exti) = setup();
        exti.write(RTSR, 0b10, u32::MAX, &mut host.ctx());
        exti.input(1, Signal::Level(true), &mut host.ctx()); // PA1: not the selected port
        assert_eq!(exti.pending(), 0);
        exti.input(17, Signal::Level(true), &mut host.ctx()); // PB1
        assert_eq!(exti.pending(), 0b10);
        assert!(!host.irq(7), "masked");
        exti.write(IMR, 0b10, u32::MAX, &mut host.ctx());
        assert!(host.irq(7));
        exti.input(17, Signal::Level(false), &mut host.ctx());
        exti.write(PR, 0b10, u32::MAX, &mut host.ctx());
        assert!(!host.irq(7));
        exti.input(17, Signal::Level(false), &mut host.ctx());
        assert_eq!(exti.pending(), 0, "falling edges are not enabled");
    }

    #[test]
    fn grouped_lines_share_one_interrupt() {
        let (mut host, mut exti) = setup();
        exti.write(IMR, 0xFC00, u32::MAX, &mut host.ctx());
        exti.write(FTSR, 0x8000, u32::MAX, &mut host.ctx());
        exti.input(15, Signal::Level(false), &mut host.ctx()); // PA15 falls
        assert!(host.irq(40));
        exti.write(PR, 0x8000, u32::MAX, &mut host.ctx());
        assert!(!host.irq(40));
    }

    #[test]
    fn software_interrupts_and_internal_lines() {
        let (mut host, mut exti) = setup();
        exti.write(IMR, 1, u32::MAX, &mut host.ctx());
        exti.write(SWIER, 1, u32::MAX, &mut host.ctx());
        assert!(host.irq(6));
        exti.write(SWIER, 1, u32::MAX, &mut host.ctx());
        exti.write(PR, 1, u32::MAX, &mut host.ctx());
        assert_eq!((exti.pending(), exti.read(SWIER, u32::MAX, &mut host.ctx())), (0, 0));
        exti.write(SWIER, 2, u32::MAX, &mut host.ctx());
        assert_eq!(exti.pending(), 0, "SWIER on a masked line does not pend");
        exti.write(IMR, 1 << 17, u32::MAX, &mut host.ctx());
        exti.write(RTSR, 1 << 17, u32::MAX, &mut host.ctx());
        exti.input(exti.port("rtc_alarm").unwrap(), Signal::Pulse, &mut host.ctx());
        assert!(host.irq(41));
    }
}

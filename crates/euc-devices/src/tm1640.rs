//! TM1640-style LED/7-segment driver on a 2-wire bit-banged bus.

use euc_core::{Build, ConfigError, Ctx, Peripheral, ResetKind, Signal};

use crate::NoParams;

const PORT_CLK: u8 = 0;
const PORT_DIN: u8 = 1;

/// Number of grid (digit) registers.
pub const GRIDS: usize = 16;

/// Receiver for a TM1640 LED driver, decoded from its two lines (CLK and DIN).
///
/// Bus rules (TM1640 datasheet): a frame starts when DIN falls while CLK is high and ends when DIN
/// rises while CLK is high; between them, DIN is sampled on each CLK rising edge, least
/// significant bit first. The first byte of a frame is a command:
///
/// | Command | Meaning |
/// |---|---|
/// | `0b01xx_xxxx` | data command: bit 2 set = fixed address, clear = auto-increment |
/// | `0b10xx_xxxx` | display control: bit 3 = display on, bits 0-2 = brightness (pulse width) |
/// | `0b11xx_aaaa` | address command: grid `aaaa`, followed by the data bytes |
///
/// The Begode Extreme drives its display this way on PC5 (CLK) and PC4 (DIN); that the chip is a
/// TM1640 is INFERRED from the protocol.
#[derive(Debug, Clone)]
pub struct Tm1640 {
    clk: bool,
    din: bool,
    in_frame: bool,
    shift: u8,
    bits: u8,
    frame: Vec<u8>,
    grid: [u8; GRIDS],
    /// Last data command; bit 2 selects fixed addressing.
    data_command: u8,
    /// Last display-control command.
    control: u8,
    frames: u64,
}

impl Default for Tm1640 {
    fn default() -> Self {
        Self::new()
    }
}

impl Tm1640 {
    /// A driver at power-on: lines idle high, display off, grid blank.
    pub fn new() -> Self {
        Self {
            clk: true,
            din: true,
            in_frame: false,
            shift: 0,
            bits: 0,
            frame: Vec::new(),
            grid: [0; GRIDS],
            data_command: 0x40,
            control: 0,
            frames: 0,
        }
    }

    /// Updates the line levels. Call it whenever either output changes; when both change in one
    /// call, CLK is applied first.
    pub fn set_lines(&mut self, clk: bool, din: bool) {
        if clk != self.clk {
            if clk && self.in_frame {
                self.shift |= u8::from(self.din) << self.bits;
                self.bits += 1;
                if self.bits == 8 {
                    self.frame.push(self.shift);
                    (self.shift, self.bits) = (0, 0);
                }
            }
            self.clk = clk;
        }
        if din != self.din {
            if self.clk && !din {
                self.in_frame = true;
                self.frame.clear();
                (self.shift, self.bits) = (0, 0);
            } else if self.clk && din && self.in_frame {
                self.in_frame = false;
                self.end_frame();
            }
            self.din = din;
        }
    }

    /// The 16 grid registers (one byte of segments per digit or LED group).
    pub const fn grid(&self) -> &[u8; GRIDS] {
        &self.grid
    }

    /// Display switched on by the last display-control command.
    pub const fn display_on(&self) -> bool {
        self.control & 0x08 != 0
    }

    /// Brightness 0..=7 from the last display-control command.
    pub const fn brightness(&self) -> u8 {
        self.control & 0x07
    }

    /// Complete frames received.
    pub const fn frames(&self) -> u64 {
        self.frames
    }

    fn end_frame(&mut self) {
        let Some((&command, data)) = self.frame.split_first() else { return };
        self.frames += 1;
        match command >> 6 {
            0b01 => self.data_command = command,
            0b10 => self.control = command & 0x0F,
            0b11 => {
                let mut addr = usize::from(command & 0x0F);
                for &byte in data {
                    self.grid[addr] = byte;
                    if self.data_command & 0x04 == 0 {
                        addr = (addr + 1) % GRIDS;
                    }
                }
            }
            _ => {}
        }
    }
}

impl Tm1640 {
    /// Builds the `tm1640` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }
}

/// As a board component (kind `tm1640`): `"clk"` and `"din"` take the two line levels; a released
/// line reads high (the bus has pull-ups).
impl Peripheral for Tm1640 {
    fn reset(&mut self, kind: ResetKind, _ctx: &mut Ctx<'_>) {
        if kind == ResetKind::Power {
            *self = Self::new();
        }
    }

    fn input(&mut self, port: u8, signal: Signal, _ctx: &mut Ctx<'_>) {
        let level = match signal {
            Signal::Level(level) => level,
            Signal::Released => true,
            _ => return,
        };
        match port {
            PORT_CLK => self.set_lines(level, self.din),
            PORT_DIN => self.set_lines(self.clk, level),
            _ => {}
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "clk" => Some(PORT_CLK),
            "din" => Some(PORT_DIN),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bit-bangs one frame the way the firmware does: start, bytes LSB first, stop.
    fn send(tm: &mut Tm1640, bytes: &[u8]) {
        tm.set_lines(true, true);
        tm.set_lines(true, false); // start
        for &byte in bytes {
            for bit in 0..8 {
                let level = byte >> bit & 1 == 1;
                tm.set_lines(false, false);
                tm.set_lines(false, level);
                tm.set_lines(true, level);
            }
        }
        tm.set_lines(false, false);
        tm.set_lines(true, false);
        tm.set_lines(true, true); // stop
    }

    #[test]
    fn auto_increment_write_fills_the_grid() {
        let mut tm = Tm1640::new();
        send(&mut tm, &[0x40]);
        send(&mut tm, &[0xC0, 0x3F, 0x06, 0x5B, 0x4F]);
        send(&mut tm, &[0x8F]);
        assert_eq!(tm.grid()[..5], [0x3F, 0x06, 0x5B, 0x4F, 0x00]);
        assert!(tm.display_on());
        assert_eq!(tm.brightness(), 7);
        assert_eq!(tm.frames(), 3);
    }

    #[test]
    fn fixed_address_mode_overwrites_one_grid() {
        let mut tm = Tm1640::new();
        send(&mut tm, &[0x44]);
        send(&mut tm, &[0xCF, 0x01, 0x02]);
        assert_eq!(tm.grid()[15], 0x02);
        assert_eq!(tm.grid()[0], 0x00, "no wrap in fixed mode");
    }

    #[test]
    fn auto_increment_wraps_after_grid_15() {
        let mut tm = Tm1640::new();
        send(&mut tm, &[0xCF, 0xAA, 0xBB]);
        assert_eq!((tm.grid()[15], tm.grid()[0]), (0xAA, 0xBB));
    }

    #[test]
    fn ignores_clocks_outside_a_frame_and_empty_frames() {
        let mut tm = Tm1640::new();
        for _ in 0..20 {
            tm.set_lines(false, true);
            tm.set_lines(true, true);
        }
        tm.set_lines(true, false);
        tm.set_lines(true, true);
        assert_eq!(tm.frames(), 0);
        assert_eq!(tm.grid(), &[0; GRIDS]);
    }

    #[test]
    fn display_off_command() {
        let mut tm = Tm1640::new();
        send(&mut tm, &[0x8C]);
        send(&mut tm, &[0x80]);
        assert!(!tm.display_on());
    }

    #[test]
    fn as_a_component_it_listens_to_its_two_lines() {
        use euc_core::testing::TestHost;

        let mut host = TestHost::new(72_000_000);
        let mut tm = Tm1640::new();
        let (clk, din) = (tm.port("clk").unwrap(), tm.port("din").unwrap());
        let mut line = |port: u8, level: bool| tm.input(port, Signal::Level(level), &mut host.ctx());
        line(din, false); // start
        for bit in 0..8 {
            line(clk, false);
            line(din, 0x8F_u8 >> bit & 1 == 1);
            line(clk, true);
        }
        line(clk, false);
        line(din, false);
        line(clk, true);
        tm.input(din, Signal::Released, &mut host.ctx()); // stop
        assert!(tm.display_on());
        assert_eq!(tm.frames(), 1);
    }
}

//! `board.toml` and `stockmap.toml`: what the machine is made of and what we know about the
//! stock firmware.

use std::collections::BTreeMap;

use euc_core::InstanceSpec;
use serde::Deserialize;

/// A board file (`profiles/<wheel>/board.toml`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardFile {
    /// Schema version, 1.
    pub schema: u32,
    /// The microcontroller's interrupt controller.
    pub mcu: Mcu,
    /// Address map.
    pub memory: MemoryMap,
    /// Components, in event-dispatch order.
    #[serde(default, rename = "instance")]
    pub instances: Vec<InstanceFile>,
    /// DMA request routes.
    #[serde(default)]
    pub dma: Vec<DmaRoute>,
    /// Wires between component ports.
    #[serde(default, rename = "wire")]
    pub wires: Vec<Wire>,
    /// What each ADC channel measures.
    #[serde(default)]
    pub analog: Vec<Analog>,
    /// The simulator's board logic.
    pub board: BoardLogic,
}

/// Interrupt controller facts.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mcu {
    /// Display name.
    pub name: String,
    /// External interrupt lines.
    pub irqs: u16,
    /// Implemented priority bits.
    pub priority_bits: u8,
    /// IRQ names by number, for logs and reports.
    pub irq_names: Vec<String>,
}

/// A range of addresses.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Span {
    /// First address.
    pub base: u32,
    /// Size in bytes.
    pub size: u32,
}

impl Span {
    /// First address past the range.
    pub const fn end(&self) -> u32 {
        self.base + self.size
    }

    /// True when `addr` is inside.
    pub const fn contains(&self, addr: u32) -> bool {
        addr >= self.base && addr - self.base < self.size
    }
}

/// The address map.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MemoryMap {
    /// All of flash.
    pub flash: Span,
    /// End of the plain-memory part of flash (the app); the data-flash model maps above it.
    pub code_end: u32,
    /// SRAM.
    pub sram: Span,
    /// Base of the SRAM bit-band alias.
    pub sram_bitband: u32,
    /// The peripheral region.
    pub periph: Span,
    /// Base of the peripheral bit-band alias.
    pub periph_bitband: u32,
    /// The private peripheral bus (SCS, DWT, ...).
    pub ppb: Span,
}

/// One `[[instance]]` table.
#[derive(Debug, Clone, Deserialize)]
pub struct InstanceFile {
    /// Unique name.
    pub name: String,
    /// Registered kind.
    pub kind: String,
    /// Bus address, absent for off-chip devices.
    pub base: Option<u32>,
    /// Window size; 0x400 when `base` is set and this is not.
    pub size: Option<u32>,
    /// Interrupt lines by role.
    #[serde(default)]
    pub irq: BTreeMap<String, u16>,
    /// Everything else, for the kind's constructor.
    #[serde(flatten)]
    pub params: toml::Table,
}

impl InstanceFile {
    /// The spec the registry builds from.
    pub fn spec(&self) -> InstanceSpec {
        InstanceSpec {
            name: self.name.clone(),
            kind: self.kind.clone(),
            base: self.base,
            size: self.size.unwrap_or(if self.base.is_some() { 0x400 } else { 0 }),
            irqs: self.irq.clone(),
            params: self.params.clone(),
        }
    }
}

/// `request` (`"USART3.rx"`) is served by DMA `channel` (`"DMA1.3"`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DmaRoute {
    /// `instance.line`.
    pub request: String,
    /// `controller.channel`.
    pub channel: String,
}

/// A wire from an output port to an input port (`"instance.port"`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Wire {
    /// Output end.
    pub from: String,
    /// Input end.
    pub to: String,
}

/// What ADC `adc` channel `channel` measures.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Analog {
    /// ADC number (1-based).
    pub adc: u8,
    /// Channel.
    pub channel: u8,
    /// Plant quantity: `battery_v`, `board_temp_c` or `motor_temp_c`.
    pub signal: String,
    /// Linear sensor: counts per unit as a fraction `[numerator, denominator]`.
    pub scale: Option<[u32; 2]>,
    /// NTC thermistor to ground with a pull-up.
    pub ntc: Option<Ntc>,
}

/// NTC divider constants.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ntc {
    /// Resistance at 25 °C in ohms.
    pub r25: f64,
    /// Beta constant in kelvin.
    pub beta: f64,
    /// Pull-up resistance in ohms.
    pub pullup: f64,
}

/// The simulator's board logic (`board` instance).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardLogic {
    /// The power button reads low when pressed.
    pub button_active_low: bool,
    /// Seconds after the firmware sets the power latch that the rider lets go of the button.
    pub release_after_latch_s: f64,
}

/// `profiles/<wheel>/stockmap.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StockMap {
    /// Schema version, 1.
    pub schema: u32,
    /// Label of the pinned build the addresses belong to.
    pub build: String,
    /// Named entry points.
    #[serde(default, rename = "entry")]
    pub entries: Vec<Entry>,
    /// Named RAM variables.
    #[serde(default, rename = "ram")]
    pub ram: Vec<RamVar>,
}

/// A named code address.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Entry {
    /// Our name.
    pub name: String,
    /// Address (Thumb bit clear).
    pub addr: u32,
    /// Record the first time execution reaches it.
    #[serde(default)]
    pub marker: bool,
}

/// A named RAM variable.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RamVar {
    /// Our name.
    pub name: String,
    /// Address.
    pub addr: u32,
    /// Size in bytes.
    pub size: u32,
    /// How to show it: `u8`, `u16`, `u32`, `ascii`.
    pub kind: String,
    /// What it is.
    pub note: Option<String>,
    /// What the simulator uses it for: `cutoff` (non-zero while the firmware has the motor cut
    /// or its balance output disabled).
    pub role: Option<String>,
}

/// Role of a RAM variable that is non-zero while the firmware cuts the motor.
pub const ROLE_CUTOFF: &str = "cutoff";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_begode_extreme_files_parse() {
        let source = euc_profile::ProfileSource::Builtin;
        let board: BoardFile = toml::from_str(&source.read("begode-extreme", "board.toml").unwrap()).unwrap();
        assert_eq!(board.mcu.irq_names.len(), usize::from(board.mcu.irqs));
        assert!(board.instances.iter().any(|i| i.name == "USART1" && i.irq["irq"] == 37));
        let stock: StockMap = toml::from_str(&source.read("begode-extreme", "stockmap.toml").unwrap()).unwrap();
        assert!(stock.entries.iter().any(|e| e.name == "main" && e.marker));
    }

    #[test]
    fn instance_defaults() {
        let file: InstanceFile = toml::from_str("name = \"X\"\nkind = \"k\"\nbase = 0x4000_0000\nfoo = 3").unwrap();
        let spec = file.spec();
        assert_eq!((spec.size, spec.params["foo"].as_integer()), (0x400, Some(3)));
        let device: InstanceFile = toml::from_str("name = \"imu\"\nkind = \"mpu6500\"").unwrap();
        assert_eq!(device.spec().size, 0);
    }
}

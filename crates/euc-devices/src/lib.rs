//! Off-chip devices of an EUC controller board: the parts the firmware talks to through pins,
//! SPI and UARTs.
//!
//! Each model is plain state with methods that know nothing about the CPU or the bus, plus a
//! [`euc_core::Peripheral`] implementation so a wheel's `board.toml` can place it and wire its
//! ports to the MCU's pins, UARTs, SPI ports and timer outputs ([`register`] adds the kinds).
//! Sensor and battery values come from the plant. The LED strip has no component: the board
//! decodes the RAM buffer behind its DMA channel with [`ws2812::decode`].
//!
//! | Kind | Model | Ports | Begode Extreme wiring |
//! |---|---|---|---|
//! | `mpu6500` | [`Mpu6500`] | `spi` (in and out), `cs` | SPI3 (PB3/PB4/PB5), chip select PA15 |
//! | `smart-bms` | [`SmartBms`] | `rx`, `tx` | USART3 (PB10/PB11) at 9600, reply by DMA1 channel 3 |
//! | `tm1640` | [`Tm1640`] | `clk`, `din` | CLK PC5, DIN PC4 |
//! | `buzzer` | [`Buzzer`] | `pwm` | TIM4 CH4 (PB9) |
//! | (none) | [`ws2812::decode`] | | TIM3 CH1 (PA6) fed by DMA1 channel 2 |

use euc_core::Registry;
use serde::Deserialize;

pub mod bms;
pub mod buzzer;
pub mod mpu6500;
pub mod tm1640;
pub mod ws2812;

pub use bms::SmartBms;
pub use buzzer::{Buzzer, BuzzerEvent, Pwm};
pub use mpu6500::Mpu6500;
pub use tm1640::Tm1640;
pub use ws2812::Rgb;

/// Registers every device kind: `mpu6500`, `smart-bms`, `tm1640`, `buzzer`.
pub fn register(registry: &mut Registry) {
    registry.register("mpu6500", Mpu6500::build);
    registry.register("smart-bms", SmartBms::build);
    registry.register("tm1640", Tm1640::build);
    registry.register("buzzer", Buzzer::build);
}

/// Parameters of a kind that takes none: any key in its `board.toml` table is an error.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NoParams {}

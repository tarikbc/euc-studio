//! Register-level models of the STM32F1 high-density peripherals (reference manual RM0008).
//!
//! Each model implements [`euc_core::Peripheral`]: the engine maps it at the base address that
//! `board.toml` gives, routes bus accesses, events and wire signals to it, and the model talks
//! back only through [`euc_core::Ctx`]. [`register`] adds every kind to a [`Registry`] so a board
//! file can instantiate them by name.
//!
//! The models implement what firmware observes: status bits that hardware sets on its own (clock
//! ready, conversion done, transmit empty, ...), flag clearing sequences, interrupt and DMA
//! request lines, and timing derived from the configured clocks. Registers a model does not
//! interpret read back what was written. Each model's documentation lists what it leaves out.
//!
//! Models that need another block's state name it with a parameter (`clock = "RCC"`,
//! `pwr = "PWR"`, `afio = "AFIO"`) and read it with a peer call.
//!
//! | Kind | Model | RM0008 |
//! |---|---|---|
//! | `stm32f1.rcc` | [`Rcc`] | §7.3 |
//! | `stm32f1.pwr` | [`Pwr`] | §5.4 |
//! | `stm32f1.bkp` | [`Bkp`] | §6.4 |
//! | `stm32f1.gpio` | [`Gpio`] | §9.2 |
//! | `stm32f1.afio` | [`Afio`] | §9.4 |
//! | `stm32f1.exti` | [`Exti`] | §10.3 |
//! | `stm32f1.tim` | [`Timer`] | §14, §15, §17 |
//! | `stm32f1.adc` | [`Adc`] | §11 |
//! | `stm32f1.usart` | [`Usart`] | §27 |
//! | `stm32f1.spi` | [`Spi`] | §25 |
//! | `stm32f1.dma` | [`Dma`] | §13 |
//! | `stm32f1.rtc` | [`Rtc`] | §18 |
//! | `stm32f1.iwdg` | [`Iwdg`] | §19 |
//! | `stm32f1.flash` | [`FlashCtl`] | §3.3.3 |
//! | `stm32f1.flash-array` | [`FlashArray`] | §3.3, PM0075 |
//! | `stm32f1.plain` | [`Plain`] | (no behavior) |

use euc_core::Registry;

mod adc;
mod afio;
mod bkp;
mod dma;
mod exti;
mod flash;
mod gpio;
mod iwdg;
mod plain;
mod pwr;
mod rcc;
mod rtc;
mod spi;
mod tim;
mod usart;
mod util;

pub use adc::{Adc, Conversion};
pub use afio::Afio;
pub use bkp::Bkp;
pub use dma::Dma;
pub use exti::Exti;
pub use flash::{FlashArray, FlashCtl};
pub use gpio::{Gpio, PINS, PinMode};
pub use iwdg::Iwdg;
pub use plain::Plain;
pub use pwr::Pwr;
pub use rcc::{Apb, Clocks, HSI_HZ, LSE_HZ, LSI_HZ, Rcc};
pub use rtc::Rtc;
pub use spi::Spi;
pub use tim::{Class, Timer};
pub use usart::Usart;

/// Registers every STM32F1 kind (`stm32f1.*`).
pub fn register(registry: &mut Registry) {
    registry.register("stm32f1.rcc", Rcc::build);
    registry.register("stm32f1.pwr", Pwr::build);
    registry.register("stm32f1.bkp", Bkp::build);
    registry.register("stm32f1.gpio", Gpio::build);
    registry.register("stm32f1.afio", Afio::build);
    registry.register("stm32f1.exti", Exti::build);
    registry.register("stm32f1.tim", Timer::build);
    registry.register("stm32f1.adc", Adc::build);
    registry.register("stm32f1.usart", Usart::build);
    registry.register("stm32f1.spi", Spi::build);
    registry.register("stm32f1.dma", Dma::build);
    registry.register("stm32f1.rtc", Rtc::build);
    registry.register("stm32f1.iwdg", Iwdg::build);
    registry.register("stm32f1.flash", FlashCtl::build);
    registry.register("stm32f1.flash-array", FlashArray::build);
    registry.register("stm32f1.plain", Plain::build);
}

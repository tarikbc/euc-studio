//! Engine backbone shared by every CPU backend, peripheral model and wheel.
//!
//! The simulator is deterministic: one [`Clock`] counts HCLK cycles, a [`Scheduler`] orders
//! every future event by `(cycle, sequence)`, and nothing inside the engine reads wall time.
//! A CPU backend implements [`Cpu`] and reaches memory and peripherals only through a [`Bus`],
//! so the interpreter we ship and the Unicorn test oracle are interchangeable.
//!
//! Board components implement [`Peripheral`] and reach the machine only through [`Ctx`]; the
//! [`ExceptionUnit`] is the NVIC/SCB/SysTick every backend shares, and [`Registry`] builds
//! components from `board.toml`.
//!
//! The run loop (`euc-sim`) built on these pieces:
//!
//! ```text
//! loop {
//!     fire every event due at clock.now()
//!     take the highest-priority pending exception (ExceptionUnit::best_pending + enter)
//!     budget = instructions until the next event, capped at the quantum
//!     cpu.run(bus, budget)      // MMIO and exception returns call back into the bus
//!     clock.advance_by(cycles spent)
//! }
//! ```

mod bus;
mod clock;
mod cpu;
mod nvic;
mod periph;
mod poll;
mod registry;
mod regs;
mod sched;
pub mod testing;

pub use bus::{Access, Bus, BusFault, BusFaultKind, Lane, Size};
pub use clock::Clock;
pub use cpu::{Cpu, Exit, Fault, Region, RegionKind};
pub use nvic::{Entry, ExceptionUnit, IRQ0, PENDSV, SCS_BASE, SYSTICK, ScsWrite, SysTick};
pub use periph::{Ctx, DeviceId, Host, ImuSample, Peripheral, PlantView, ResetKind, Signal};
pub use poll::PollDetector;
pub use registry::{Build, ConfigError, Constructor, InstanceSpec, Registry};
pub use regs::CoreRegs;
pub use sched::{EventId, Scheduler};

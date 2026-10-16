//! The simulated controller: memory map, board components, run loop and reports.
//!
//! A [`Machine`] is built from a wheel's `board.toml` (instances, wiring, analog channels) and
//! a firmware image, with any [`euc_core::Cpu`] backend. Components come from a
//! [`euc_core::Registry`]; [`registry`] holds every kind this repo models. The run loop and the
//! access path are ports of the Python reference harness, so a run makes the same decisions at
//! the same simulated cycles.

mod board;
pub mod config;
mod machine;
mod memory;
mod plant;
mod report;
mod system;

use euc_core::Registry;

pub use board::{Action, BoardLogic, Dwt};
pub use machine::{BOARD, BuildError, CpuFactory, Machine, MachineConfig, exception_name};
pub use plant::{Plant, PlantState};
pub use report::Report;
pub use system::{Blocker, Diagnostics};

/// Every component kind this repo models: STM32F1 peripherals, board devices and the
/// simulator's own.
pub fn registry() -> Registry {
    let mut registry = Registry::new();
    euc_stm32f1::register(&mut registry);
    euc_devices::register(&mut registry);
    board::register(&mut registry);
    registry
}

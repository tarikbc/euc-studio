//! Electrical plant of an EUC: what the controller firmware drives and senses.
//!
//! ```text
//! TIM8 CCR1/CCR3/CCR2 ──▶ inverter ──▶ PMSM (dq) ──▶ torque ──▶ body physics
//!     (or MOE off)          ▲   │ bus current   │ angle, speed ◀──┘
//!                           │   ▼               ▼
//!                  battery (2 × 32S)       hall sensors ──▶ GPIO
//!                           │                    phase currents, bus V/I, NTC ──▶ ADC
//!                           └──▶ smart BMS readings
//! ```
//!
//! [`ElectricalPlant`] runs the models on one fixed grid (5 µs by default), advanced lazily to
//! the engine's cycle whenever the engine is about to let the firmware look at it. Every
//! parameter comes from the wheel's `plant.toml` ([`PlantParams`]); `docs/models/electrical.md`
//! explains each model, the sign conventions that make the stock Begode firmware's FOC produce
//! the right torque, and why each default value was chosen.
//!
//! The models, each usable on its own:
//! - [`motor`]: PMSM in the rotor frame, backward-Euler step, torque, back-EMF;
//! - [`inverter`]: averaged pole voltages with dead-time error, body-diode conduction when the
//!   outputs are off;
//! - [`halls`]: three digital sensors with the firmware's sector table, offset, noise, stuck lines;
//! - [`sensors`]: phase and bus current, bus voltage and NTC counts for the ADC channels;
//! - [`battery`]: parallel packs of series cell groups with OCV(SOC), R0 and one RC pair;
//! - [`thermal`]: lumped heat for the power stage and the windings;
//! - [`fault`]: faults a scenario injects (stuck halls, sensor errors, an open phase, overheating,
//!   weak cells);
//! - [`rig`]: a held or free-spinning wheel to test the drive without body physics.

pub mod battery;
pub mod fault;
pub mod halls;
pub mod inverter;
pub mod motor;
mod plant;
pub mod rig;
mod rng;
pub mod sensors;
pub mod thermal;

pub use plant::{ElectricalPlant, PlantError, PlantParams, PlantState};

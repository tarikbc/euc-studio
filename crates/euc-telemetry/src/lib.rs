//! What the simulator streams to the studio, the CLI and recordings.
//!
//! - [`Channel`]: the schema of every scalar signal, with a stable numeric id, a unit and a
//!   rate class.
//! - [`Snapshot`]: the state at one instant: signal values ([`Signals`]) and the pose of every
//!   body ([`Pose`], described once by [`BodyInfo`]).
//! - [`Event`]: sparse things that happen (alarms, cutoffs, tiltback, commands, firmware
//!   milestones).
//! - [`batch`]: snapshots packed into one compact, columnar binary message for IPC.
//! - [`typescript`]: the same schema as TypeScript for the studio front end.
//!
//! This crate has no engine dependencies, so producers (`euc-sim`) and consumers (the studio
//! backend, recorders) agree on one definition.

pub mod batch;
mod model;
mod schema;
mod ts;

pub use model::{BodyInfo, BodyRole, Event, EventKind, Geom, GeomKind, Outputs, Pose, Signals, Snapshot};
pub use schema::{CHANNEL_COUNT, CHANNELS, Channel, ChannelSpec, RateClass};
pub use ts::{TS_PATH, typescript};

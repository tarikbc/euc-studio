//! The peripheral framework: how a memory-mapped block or an off-chip device talks to the engine.
//!
//! Every component of a board (an STM32 USART, an MPU-6500, a BMS pack) is a [`Peripheral`].
//! The engine calls it for bus accesses, scheduled events and incoming signals, always with a
//! [`Ctx`] that is the component's only way back into the machine: the clock, the scheduler,
//! interrupt lines, DMA requests, wires to other components, the plant and diagnostics.
//!
//! Two kinds of coupling exist, mirroring hardware:
//! - **Wires** (board level, data driven): [`Ctx::emit`] puts a [`Signal`] on one of the
//!   component's ports; the board wiring delivers it to [`Peripheral::input`] of whatever the
//!   port is connected to. Delivery happens after the current call returns, but before the CPU
//!   executes its next instruction, so a SPI device answers before the firmware reads `DR`.
//! - **Peers** (inside one chip): [`Ctx::peer`] borrows another component by id for a direct,
//!   synchronous call that needs no `Ctx` (the RTC asking RCC for its clock). Bus mastering
//!   (DMA) goes through [`Ctx::bus_read`] and [`Ctx::bus_write`], which reach memory and other
//!   peripherals with full side effects.

use std::any::Any;

use crate::{BusFault, Size};

/// Index of a component in the machine, fixed when the board is built.
pub type DeviceId = u16;

/// Why a component is being reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ResetKind {
    /// Core and peripherals restart (SYSRESETREQ, watchdog). SRAM, flash and the backup domain
    /// (RTC, BKP and the RCC backup control register) keep their contents.
    System,
    /// The battery was switched off and on: everything restarts, backup domain included.
    Power,
}

/// A value travelling along a wire between two components (`[[wire]]` in `board.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Signal {
    /// A driven digital level.
    Level(bool),
    /// The driver let go: the pin floats and its pull-up or pull-down decides.
    Released,
    /// One data unit on a serial or SPI line.
    Byte {
        /// The data (8 or 9 bits).
        value: u16,
        /// The cycle the unit is on the wire: when a transmitter started shifting it out, or the
        /// earliest cycle an injected receive may arrive.
        at: u64,
    },
    /// A one-shot trigger (a timer's TRGO, a master ADC starting its slave).
    Pulse,
    /// A PWM waveform from now on (a timer output compare channel). Sent when it changes.
    Pwm {
        /// Counter tick rate in Hz.
        tick_hz: u32,
        /// Period in ticks.
        period: u32,
        /// High time in ticks; 0 is a steady low output.
        high: u32,
    },
}

/// The read-only view of the physical world that sensors sample.
///
/// The engine owns the plant (motor, battery, body physics, or static values in M1) and keeps
/// this view current; components only read it, at the instant they sample.
pub trait PlantView {
    /// The 12-bit count ADC `adc` (1-based) converts on `channel` right now. The board wiring
    /// decides what each channel measures.
    fn adc_raw(&self, adc: u8, channel: u8) -> u16;

    /// What an IMU on the wheel measures right now.
    fn imu(&self) -> ImuSample;

    /// Battery pack voltage in volts (smart BMS models report it per cell).
    fn battery_v(&self) -> f64;
}

/// One IMU sample in the sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Specific force in g; `[0, 0, 1]` when the wheel stands level and still.
    pub accel_g: [f64; 3],
    /// Angular rate in degrees per second.
    pub gyro_dps: [f64; 3],
    /// Die temperature in °C.
    pub temp_c: f64,
}

/// A board component: a memory-mapped peripheral, an off-chip device, or both.
///
/// Offsets are relative to the component's base address in `board.toml`. Components that are not
/// on the bus (an IMU behind SPI) keep the default `read`/`write` and are driven by `input`.
pub trait Peripheral: Any {
    /// Returns to the reset state. The engine has already cancelled this component's pending
    /// events and dropped its interrupt lines; schedule again whatever must run after reset.
    /// On [`ResetKind::System`] backup-domain state stays.
    fn reset(&mut self, kind: ResetKind, ctx: &mut Ctx<'_>);

    /// Reads the aligned 32-bit register at `off` (a multiple of 4). `mask` marks the byte lanes
    /// the CPU reads (`0x0000_FF00` for a byte load at `off + 1`); the engine extracts them from
    /// the returned word. Clear-on-read side effects happen here. Registers a model does not
    /// know read back what was last written.
    fn read(&mut self, off: u32, mask: u32, ctx: &mut Ctx<'_>) -> u32 {
        let _ = (off, mask, ctx);
        0
    }

    /// Writes the aligned register at `off`: `value` is already shifted into its byte lanes and
    /// `mask` marks them, so `old & !mask | value & mask` is a plain store.
    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let _ = (off, value, mask, ctx);
    }

    /// An event scheduled with [`Ctx::schedule`] under `tag` is due. Events fire at quantum
    /// boundaries, so `ctx.now()` can be later than the scheduled cycle: catch up whole periods.
    fn on_event(&mut self, tag: u32, ctx: &mut Ctx<'_>) {
        let _ = (tag, ctx);
    }

    /// A signal arrived on input `port` (see [`Peripheral::port`]).
    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let _ = (port, signal, ctx);
    }

    /// The port number behind a wire end named in `board.toml` (`"tx"`, `"rx"`, `"15"`, `"cs"`),
    /// for both inputs and outputs. `None`: no such port, and the board fails to build.
    fn port(&self, name: &str) -> Option<u8> {
        let _ = name;
        None
    }

    /// DMA controllers only: moves one unit on `channel` (1-based) because a peripheral raised a
    /// request routed to it. Transfers go through [`Ctx::bus_read`] and [`Ctx::bus_write`].
    /// Returns `false` when the channel is disabled or has nothing left to move.
    fn dma_request(&mut self, channel: u8, ctx: &mut Ctx<'_>) -> bool {
        let _ = (channel, ctx);
        false
    }

    /// The register name at `off`, for traces and reports.
    fn reg_name(&self, off: u32) -> Option<&'static str> {
        let _ = off;
        None
    }
}

/// The engine side of [`Ctx`]. The simulator implements it; [`crate::testing::TestHost`] is a
/// recording implementation for unit tests of a single model.
///
/// Every method that acts for a component takes its id; [`Ctx`] fills it in.
pub trait Host {
    /// HCLK frequency in Hz.
    fn hclk(&self) -> u64;
    /// See [`Ctx::schedule`].
    fn schedule(&mut self, dev: DeviceId, at: u64, tag: u32);
    /// See [`Ctx::cancel`].
    fn cancel(&mut self, dev: DeviceId, tag: u32);
    /// See [`Ctx::set_irq`].
    fn set_irq(&mut self, dev: DeviceId, irq: u16, level: bool);
    /// See [`Ctx::pulse_irq`].
    fn pulse_irq(&mut self, irq: u16);
    /// See [`Ctx::dma_request`].
    fn dma_request(&mut self, dev: DeviceId, line: u8);
    /// See [`Ctx::emit`].
    fn emit(&mut self, dev: DeviceId, port: u8, signal: Signal);
    /// See [`Ctx::bus_read`].
    fn bus_read(&mut self, dev: DeviceId, addr: u32, size: Size) -> Result<u32, BusFault>;
    /// See [`Ctx::bus_write`].
    fn bus_write(&mut self, dev: DeviceId, addr: u32, size: Size, value: u32) -> Result<(), BusFault>;
    /// See [`Ctx::peer`].
    fn peer(&mut self, id: DeviceId) -> Option<&mut dyn Peripheral>;
    /// See [`Ctx::plant`].
    fn plant(&self) -> &dyn PlantView;
    /// See [`Ctx::log`].
    fn log(&mut self, dev: DeviceId, text: String);
    /// See [`Ctx::blocker`].
    fn blocker(&mut self, dev: DeviceId, kind: &'static str, text: String, fatal: bool);
    /// See [`Ctx::request_reset`].
    fn request_reset(&mut self, dev: DeviceId, kind: ResetKind, why: String);
}

/// A component's handle on the machine for the duration of one call.
pub struct Ctx<'a> {
    now: u64,
    dev: DeviceId,
    host: &'a mut dyn Host,
}

impl<'a> Ctx<'a> {
    /// A context for component `dev` at cycle `now`. Engines and test hosts build these.
    pub fn new(now: u64, dev: DeviceId, host: &'a mut dyn Host) -> Self {
        Self { now, dev, host }
    }

    /// The current cycle as this component sees it. Inside a CPU quantum this is the quantum
    /// start (plus any poll fast-forward), so in-quantum timing is accurate to one quantum.
    pub const fn now(&self) -> u64 {
        self.now
    }

    /// This component's id.
    pub const fn id(&self) -> DeviceId {
        self.dev
    }

    /// HCLK frequency in Hz.
    pub fn hclk(&self) -> u64 {
        self.host.hclk()
    }

    /// Calls [`Peripheral::on_event`] with `tag` at cycle `at`, replacing any pending event this
    /// component has under the same tag. One tag per independent timer keeps rescheduling simple.
    pub fn schedule(&mut self, at: u64, tag: u32) {
        self.host.schedule(self.dev, at, tag);
    }

    /// Cancels this component's pending event under `tag`, if any.
    pub fn cancel(&mut self, tag: u32) {
        self.host.cancel(self.dev, tag);
    }

    /// Drives this component's contribution to interrupt line `irq` (the NVIC IRQ number, 0 for
    /// WWDG). Lines are level sensitive and wired-OR across components (ADC1 and ADC2 share one):
    /// the line is high while any component holds it high. A high line pends its interrupt
    /// whenever that interrupt is not active, as on a Cortex-M3.
    pub fn set_irq(&mut self, irq: u16, level: bool) {
        self.host.set_irq(self.dev, irq, level);
    }

    /// Pends interrupt `irq` once, like a write to `NVIC_ISPR` or `STIR`.
    pub fn pulse_irq(&mut self, irq: u16) {
        self.host.pulse_irq(irq);
    }

    /// Raises this component's DMA request `line` (its own numbering, e.g. 0 = RX, 1 = TX). The
    /// board wiring routes it to a DMA controller channel, which then calls
    /// [`Peripheral::dma_request`]; that happens before the current CPU access completes.
    pub fn dma_request(&mut self, line: u8) {
        self.host.dma_request(self.dev, line);
    }

    /// Puts `signal` on output `port`; the board wiring delivers it to every connected input.
    pub fn emit(&mut self, port: u8, signal: Signal) {
        self.host.emit(self.dev, port, signal);
    }

    /// Reads memory or a peripheral as a bus master (DMA), with all side effects. The component
    /// itself is not reachable this way.
    pub fn bus_read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.host.bus_read(self.dev, addr, size)
    }

    /// Writes memory or a peripheral as a bus master (DMA), with all side effects.
    pub fn bus_write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.host.bus_write(self.dev, addr, size, value)
    }

    /// Borrows another component of a known type for a direct call (RTC reading the RCC clock
    /// configuration). The peer gets no `Ctx`, so call only methods that touch its own state.
    /// `None` if `id` is this component, unknown, or of another type.
    pub fn peer<T: Peripheral>(&mut self, id: DeviceId) -> Option<&mut T> {
        let peer: &mut dyn Any = self.host.peer(id)?;
        peer.downcast_mut()
    }

    /// The plant, as sensors see it now.
    pub fn plant(&self) -> &dyn PlantView {
        self.host.plant()
    }

    /// Records a timeline event for the run report ("FLASH unlocked").
    pub fn log(&mut self, text: impl Into<String>) {
        self.host.log(self.dev, text.into());
    }

    /// Reports something the model cannot represent or the firmware did wrong. A fatal blocker
    /// stops the run after the current instruction; others are counted and the run goes on.
    pub fn blocker(&mut self, kind: &'static str, text: impl Into<String>, fatal: bool) {
        self.host.blocker(self.dev, kind, text.into(), fatal);
    }

    /// Asks for a reset at the next quantum boundary (watchdog timeout, power switch).
    pub fn request_reset(&mut self, kind: ResetKind, why: impl Into<String>) {
        self.host.request_reset(self.dev, kind, why.into());
    }
}

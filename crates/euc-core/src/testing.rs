//! A recording [`Host`] for unit tests of a single model, without a machine around it.
//!
//! ```
//! use euc_core::testing::TestHost;
//! use euc_core::{Ctx, Peripheral, ResetKind, Signal};
//!
//! /// Echoes every written byte on port 0 and raises IRQ 37 while the last byte was odd.
//! struct Echo;
//!
//! impl Peripheral for Echo {
//!     fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {}
//!     fn write(&mut self, _off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
//!         let byte = (value & mask) as u16;
//!         ctx.emit(0, Signal::Byte { value: byte, at: ctx.now() });
//!         ctx.set_irq(37, byte & 1 == 1);
//!     }
//! }
//!
//! let mut host = TestHost::new(72_000_000);
//! host.now = 500;
//! Echo.write(0, 0x41, 0xFF, &mut host.ctx());
//! assert_eq!(host.emitted, [(0, Signal::Byte { value: 0x41, at: 500 })]);
//! assert!(host.irq(37));
//! ```

use std::collections::{BTreeMap, HashMap};

use crate::{BusFault, Ctx, DeviceId, Host, ImuSample, Lane, Peripheral, PlantView, ResetKind, Signal, Size};

/// Id the device under test gets in [`TestHost::ctx`].
pub const UNDER_TEST: DeviceId = 0;

/// Records everything the device under test asks of the machine.
pub struct TestHost {
    /// HCLK in Hz.
    pub hclk: u64,
    /// The cycle `ctx()` reports.
    pub now: u64,
    /// Sensor values.
    pub plant: TestPlant,
    /// Pending events by tag (cycle due).
    pub scheduled: BTreeMap<u32, u64>,
    /// Interrupt line levels driven by the device.
    pub irqs: BTreeMap<u16, bool>,
    /// Interrupts pulsed.
    pub pulses: Vec<u16>,
    /// DMA request lines raised, in order.
    pub dma_requests: Vec<u8>,
    /// Signals emitted, in order: `(port, signal)`.
    pub emitted: Vec<(u8, Signal)>,
    /// Memory behind `bus_read`/`bus_write` (DMA tests), byte addressed; unwritten bytes read 0.
    pub memory: HashMap<u32, u8>,
    /// Other components reachable through `Ctx::peer`.
    pub peers: BTreeMap<DeviceId, Box<dyn Peripheral>>,
    /// Log lines.
    pub log: Vec<String>,
    /// Blockers: `(kind, text, fatal)`.
    pub blockers: Vec<(&'static str, String, bool)>,
    /// Reset requests.
    pub resets: Vec<(ResetKind, String)>,
}

impl TestHost {
    /// A host at cycle 0 with a level, still plant at 25 °C and 130 V.
    pub fn new(hclk: u64) -> Self {
        Self {
            hclk,
            now: 0,
            plant: TestPlant::default(),
            scheduled: BTreeMap::new(),
            irqs: BTreeMap::new(),
            pulses: Vec::new(),
            dma_requests: Vec::new(),
            emitted: Vec::new(),
            memory: HashMap::new(),
            peers: BTreeMap::new(),
            log: Vec::new(),
            blockers: Vec::new(),
            resets: Vec::new(),
        }
    }

    /// A context for the device under test at [`TestHost::now`].
    pub fn ctx(&mut self) -> Ctx<'_> {
        Ctx::new(self.now, UNDER_TEST, self)
    }

    /// The level the device drives on `irq` (false if never driven).
    pub fn irq(&self, irq: u16) -> bool {
        self.irqs.get(&irq).copied().unwrap_or(false)
    }
}

impl Host for TestHost {
    fn hclk(&self) -> u64 {
        self.hclk
    }

    fn schedule(&mut self, _dev: DeviceId, at: u64, tag: u32) {
        self.scheduled.insert(tag, at);
    }

    fn cancel(&mut self, _dev: DeviceId, tag: u32) {
        self.scheduled.remove(&tag);
    }

    fn set_irq(&mut self, _dev: DeviceId, irq: u16, level: bool) {
        self.irqs.insert(irq, level);
    }

    fn pulse_irq(&mut self, irq: u16) {
        self.pulses.push(irq);
    }

    fn dma_request(&mut self, _dev: DeviceId, line: u8) {
        self.dma_requests.push(line);
    }

    fn emit(&mut self, _dev: DeviceId, port: u8, signal: Signal) {
        self.emitted.push((port, signal));
    }

    fn bus_read(&mut self, _dev: DeviceId, addr: u32, size: Size) -> Result<u32, BusFault> {
        let lane = Lane::new(addr, size);
        let word = (0..4).fold(0, |w, b| w | u32::from(*self.memory.get(&(lane.word + b)).unwrap_or(&0)) << (8 * b));
        Ok(lane.extract(word))
    }

    fn bus_write(&mut self, _dev: DeviceId, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        for b in 0..size.bytes() {
            self.memory.insert(addr + b, (value >> (8 * b)) as u8);
        }
        Ok(())
    }

    fn peer(&mut self, id: DeviceId) -> Option<&mut dyn Peripheral> {
        self.peers.get_mut(&id).map(|peer| peer.as_mut() as &mut dyn Peripheral)
    }

    fn plant(&self) -> &dyn PlantView {
        &self.plant
    }

    fn log(&mut self, _dev: DeviceId, text: String) {
        self.log.push(text);
    }

    fn blocker(&mut self, _dev: DeviceId, kind: &'static str, text: String, fatal: bool) {
        self.blockers.push((kind, text, fatal));
    }

    fn request_reset(&mut self, _dev: DeviceId, kind: ResetKind, why: String) {
        self.resets.push((kind, why));
    }
}

/// Settable sensor values. ADC channels without an entry read mid-scale (2048).
#[derive(Debug, Clone)]
pub struct TestPlant {
    /// Raw counts by `(adc, channel)`.
    pub adc: BTreeMap<(u8, u8), u16>,
    /// The IMU sample.
    pub imu: ImuSample,
    /// Pack voltage.
    pub battery_v: f64,
}

impl Default for TestPlant {
    fn default() -> Self {
        Self {
            adc: BTreeMap::new(),
            imu: ImuSample { accel_g: [0.0, 0.0, 1.0], gyro_dps: [0.0; 3], temp_c: 25.0 },
            battery_v: 130.0,
        }
    }
}

impl PlantView for TestPlant {
    fn adc_raw(&self, adc: u8, channel: u8) -> u16 {
        self.adc.get(&(adc, channel)).copied().unwrap_or(2048)
    }

    fn imu(&self) -> ImuSample {
        self.imu
    }

    fn battery_v(&self) -> f64 {
        self.battery_v
    }
}

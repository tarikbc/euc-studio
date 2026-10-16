//! MPU-6500 6-axis IMU on a 4-wire SPI bus.

use euc_core::{Build, ConfigError, Ctx, ImuSample, Peripheral, ResetKind, Signal};

use crate::NoParams;
use crate::fault::{DeviceFault, Schedule};
use crate::rng::Rng;

/// Register addresses the model gives meaning to (MPU-6500 register map, rev 2.1).
pub mod reg {
    /// Gyroscope configuration: `FS_SEL` in bits 3-4.
    pub const GYRO_CONFIG: u8 = 0x1B;
    /// Accelerometer configuration: `AFS_SEL` in bits 3-4.
    pub const ACCEL_CONFIG: u8 = 0x1C;
    /// First of the 14 sample bytes: accel X/Y/Z, temperature, gyro X/Y/Z, big-endian.
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    /// Last sample byte.
    pub const GYRO_ZOUT_L: u8 = 0x48;
    /// Power management 1: bit 7 `DEVICE_RESET`.
    pub const PWR_MGMT_1: u8 = 0x6B;
    /// Identity register.
    pub const WHO_AM_I: u8 = 0x75;
}

/// Value of `WHO_AM_I` on an MPU-6500.
pub const WHO_AM_I_VALUE: u8 = 0x70;

/// Temperature sensor gain (datasheet: 333.87 LSB/°C, 0 LSB at 21 °C).
const TEMP_LSB_PER_C: f64 = 333.87;
const TEMP_ZERO_C: f64 = 21.0;

/// A level, still sensor at 25 °C.
pub const LEVEL: ImuSample = ImuSample { accel_g: [0.0, 0.0, 1.0], gyro_dps: [0.0; 3], temp_c: 25.0 };

const PORT_SPI: u8 = 0;
const PORT_CS: u8 = 1;

/// Where the SPI transaction is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Chip select high: the device ignores the bus and leaves MISO idle.
    Deselected,
    /// The next byte is `R/W | address`.
    Address,
    /// Data bytes to or from consecutive registers.
    Data { addr: u8, read: bool },
}

/// Register model of an MPU-6500 on SPI (mode 3, MSB first).
///
/// A transaction starts when chip select goes low. The first byte is the register address with
/// bit 7 set for a read; every following byte reads or writes the next register (auto-increment).
/// The 14 sample bytes are latched from the [`ImuSample`] input when a read transaction addresses
/// them, so a burst read sees one coherent sample, as the real part's shadow registers guarantee.
///
/// Modeled: the register file with its reset values, `WHO_AM_I`, `DEVICE_RESET`, accelerometer
/// and gyroscope full-scale ranges, the temperature formula and saturation to `i16`. Not modeled:
/// FIFO, interrupts, sample-rate divider, digital low-pass filters, self-test, I2C.
#[derive(Debug, Clone)]
pub struct Mpu6500 {
    regs: [u8; 128],
    phase: Phase,
    input: ImuSample,
    sample: [u8; 14],
    writes: u64,
    sample_reads: u64,
    faults: Schedule,
}

impl Default for Mpu6500 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mpu6500 {
    /// A device at power-on, deselected, measuring [`LEVEL`].
    pub fn new() -> Self {
        let mut imu = Self {
            regs: [0; 128],
            phase: Phase::Deselected,
            input: LEVEL,
            sample: [0; 14],
            writes: 0,
            sample_reads: 0,
            faults: Schedule::default(),
        };
        imu.reset_registers();
        imu
    }

    /// Updates what the sensor measures. The next read transaction of the sample registers sees it.
    pub fn set_sensor(&mut self, input: ImuSample) {
        self.input = input;
    }

    /// The current sensor input.
    pub const fn sensor(&self) -> &ImuSample {
        &self.input
    }

    /// Adds an IMU fault (`imu_noise`, `imu_bias`, `imu_stuck`) for the cycles `from..until`
    /// (`None`: to the end of the run). As a board component the device applies it to every
    /// sample it takes from the plant inside the window.
    pub fn add_fault(&mut self, from: u64, until: Option<u64>, fault: DeviceFault) -> Result<(), String> {
        self.faults.add("mpu6500", from, until, fault)
    }

    /// What the sensor measures at `now`, given the plant's `truth` and the active faults; `None`
    /// while a stuck fault freezes the sample.
    fn measure(&self, truth: ImuSample, now: u64) -> Option<ImuSample> {
        let mut sample = truth;
        let mut rng = Rng::new(now ^ 0x494D_5530);
        for fault in self.faults.active(now) {
            match fault {
                DeviceFault::ImuStuck => return None,
                DeviceFault::ImuBias { accel_g, gyro_dps } => {
                    for axis in 0..3 {
                        sample.accel_g[axis] += accel_g[axis];
                        sample.gyro_dps[axis] += gyro_dps[axis];
                    }
                }
                DeviceFault::ImuNoise { accel_g, gyro_dps } => {
                    for axis in 0..3 {
                        sample.accel_g[axis] += rng.gaussian(accel_g);
                        sample.gyro_dps[axis] += rng.gaussian(gyro_dps);
                    }
                }
                DeviceFault::BmsSilent | DeviceFault::BmsGarbled { .. } => {}
            }
        }
        Some(sample)
    }

    /// Drives the chip-select line (`true` = high = deselected). A falling edge starts a transaction.
    pub fn chip_select(&mut self, high: bool) {
        self.phase = match (high, self.phase) {
            (true, _) => Phase::Deselected,
            (false, Phase::Deselected) => Phase::Address,
            (false, phase) => phase,
        };
    }

    /// One full-duplex byte: the master sends `mosi` and receives the returned byte.
    ///
    /// Returns `0xFF` (MISO pulled up, nobody driving it) while deselected.
    pub fn transfer(&mut self, mosi: u8) -> u8 {
        match self.phase {
            Phase::Deselected => 0xFF,
            Phase::Address => {
                let addr = mosi & 0x7F;
                let read = mosi & 0x80 != 0;
                if read && (reg::ACCEL_XOUT_H..=reg::GYRO_ZOUT_L).contains(&addr) {
                    self.sample = self.sample_bytes();
                }
                self.phase = Phase::Data { addr, read };
                0x00
            }
            Phase::Data { addr, read } => {
                self.phase = Phase::Data { addr: (addr + 1) & 0x7F, read };
                if read {
                    self.read(addr)
                } else {
                    self.write(addr, mosi);
                    0x00
                }
            }
        }
    }

    /// A register as the firmware would read it (sample registers from the last latched sample).
    pub fn register(&self, addr: u8) -> u8 {
        match addr {
            reg::ACCEL_XOUT_H..=reg::GYRO_ZOUT_L => self.sample[usize::from(addr - reg::ACCEL_XOUT_H)],
            _ => self.regs[usize::from(addr & 0x7F)],
        }
    }

    /// Register writes received since power-on.
    pub const fn writes(&self) -> u64 {
        self.writes
    }

    /// Sample bytes read since power-on.
    pub const fn sample_reads(&self) -> u64 {
        self.sample_reads
    }

    /// Accelerometer sensitivity for the configured range: 16384 LSB/g at ±2 g, halving per step.
    pub fn accel_lsb_per_g(&self) -> f64 {
        f64::from(16_384_u16 >> ((self.regs[usize::from(reg::ACCEL_CONFIG)] >> 3) & 3))
    }

    /// Gyroscope sensitivity for the configured range: 131 LSB/(°/s) at ±250 °/s, halving per step.
    pub fn gyro_lsb_per_dps(&self) -> f64 {
        131.0 / f64::from(1_u8 << ((self.regs[usize::from(reg::GYRO_CONFIG)] >> 3) & 3))
    }

    fn read(&mut self, addr: u8) -> u8 {
        if (reg::ACCEL_XOUT_H..=reg::GYRO_ZOUT_L).contains(&addr) {
            self.sample_reads += 1;
        }
        self.register(addr)
    }

    fn write(&mut self, addr: u8, value: u8) {
        self.writes += 1;
        if addr == reg::PWR_MGMT_1 && value & 0x80 != 0 {
            self.reset_registers();
        } else if addr != reg::WHO_AM_I {
            self.regs[usize::from(addr)] = value;
        }
    }

    fn reset_registers(&mut self) {
        self.regs = [0; 128];
        self.regs[usize::from(reg::PWR_MGMT_1)] = 0x01;
        self.regs[usize::from(reg::WHO_AM_I)] = WHO_AM_I_VALUE;
    }

    fn sample_bytes(&self) -> [u8; 14] {
        let accel = self.accel_lsb_per_g();
        let gyro = self.gyro_lsb_per_dps();
        let input = &self.input;
        let values = [
            input.accel_g[0] * accel,
            input.accel_g[1] * accel,
            input.accel_g[2] * accel,
            (input.temp_c - TEMP_ZERO_C) * TEMP_LSB_PER_C,
            input.gyro_dps[0] * gyro,
            input.gyro_dps[1] * gyro,
            input.gyro_dps[2] * gyro,
        ];
        let mut bytes = [0; 14];
        let (pairs, _) = bytes.as_chunks_mut::<2>();
        for (pair, value) in pairs.iter_mut().zip(values) {
            // Ties to even, as the reference harness rounds.
            *pair = (value.round_ties_even().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16).to_be_bytes();
        }
        bytes
    }
}

impl Mpu6500 {
    /// Builds the `mpu6500` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        build.params::<NoParams>()?;
        Ok(Box::new(Self::new()))
    }
}

/// As a board component (kind `mpu6500`): `"cs"` takes the chip-select level (released counts as
/// high: the line has a pull-up), `"spi"` takes the master's bytes and answers each with a byte on
/// `"spi"`. The sensor input is the plant's IMU sample at the start of each transaction.
impl Peripheral for Mpu6500 {
    fn reset(&mut self, kind: ResetKind, _ctx: &mut Ctx<'_>) {
        // The IMU is powered by the board: only a power cycle resets it. Its faults stay.
        if kind == ResetKind::Power {
            let faults = std::mem::take(&mut self.faults);
            *self = Self { faults, ..Self::new() };
        }
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        match (port, signal) {
            (PORT_CS, Signal::Level(high)) => {
                if !high {
                    let truth = ctx.plant().imu();
                    if let Some(sample) = self.measure(truth, ctx.now()) {
                        self.input = sample;
                    }
                }
                self.chip_select(high);
            }
            (PORT_CS, Signal::Released) => self.chip_select(true),
            (PORT_SPI, Signal::Byte { value, at }) => {
                let miso = self.transfer(value as u8);
                ctx.emit(PORT_SPI, Signal::Byte { value: u16::from(miso), at });
            }
            _ => {}
        }
    }

    fn port(&self, name: &str) -> Option<u8> {
        match name {
            "spi" => Some(PORT_SPI),
            "cs" => Some(PORT_CS),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One CS-low transaction: address byte, then `data`; returns what MISO carried.
    fn transaction(imu: &mut Mpu6500, addr: u8, data: &[u8]) -> Vec<u8> {
        imu.chip_select(false);
        let mut miso = vec![imu.transfer(addr)];
        miso.extend(data.iter().map(|&b| imu.transfer(b)));
        imu.chip_select(true);
        miso
    }

    fn read16(bytes: &[u8], k: usize) -> i16 {
        i16::from_be_bytes([bytes[2 * k], bytes[2 * k + 1]])
    }

    #[test]
    fn identifies_itself() {
        let mut imu = Mpu6500::new();
        assert_eq!(transaction(&mut imu, 0x80 | reg::WHO_AM_I, &[0]), [0x00, WHO_AM_I_VALUE]);
        assert_eq!(imu.register(reg::PWR_MGMT_1), 0x01);
    }

    #[test]
    fn ignores_the_bus_while_deselected() {
        let mut imu = Mpu6500::new();
        assert_eq!(imu.transfer(0x80 | reg::WHO_AM_I), 0xFF);
        assert_eq!(imu.transfer(0), 0xFF);
        assert_eq!(imu.writes(), 0);
    }

    #[test]
    fn writes_auto_increment_and_device_reset_restores_defaults() {
        let mut imu = Mpu6500::new();
        transaction(&mut imu, reg::GYRO_CONFIG, &[0x08, 0x08, 0x00]);
        assert_eq!((imu.register(reg::GYRO_CONFIG), imu.register(reg::ACCEL_CONFIG)), (0x08, 0x08));
        assert_eq!(imu.writes(), 3);
        transaction(&mut imu, reg::WHO_AM_I, &[0x12]);
        assert_eq!(imu.register(reg::WHO_AM_I), WHO_AM_I_VALUE, "read-only");
        transaction(&mut imu, reg::PWR_MGMT_1, &[0x80]);
        assert_eq!(imu.register(reg::GYRO_CONFIG), 0);
        assert_eq!(imu.register(reg::PWR_MGMT_1), 0x01);
    }

    #[test]
    fn burst_read_scales_with_the_configured_ranges() {
        // The Begode Extreme set-up: gyro ±500 °/s, accel ±4 g.
        let mut imu = Mpu6500::new();
        transaction(&mut imu, reg::GYRO_CONFIG, &[0x08, 0x08]);
        imu.set_sensor(ImuSample { accel_g: [0.5, -0.25, 1.0], gyro_dps: [10.0, -100.0, 0.5], temp_c: 31.0 });
        let miso = transaction(&mut imu, 0x80 | reg::ACCEL_XOUT_H, &[0; 14]);
        let sample = &miso[1..];
        assert_eq!([read16(sample, 0), read16(sample, 1), read16(sample, 2)], [4096, -2048, 8192]);
        assert_eq!(read16(sample, 3), 3339, "(31 - 21) * 333.87");
        assert_eq!([read16(sample, 4), read16(sample, 5), read16(sample, 6)], [655, -6550, 33]);
        assert_eq!(imu.sample_reads(), 14);
    }

    #[test]
    fn a_burst_sees_one_coherent_sample() {
        let mut imu = Mpu6500::new();
        imu.chip_select(false);
        imu.transfer(0x80 | reg::ACCEL_XOUT_H);
        let high = imu.transfer(0);
        imu.set_sensor(ImuSample { accel_g: [1.0, 0.0, 0.0], ..LEVEL });
        let low = imu.transfer(0);
        imu.chip_select(true);
        assert_eq!((high, low), (0, 0), "the input change waits for the next transaction");
    }

    #[test]
    fn saturates_out_of_range_inputs() {
        let mut imu = Mpu6500::new();
        imu.set_sensor(ImuSample { accel_g: [3.0, -3.0, 0.0], gyro_dps: [1000.0, 0.0, 0.0], temp_c: 21.0 });
        let miso = transaction(&mut imu, 0x80 | reg::ACCEL_XOUT_H, &[0; 14]);
        let sample = &miso[1..];
        assert_eq!((read16(sample, 0), read16(sample, 1)), (i16::MAX, i16::MIN));
        assert_eq!(read16(sample, 4), i16::MAX);
    }

    #[test]
    fn faults_bias_add_noise_and_freeze_samples() {
        use euc_core::testing::TestHost;

        let mut host = TestHost::new(72_000_000);
        let mut imu = Mpu6500::new();
        let cs = imu.port("cs").unwrap();
        let bias = DeviceFault::ImuBias { accel_g: [0.1, 0.0, 0.0], gyro_dps: [0.0, 0.0, -2.0] };
        imu.add_fault(100, Some(200), bias).unwrap();
        imu.add_fault(300, None, DeviceFault::ImuStuck).unwrap();
        imu.add_fault(500, None, DeviceFault::ImuNoise { accel_g: 0.0, gyro_dps: 1.0 }).unwrap();
        assert!(imu.add_fault(0, None, DeviceFault::BmsSilent).is_err());
        // One chip-select cycle at `now` with the plant's accelerometer x at `accel_x`.
        let sample_at = |imu: &mut Mpu6500, host: &mut TestHost, now: u64, accel_x: f64| {
            host.now = now;
            host.plant.imu = ImuSample { accel_g: [accel_x, 0.0, 1.0], ..LEVEL };
            imu.input(cs, Signal::Level(false), &mut host.ctx());
            imu.input(cs, Signal::Level(true), &mut host.ctx());
            *imu.sensor()
        };
        assert_eq!(sample_at(&mut imu, &mut host, 50, 0.0).accel_g[0], 0.0);
        let biased = sample_at(&mut imu, &mut host, 150, 0.2);
        assert!((biased.accel_g[0] - 0.3).abs() < 1e-12);
        assert_eq!(biased.gyro_dps, [0.0, 0.0, -2.0]);
        assert_eq!(sample_at(&mut imu, &mut host, 250, 0.4).accel_g[0], 0.4, "the bias window is over");
        assert_eq!(sample_at(&mut imu, &mut host, 350, 0.9).accel_g[0], 0.4, "stuck: the last sample stays");
        imu.reset(ResetKind::Power, &mut host.ctx());
        let after_reset = sample_at(&mut imu, &mut host, 360, 0.9);
        assert_eq!(after_reset.accel_g[0], LEVEL.accel_g[0], "still stuck after a power cycle");
    }

    #[test]
    fn noise_is_deterministic() {
        let mut imu = Mpu6500::new();
        imu.add_fault(0, None, DeviceFault::ImuNoise { accel_g: 0.01, gyro_dps: 0.5 }).unwrap();
        let a = imu.measure(LEVEL, 1234).unwrap();
        assert_eq!(a, imu.measure(LEVEL, 1234).unwrap());
        assert_ne!(a, imu.measure(LEVEL, 1235).unwrap());
        assert!(a.gyro_dps.iter().all(|g| g.abs() < 5.0) && a.gyro_dps.iter().any(|g| *g != 0.0));
    }

    #[test]
    fn as_a_component_it_answers_on_its_spi_port() {
        use euc_core::testing::TestHost;

        let mut host = TestHost::new(72_000_000);
        host.plant.imu = ImuSample { accel_g: [0.0, 0.5, 1.0], ..LEVEL };
        let mut imu = Mpu6500::new();
        let (spi, cs) = (imu.port("spi").unwrap(), imu.port("cs").unwrap());
        imu.input(cs, Signal::Level(false), &mut host.ctx());
        for byte in [0x80 | reg::ACCEL_XOUT_H, 0, 0, 0, 0] {
            imu.input(spi, Signal::Byte { value: u16::from(byte), at: 9 }, &mut host.ctx());
        }
        imu.input(cs, Signal::Released, &mut host.ctx());
        let miso: Vec<u16> =
            host.emitted.iter().map(|(_, s)| if let Signal::Byte { value, .. } = s { *value } else { 0 }).collect();
        assert_eq!(miso, [0x00, 0x00, 0x00, 0x20, 0x00], "AY = 0.5 g = 0x2000 at +-2 g");
        assert!(host.emitted.iter().all(|&(port, _)| port == spi));
        imu.input(spi, Signal::Byte { value: 0x75 | 0x80, at: 10 }, &mut host.ctx());
        assert_eq!(host.emitted.last(), Some(&(spi, Signal::Byte { value: 0xFF, at: 10 })), "deselected");
    }
}

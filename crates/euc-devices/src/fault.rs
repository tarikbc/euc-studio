//! Faults to inject into the board's devices: a noisy, biased or frozen IMU, a silent or garbled
//! smart BMS.
//!
//! A scenario lists faults on its timeline, one `[[timeline]]` table each with `fault = "<kind>"`
//! and the kind's fields (`docs/models/electrical.md` has the list). The simulator hands each one
//! to the device that [`DeviceFault::device_kind`] names, with its window in engine cycles
//! ([`Mpu6500::add_fault`](crate::Mpu6500::add_fault),
//! [`SmartBms::add_fault`](crate::SmartBms::add_fault)); the device applies it whenever it
//! samples or answers inside the window.

use serde::Deserialize;

use crate::bms::REPLY_LEN;

/// A fault of a board device, as a scenario's `[[timeline]]` entry writes it
/// (`fault = "imu_noise"`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(tag = "fault", rename_all = "snake_case", deny_unknown_fields)]
pub enum DeviceFault {
    /// Gaussian noise on every IMU sample, per axis.
    ImuNoise {
        /// Standard deviation of each accelerometer axis, g.
        #[serde(default)]
        accel_g: f64,
        /// Standard deviation of each gyroscope axis, °/s.
        #[serde(default)]
        gyro_dps: f64,
    },
    /// Constant offsets on the IMU's axes (a knocked or drifting sensor).
    ImuBias {
        /// Accelerometer offsets x, y, z, g.
        #[serde(default)]
        accel_g: [f64; 3],
        /// Gyroscope offsets x, y, z, °/s.
        #[serde(default)]
        gyro_dps: [f64; 3],
    },
    /// The IMU's sample registers stop updating: every read returns the last sample before the
    /// fault. The bus and `WHO_AM_I` keep working.
    ImuStuck,
    /// The smart BMS never answers the controller's poll.
    BmsSilent,
    /// The smart BMS answers with damaged bytes: each byte is replaced by a random one with
    /// probability `byte_error_rate`, and the last `missing_bytes` bytes never arrive.
    BmsGarbled {
        /// Chance that a byte is wrong, 0..=1.
        #[serde(default)]
        byte_error_rate: f64,
        /// Bytes cut from the end of the reply (of 176).
        #[serde(default)]
        missing_bytes: u16,
    },
}

impl DeviceFault {
    /// Every `fault` name of this enum, for a timeline parser that dispatches on it.
    pub const KINDS: &[&str] = &["imu_noise", "imu_bias", "imu_stuck", "bms_silent", "bms_garbled"];

    /// The `fault` name of this fault.
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::ImuNoise { .. } => "imu_noise",
            Self::ImuBias { .. } => "imu_bias",
            Self::ImuStuck => "imu_stuck",
            Self::BmsSilent => "bms_silent",
            Self::BmsGarbled { .. } => "bms_garbled",
        }
    }

    /// The `board.toml` kind of the device this fault goes to: `mpu6500` or `smart-bms`.
    pub const fn device_kind(&self) -> &'static str {
        match self {
            Self::ImuNoise { .. } | Self::ImuBias { .. } | Self::ImuStuck => "mpu6500",
            Self::BmsSilent | Self::BmsGarbled { .. } => "smart-bms",
        }
    }

    /// Checks the fault's values.
    pub fn validate(&self) -> Result<(), String> {
        let ok = match *self {
            Self::ImuNoise { accel_g, gyro_dps } => [accel_g, gyro_dps].iter().all(|v| *v >= 0.0 && v.is_finite()),
            Self::ImuBias { accel_g, gyro_dps } => accel_g.iter().chain(&gyro_dps).all(|v| v.is_finite()),
            Self::BmsGarbled { byte_error_rate, missing_bytes } => {
                (0.0..=1.0).contains(&byte_error_rate) && usize::from(missing_bytes) <= REPLY_LEN
            }
            Self::ImuStuck | Self::BmsSilent => true,
        };
        if ok { Ok(()) } else { Err(format!("fault {}: a value is out of range", self.kind())) }
    }
}

/// A device's faults with their windows.
#[derive(Debug, Clone, Default)]
pub(crate) struct Schedule {
    list: Vec<(u64, Option<u64>, DeviceFault)>,
}

impl Schedule {
    /// Adds `fault` for `from..until` if it goes to a device of kind `device_kind`.
    pub(crate) fn add(
        &mut self,
        device_kind: &str,
        from: u64,
        until: Option<u64>,
        fault: DeviceFault,
    ) -> Result<(), String> {
        fault.validate()?;
        if fault.device_kind() != device_kind {
            return Err(format!("fault {} goes to a {}, not a {device_kind}", fault.kind(), fault.device_kind()));
        }
        if until.is_some_and(|until| until <= from) {
            return Err(format!("fault {}: its window ends before it starts", fault.kind()));
        }
        self.list.push((from, until, fault));
        Ok(())
    }

    /// The faults active at `now`.
    pub(crate) fn active(&self, now: u64) -> impl Iterator<Item = DeviceFault> + '_ {
        self.list
            .iter()
            .filter(move |&&(from, until, _)| from <= now && until.is_none_or(|until| now < until))
            .map(|&(_, _, fault)| fault)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A timeline entry as a scenario parser would hold it.
    #[derive(Debug, Deserialize)]
    #[allow(dead_code)]
    struct Entry {
        at_s: f64,
        for_s: Option<f64>,
        device: Option<String>,
        #[serde(flatten)]
        fault: DeviceFault,
    }

    #[test]
    fn timeline_entries_parse() {
        let entry: Entry = toml::from_str("at_s = 3.0\nfault = \"imu_bias\"\ngyro_dps = [0.0, 2.0, 0.0]").unwrap();
        assert_eq!(entry.fault, DeviceFault::ImuBias { accel_g: [0.0; 3], gyro_dps: [0.0, 2.0, 0.0] });
        let entry: Entry = toml::from_str("at_s = 3.0\nfor_s = 1.0\nfault = \"bms_silent\"\ndevice = \"bms\"").unwrap();
        assert_eq!((entry.fault, entry.device.as_deref()), (DeviceFault::BmsSilent, Some("bms")));
        assert!(toml::from_str::<Entry>("at_s = 3.0\nfault = \"imu_noise\"\nnoise = 1.0").is_err());
    }

    #[test]
    fn kinds_list_every_variant() {
        let samples = [
            "fault = \"imu_noise\"\naccel_g = 0.01",
            "fault = \"imu_bias\"",
            "fault = \"imu_stuck\"",
            "fault = \"bms_silent\"",
            "fault = \"bms_garbled\"\nbyte_error_rate = 0.1",
        ];
        let kinds: Vec<&str> = samples.iter().map(|s| toml::from_str::<DeviceFault>(s).unwrap().kind()).collect();
        assert_eq!(kinds, DeviceFault::KINDS);
    }

    #[test]
    fn schedules_check_the_target_and_the_window() {
        let mut schedule = Schedule::default();
        let err = schedule.add("mpu6500", 0, None, DeviceFault::BmsSilent).unwrap_err();
        assert_eq!(err, "fault bms_silent goes to a smart-bms, not a mpu6500");
        assert!(schedule.add("mpu6500", 10, Some(10), DeviceFault::ImuStuck).is_err());
        let bad = DeviceFault::BmsGarbled { byte_error_rate: 2.0, missing_bytes: 0 };
        assert_eq!(bad.validate().unwrap_err(), "fault bms_garbled: a value is out of range");
        schedule.add("mpu6500", 10, Some(20), DeviceFault::ImuStuck).unwrap();
        assert_eq!(schedule.active(9).count(), 0);
        assert_eq!(schedule.active(10).count(), 1);
        assert_eq!(schedule.active(20).count(), 0);
    }
}

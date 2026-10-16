//! The channel schema: every scalar the simulator streams.
//!
//! A channel's id is its index in [`CHANNELS`] and in a batch. Ids are append-only: new
//! channels go at the end, and an id is never reused, so recordings stay readable.

use serde::Serialize;

/// How often a channel changes enough to be worth sending.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RateClass {
    /// Every snapshot (control-loop signals).
    Fast,
    /// Slowly changing (temperatures, charge); consumers may decimate it.
    Slow,
}

/// One scalar signal. The discriminant is the stable id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
#[repr(u16)]
pub enum Channel {
    /// Ground speed of the wheel, signed (forward positive).
    SpeedKmh = 0,
    /// Wheel angular velocity relative to the shell.
    WheelRadS = 1,
    /// Inverter duty cycle the firmware commands, 0..100 %.
    PwmPct = 2,
    /// Phase A current.
    PhaseAA = 3,
    /// Phase B current.
    PhaseBA = 4,
    /// Phase C current.
    PhaseCA = 5,
    /// Torque-producing (q-axis) current.
    IqA = 6,
    /// Motor torque on the wheel.
    TorqueNm = 7,
    /// Battery terminal voltage.
    BatteryV = 8,
    /// Battery current (discharge positive).
    BatteryA = 9,
    /// State of charge.
    SocPct = 10,
    /// Shell pitch, nose up positive.
    PitchDeg = 11,
    /// Shell roll, right side down positive.
    RollDeg = 12,
    /// Controller board (MOSFET) temperature.
    BoardTempC = 13,
    /// Motor winding temperature.
    MotorTempC = 14,
    /// Distance travelled since the run started.
    DistanceM = 15,
    /// Share of CPU cycles spent in interrupt handlers.
    IsrLoadPct = 16,
    /// Rider lean from vertical, forward positive.
    RiderLeanDeg = 17,
    /// Speed the rider (or scenario) is aiming for.
    TargetSpeedKmh = 18,
}

/// What [`CHANNELS`] knows about a channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct ChannelSpec {
    /// Stable id, equal to the index in [`CHANNELS`].
    pub id: u16,
    /// The channel.
    pub channel: Channel,
    /// `snake_case` key, as in JSON and TypeScript.
    pub key: &'static str,
    /// Label for people.
    pub label: &'static str,
    /// Unit symbol (empty for none).
    pub unit: &'static str,
    /// Update rate class.
    pub rate: RateClass,
}

const fn spec(
    channel: Channel,
    key: &'static str,
    label: &'static str,
    unit: &'static str,
    rate: RateClass,
) -> ChannelSpec {
    ChannelSpec { id: channel as u16, channel, key, label, unit, rate }
}

/// Every channel, in id order.
pub const CHANNELS: [ChannelSpec; 19] = [
    spec(Channel::SpeedKmh, "speed_kmh", "Speed", "km/h", RateClass::Fast),
    spec(Channel::WheelRadS, "wheel_rad_s", "Wheel speed", "rad/s", RateClass::Fast),
    spec(Channel::PwmPct, "pwm_pct", "PWM", "%", RateClass::Fast),
    spec(Channel::PhaseAA, "phase_a_a", "Phase A current", "A", RateClass::Fast),
    spec(Channel::PhaseBA, "phase_b_a", "Phase B current", "A", RateClass::Fast),
    spec(Channel::PhaseCA, "phase_c_a", "Phase C current", "A", RateClass::Fast),
    spec(Channel::IqA, "iq_a", "Torque current", "A", RateClass::Fast),
    spec(Channel::TorqueNm, "torque_nm", "Motor torque", "N·m", RateClass::Fast),
    spec(Channel::BatteryV, "battery_v", "Battery", "V", RateClass::Fast),
    spec(Channel::BatteryA, "battery_a", "Battery current", "A", RateClass::Fast),
    spec(Channel::SocPct, "soc_pct", "Charge", "%", RateClass::Slow),
    spec(Channel::PitchDeg, "pitch_deg", "Pitch", "°", RateClass::Fast),
    spec(Channel::RollDeg, "roll_deg", "Roll", "°", RateClass::Fast),
    spec(Channel::BoardTempC, "board_temp_c", "Board temp", "°C", RateClass::Slow),
    spec(Channel::MotorTempC, "motor_temp_c", "Motor temp", "°C", RateClass::Slow),
    spec(Channel::DistanceM, "distance_m", "Distance", "m", RateClass::Fast),
    spec(Channel::IsrLoadPct, "isr_load_pct", "ISR load", "%", RateClass::Slow),
    spec(Channel::RiderLeanDeg, "rider_lean_deg", "Rider lean", "°", RateClass::Fast),
    spec(Channel::TargetSpeedKmh, "target_speed_kmh", "Target speed", "km/h", RateClass::Fast),
];

/// Number of channels.
pub const CHANNEL_COUNT: usize = CHANNELS.len();

impl Channel {
    /// The channel's entry in [`CHANNELS`].
    pub const fn spec(self) -> &'static ChannelSpec {
        &CHANNELS[self as usize]
    }

    /// The channel with this id, if there is one.
    pub fn from_id(id: u16) -> Option<Self> {
        CHANNELS.get(usize::from(id)).map(|s| s.channel)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_dense_and_in_order() {
        for (i, spec) in CHANNELS.iter().enumerate() {
            assert_eq!(usize::from(spec.id), i, "{}", spec.key);
            assert_eq!(spec.channel as usize, i, "{}", spec.key);
            assert_eq!(Channel::from_id(spec.id), Some(spec.channel));
        }
        assert_eq!(Channel::from_id(CHANNEL_COUNT as u16), None);
    }

    #[test]
    fn keys_are_unique_snake_case() {
        let mut keys: Vec<_> = CHANNELS.iter().map(|s| s.key).collect();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(keys.len(), CHANNEL_COUNT);
        assert!(
            CHANNELS.iter().all(|s| s.key.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'))
        );
    }
}

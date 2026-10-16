//! Snapshots, bodies and events.

use serde::{Deserialize, Serialize};

use crate::schema::Channel;

/// Position and orientation of a body in the world frame of the physics model: z up, metres,
/// unit quaternion `[w, x, y, z]`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    /// Position in metres.
    pub pos: [f32; 3],
    /// Orientation, `[w, x, y, z]`.
    pub quat: [f32; 4],
}

impl Pose {
    /// At the origin, not rotated.
    pub const IDENTITY: Self = Self { pos: [0.0; 3], quat: [1.0, 0.0, 0.0, 0.0] };

    /// The seven floats a batch stores: position, then quaternion.
    pub fn to_array(self) -> [f32; 7] {
        let [x, y, z] = self.pos;
        let [w, qx, qy, qz] = self.quat;
        [x, y, z, w, qx, qy, qz]
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// What a body is, so a viewer can draw it in its own style.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyRole {
    /// The wheel: tire, rim and hub motor. Its frame's origin is the axle centre.
    Wheel,
    /// The shell with pedals and electronics. Its frame's origin is the axle centre.
    Shell,
    /// Part of the rider.
    Rider,
    /// Anything else (terrain props, debug bodies).
    Other,
}

/// Primitive geometry of a body, as in MJCF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeomKind {
    /// `size[0]` radius, `size[1]` half length along local z.
    Capsule,
    /// `size` half extents.
    Box,
    /// `size[0]` radius.
    Sphere,
    /// `size[0]` radius, `size[1]` half length along local z.
    Cylinder,
    /// `size` semi-axes.
    Ellipsoid,
}

/// One geom of a body, in the body frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Geom {
    /// Shape.
    pub kind: GeomKind,
    /// MJCF size parameters (meaning depends on `kind`); unused entries are zero.
    pub size: [f32; 3],
    /// Offset from the body frame, metres.
    pub pos: [f32; 3],
    /// Rotation relative to the body frame, `[w, x, y, z]`.
    pub quat: [f32; 4],
}

/// A body the simulator moves. Sent once per run; snapshots carry its [`Pose`] in the same
/// order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyInfo {
    /// Name in the model (`wheel`, `shell`, `pelvis`, ...).
    pub name: String,
    /// What it is.
    pub role: BodyRole,
    /// Its shapes, for viewers that draw the model as is.
    pub geoms: Vec<Geom>,
}

/// Every scalar signal at one instant. `NaN` means the value is not modelled (yet).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[allow(missing_docs)] // Each field is the channel of the same name; see [`Channel`].
pub struct Signals {
    pub speed_kmh: f32,
    pub wheel_rad_s: f32,
    pub pwm_pct: f32,
    pub phase_a_a: f32,
    pub phase_b_a: f32,
    pub phase_c_a: f32,
    pub iq_a: f32,
    pub torque_nm: f32,
    pub battery_v: f32,
    pub battery_a: f32,
    pub soc_pct: f32,
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub board_temp_c: f32,
    pub motor_temp_c: f32,
    pub distance_m: f32,
    pub isr_load_pct: f32,
    pub rider_lean_deg: f32,
    pub target_speed_kmh: f32,
}

impl Default for Signals {
    /// Everything not modelled.
    fn default() -> Self {
        Self {
            speed_kmh: f32::NAN,
            wheel_rad_s: f32::NAN,
            pwm_pct: f32::NAN,
            phase_a_a: f32::NAN,
            phase_b_a: f32::NAN,
            phase_c_a: f32::NAN,
            iq_a: f32::NAN,
            torque_nm: f32::NAN,
            battery_v: f32::NAN,
            battery_a: f32::NAN,
            soc_pct: f32::NAN,
            pitch_deg: f32::NAN,
            roll_deg: f32::NAN,
            board_temp_c: f32::NAN,
            motor_temp_c: f32::NAN,
            distance_m: f32::NAN,
            isr_load_pct: f32::NAN,
            rider_lean_deg: f32::NAN,
            target_speed_kmh: f32::NAN,
        }
    }
}

impl Signals {
    /// The value of `channel`.
    pub const fn get(&self, channel: Channel) -> f32 {
        match channel {
            Channel::SpeedKmh => self.speed_kmh,
            Channel::WheelRadS => self.wheel_rad_s,
            Channel::PwmPct => self.pwm_pct,
            Channel::PhaseAA => self.phase_a_a,
            Channel::PhaseBA => self.phase_b_a,
            Channel::PhaseCA => self.phase_c_a,
            Channel::IqA => self.iq_a,
            Channel::TorqueNm => self.torque_nm,
            Channel::BatteryV => self.battery_v,
            Channel::BatteryA => self.battery_a,
            Channel::SocPct => self.soc_pct,
            Channel::PitchDeg => self.pitch_deg,
            Channel::RollDeg => self.roll_deg,
            Channel::BoardTempC => self.board_temp_c,
            Channel::MotorTempC => self.motor_temp_c,
            Channel::DistanceM => self.distance_m,
            Channel::IsrLoadPct => self.isr_load_pct,
            Channel::RiderLeanDeg => self.rider_lean_deg,
            Channel::TargetSpeedKmh => self.target_speed_kmh,
        }
    }

    /// Sets the value of `channel`.
    pub const fn set(&mut self, channel: Channel, value: f32) {
        let slot = match channel {
            Channel::SpeedKmh => &mut self.speed_kmh,
            Channel::WheelRadS => &mut self.wheel_rad_s,
            Channel::PwmPct => &mut self.pwm_pct,
            Channel::PhaseAA => &mut self.phase_a_a,
            Channel::PhaseBA => &mut self.phase_b_a,
            Channel::PhaseCA => &mut self.phase_c_a,
            Channel::IqA => &mut self.iq_a,
            Channel::TorqueNm => &mut self.torque_nm,
            Channel::BatteryV => &mut self.battery_v,
            Channel::BatteryA => &mut self.battery_a,
            Channel::SocPct => &mut self.soc_pct,
            Channel::PitchDeg => &mut self.pitch_deg,
            Channel::RollDeg => &mut self.roll_deg,
            Channel::BoardTempC => &mut self.board_temp_c,
            Channel::MotorTempC => &mut self.motor_temp_c,
            Channel::DistanceM => &mut self.distance_m,
            Channel::IsrLoadPct => &mut self.isr_load_pct,
            Channel::RiderLeanDeg => &mut self.rider_lean_deg,
            Channel::TargetSpeedKmh => &mut self.target_speed_kmh,
        };
        *slot = value;
    }
}

/// The simulated state at one instant.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Snapshot {
    /// Simulated seconds since power-on.
    pub t_s: f64,
    /// Scalar signals.
    pub signals: Signals,
    /// One pose per body, in [`BodyInfo`] order.
    pub bodies: Vec<Pose>,
}

/// What the wheel shows and sounds: the board's outputs, as the firmware drives them. Sent when
/// they change, not with every snapshot.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Outputs {
    /// The display driver's grid memory (TM1640: 16 bytes, one per grid, one bit per segment).
    /// How grids and bits map to the digits and icons is board data (`display.toml`).
    pub display_grid: Vec<u8>,
    /// The display is switched on.
    pub display_on: bool,
    /// Display brightness step, 0..=7.
    pub display_brightness: u8,
    /// LED strip colours in strip order, `[r, g, b]`.
    pub leds: Vec<[u8; 3]>,
    /// Buzzer tone in Hz; 0 when silent.
    pub buzzer_hz: u32,
}

/// What kind of thing happened.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventKind {
    /// The firmware raised an alarm (speed, PWM, voltage, temperature).
    Alarm,
    /// The firmware cut the motor out.
    Cutoff,
    /// The firmware tilted the pedals back to slow the rider down.
    Tiltback,
    /// A command reached the wheel (BLE byte or sequence).
    Command,
    /// A firmware milestone (boot markers, first frame, resets).
    Firmware,
    /// A scenario step (intent changes, scripted inputs).
    Scenario,
    /// An injected fault or a problem the simulator found (blockers).
    Fault,
}

/// Something that happened at one instant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Event {
    /// Simulated seconds since power-on.
    pub t_s: f64,
    /// Kind.
    pub kind: EventKind,
    /// What happened, for people.
    pub text: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::CHANNELS;

    #[test]
    fn signals_get_and_set_cover_every_channel() {
        let mut s = Signals::default();
        for spec in CHANNELS {
            assert!(s.get(spec.channel).is_nan(), "{} starts unmodelled", spec.key);
            s.set(spec.channel, f32::from(spec.id) + 0.5);
        }
        for spec in CHANNELS {
            assert!((s.get(spec.channel) - (f32::from(spec.id) + 0.5)).abs() < f32::EPSILON, "{}", spec.key);
        }
    }

    #[test]
    fn json_uses_snake_case() {
        let event = Event { t_s: 1.5, kind: EventKind::Tiltback, text: "speed".into() };
        assert_eq!(serde_json::to_string(&event).unwrap(), r#"{"t_s":1.5,"kind":"tiltback","text":"speed"}"#);
        let body = BodyInfo { name: "wheel".into(), role: BodyRole::Wheel, geoms: vec![] };
        assert!(serde_json::to_string(&body).unwrap().contains(r#""role":"wheel""#));
    }
}

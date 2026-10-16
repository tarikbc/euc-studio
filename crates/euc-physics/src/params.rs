//! Parameters of the mechanical model: the wheel from its profile, the rider and the aids from the
//! scenario (or their defaults).
//!
//! `docs/models/body.md` explains every value and says which ones are defaults rather than
//! measurements.

use serde::Deserialize;
use thiserror::Error;

/// The wheel's mechanics (`[body]` in the wheel's `plant.toml`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WheelBody {
    /// The tire and its contact with the ground.
    pub tire: Tire,
    /// What spins: hub-motor rotor, rim and tire.
    pub wheel: Wheel,
    /// Everything that does not spin: shell, battery, controller, stator, pedals.
    pub shell: Shell,
    /// Measured collision shapes (the wheel's `geometry.toml` `[collision]`), if any; without them
    /// the tire is an ellipsoid and the shell a few generic boxes.
    #[serde(skip)]
    pub shapes: Option<Shapes>,
}

/// Collision shapes measured on a real wheel, in the shell frame (x forward, y left, z up,
/// origin on the axle centre).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Shapes {
    /// The tire as a torus around the axle (rolls round in the wheel plane, rounded across).
    pub tire: Option<Torus>,
    /// Solids of the shell (packs, seat, handle, brackets, pedals), by name.
    pub solids: Vec<Solid>,
}

/// A torus around the y axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Torus {
    /// Distance from the axle to the centre of the tube, metres.
    pub major_m: f64,
    /// Radius of the tube (the crown's lateral radius), metres.
    pub minor_m: f64,
}

/// One shell solid.
#[derive(Debug, Clone, PartialEq)]
pub enum Solid {
    /// A box.
    Box {
        /// Name (`pack_left`, `pedal_right`, ...).
        name: String,
        /// Centre, metres.
        centre: [f64; 3],
        /// Half sizes, metres.
        half: [f64; 3],
    },
    /// A capsule.
    Capsule {
        /// Name.
        name: String,
        /// One end of the axis, metres.
        from: [f64; 3],
        /// The other end.
        to: [f64; 3],
        /// Radius, metres.
        radius: f64,
    },
}

impl Solid {
    /// The name.
    pub fn name(&self) -> &str {
        match self {
            Self::Box { name, .. } | Self::Capsule { name, .. } => name,
        }
    }
}

/// `[body.tire]`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tire {
    /// Rolling radius under load, metres: from the axle to the ground. With a measured torus,
    /// the torus is sized to it (its tube keeps the measured crown radius).
    pub radius_m: f64,
    /// Half width of the tread, metres (the lateral semi-axis of the ellipsoid without a measured
    /// torus).
    pub half_width_m: f64,
    /// Sliding friction coefficient on the ground.
    pub friction: f64,
    /// Rolling resistance coefficient: resisting force over normal load.
    pub crr: f64,
    /// Contact time constant, seconds (MuJoCo `solref`): how soft the tire is.
    #[serde(default = "default_contact_time_s")]
    pub contact_time_s: f64,
}

const fn default_contact_time_s() -> f64 {
    0.02
}

/// `[body.wheel]`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Wheel {
    /// Spinning mass, kilograms.
    pub mass_kg: f64,
    /// Moment of inertia about the axle, kg·m².
    pub spin_inertia_kgm2: f64,
}

/// `[body.shell]`. Heights are measured from the axle, positive up.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Shell {
    /// Mass of everything that does not spin, kilograms.
    pub mass_kg: f64,
    /// Height of its centre of mass above the axle, metres.
    pub com_above_axle_m: f64,
    /// Principal moments of inertia about the centre of mass (roll, pitch, yaw), kg·m².
    pub inertia_kgm2: [f64; 3],
    /// Depth of the pedal surface below the axle, metres.
    pub pedal_below_axle_m: f64,
    /// Height of the IMU above the axle, metres.
    pub imu_above_axle_m: f64,
}

/// The rider: a rigid body standing on the pedals, hinged at the ankles.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Rider {
    /// Mass, kilograms.
    pub mass_kg: f64,
    /// Height of the centre of mass above the pedals, metres.
    pub com_above_pedals_m: f64,
    /// Height of the ankle joint above the pedals, metres.
    pub ankle_above_pedals_m: f64,
    /// Radius of gyration about the centre of mass for pitch and roll, metres.
    pub gyration_m: f64,
    /// Standing height, metres (for drawing).
    pub height_m: f64,
    /// Drag area of rider and wheel, m².
    pub cda_m2: f64,
    /// How the rider controls the lean.
    pub control: RiderControl,
}

impl Default for Rider {
    fn default() -> Self {
        Self {
            mass_kg: 80.0,
            com_above_pedals_m: 1.0,
            ankle_above_pedals_m: 0.08,
            gyration_m: 0.35,
            height_m: 1.78,
            cda_m2: 0.6,
            control: RiderControl::default(),
        }
    }
}

/// The rider's controller: a target speed sets a lean, the ankles hold it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct RiderControl {
    /// Ankle stiffness about the commanded lean, N·m per radian.
    pub lean_kp: f64,
    /// Ankle damping, N·m per rad/s.
    pub lean_kd: f64,
    /// Strongest ankle torque, N·m (both legs).
    pub ankle_max_nm: f64,
    /// Acceleration asked per m/s of speed error, 1/s.
    pub speed_kp: f64,
    /// Integral of the speed error, 1/s².
    pub speed_ki: f64,
    /// Strongest acceleration the rider asks for, m/s².
    pub accel_max: f64,
    /// Strongest braking the rider asks for, m/s².
    pub brake_max: f64,
    /// Largest lean from vertical, degrees.
    pub lean_max_deg: f64,
    /// Fastest change of the lean, degrees per second.
    pub lean_rate_deg_s: f64,
}

impl Default for RiderControl {
    fn default() -> Self {
        Self {
            lean_kp: 6000.0,
            lean_kd: 900.0,
            ankle_max_nm: 400.0,
            speed_kp: 0.6,
            speed_ki: 0.03,
            accel_max: 1.5,
            brake_max: 2.5,
            lean_max_deg: 15.0,
            lean_rate_deg_s: 12.0,
        }
    }
}

/// Aids the M2 model needs because the rider is rigid.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Assist {
    /// Roll spring on the shell, N·m per radian (0 = none). A rigid rider cannot steer or shift
    /// sideways, so without it the wheel falls over sideways.
    pub roll_stiffness: f64,
    /// Roll damper on the shell, N·m per rad/s.
    pub roll_damping: f64,
}

impl Default for Assist {
    fn default() -> Self {
        Self { roll_stiffness: 5000.0, roll_damping: 500.0 }
    }
}

/// Everything the model is built from.
#[derive(Debug, Clone, PartialEq)]
pub struct Setup {
    /// The wheel.
    pub wheel: WheelBody,
    /// The rider.
    pub rider: Rider,
    /// The aids.
    pub assist: Assist,
    /// Air density, kg/m³.
    pub air_density: f64,
    /// Gravity, m/s².
    pub gravity: f64,
    /// The ground.
    pub terrain: crate::Terrain,
}

impl Setup {
    /// The wheel with the default rider and aids, at sea level, on flat ground.
    pub fn new(wheel: WheelBody) -> Self {
        Self {
            wheel,
            rider: Rider::default(),
            assist: Assist::default(),
            air_density: 1.2,
            gravity: 9.81,
            terrain: crate::Terrain::flat(),
        }
    }

    /// Checks ranges that the types cannot.
    pub fn validate(&self) -> Result<(), ParamError> {
        let w = &self.wheel;
        let r = &self.rider;
        let positive = [
            ("body.tire.radius_m", w.tire.radius_m),
            ("body.tire.half_width_m", w.tire.half_width_m),
            ("body.tire.friction", w.tire.friction),
            ("body.wheel.mass_kg", w.wheel.mass_kg),
            ("body.wheel.spin_inertia_kgm2", w.wheel.spin_inertia_kgm2),
            ("body.shell.mass_kg", w.shell.mass_kg),
            ("rider.mass_kg", r.mass_kg),
            ("rider.com_above_pedals_m", r.com_above_pedals_m),
            ("rider.gyration_m", r.gyration_m),
            ("rider.height_m", r.height_m),
            ("gravity", self.gravity),
        ];
        if let Some((name, _)) = positive.iter().find(|(_, v)| !(*v > 0.0 && v.is_finite())) {
            return Err(ParamError(format!("{name} must be positive")));
        }
        let not_negative = [
            ("body.tire.crr", w.tire.crr),
            ("rider.cda_m2", r.cda_m2),
            ("assist.roll_stiffness", self.assist.roll_stiffness),
            ("assist.roll_damping", self.assist.roll_damping),
            ("air_density", self.air_density),
        ];
        if let Some((name, _)) = not_negative.iter().find(|(_, v)| !(*v >= 0.0 && v.is_finite())) {
            return Err(ParamError(format!("{name} must not be negative")));
        }
        if w.shell.inertia_kgm2.iter().any(|i| *i <= 0.0 || !i.is_finite()) {
            return Err(ParamError("body.shell.inertia_kgm2 must be positive".into()));
        }
        // MuJoCo needs at least two steps per contact time constant to stay stable.
        if w.tire.contact_time_s < 2.0 * crate::TIMESTEP_S {
            return Err(ParamError(format!("body.tire.contact_time_s must be at least {} s", 2.0 * crate::TIMESTEP_S)));
        }
        if let Some(torus) = w.shapes.as_ref().and_then(|s| s.tire)
            && !(torus.minor_m > 0.0 && torus.minor_m < w.tire.radius_m)
        {
            return Err(ParamError("the tire torus's tube must be thinner than the rolling radius".into()));
        }
        if r.ankle_above_pedals_m >= r.com_above_pedals_m {
            return Err(ParamError("rider.ankle_above_pedals_m must be below the centre of mass".into()));
        }
        self.terrain.validate().map_err(ParamError)
    }
}

/// A parameter out of its range.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("{0}")]
pub struct ParamError(pub String);

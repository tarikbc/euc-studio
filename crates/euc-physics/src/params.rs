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
    /// Everything that does not spin: shell, battery, controller, pedals, and the stator unless a
    /// suspension carries it.
    pub shell: Shell,
    /// The suspension between the axle and the shell, if the wheel has one; without it the axle
    /// is fixed in the shell.
    #[serde(default)]
    pub suspension: Option<Suspension>,
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

/// `[body.suspension]`: the axle rides on a carriage that slides in the shell, held down by a
/// coil-over shock through a linkage. The shell's heights are from the axle at full extension
/// (topped out); the carriage moves up into the shell as the suspension compresses. The linkage's
/// leverage (axle travel per shock travel) may change along the travel (`ratio_curve`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Suspension {
    /// Mass that moves with the axle besides the spinning wheel (stator, axle clamps, slider
    /// blocks), kilograms.
    pub unsprung_mass_kg: f64,
    /// Direction the axle moves as the suspension compresses, degrees from the shell's vertical,
    /// positive when the axle moves forward as it rises.
    pub axis_forward_deg: f64,
    /// Spring rate of the shock's coil, N/mm.
    pub spring_rate_n_per_mm: f64,
    /// Spring preload, mm of coil compression at full extension.
    pub preload_mm: f64,
    /// Shock stroke, mm.
    pub stroke_mm: f64,
    /// Shock length between the eye centres at full extension, mm (for drawing).
    pub eye_to_eye_mm: f64,
    /// Axle travel per unit of shock travel (the linkage's leverage), along the whole travel when
    /// `ratio_curve` is empty.
    pub linkage_ratio: f64,
    /// The leverage along the travel: points `[axle travel from full extension in mm, axle mm per
    /// shock mm]` in rising travel, joined by straight lines and held at the end values beyond
    /// them. Empty: `linkage_ratio` throughout.
    #[serde(default)]
    pub ratio_curve: Vec<[f64; 2]>,
    /// Shock damping, N·s/m at the shock, compression and rebound alike.
    pub damping_n_s_per_m: f64,
}

impl Suspension {
    /// The leverage at axle travel `x` (metres from full extension): axle travel per shock travel.
    pub fn ratio(&self, x: f64) -> f64 {
        let curve = &self.ratio_curve;
        let mm = 1000.0 * x;
        if curve.is_empty() {
            return self.linkage_ratio;
        }
        match curve.iter().position(|[travel, _]| *travel > mm) {
            Some(0) => curve[0][1],
            None => curve[curve.len() - 1][1],
            Some(i) => {
                let ([x0, r0], [x1, r1]) = (curve[i - 1], curve[i]);
                r0 + (r1 - r0) * (mm - x0) / (x1 - x0)
            }
        }
    }

    /// Shock travel from full extension, metres, at axle travel `x`: the integral of 1 / ratio
    /// along the axle travel (beyond full extension, at the first ratio).
    pub fn shock_m(&self, x: f64) -> f64 {
        if x <= 0.0 {
            return x / self.ratio(0.0);
        }
        // The ratio is straight between the curve's points, so each piece integrates exactly:
        // the integral of dx / r over a length l where r runs from r0 to r1 is l·ln(r1/r0)/(r1 − r0).
        let piece = |from: f64, to: f64| {
            let (r0, r1) = (self.ratio(from), self.ratio(to));
            if (r1 - r0).abs() < 1e-12 { (to - from) / r0 } else { (to - from) * (r1 / r0).ln() / (r1 - r0) }
        };
        let (mut shock, mut at) = (0.0, 0.0);
        for [travel_mm, _] in &self.ratio_curve {
            let knot = travel_mm / 1000.0;
            if knot <= at {
                continue;
            }
            if knot >= x {
                break;
            }
            shock += piece(at, knot);
            at = knot;
        }
        shock + piece(at, x)
    }

    /// Axle travel from full extension to full compression (the shock's whole stroke), metres.
    pub fn travel_m(&self) -> f64 {
        let stroke = self.stroke_mm / 1000.0;
        let most = self.ratio_curve.iter().map(|[_, r]| *r).fold(self.ratio(0.0), f64::max);
        bisect(0.0, stroke * most, |x| self.shock_m(x) - stroke)
    }

    /// The force the shock pushes the axle down with (towards full extension), newtons, at axle
    /// travel `x` (metres from full extension) and compression rate `rate` (m/s): the coil (never
    /// pulling) and the damper, each through the leverage at `x`.
    pub fn axle_force_n(&self, x: f64, rate: f64) -> f64 {
        let ratio = self.ratio(x);
        let coil = 1000.0 * self.spring_rate_n_per_mm * (self.shock_m(x) + self.preload_mm / 1000.0);
        (coil.max(0.0) + self.damping_n_s_per_m * rate / ratio) / ratio
    }

    /// Unit vector of the compression direction in the shell frame (x forward, z up).
    pub fn axis(&self) -> [f64; 3] {
        let a = self.axis_forward_deg.to_radians();
        [a.sin(), 0.0, a.cos()]
    }

    /// Compression at rest under `sprung_kg` (shell and rider) on level ground, metres from full
    /// extension, within the travel.
    pub fn sag_m(&self, sprung_kg: f64, gravity: f64) -> f64 {
        let load = sprung_kg * gravity * self.axis_forward_deg.to_radians().cos();
        let travel = self.travel_m();
        if self.axle_force_n(0.0, 0.0) >= load {
            0.0
        } else if self.axle_force_n(travel, 0.0) <= load {
            travel
        } else {
            bisect(0.0, travel, |x| self.axle_force_n(x, 0.0) - load)
        }
    }

    fn validate(&self) -> Result<(), ParamError> {
        let positive = [
            ("spring_rate_n_per_mm", self.spring_rate_n_per_mm),
            ("stroke_mm", self.stroke_mm),
            ("eye_to_eye_mm", self.eye_to_eye_mm),
            ("linkage_ratio", self.linkage_ratio),
        ];
        let not_negative = [
            ("unsprung_mass_kg", self.unsprung_mass_kg),
            ("preload_mm", self.preload_mm),
            ("damping_n_s_per_m", self.damping_n_s_per_m),
        ];
        if let Some((name, _)) = positive.iter().find(|(_, v)| !(*v > 0.0 && v.is_finite())) {
            return Err(ParamError(format!("body.suspension.{name} must be positive")));
        }
        if let Some((name, _)) = not_negative.iter().find(|(_, v)| !(*v >= 0.0 && v.is_finite())) {
            return Err(ParamError(format!("body.suspension.{name} must not be negative")));
        }
        if self.stroke_mm >= self.eye_to_eye_mm {
            return Err(ParamError("body.suspension.stroke_mm must be shorter than eye_to_eye_mm".into()));
        }
        if !self.axis_forward_deg.is_finite() || self.axis_forward_deg.abs() >= 45.0 {
            return Err(ParamError("body.suspension.axis_forward_deg must be within ±45°".into()));
        }
        let points_ok = self.ratio_curve.iter().all(|[x, r]| *x >= 0.0 && x.is_finite() && *r > 0.0 && r.is_finite());
        let rising = self.ratio_curve.windows(2).all(|w| w[1][0] > w[0][0]);
        if !(points_ok && rising) {
            return Err(ParamError(
                "body.suspension.ratio_curve needs [travel mm >= 0, ratio > 0] points in rising travel".into(),
            ));
        }
        Ok(())
    }
}

/// The root of `f` between `low` and `high`, where `f` rises through zero, to well under a micron.
fn bisect(mut low: f64, mut high: f64, f: impl Fn(f64) -> f64) -> f64 {
    for _ in 0..64 {
        let mid = f64::midpoint(low, high);
        if f(mid) < 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    f64::midpoint(low, high)
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
    /// When the rider lets go of the pedals of a wheel that goes down.
    pub let_go: LetGo,
}

/// When the rider lets go of the pedals on their own (`[rider.let_go]`): a real rider leaves a
/// wheel that goes down instead of riding it into the ground.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct LetGo {
    /// The rider lets go at all (false: the rider stays on the pedals whatever happens).
    pub enabled: bool,
    /// Shell pitch or roll beyond which the rider lets go once the firmware has cut the drive,
    /// degrees.
    pub after_cut_deg: f64,
    /// Shell pitch or roll beyond which the rider lets go in any case (a fall), degrees.
    pub fall_deg: f64,
    /// Time from the trigger to letting go, seconds: the rider's reaction.
    pub reaction_s: f64,
}

impl Default for LetGo {
    fn default() -> Self {
        Self { enabled: true, after_cut_deg: 20.0, fall_deg: 45.0, reaction_s: 0.1 }
    }
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
            let_go: LetGo::default(),
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
    /// Tiltback the rider tolerates before easing off, degrees, while riding forward: the
    /// firmware's own tiltback when the simulator can read it, else the pedals' pitch above the
    /// riding pitch (the angle the rider is used to). 90: the rider ignores tiltback.
    pub tiltback_feel_deg: f64,
    /// How much speed the rider gives up per degree of tiltback beyond `tiltback_feel_deg`, km/h
    /// (0: the rider ignores tiltback).
    pub tiltback_ease_kmh_per_deg: f64,
    /// Strongest braking while easing off for tiltback, m/s²: the rider stops pushing and lets
    /// the wheel slow, without braking hard.
    pub tiltback_ease_brake: f64,
    /// Steering: twist about the vertical per radian of heading away from the line, N·m.
    pub steer_kp: f64,
    /// Steering damping per rad/s of turn, N·m·s.
    pub steer_kd: f64,
    /// Strongest twist feet and legs give the wheel, N·m.
    pub steer_max_nm: f64,
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
            tiltback_feel_deg: 2.0,
            tiltback_ease_kmh_per_deg: 1.5,
            tiltback_ease_brake: 1.0,
            steer_kp: 2000.0,
            steer_kd: 200.0,
            steer_max_nm: 60.0,
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
            ("rider.let_go.after_cut_deg", r.let_go.after_cut_deg),
            ("rider.let_go.fall_deg", r.let_go.fall_deg),
            ("gravity", self.gravity),
        ];
        if let Some((name, _)) = positive.iter().find(|(_, v)| !(*v > 0.0 && v.is_finite())) {
            return Err(ParamError(format!("{name} must be positive")));
        }
        let not_negative = [
            ("body.tire.crr", w.tire.crr),
            ("rider.cda_m2", r.cda_m2),
            ("rider.let_go.reaction_s", r.let_go.reaction_s),
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
        if let Some(s) = &w.suspension {
            s.validate()?;
        }
        self.terrain.validate().map_err(ParamError)
    }
}

/// A parameter out of its range.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("{0}")]
pub struct ParamError(pub String);

#[cfg(test)]
mod tests {
    use super::*;

    fn shock(ratio_curve: Vec<[f64; 2]>) -> Suspension {
        Suspension {
            unsprung_mass_kg: 6.0,
            axis_forward_deg: 0.0,
            spring_rate_n_per_mm: 131.3,
            preload_mm: 2.0,
            stroke_mm: 55.0,
            eye_to_eye_mm: 200.0,
            linkage_ratio: 1.2,
            ratio_curve,
            damping_n_s_per_m: 2200.0,
        }
    }

    #[test]
    fn a_constant_leverage_scales_the_shock() {
        let s = shock(Vec::new());
        assert!((s.shock_m(0.012) - 0.010).abs() < 1e-12);
        assert!((s.travel_m() - 0.066).abs() < 1e-9);
        // The coil's force through the leverage: k (x / r + preload) / r.
        let expected = 131_300.0 * (0.012 / 1.2 + 0.002) / 1.2;
        assert!((s.axle_force_n(0.012, 0.0) - expected).abs() < 1e-6);
        let sag = s.sag_m(100.0, 9.81);
        assert!((s.axle_force_n(sag, 0.0) - 981.0).abs() < 1e-6, "balances the load at {sag} m");
        // A flat curve is the same as no curve.
        let flat = shock(vec![[0.0, 1.2], [60.0, 1.2]]);
        assert!((flat.shock_m(0.05) - s.shock_m(0.05)).abs() < 1e-12);
    }

    #[test]
    fn a_rising_leverage_integrates_exactly() {
        let s = shock(vec![[0.0, 1.4], [30.0, 1.6], [60.0, 2.0]]);
        assert!((s.ratio(0.015) - 1.5).abs() < 1e-12 && (s.ratio(0.1) - 2.0).abs() < 1e-12);
        // Against a fine midpoint sum.
        let n = 100_000;
        let x = 0.05;
        let numeric: f64 = (0..n).map(|i| x / f64::from(n) / s.ratio(x * (f64::from(i) + 0.5) / f64::from(n))).sum();
        assert!((s.shock_m(x) - numeric).abs() < 1e-9, "{} vs {numeric}", s.shock_m(x));
        assert!((s.shock_m(s.travel_m()) - 0.055).abs() < 1e-9);
        // More leverage further in: less force per mm of axle travel.
        let rate = |x: f64| (s.axle_force_n(x + 1e-4, 0.0) - s.axle_force_n(x, 0.0)) / 1e-4;
        assert!(rate(0.05) < rate(0.005));
        assert!(shock(vec![[10.0, 1.4], [5.0, 1.6]]).validate().is_err(), "travel must rise");
        assert!(shock(vec![[0.0, 0.0]]).validate().is_err(), "ratio must be positive");
    }
}

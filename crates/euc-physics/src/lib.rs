//! The mechanical world of a wheel and its rider, in MuJoCo.
//!
//! [`Physics`] builds an MJCF model from [`Setup`] (the wheel's `[body]` parameters, the rider and
//! the aids), steps it at [`TIMESTEP_S`] and exchanges three things with the rest of the engine:
//! - **torque in**: the motor's torque on the axle hinge, with its reaction on the shell;
//! - **wheel out**: the rotor's angle and speed relative to the stator, for the electrical plant;
//! - **IMU out**: specific force and angular rate at the IMU site, in the sensor frame.
//!
//! The rider is one rigid body hinged at the ankles ([`RiderController`] sets the ankle torque
//! from a target speed). A roll spring on the shell stands in for the lateral balance a rigid
//! rider cannot do ([`Assist`]). `docs/models/body.md` describes the model and every parameter.
//!
//! Frames: world and shell are x forward, y left, z up; positive wheel angle rolls forward;
//! the IMU site is the shell frame moved up to the sensor.

mod mjcf;
mod params;
mod rider;
mod terrain;

use euc_telemetry::{BodyInfo, BodyRole, Geom, GeomKind, Pose};
use mujoco_rs::prelude::*;
use thiserror::Error;

pub use params::{Assist, ParamError, Rider, RiderControl, Setup, Shapes, Shell, Solid, Tire, Torus, Wheel, WheelBody};
pub use rider::{Intent, RiderController};
pub use terrain::{Feature, Terrain};

use mjcf::names;

/// Physics step, seconds.
pub const TIMESTEP_S: f64 = 0.0005;

/// Pitch or roll beyond which the wheel counts as fallen, radians (45°).
const FALLEN_RAD: f64 = std::f64::consts::FRAC_PI_4;

/// The push of the foot that steps off: a roll torque on the shell, N·m, for [`STEP_OFF_KICK_S`].
/// Without it a riderless wheel in this symmetric model would roll on upright for ever.
const STEP_OFF_KICK_NM: f64 = 30.0;

/// How long the stepping foot pushes, seconds.
const STEP_OFF_KICK_S: f64 = 0.2;

/// Time the model settles on its stand before the clock starts, seconds.
const SETTLE_S: f64 = 0.2;

/// The model could not be built.
#[derive(Debug, Error)]
pub enum PhysicsError {
    /// A parameter is out of range.
    #[error(transparent)]
    Param(#[from] ParamError),
    /// MuJoCo refused the generated model (a bug, not a user error).
    #[error("MuJoCo rejected the generated model: {0}")]
    Model(String),
}

/// One IMU sample in the sensor frame (x forward, y left, z up on a level wheel).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// Specific force, m/s²: `[0, 0, g]` standing still and level.
    pub accel_mps2: [f64; 3],
    /// Angular rate, rad/s.
    pub gyro_rad_s: [f64; 3],
}

/// The state worth reporting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    /// Simulated seconds since the model was built.
    pub time_s: f64,
    /// Ground speed of the axle along the heading, m/s, forward positive.
    pub speed_mps: f64,
    /// Distance along the heading since the start, metres.
    pub distance_m: f64,
    /// Shell pitch, radians, nose up positive.
    pub pitch_rad: f64,
    /// Shell roll, radians, right side down positive.
    pub roll_rad: f64,
    /// Rider lean from vertical, radians, forward positive.
    pub rider_lean_rad: f64,
    /// Wheel speed relative to the shell, rad/s, forward positive.
    pub wheel_rad_s: f64,
    /// Motor torque applied in the last step, N·m, forward positive.
    pub motor_torque_nm: f64,
    /// Ankle torque applied in the last step, N·m, tipping the rider forward when positive.
    pub ankle_torque_nm: f64,
    /// The stand holds the wheel.
    pub held: bool,
    /// Pitch or roll beyond 45°: the wheel is down.
    pub fallen: bool,
}

/// Indices into the compiled model.
#[derive(Debug, Clone, Copy)]
struct Ids {
    shell: usize,
    wheel: usize,
    rider: usize,
    root_qpos: usize,
    root_dof: usize,
    axle_qpos: usize,
    axle_dof: usize,
    lean_qpos: usize,
    lean_dof: usize,
    ankle: usize,
    accel: usize,
    gyro: usize,
    stand: usize,
    on_pedals: usize,
}

/// The shell's attitude and motion, from the free joint.
#[derive(Debug, Clone, Copy)]
struct Attitude {
    /// Horizontal unit vector of the heading.
    heading: [f64; 2],
    pitch: f64,
    roll: f64,
    /// Pitch rate, nose down positive (rotation about the shell's y axis).
    pitch_down_rate: f64,
    /// Roll rate about the heading, right side down positive.
    roll_rate: f64,
    speed: f64,
}

/// The mechanical model and its state.
pub struct Physics {
    data: MjData<Box<MjModel>>,
    ids: Ids,
    setup: Setup,
    rider: RiderController,
    total_mass: f64,
    distance_m: f64,
    motor_torque: f64,
    ankle_torque: f64,
    push_n: f64,
    /// When the rider stepped off, seconds: no ankle, push, drag or roll aid any more.
    stepped_off_at: Option<f64>,
}

impl std::fmt::Debug for Physics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Physics").field("state", &self.state()).finish_non_exhaustive()
    }
}

impl Physics {
    /// Builds the model with the wheel upright on its stand, the rider standing still.
    pub fn new(setup: Setup) -> Result<Self, PhysicsError> {
        setup.validate()?;
        let model = MjModel::from_xml_string(&mjcf::mjcf(&setup)).map_err(|e| PhysicsError::Model(e.to_string()))?;
        let id = |kind: MjtObj, name: &str| {
            model.name_to_id(kind, name).ok_or_else(|| PhysicsError::Model(format!("no {name} in the model")))
        };
        let (shell, wheel, rider) = (
            id(MjtObj::mjOBJ_BODY, names::SHELL)?,
            id(MjtObj::mjOBJ_BODY, names::WHEEL)?,
            id(MjtObj::mjOBJ_BODY, names::RIDER)?,
        );
        let (root, axle, lean) = (
            id(MjtObj::mjOBJ_JOINT, names::ROOT)?,
            id(MjtObj::mjOBJ_JOINT, names::AXLE)?,
            id(MjtObj::mjOBJ_JOINT, names::LEAN)?,
        );
        let qpos = |joint: usize| model.jnt_qposadr()[joint] as usize;
        let dof = |joint: usize| model.jnt_dofadr()[joint] as usize;
        let sensor = |name: &str| id(MjtObj::mjOBJ_SENSOR, name).map(|s| model.sensor_adr()[s] as usize);
        let ids = Ids {
            shell,
            wheel,
            rider,
            root_qpos: qpos(root),
            root_dof: dof(root),
            axle_qpos: qpos(axle),
            axle_dof: dof(axle),
            lean_qpos: qpos(lean),
            lean_dof: dof(lean),
            ankle: id(MjtObj::mjOBJ_ACTUATOR, names::ANKLE)?,
            accel: sensor(names::IMU_ACCEL)?,
            gyro: sensor(names::IMU_GYRO)?,
            stand: id(MjtObj::mjOBJ_EQUALITY, names::STAND)?,
            on_pedals: id(MjtObj::mjOBJ_EQUALITY, names::ON_PEDALS)?,
        };
        let total_mass = model.body_mass().iter().sum();
        let data = MjData::new(Box::new(model));
        let rider = RiderController::new(setup.rider.control);
        let mut physics = Self {
            data,
            ids,
            setup,
            rider,
            total_mass,
            distance_m: 0.0,
            motor_torque: 0.0,
            ankle_torque: 0.0,
            push_n: 0.0,
            stepped_off_at: None,
        };
        // The stand and the tire are soft constraints: let them take the load before time starts,
        // so the first IMU sample already reads a wheel at rest.
        for _ in 0..(SETTLE_S / TIMESTEP_S).round() as usize {
            physics.step(0.0);
        }
        physics.data.set_time(0.0);
        physics.distance_m = 0.0;
        physics.data.forward();
        Ok(physics)
    }

    /// The parameters the model was built from.
    pub const fn setup(&self) -> &Setup {
        &self.setup
    }

    /// The MJCF text of the model (for inspection and the docs).
    pub fn mjcf(&self) -> String {
        mjcf::mjcf(&self.setup)
    }

    /// The rider's controller (set what the rider wants here).
    pub const fn rider(&mut self) -> &mut RiderController {
        &mut self.rider
    }

    /// Pushes the rider forward (backward when negative) with a horizontal force at the centre of
    /// mass, newtons, from the next step until changed: a nudge, a gust, a hand on the back.
    pub const fn set_push(&mut self, newtons: f64) {
        self.push_n = newtons;
    }

    /// The rider steps off: the feet weld lets go and the rider's body lands beside the wheel; the
    /// ankle torque, the rider's pushes and drag and the roll aid stop, and the stepping foot gives
    /// the shell a short sideways push. It cannot be undone.
    pub fn step_off(&mut self) {
        if self.stepped_off_at.is_none() {
            self.data.eq_active_mut()[self.ids.on_pedals] = false;
            self.stepped_off_at = Some(self.time_s());
        }
    }

    /// The rider has stepped off.
    pub const fn rider_off(&self) -> bool {
        self.stepped_off_at.is_some()
    }

    /// Holds the wheel upright and still on a stand, or lets it go.
    pub fn set_held(&mut self, held: bool) {
        self.data.eq_active_mut()[self.ids.stand] = held;
    }

    /// The stand holds the wheel.
    pub fn held(&self) -> bool {
        self.data.eq_active()[self.ids.stand]
    }

    /// Advances one [`TIMESTEP_S`] with `motor_torque_nm` on the wheel (forward positive; the
    /// shell takes the reaction).
    pub fn step(&mut self, motor_torque_nm: f64) {
        let ids = self.ids;
        // Positions and velocities first, so the rider and the aids act on the current state.
        self.data.step1();
        let att = self.attitude();
        let lean = -att.pitch + self.data.qpos()[ids.lean_qpos];
        let lean_rate = att.pitch_down_rate + self.data.qvel()[ids.lean_dof];
        let setup = &self.setup;
        let drag_area = 0.5 * setup.air_density * setup.rider.cda_m2;
        let aero = drag_area * att.speed * att.speed.abs();
        let drag = rider::Drag { rolling: setup.wheel.tire.crr * setup.gravity, aero: drag_area / self.total_mass };
        let sense = rider::Sense {
            dt: TIMESTEP_S,
            speed: att.speed,
            lean,
            lean_rate,
            drag,
            gravity: setup.gravity,
            held: self.held(),
            pitch_up: att.pitch,
        };
        let rider_off = self.rider_off();
        self.ankle_torque = if rider_off { 0.0 } else { self.rider.ankle_torque(&sense) };
        self.motor_torque = motor_torque_nm;

        let assist = if rider_off { Assist { roll_stiffness: 0.0, roll_damping: 0.0 } } else { self.setup.assist };
        let kick = match self.stepped_off_at {
            Some(at) if self.time_s() - at < STEP_OFF_KICK_S => STEP_OFF_KICK_NM,
            _ => 0.0,
        };
        let roll_torque = kick - (assist.roll_stiffness * att.roll + assist.roll_damping * att.roll_rate);
        let [hx, hy] = att.heading;
        self.data.ctrl_mut()[ids.ankle] = self.ankle_torque;
        self.data.qfrc_applied_mut()[ids.axle_dof] = motor_torque_nm;
        self.data.xfrc_applied_mut()[ids.shell] = [0.0, 0.0, 0.0, roll_torque * hx, roll_torque * hy, 0.0];
        let along = if rider_off { 0.0 } else { self.push_n - aero };
        self.data.xfrc_applied_mut()[ids.rider] = [along * hx, along * hy, 0.0, 0.0, 0.0, 0.0];
        self.data.step2();
        self.distance_m += att.speed * TIMESTEP_S;
    }

    /// Simulated seconds since the model was built.
    pub fn time_s(&self) -> f64 {
        self.data.time()
    }

    /// The rotor's angle (radians) and speed (rad/s) relative to the stator, forward positive.
    pub fn wheel(&self) -> (f64, f64) {
        (self.data.qpos()[self.ids.axle_qpos], self.data.qvel()[self.ids.axle_dof])
    }

    /// What the IMU measures, as of the start of the last step.
    pub fn imu(&self) -> ImuReading {
        let s = self.data.sensordata();
        let (a, g) = (self.ids.accel, self.ids.gyro);
        ImuReading { accel_mps2: [s[a], s[a + 1], s[a + 2]], gyro_rad_s: [s[g], s[g + 1], s[g + 2]] }
    }

    /// Gravity, m/s² (what a level IMU at rest reads as 1 g).
    pub const fn gravity(&self) -> f64 {
        self.setup.gravity
    }

    /// Total mass of wheel, shell and rider, kilograms.
    pub const fn total_mass(&self) -> f64 {
        self.total_mass
    }

    /// The state now.
    pub fn state(&self) -> BodyState {
        let att = self.attitude();
        let lean = -att.pitch + self.data.qpos()[self.ids.lean_qpos];
        BodyState {
            time_s: self.time_s(),
            speed_mps: att.speed,
            distance_m: self.distance_m,
            pitch_rad: att.pitch,
            roll_rad: att.roll,
            rider_lean_rad: lean,
            wheel_rad_s: self.wheel().1,
            motor_torque_nm: self.motor_torque,
            ankle_torque_nm: self.ankle_torque,
            held: self.held(),
            fallen: att.pitch.abs() > FALLEN_RAD || att.roll.abs() > FALLEN_RAD,
        }
    }

    /// The bodies a viewer draws, in the order [`Physics::poses`] reports them.
    pub fn bodies(&self) -> Vec<BodyInfo> {
        let model = self.data.model();
        let ids = self.ids;
        [
            (ids.wheel, BodyRole::Wheel, names::WHEEL),
            (ids.shell, BodyRole::Shell, names::SHELL),
            (ids.rider, BodyRole::Rider, names::RIDER),
        ]
        .into_iter()
        .map(|(body, role, name)| {
            let geoms = (0..model.geom_bodyid().len())
                .filter(|&g| model.geom_bodyid()[g] as usize == body)
                .filter_map(|g| {
                    let kind = match model.geom_type()[g] {
                        MjtGeom::mjGEOM_CAPSULE => GeomKind::Capsule,
                        MjtGeom::mjGEOM_BOX => GeomKind::Box,
                        MjtGeom::mjGEOM_SPHERE => GeomKind::Sphere,
                        MjtGeom::mjGEOM_CYLINDER => GeomKind::Cylinder,
                        MjtGeom::mjGEOM_ELLIPSOID => GeomKind::Ellipsoid,
                        _ => return None,
                    };
                    Some(Geom {
                        kind,
                        size: model.geom_size()[g].map(|v| v as f32),
                        pos: model.geom_pos()[g].map(|v| v as f32),
                        quat: model.geom_quat()[g].map(|v| v as f32),
                    })
                })
                .collect();
            BodyInfo { name: name.to_owned(), role, geoms }
        })
        .collect()
    }

    /// Where each body of [`Physics::bodies`] is, as of the start of the last step.
    pub fn poses(&self) -> Vec<Pose> {
        let ids = self.ids;
        [ids.wheel, ids.shell, ids.rider]
            .into_iter()
            .map(|body| Pose {
                pos: self.data.xpos()[body].map(|v| v as f32),
                quat: self.data.xquat()[body].map(|v| v as f32),
            })
            .collect()
    }

    /// Attitude, rates and ground speed of the shell from the free joint (current, unlike the
    /// derived body poses, which MuJoCo computes at the start of a step).
    fn attitude(&self) -> Attitude {
        let quat = &self.data.qpos()[self.ids.root_qpos + 3..self.ids.root_qpos + 7];
        let vel = &self.data.qvel()[self.ids.root_dof..self.ids.root_dof + 6];
        let [qw, qx, qy, qz] = [quat[0], quat[1], quat[2], quat[3]];
        // Columns of the rotation matrix: the shell's x (forward) and y (left) axes in the world.
        let fwd = [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy + qw * qz), 2.0 * (qx * qz - qw * qy)];
        let left = [2.0 * (qx * qy - qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz + qw * qx)];
        let norm = fwd[0].hypot(fwd[1]).max(1e-9);
        let heading = [fwd[0] / norm, fwd[1] / norm];
        Attitude {
            heading,
            pitch: fwd[2].clamp(-1.0, 1.0).asin(),
            roll: left[2].clamp(-1.0, 1.0).asin(),
            // The free joint's angular velocity is in the shell frame: x roll, y pitch (nose down).
            pitch_down_rate: vel[4],
            roll_rate: vel[3],
            speed: vel[0] * heading[0] + vel[1] * heading[1],
        }
    }
}

//! The mechanical world of a wheel and its rider, in MuJoCo.
//!
//! [`Physics`] builds an MJCF model from [`Setup`] (the wheel's `[body]` parameters, the rider and
//! the aids), steps it at [`TIMESTEP_S`] and exchanges three things with the rest of the engine:
//! - **torque in**: the motor's torque on the axle hinge, with its reaction on the shell;
//! - **wheel out**: the rotor's angle and speed relative to the stator, for the electrical plant;
//! - **IMU out**: specific force and angular rate at the IMU site, in the sensor frame.
//!
//! The default rider is articulated: an upper body on two legs with hip, knee and ankle hinges
//! ([`RiderModel::ArticulatedSagittalV1`]); the rigid rider, one body hinged at the ankles, stays
//! selectable as the controlled baseline, and the lateral rider ([`RiderModel::ArticulatedLateralV1`],
//! opt-in) adds the legs' sideways and twisting joints. [`RiderController`] turns a target speed into the lean
//! the ankles (and, for the articulated rider, the hips and knees) hold. A roll spring on the
//! shell stands in for the lateral balance neither rider can do ([`Assist`]). Steering retains a
//! capped external yaw aid on the shell, without an internal rider reaction.
//! `docs/models/body.md` describes the model and every parameter.
//!
//! Frames: world and shell are x forward, y left, z up; positive wheel angle rolls forward;
//! the IMU site is the shell frame moved up to the sensor.

mod anthropometry;
pub mod map;
mod mjcf;
mod params;
mod rider;
mod terrain;
pub mod world;

use euc_telemetry::{BodyInfo, BodyRole, Geom, GeomKind, Pose};
use mujoco_rs::prelude::*;
use thiserror::Error;

pub use map::{GeneratedMapSpec, Ground, GroundSample, HeightMap, MapFeature, MapFeatureKind};
pub use params::{
    Articulation, Assist, LetGo, ParamError, Rider, RiderControl, RiderModel, Setup, Shapes, Shell, Solid, Suspension,
    Tire, Torus, Wheel, WheelBody,
};
pub use rider::{Intent, RiderController, wrap_angle};
pub use terrain::{Feature, Terrain};
pub use world::{WorldGround, WorldStart};

use mjcf::{JointKind, SOLE_HALF_WIDTH_M, names};

/// Physics step, seconds.
pub const TIMESTEP_S: f64 = 0.0005;

/// MJCF geom group of every ground geom (`group="3"` on the floor or strip, curb boxes, a height
/// map and the world's geoms); [`Physics::ground_ray_m`] sees only it. Groups only filter rays
/// and drawing in MuJoCo, never contacts.
pub const GROUND_GROUP: usize = 3;

/// The default rider model's id ([`RiderModel::default`]).
pub const RIDER_MODEL: &str = "articulated_sagittal_v1";

/// The leg joints of `model` in [`Physics::rider_joints`] order (none for the rigid rider).
pub fn leg_joint_names(model: RiderModel) -> Vec<&'static str> {
    mjcf::leg_joints(model).iter().map(|(name, _, _)| *name).collect()
}

/// Pitch or roll beyond which the wheel counts as fallen, radians (45°).
const FALLEN_RAD: f64 = std::f64::consts::FRAC_PI_4;

/// The push of the foot that steps off: its roll moment on the shell, N·m, for [`STEP_OFF_KICK_S`]
/// (the rigid rider: an external torque; the articulated rider: the right foot pressing its
/// pedal, an internal force pair). Without it a riderless wheel in this symmetric model would roll
/// on upright for ever.
const STEP_OFF_KICK_NM: f64 = 30.0;

/// How long the stepping foot pushes, seconds.
const STEP_OFF_KICK_S: f64 = 0.2;

/// Time the model settles on its stand before the clock starts, seconds.
const SETTLE_S: f64 = 0.2;

/// Contact margin of a released sole-on-pedal pair, metres: far below any penetration, so the
/// pair never makes a contact again.
const RELEASED_MARGIN_M: f64 = -1.0e3;

/// Normal load above which a sole counts as standing on its pedal, newtons.
const FOOT_CONTACT_N: f64 = 1.0;

/// Largest requested foot lift, radians of hip flexion (about 15 cm at the default leg length).
const FOOT_LIFT_MAX_RAD: f64 = 0.6;

/// How fast a leg lifts or lowers its foot, radians of hip flexion per second: a step, not a kick.
const FOOT_LIFT_RATE_RAD_S: f64 = 3.0;

/// How fast the knees bend into a crouch or straighten out of one, radians per second: the full
/// 1.2 rad in 0.4 s. A full crouch asked for at once pitched the shell up to 10°.
const CROUCH_RATE_RAD_S: f64 = 3.0;

/// The deepest crouch, radians of knee flexion.
pub(crate) const CROUCH_MAX_RAD: f64 = 1.2;

/// How far a lateral knee swings out at most, radians of the hip's external rotation: about as
/// far as the tibia's internal rotation lets the knee go with the foot on its pedal.
const SWING_MAX_RAD: f64 = 0.45;

/// How long a knee that cannot swing further waits for its leg to follow before the contact pairs
/// come on, seconds.
const STANCE_SWING_WAIT_S: f64 = 0.2;

/// The most a lateral leg may press into the shell when its contact pairs come on, metres: the
/// contact then pushes it back out (the pads' depth is uncertain by about ±1 cm).
const STANCE_PENETRATION_MAX_M: f64 = 0.01;

/// How far a lateral hip abducts per radian of knee swing, radians: measured on the model with
/// the knees bent 21° to 30° (0.15 to 0.18).
const ADDUCTION_PER_SWING: f64 = 0.17;

/// In an absorb, how stiff the hips and knees hold their posture, as a share of `joint_kp` at
/// its start (ESTIMATE: legs that give).
const ABSORB_STIFFNESS: f64 = 0.15;

/// How often the scripted rider looks ahead for a step up, steps (every 10 ms).
const BRACE_LOOK_STEPS: u64 = 20;

/// The farthest apart the points the scripted rider looks at are, metres: a rise of
/// `brace_step_m` within it is a step, not a slope.
const BRACE_SPACING_M: f64 = 0.15;

/// The most points the scripted rider looks at (0.55 s at 30 km/h).
const BRACE_SAMPLES_MAX: usize = 32;

/// The slowest speed the scripted rider braces at, m/s: at walking pace a curb is stepped over.
const BRACE_SPEED_MIN_MPS: f64 = 1.0;

/// How long the wheel's speed, and then its fall, are averaged over for the absorb's trigger,
/// seconds.
const ABSORB_DECEL_TIME_S: f64 = 0.01;

/// In an absorb, how much of `joint_kd` the legs keep at its start (ESTIMATE: with the full
/// damping the ankles still turned the pedals over).
const ABSORB_DAMPING: f64 = 0.2;

/// How fast a lateral knee swings out or in, radians per second.
const SWING_RATE_RAD_S: f64 = 2.0;

/// How fast the stance swings a knee per newton of pad load off the preload, radians per second
/// per newton: 100 N too much moves it out at 0.2 rad/s.
const SWING_PER_N: f64 = 0.01;

/// How far clear of the shell the lateral legs stand before their contact pairs come on, metres.
const STANCE_CLEARANCE_M: f64 = 0.002;

/// Longest extra settle for the lateral legs to stand clear of the pads, seconds.
const STANCE_SETTLE_MAX_S: f64 = 1.0;

/// Settle after the pads come on, for the legs to come to rest on them, seconds.
const STANCE_REST_S: f64 = 0.5;

/// The hips' share of a crouch's knee flexion. The upper body keeps the lean the controller
/// holds; at 0.58 the hips go back about as far as the knees go forward, so the centre of mass
/// stays within 1° of the ankles at any depth (at 1/2 a full crouch put it 7.8° ahead, and the
/// wheel rode off with the stick at rest). Fitted to the model's estimated segment masses.
const CROUCH_HIP_SHARE: f64 = 0.58;

/// The same share for the lateral rider's segments (de Leva's, longer shanks than thighs).
const LATERAL_CROUCH_HIP_SHARE: f64 = 0.60;

/// How far [`Physics::leg_pads`] looks for a pad from a leg, metres; a leg further away reports
/// this gap.
const LEG_PAD_GAP_MAX_M: f64 = 0.2;

/// A sole pulling on its pedal harder than this, newtons, is counted as a pull: a welded foot can
/// pull, a real one cannot.
const SOLE_PULL_N: f64 = 5.0;

/// Time constant over which a welded sole's load is averaged, seconds: 20 ms keeps a step's or a
/// bump's load and takes out the fast part (on a wheel without a suspension the tire's mesh still
/// ripples the load near 1 kHz, 41 N RMS). Until 2026-10 the tire's ring of 48 capsules rippled the
/// wheel at 48 times its turn rate (144 Hz at 15 km/h), swinging a weld's load by up to ±250 N
/// without a suspension (±17 N on the Extreme's profile with its suspension).
const SOLE_LOAD_TIME_S: f64 = 0.02;

/// How the rider left the pedals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Leaving {
    /// Stepped off on purpose ([`Physics::step_off`]), with a push of the stepping foot.
    SteppedOff,
    /// Let go after the firmware cut the drive and the wheel tipped ([`LetGo::after_cut_deg`]).
    LetGoAfterCut,
    /// Let go as the wheel fell ([`LetGo::fall_deg`]).
    LetGoFalling,
    /// Both articulated feet were off their pedals for [`LOST_FOOTING_S`]: the rider fell or
    /// jumped off while the wheel stayed up.
    LostFooting,
}

/// Time both contact feet must be off their pedals before the rider counts as off, seconds:
/// longer than the airtime of the shipped impacts (a 2 cm pothole at 12 km/h unloads both soles
/// for about 0.06 s).
pub const LOST_FOOTING_S: f64 = 0.5;

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
    /// Suspension compression from full extension, metres of axle travel (0 without a suspension).
    pub suspension_m: f64,
    /// The shock's compression from full extension, metres (0 without a suspension).
    pub shock_m: f64,
    /// Motor torque applied in the last step, N·m, forward positive.
    pub motor_torque_nm: f64,
    /// Ankle torque applied in the last step, N·m, tipping the rider forward when positive.
    pub ankle_torque_nm: f64,
    /// The stand holds the wheel.
    pub held: bool,
    /// Pitch or roll beyond 45°: the wheel is down.
    pub fallen: bool,
}

/// Read-only human controller and mechanical state for free ride and telemetry.
/// Desired lean is input; commanded lean follows delay and slew limits; measured lean is physical.
/// External aid torques and signed work are model energy sources, not articulated human motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiderState {
    /// Human model identifier; the current rider remains rigid.
    pub model: &'static str,
    /// Desired manual lean before delay/limits, radians, forward positive. In speed mode this
    /// is the most recent computed lean target, not an equivalent speed input.
    pub desired_lean_rad: f64,
    /// Ankle target after command delay and slew limiting, radians, forward positive.
    pub commanded_lean_rad: f64,
    /// Upper-body lean projected along shell heading, radians, forward positive.
    pub measured_lean_rad: f64,
    /// Desired heading before command delay, radians, left positive from world +x.
    pub desired_heading_rad: f64,
    /// Heading target after command delay, radians.
    pub applied_heading_rad: f64,
    /// Physical shell heading, radians, left positive from world +x.
    pub heading_rad: f64,
    /// The lean into the turn the rider holds, radians of roll, right side down positive.
    pub turn_lean_rad: f64,
    /// Current shell origin in world coordinates, metres; not the detached rider position.
    pub position_m: [f64; 3],
    /// Current rigid rider origin in world coordinates, metres, including after detachment.
    pub rider_position_m: [f64; 3],
    /// Horizontal length of the shell's physical path, metres, always nonnegative.
    pub path_length_m: f64,
    /// The rider is still on the pedals (has not stepped off or let go).
    pub attached: bool,
    /// The stand holds the shell.
    pub held: bool,
    /// Last ankle actuator torque, N·m, forward positive.
    pub ankle_torque_nm: f64,
    /// The last ankle torque reached its nonzero configured cap.
    pub ankle_saturated: bool,
    /// Total simulated time at the ankle torque cap, seconds.
    pub ankle_saturation_s: f64,
    /// How far the legs give way now, 0 to 1, after something stopped the wheel (see
    /// [`Articulation::absorb_decel_mps2`]); zero for the rigid and contact-feet riders.
    pub absorb: f64,
    /// How far the scripted rider leans back for a step up ahead now, radians; 0 without a brace
    /// (see [`RiderControl::brace_lead_s`]).
    pub brace_lean_rad: f64,
    /// Command minus measured lean, radians.
    pub lean_error_rad: f64,
    /// Last external yaw aid on the shell, N·m, left
    /// positive. Heading twist fades with speed; yaw-rate damping remains active at every speed.
    pub yaw_aid_torque_nm: f64,
    /// Last external roll aid torque, N·m, excluding the artificial step-off kick.
    pub roll_aid_torque_nm: f64,
    /// Signed work of the external yaw aid, joules, integrated on physics ticks.
    pub yaw_aid_work_j: f64,
    /// Signed work of the external roll aid, joules, excluding the step-off kick.
    pub roll_aid_work_j: f64,
    /// Configured experimental request delay, seconds, rounded up to physics ticks in control.
    pub command_delay_s: f64,
    /// Configured experimental observation delay, seconds, separate from release delay.
    pub feedback_delay_s: f64,
}

/// A real articulated joint, left leg then right, each from the hip down (hip, knee, ankle; the
/// lateral rider also has hip adduction and rotation after the hip and ankle roll after the
/// ankle).
#[derive(Debug, Clone, PartialEq)]
pub struct RiderJointState {
    /// Stable physical joint name.
    pub name: &'static str,
    /// Joint posture target, radians. The upper-body lean task adds a separate internal moment.
    pub posture_target_rad: f64,
    /// Actual hinge angle, radians.
    pub angle_rad: f64,
    /// Actual hinge rate, radians per second.
    pub rate_rad_s: f64,
    /// Applied, saturated internal motor torque, N·m.
    pub torque_nm: f64,
}

/// What a welded sole does to its pedal, as of the last step (`articulated_sagittal_v1` and
/// `articulated_lateral_v1`): the weld's share of the load, and the ankle's roll moment against
/// what a real sole could carry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoleLoad {
    /// `left` or `right`.
    pub name: &'static str,
    /// How hard the pedal pushes the sole along the deck's normal, newtons, averaged over the
    /// last 20 ms (its fast part dropped). Negative: the foot pulls the pedal, which only the weld can
    /// do.
    pub normal_n: f64,
    /// The ankle's roll moment on the foot, N·m, inversion positive (zero without ankle roll).
    pub roll_nm: f64,
    /// The roll moment the ankle was held to, N·m: the sole's load as the step began times half
    /// its width (the centre of pressure at the edge of the foot), within the ankle's strength;
    /// zero without ankle roll.
    pub roll_limit_nm: f64,
}

const HUMAN_BODIES: [&str; 7] =
    ["rider", "left_thigh", "left_shank", "left_foot", "right_thigh", "right_shank", "right_foot"];

/// The most leg joints a model has (the lateral rider's 14).
const MAX_LEG_JOINTS: usize = 14;

/// The most leg-on-shell contact pairs a model has (knee, shin and calf on up to 10 shell parts a
/// side).
const MAX_PAD_PAIRS: usize = 64;

/// One lateral leg part (knee, shin or calf) on one shell part (a side pad, the front or the top
/// solid): the contact pair and its two geoms.
#[derive(Debug, Clone, Copy, Default)]
struct PadPair {
    pair: usize,
    /// 0 left, 1 right.
    leg: usize,
    /// The leg part's geom.
    part: usize,
    /// The shell part's geom.
    pad: usize,
}

/// One lateral leg against the shell (its side's pads, the front and top solids), as of the last
/// step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegPad {
    /// `left` or `right`.
    pub name: &'static str,
    /// How hard the shell pushes the knee, shin and calf, newtons (the normal forces summed).
    pub force_n: f64,
    /// The smallest distance from the knee, shin or calf to the shell, metres; negative: pressed
    /// in.
    pub gap_m: f64,
    /// How far the knee swings out about the line from hip to ankle, radians (the hip's external
    /// rotation that the stance and, for the inside knee in a turn, its spread hold).
    pub swing_rad: f64,
}

#[derive(Debug, Clone, Copy)]
struct LegJoint {
    name: &'static str,
    kind: JointKind,
    /// 0 left, 1 right.
    leg: usize,
    qpos: usize,
    dof: usize,
    motor: usize,
    /// The angle the model was built at (the rest stance), radians.
    rest: f64,
}

#[derive(Debug, Clone, Copy)]
struct ArticulatedIds {
    /// The leg joints in `mjcf::leg_joints` order; the first `joint_count` are real.
    joints: [LegJoint; MAX_LEG_JOINTS],
    joint_count: usize,
    /// The pedal welds (`articulated_sagittal_v1`), left then right.
    welds: Option<[usize; 2]>,
    /// The sole-on-pedal contact pairs (`articulated_contact_v1`), left then right.
    pairs: Option<[usize; 2]>,
    /// The leg-on-pad contact pairs (`articulated_lateral_v1` on a wheel with measured pads);
    /// the first `pad_pair_count` are real.
    pad_pairs: [PadPair; MAX_PAD_PAIRS],
    pad_pair_count: usize,
    /// The sole geoms, left then right.
    soles: [usize; 2],
    /// The pedal geoms on the shell, left then right.
    pedals: [usize; 2],
    /// The foot bodies, left then right.
    feet: [usize; 2],
}

impl ArticulatedIds {
    fn resolve(model: &MjModel, setup: &Setup) -> Result<Self, PhysicsError> {
        let id = |kind, name: &str| {
            model.name_to_id(kind, name).ok_or_else(|| PhysicsError::Model(format!("no {name} in the model")))
        };
        let rider = setup.rider.model;
        let contact = rider.has_contact_feet();
        let mut pad_pairs = [PadPair::default(); MAX_PAD_PAIRS];
        let mut pad_pair_count = 0;
        for (leg, side) in ["left", "right"].into_iter().enumerate() {
            for (part, target) in mjcf::leg_pairs(setup, leg) {
                let slot = pad_pairs
                    .get_mut(pad_pair_count)
                    .ok_or_else(|| PhysicsError::Model(format!("more than {MAX_PAD_PAIRS} leg-on-shell pairs")))?;
                *slot = PadPair {
                    pair: id(MjtObj::mjOBJ_PAIR, &format!("{side}_{part}{}{target}", mjcf::PAD_PAIR_INFIX))?,
                    leg,
                    part: id(MjtObj::mjOBJ_GEOM, &format!("{side}_{part}"))?,
                    pad: id(MjtObj::mjOBJ_GEOM, target)?,
                };
                pad_pair_count += 1;
            }
        }
        let unused = LegJoint { name: "", kind: JointKind::Hip, leg: 0, qpos: 0, dof: 0, motor: 0, rest: 0.0 };
        let mut joints = [unused; MAX_LEG_JOINTS];
        let leg_joints = mjcf::leg_joints(rider);
        for (slot, &(name, kind, leg)) in joints.iter_mut().zip(leg_joints) {
            let joint = id(MjtObj::mjOBJ_JOINT, name)?;
            let qpos = model.jnt_qposadr()[joint] as usize;
            *slot = LegJoint {
                name,
                kind,
                leg,
                qpos,
                dof: model.jnt_dofadr()[joint] as usize,
                motor: id(MjtObj::mjOBJ_ACTUATOR, &format!("{name}_motor"))?,
                rest: model.qpos0()[qpos],
            };
        }
        let sides = |kind, name: &dyn Fn(&str) -> String| -> Result<[usize; 2], PhysicsError> {
            Ok([id(kind, &name("left"))?, id(kind, &name("right"))?])
        };
        let (welds, pairs) = if contact {
            (None, Some(sides(MjtObj::mjOBJ_PAIR, &|side| format!("{side}{}", mjcf::SOLE_PAIR_SUFFIX))?))
        } else {
            (Some(sides(MjtObj::mjOBJ_EQUALITY, &|side| format!("on_{side}_pedal"))?), None)
        };
        Ok(Self {
            joints,
            joint_count: leg_joints.len(),
            welds,
            pairs,
            pad_pairs,
            pad_pair_count,
            soles: sides(MjtObj::mjOBJ_GEOM, &|side| format!("{side}_sole"))?,
            pedals: sides(MjtObj::mjOBJ_GEOM, &|side| format!("pedal_{side}"))?,
            feet: sides(MjtObj::mjOBJ_BODY, &|side| format!("{side}_foot"))?,
        })
    }

    /// The model's leg joints.
    fn joints(&self) -> &[LegJoint] {
        &self.joints[..self.joint_count]
    }

    /// The model's leg-on-pad contact pairs.
    fn pad_pairs(&self) -> &[PadPair] {
        &self.pad_pairs[..self.pad_pair_count]
    }
}

/// One of the rider's feet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foot {
    /// The left foot, on the left pedal.
    Left,
    /// The right foot, on the right pedal.
    Right,
}

impl Foot {
    const fn index(self) -> usize {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }
}

/// One articulated foot on its pedal, as of the last step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FootState {
    /// `left` or `right`.
    pub name: &'static str,
    /// The sole stands on its pedal: an active contact pushing with more than 1 N, or, for
    /// welded feet, an attached rider.
    pub in_contact: bool,
    /// How hard the pedal pushes on the sole along the deck's normal, newtons (NaN for welded
    /// feet: the weld's force is not a contact load).
    pub normal_n: f64,
    /// Friction of the pedal on the sole, along the deck, newtons (NaN for welded feet).
    pub friction_n: f64,
    /// How far the sole's centre has moved along the deck from where it stood when the model was
    /// built (slip, or a foot put down elsewhere), metres.
    pub slip_m: f64,
    /// That move along the deck, metres: forward, then to the left.
    pub offset_m: [f64; 2],
    /// Height of the sole's centre above where it stood when the model was built, normal to the
    /// deck, metres (positive: lifted).
    pub lift_m: f64,
}

/// Indices into the compiled model.
#[derive(Debug, Clone, Copy)]
struct Ids {
    shell: usize,
    wheel: usize,
    /// The tire's geom.
    tire: usize,
    rider: usize,
    root_qpos: usize,
    root_dof: usize,
    axle_qpos: usize,
    axle_dof: usize,
    lean_qpos: usize,
    lean_dof: usize,
    rider_qpos: usize,
    rider_dof: usize,
    /// The suspension joint's qpos and dof, if the wheel has one.
    suspension: Option<(usize, usize)>,
    ankle: usize,
    accel: usize,
    gyro: usize,
    stand: usize,
    /// The rigid rider's weld to the feet (the articulated feet have their own welds or pairs).
    on_pedals: Option<usize>,
    articulated: Option<ArticulatedIds>,
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
    /// World angular velocity projected onto horizontal heading, for external aid work.
    roll_aid_rate: f64,
    /// Turn rate about the vertical, left positive.
    yaw_rate: f64,
    /// Ground speed of the axle along the heading, m/s.
    speed: f64,
    /// Ground speed of the axle in any direction across the ground, m/s.
    ground_speed: f64,
}

/// The lateral legs' stance against the shell (`Physics::stance`).
#[derive(Debug, Clone, Copy, Default)]
struct Stance {
    /// How far each knee swings out about the line from hip to ankle, radians (left, right): the
    /// hip's external rotation the stance holds.
    swing_rad: [f64; 2],
    /// The legs' contact pairs with the shell are on.
    pads_on: bool,
    /// The model has settled: the pairs stay as they are (on, or off when the legs could not stand
    /// clear of the shell).
    done: bool,
    /// How long a knee has swung as far as it goes while the pairs are off, seconds.
    wait_s: f64,
    /// What the lateral joints held once the model settled, less what the swing then asked of
    /// them, radians (in `ArticulatedIds::joints` order): their posture holds the settled stance,
    /// so the springs carry no load at rest.
    offset_rad: [f64; MAX_LEG_JOINTS],
}

/// The articulated rider's answer to a curb: a brace before it (scripted riding), legs that give
/// way when it stops the wheel (the welded riders).
#[derive(Debug, Clone, Copy, Default)]
struct Absorb {
    /// The scripted rider braces for a step up ahead (its look ahead, every 10 ms).
    brace: bool,
    /// How far the legs give way now, 0 to 1.
    level: f64,
    /// How fast `speed` falls, m/s², averaged over [`ABSORB_DECEL_TIME_S`].
    decel: f64,
    /// The axle's speed across the ground, m/s, averaged over [`ABSORB_DECEL_TIME_S`].
    speed: f64,
}

/// The mechanical model and its state.
pub struct Physics {
    data: MjData<Box<MjModel>>,
    ids: Ids,
    setup: Setup,
    rider: RiderController,
    total_mass: f64,
    body_ids: Vec<(usize, BodyRole, &'static str)>,
    /// Every body of wheel and rider with mass (the trees of the shell's and the rider's free
    /// joints): what the lateral rider's roll aid leans as one.
    machine: Vec<usize>,
    /// The tire's rolling resistance from the last step's contacts, N·m against the spin
    /// ([`Self::rolling_resistance_nm`]); the next step applies it.
    rolling_nm: f64,
    /// Requested crouch, radians of knee flexion.
    crouch_target_rad: f64,
    /// The crouch the legs are moving to now: the request at [`CROUCH_RATE_RAD_S`].
    crouch_rad: f64,
    /// The knees' bend in the riding stance, radians, moving from straight (as built) to the
    /// soft knees of the lateral rider at [`CROUCH_RATE_RAD_S`] (zero for the other models).
    stance_rad: f64,
    /// Each welded sole's load along the deck's normal as of the last step, newtons (left,
    /// right); before the first step, half the rider's weight.
    sole_normal_n: [f64; 2],
    /// The roll moment each lateral ankle was held to in the last step, N·m (left, right).
    sole_roll_limit_nm: [f64; 2],
    /// The lateral legs' stance against the shell.
    stance: Stance,
    /// Requested lift of each articulated foot, radians of hip flexion (left, right).
    foot_lift_target_rad: [f64; 2],
    /// The lift the legs are moving to now: the request at [`FOOT_LIFT_RATE_RAD_S`].
    foot_lift_rad: [f64; 2],
    /// Since when both contact feet have been off their pedals, seconds.
    feet_off_since: Option<f64>,
    /// At least one sole carried load in the last complete step (always true without contact feet).
    soles_loaded: bool,
    /// Where each articulated sole's centre stood in the shell frame when the model was built.
    foot_home: [[f64; 3]; 2],
    distance_m: f64,
    path_length_m: f64,
    motor_torque: f64,
    ankle_torque: f64,
    ankle_saturation_s: f64,
    /// The welded rider's absorb.
    absorb: Absorb,
    /// The rider's lean from vertical as of the last step, radians, forward positive: for the
    /// articulated rider the upper-body pitch along the shell heading, for the rigid one its pitch.
    lean: f64,
    yaw_aid_torque: f64,
    roll_aid_torque: f64,
    yaw_aid_work_j: f64,
    roll_aid_work_j: f64,
    push_n: f64,
    /// When and how the rider left the pedals: no ankle, push, drag or roll aid any more.
    left: Option<(f64, Leaving)>,
    /// When the rider will let go (the trigger came, the reaction runs), and why.
    letting_go: Option<(f64, Leaving)>,
    /// The firmware has cut the drive (the session tells).
    drive_cut: bool,
    /// The streaming world's window and box pool, when the ground is a world.
    world: Option<world::Window>,
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
        let articulated = setup.rider.model.is_articulated();
        let (root, axle, lean) = (
            id(MjtObj::mjOBJ_JOINT, names::ROOT)?,
            id(MjtObj::mjOBJ_JOINT, names::AXLE)?,
            id(MjtObj::mjOBJ_JOINT, if articulated { "left_ankle" } else { names::LEAN })?,
        );
        let qpos = |joint: usize| model.jnt_qposadr()[joint] as usize;
        let dof = |joint: usize| model.jnt_dofadr()[joint] as usize;
        let sensor = |name: &str| id(MjtObj::mjOBJ_SENSOR, name).map(|s| model.sensor_adr()[s] as usize);
        let suspension = match &setup.wheel.suspension {
            Some(_) => {
                let joint = id(MjtObj::mjOBJ_JOINT, names::SUSPENSION)?;
                Some((qpos(joint), dof(joint)))
            }
            None => None,
        };
        let articulated_ids = articulated.then(|| ArticulatedIds::resolve(&model, &setup)).transpose()?;
        let mut body_ids = vec![(wheel, BodyRole::Wheel, names::WHEEL), (shell, BodyRole::Shell, names::SHELL)];
        let riders: &[&'static str] = if articulated { &HUMAN_BODIES } else { &[names::RIDER] };
        for &name in riders {
            body_ids.push((id(MjtObj::mjOBJ_BODY, name)?, BodyRole::Rider, name));
        }
        let ids = Ids {
            shell,
            wheel,
            tire: id(MjtObj::mjOBJ_GEOM, names::TIRE)?,
            rider,
            root_qpos: qpos(root),
            root_dof: dof(root),
            axle_qpos: qpos(axle),
            axle_dof: dof(axle),
            lean_qpos: qpos(lean),
            lean_dof: dof(lean),
            rider_qpos: qpos(id(MjtObj::mjOBJ_JOINT, names::RIDER_FREE)?),
            rider_dof: dof(id(MjtObj::mjOBJ_JOINT, names::RIDER_FREE)?),
            suspension,
            ankle: id(MjtObj::mjOBJ_ACTUATOR, if articulated { "left_ankle_motor" } else { names::ANKLE })?,
            accel: sensor(names::IMU_ACCEL)?,
            gyro: sensor(names::IMU_GYRO)?,
            stand: id(MjtObj::mjOBJ_EQUALITY, names::STAND)?,
            on_pedals: if articulated { None } else { Some(id(MjtObj::mjOBJ_EQUALITY, names::ON_PEDALS)?) },
            articulated: articulated_ids,
        };
        let (total_mass, machine) = (moving_mass(&model), machine_bodies(&model, [shell, rider]));
        let mut data = MjData::new(Box::new(model));
        // The world scales the tire's ground pairs by the surface under it.
        let world = setup.world.clone().map(|ground| world::Window::new(ground, &mut data)).transpose()?;
        let rider = RiderController::new(setup.rider.control);
        let half_weight = 0.5 * setup.rider.mass_kg * setup.gravity;
        let mut physics = Self {
            data,
            ids,
            setup,
            rider,
            total_mass,
            body_ids,
            machine,
            rolling_nm: 0.0,
            crouch_target_rad: 0.0,
            crouch_rad: 0.0,
            stance_rad: 0.0,
            sole_normal_n: [half_weight; 2],
            sole_roll_limit_nm: [0.0; 2],
            stance: Stance::default(),
            foot_lift_target_rad: [0.0; 2],
            foot_lift_rad: [0.0; 2],
            feet_off_since: None,
            soles_loaded: true,
            foot_home: [[0.0; 3]; 2],
            distance_m: 0.0,
            path_length_m: 0.0,
            motor_torque: 0.0,
            ankle_torque: 0.0,
            ankle_saturation_s: 0.0,
            absorb: Absorb::default(),
            lean: 0.0,
            yaw_aid_torque: 0.0,
            roll_aid_torque: 0.0,
            yaw_aid_work_j: 0.0,
            roll_aid_work_j: 0.0,
            push_n: 0.0,
            left: None,
            letting_go: None,
            drive_cut: false,
            world,
        };
        physics.settle();
        Ok(physics)
    }

    /// The stand and the tire are soft constraints: let them take the load before time starts,
    /// so the first IMU sample already reads a wheel at rest.
    fn settle(&mut self) {
        for _ in 0..(SETTLE_S / TIMESTEP_S).round() as usize {
            self.step(0.0);
        }
        // A lateral rider on a wheel with pads stands until the legs are clear of the pads and
        // the pairs are on, then until the legs rest on the pads.
        let pads = self.ids.articulated.is_some_and(|legs| legs.pad_pair_count > 0);
        let mut more = (STANCE_SETTLE_MAX_S / TIMESTEP_S).round() as usize;
        while pads && !self.stance.pads_on && more > 0 {
            self.step(0.0);
            more -= 1;
        }
        if pads {
            self.stance.done = true;
            for _ in 0..(STANCE_REST_S / TIMESTEP_S).round() as usize {
                self.step(0.0);
            }
        }
        if let Some(legs) = self.ids.articulated.filter(|_| self.setup.rider.model.has_lateral_legs()) {
            for (index, joint) in legs.joints().iter().enumerate() {
                if joint.kind.lateral() {
                    self.stance.offset_rad[index] = self.data.qpos()[joint.qpos] - self.posture_target(index, joint);
                }
            }
            self.stance.done = true;
        }
        self.data.set_time(0.0);
        self.distance_m = 0.0;
        self.path_length_m = 0.0;
        self.ankle_saturation_s = 0.0;
        self.yaw_aid_work_j = 0.0;
        self.roll_aid_work_j = 0.0;
        // Settling is not part of the public simulation clock or delay history. Keep the
        // settled controller state, but discard observations and requests from that interval.
        self.rider.reset_delays();
        self.data.forward();
        if let Some(legs) = self.ids.articulated {
            self.foot_home = legs.soles.map(|sole| self.in_shell(self.data.geom_xpos()[sole]));
        }
    }

    /// A world point in the shell's frame (x forward, y left, z up along the shell), metres from
    /// the shell's origin, as of the last kinematics.
    fn in_shell(&self, point: [f64; 3]) -> [f64; 3] {
        let origin = self.data.xpos()[self.ids.shell];
        let r = self.data.xmat()[self.ids.shell];
        let d = [point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]];
        std::array::from_fn(|j| r[j] * d[0] + r[3 + j] * d[1] + r[6 + j] * d[2])
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

    /// What resists motion, as the rider senses it: rolling resistance and air drag.
    fn drag(&self) -> rider::Drag {
        let setup = &self.setup;
        let drag_area = 0.5 * setup.air_density * setup.rider.cda_m2;
        rider::Drag { rolling: setup.wheel.tire.crr * setup.gravity, aero: drag_area / self.total_mass }
    }

    /// Manual drive limits at the speed now, m/s²: the strongest forward acceleration and the
    /// strongest braking (positive) the drive lean limit can ask for, net of rolling resistance and
    /// air drag (`RiderControl::manual_drive_lean_max_deg`). The firmware may deliver less.
    pub fn drive_caps(&self) -> (f64, f64) {
        self.rider.drive_caps(self.state().speed_mps, self.setup.gravity, self.drag())
    }

    /// Sets desired world heading in radians. Rejects nonfinite input without changing state.
    /// This changes a target for the external yaw aid, never a pose or velocity.
    pub fn set_heading_target(&mut self, rad: f64) -> Result<(), String> {
        self.rider.set_heading_target(rad)
    }

    /// Clears delayed manual requests on pause/disarm and holds the current measured heading.
    /// This changes controller state only; the lean command retains its physical-step slew limit.
    pub fn clear_rider_input(&mut self) {
        let measured = self.heading_rad();
        let target = self.rider.heading_target();
        let heading = if measured.is_finite() {
            measured
        } else if target.is_finite() {
            target
        } else {
            0.0
        };
        let _ = self.rider.clear_input(heading);
    }

    /// Current physical shell heading in radians, left positive from world +x.
    pub fn heading_rad(&self) -> f64 {
        let [x, y] = self.attitude().heading;
        y.atan2(x)
    }

    /// Current shell origin in world coordinates, metres, read directly from its free joint.
    /// This is not the rider COM or axle position when the suspension moves.
    pub fn position_m(&self) -> [f64; 3] {
        let q = self.ids.root_qpos;
        let p = self.data.qpos();
        [p[q], p[q + 1], p[q + 2]]
    }

    /// The streaming world under this run, if the ground is one.
    pub fn world_ground(&self) -> Option<&WorldGround> {
        self.world.as_ref().map(world::Window::ground)
    }

    /// The world's surface under the wheel as of the last world tick (every
    /// [`world::TICK_STEPS`] steps), if the ground is a world.
    pub fn ground_surface(&self) -> Option<euc_world::Surface> {
        self.world.as_ref().map(world::Window::surface)
    }

    /// Height of the ground under `xy` (physics frame), metres, as the tire meets it: one ray cast
    /// from 50 m above against the ground geoms only ([`GROUND_GROUP`]: the floor or strip, curb
    /// boxes, a height map, the world's window, box pool and rider's window), never a body;
    /// `None` where nothing is hit (in a world, beyond its windows). Cheap enough for a few
    /// look-ahead samples every step.
    pub fn ground_ray_m(&mut self, xy: [f64; 2]) -> Option<f64> {
        let mut groups = [0; mujoco_rs::mujoco_c::mjNGROUP as usize];
        groups[GROUND_GROUP] = 1;
        let (hit, distance) = self.data.ray(&[xy[0], xy[1], 50.0], &[0.0, 0.0, -1.0], Some(&groups), true, None, None);
        hit.map(|_| 50.0 - distance)
    }

    /// World upkeep so far: window recentres, the last recentre's cost on the step thread
    /// (seconds) and the solids in the box pool now.
    pub fn world_upkeep(&self) -> Option<(u64, f64, usize)> {
        self.world.as_ref().map(|w| (w.recentres, w.last_recentre_s, w.pooled()))
    }

    /// Current rigid rider origin, metres in world coordinates, including after detachment.
    pub fn rider_position_m(&self) -> [f64; 3] {
        let q = self.ids.rider_qpos;
        let p = self.data.qpos();
        [p[q], p[q + 1], p[q + 2]]
    }

    /// World COM of the moving rider subtree at the last kinematic update, metres.
    /// Includes every articulated limb; excludes the legacy ankle proxy attached to the shell.
    pub fn rider_com_m(&self) -> [f64; 3] {
        self.data.subtree_com()[self.ids.rider]
    }

    /// Upper-body pitch projected along the shell heading. The internal task torques below use
    /// this coordinate, not the separate rider-subtree COM angle.
    fn articulated_lean(&self, heading: [f64; 2]) -> f64 {
        let offset = self.ids.rider_qpos + 3;
        let positions = self.data.qpos();
        let [w, x, y, z] = [positions[offset], positions[offset + 1], positions[offset + 2], positions[offset + 3]];
        let up = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)];
        (up[0] * heading[0] + up[1] * heading[1]).atan2(up[2])
    }

    /// The articulated upper body's roll about the heading (radians, right side down positive,
    /// as the shell's) and its rate about the heading (rad/s).
    fn rider_roll(&self, heading: [f64; 2]) -> (f64, f64) {
        let (rotation, omega) = self.rider_frame();
        let roll = rotation[2][1].clamp(-1.0, 1.0).asin();
        (roll, omega[0] * heading[0] + omega[1] * heading[1])
    }

    /// The upper body's rotation matrix (its columns: the body's x, y and z axes in the world)
    /// and its angular velocity in the world, from its free joint.
    fn rider_frame(&self) -> ([[f64; 3]; 3], [f64; 3]) {
        let offset = self.ids.rider_qpos + 3;
        let positions = self.data.qpos();
        let [w, x, y, z] = [positions[offset], positions[offset + 1], positions[offset + 2], positions[offset + 3]];
        // The rotation matrix: its left (y) column gives the roll, and it turns the free joint's
        // body-frame angular velocity into the world.
        let rotation = [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ];
        let rates = &self.data.qvel()[self.ids.rider_dof + 3..self.ids.rider_dof + 6];
        let omega: [f64; 3] =
            std::array::from_fn(|i| rotation[i][0] * rates[0] + rotation[i][1] * rates[1] + rotation[i][2] * rates[2]);
        (rotation, omega)
    }

    /// The roll aid on the contact feet's upper body (zero for the other models; the lateral
    /// rider's is in [`Physics::lean_as_one`]), N·m about the heading, toward the wheel's roll
    /// `target`; adds its torque and work to the reported roll aid.
    fn rider_roll_aid(&mut self, heading: [f64; 2], target: f64, assist: Assist) -> f64 {
        let model = self.setup.rider.model;
        if !model.upper_body_aids() || model.has_lateral_legs() || self.rider_off() {
            return 0.0;
        }
        let (roll, rate) = self.rider_roll(heading);
        let torque = -(assist.roll_stiffness * (roll - target) + assist.roll_damping * rate);
        self.roll_aid_work_j += torque * rate * TIMESTEP_S;
        self.roll_aid_torque += torque;
        torque
    }

    /// The roll aid of every rider but contact feet: `torque`, N·m about the heading through the
    /// tire's contact, leans wheel and rider as one rigid machine. Each body takes the force and
    /// the moment of its own part of that lean (its mass times its acceleration about the line,
    /// its inertia times its turn), so the aid loads no joint, pad or pedal, and the legs hold only the
    /// posture.
    fn lean_as_one(&mut self, heading: [f64; 2], torque: f64) {
        let axis = [heading[0], heading[1], 0.0];
        let contact = self.tire_contact();
        let mut inertia = 0.0;
        for index in 0..self.machine.len() {
            let (mass, lever, spin) = self.lean_part(self.machine[index], axis, contact);
            inertia += mass * dot(lever, lever) + dot(spin, axis);
        }
        let acceleration = torque / inertia;
        for index in 0..self.machine.len() {
            let body = self.machine[index];
            let (mass, lever, spin) = self.lean_part(body, axis, contact);
            let applied = &mut self.data.xfrc_applied_mut()[body];
            for i in 0..3 {
                applied[i] += mass * acceleration * lever[i];
                applied[3 + i] += acceleration * spin[i];
            }
        }
    }

    /// One body's part in a lean about `axis` through `contact`: its mass, the direction and
    /// length of its centre's path per radian (`axis` × arm, metres) and its own inertia applied
    /// to `axis` in the world, `I·axis`, kg·m²: the moment its own turn about `axis` takes per
    /// rad/s². A body tilted in that turn (a rider leaning back) needs a moment off the axis too;
    /// without it the lean also yawed such a body.
    fn lean_part(&self, body: usize, axis: [f64; 3], contact: [f64; 3]) -> (f64, [f64; 3], [f64; 3]) {
        let model = self.data.model();
        let at = self.data.xipos()[body];
        let lever = cross(axis, [at[0] - contact[0], at[1] - contact[1], at[2] - contact[2]]);
        let (principal, frame) = (model.body_inertia()[body], self.data.ximat()[body]);
        // The axis in the body's principal frame, scaled by the principal moments, back in the world.
        let local: [f64; 3] = std::array::from_fn(|k| principal[k] * dot([frame[k], frame[3 + k], frame[6 + k]], axis));
        let spin = std::array::from_fn(|i| (0..3).map(|k| frame[3 * i + k] * local[k]).sum());
        (model.body_mass()[body], lever, spin)
    }

    /// The tire's lowest point now, metres: the crown's centre below the axle in the wheel's
    /// plane, less the crown's radius (a thin disc without a measured torus).
    fn tire_contact(&self) -> [f64; 3] {
        let axle = self.data.xpos()[self.ids.wheel];
        let r = self.data.xmat()[self.ids.shell];
        let side = [r[1], r[4], r[7]];
        // Straight down less its part along the axle: down in the wheel's plane.
        let down = [side[0] * side[2], side[1] * side[2], side[2] * side[2] - 1.0];
        let length = dot(down, down).sqrt().max(1e-9);
        let crown = self.setup.wheel.shapes.as_ref().and_then(|s| s.tire).map_or(0.0, |torus| torus.minor_m);
        let reach = (self.setup.wheel.tire.radius_m - crown) / length;
        [axle[0] + reach * down[0], axle[1] + reach * down[1], axle[2] + reach * down[2] - crown]
    }

    /// Physical rider and controller state without advancing simulation time.
    pub fn rider_state(&self) -> RiderState {
        let att = self.attitude();
        let measured_lean = self.lean;
        let c = self.setup.rider.control;
        RiderState {
            model: self.setup.rider.model.id(),
            desired_lean_rad: self.rider.desired_lean(),
            commanded_lean_rad: self.rider.lean_command(),
            measured_lean_rad: measured_lean,
            desired_heading_rad: self.rider.heading_target(),
            applied_heading_rad: self.rider.applied_heading_target(),
            heading_rad: att.heading[1].atan2(att.heading[0]),
            turn_lean_rad: self.rider.turn_lean(),
            position_m: self.position_m(),
            rider_position_m: self.rider_position_m(),
            path_length_m: self.path_length_m,
            attached: !self.rider_off(),
            held: self.held(),
            ankle_torque_nm: self.ankle_torque,
            ankle_saturated: self.ankle_saturated(),
            ankle_saturation_s: self.ankle_saturation_s,
            absorb: self.absorb.level,
            brace_lean_rad: self.rider.brace_lean(),
            lean_error_rad: self.rider.lean_command() - measured_lean,
            yaw_aid_torque_nm: self.yaw_aid_torque,
            roll_aid_torque_nm: self.roll_aid_torque,
            yaw_aid_work_j: self.yaw_aid_work_j,
            roll_aid_work_j: self.roll_aid_work_j,
            command_delay_s: c.command_delay_s,
            feedback_delay_s: c.feedback_delay_s,
        }
    }

    fn ankle_saturated(&self) -> bool {
        let limit = self.setup.rider.control.ankle_max_nm;
        limit > 0.0 && self.ankle_torque.abs() >= limit
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
        self.leave(Leaving::SteppedOff);
    }

    /// The rider has left the pedals.
    pub const fn rider_off(&self) -> bool {
        self.left.is_some()
    }

    /// When and how the rider left the pedals.
    pub const fn left_pedals(&self) -> Option<(f64, Leaving)> {
        self.left
    }

    /// Tells the model whether the firmware has cut the drive: the rider then lets go once the
    /// wheel tips past [`LetGo::after_cut_deg`].
    pub const fn set_drive_cut(&mut self, cut: bool) {
        self.drive_cut = cut;
    }

    fn leave(&mut self, how: Leaving) {
        if self.left.is_none() {
            if let Some(weld) = self.ids.on_pedals {
                self.data.eq_active_mut()[weld] = false;
            }
            if let Some(legs) = self.ids.articulated {
                for weld in legs.welds.into_iter().flatten() {
                    self.data.eq_active_mut()[weld] = false;
                }
                let pads = legs.pad_pairs().iter().map(|pad| pad.pair);
                self.release_pairs(legs.pairs.into_iter().flatten().chain(pads));
                for joint in legs.joints() {
                    self.data.ctrl_mut()[joint.motor] = 0.0;
                }
                self.foot_lift_rad = [0.0; 2];
                self.foot_lift_target_rad = [0.0; 2];
            }
            self.left = Some((self.time_s(), how));
            self.clear_rider_input();
            self.ankle_torque = 0.0;
            self.yaw_aid_torque = 0.0;
            self.roll_aid_torque = 0.0;
        }
    }

    /// The rider has left: the soles stop meeting the pedals and the legs the pads, so the feet
    /// pass the decks as the welded feet do once released. MuJoCo generates a pair's contacts
    /// only within its margin (its `gap` does not apply to explicit pairs in 3.9); the same model
    /// with the pairs' margins far below any penetration goes in. Sizes and state are unchanged,
    /// so the swap keeps every coordinate and velocity.
    fn release_pairs(&mut self, pairs: impl Iterator<Item = usize>) {
        let mut pairs = pairs.peekable();
        if pairs.peek().is_none() {
            return;
        }
        let mut released = Box::new(MjModel::clone(self.data.model()));
        for pair in pairs {
            released.pair_margin_mut()[pair] = RELEASED_MARGIN_M;
        }
        self.data.swap_model(released);
    }

    /// A rider lets go of a wheel that goes down: once the drive is cut and the shell tips past
    /// `after_cut_deg`, or past `fall_deg` in any case, after the reaction time. Letting go keeps
    /// the rider's own momentum (no push, unlike a step-off).
    fn watch_for_letting_go(&mut self, att: &Attitude) {
        let rule = self.setup.rider.let_go;
        if !rule.enabled || self.left.is_some() {
            self.feet_off_since = None;
            return;
        }
        let now = self.time_s();
        // Contact feet that have both left their pedals for a while: the rider is off, on the
        // stand or not (judged on the last complete step's contact loads).
        if self.ids.articulated.is_some_and(|legs| legs.pairs.is_some()) {
            if self.soles_loaded {
                self.feet_off_since = None;
            } else {
                let since = *self.feet_off_since.get_or_insert(now);
                if now - since >= LOST_FOOTING_S {
                    self.leave(Leaving::LostFooting);
                    return;
                }
            }
        }
        if self.held() {
            return;
        }
        match self.letting_go {
            Some((at, how)) if now >= at => self.leave(how),
            Some(_) => {}
            None => {
                let tilt = att.pitch.abs().max(att.roll.abs()).to_degrees();
                let how = if tilt > rule.fall_deg {
                    Some(Leaving::LetGoFalling)
                } else if self.drive_cut && tilt > rule.after_cut_deg {
                    Some(Leaving::LetGoAfterCut)
                } else {
                    None
                };
                self.letting_go = how.map(|how| (now + rule.reaction_s, how));
            }
        }
    }

    /// Holds the wheel upright and still on a stand, or lets it go.
    pub fn set_held(&mut self, held: bool) {
        self.data.eq_active_mut()[self.ids.stand] = held;
    }

    /// The stand holds the wheel.
    pub fn held(&self) -> bool {
        self.data.eq_active()[self.ids.stand]
    }

    /// Between steps, on a fixed cadence: the streaming world's ground follows the wheel, and a
    /// rider who has left it.
    fn tick_world(&mut self) {
        let ids = self.ids;
        let off = self.rider_off();
        if let Some(window) = &mut self.world {
            let (q, v) = (self.data.qpos(), self.data.qvel());
            let position = [q[ids.root_qpos], q[ids.root_qpos + 1]];
            let velocity = [v[ids.root_dof], v[ids.root_dof + 1]];
            let rider = off.then(|| [q[ids.rider_qpos], q[ids.rider_qpos + 1]]);
            window.tick(&mut self.data, position, velocity, rider);
        }
    }

    /// Advances one [`TIMESTEP_S`] with `motor_torque_nm` on the wheel (forward positive; the
    /// shell takes the reaction).
    pub fn step(&mut self, motor_torque_nm: f64) {
        let ids = self.ids;
        self.tick_world();
        // Positions and velocities first, so the rider and the aids act on the current state.
        self.data.step1();
        let att = self.attitude();
        self.watch_for_letting_go(&att);
        let (lean, lean_rate) = match &ids.articulated {
            Some(_) => (self.articulated_lean(att.heading), self.data.qvel()[ids.rider_dof + 4]),
            None => {
                (-att.pitch + self.data.qpos()[ids.lean_qpos], att.pitch_down_rate + self.data.qvel()[ids.lean_dof])
            }
        };
        self.lean = lean;
        let setup = &self.setup;
        let drag_area = 0.5 * setup.air_density * setup.rider.cda_m2;
        let aero = drag_area * att.speed * att.speed.abs();
        let drag = self.drag();
        let sense = rider::Sense {
            dt: TIMESTEP_S,
            speed: att.speed,
            lean,
            lean_rate,
            drag,
            gravity: setup.gravity,
            held: self.held(),
            pitch_up: att.pitch,
            heading: att.heading[1].atan2(att.heading[0]),
            yaw_rate: att.yaw_rate,
        };
        let rider_off = self.rider_off();
        self.update_absorb(rider_off, att.ground_speed);
        self.look_ahead(&att, rider_off);
        // In an absorb the ankles give way too: holding the body upright over feet that stopped
        // with the wheel would turn the body's momentum into a nose-down moment on the pedals.
        self.ankle_torque = if rider_off { 0.0 } else { self.rider.ankle_torque(&sense) * (1.0 - self.absorb.level) };
        // Steering: the roll aid holds the turn lean, standing in for the hips. The heading
        // twist fades with speed; yaw-rate damping remains active. Both yaw terms are external.
        let steer = if rider_off { rider::Steer::default() } else { self.rider.steer(&sense) };
        self.yaw_aid_torque = steer.yaw_nm;
        self.yaw_aid_work_j += steer.yaw_nm * att.yaw_rate * TIMESTEP_S;
        self.motor_torque = motor_torque_nm;

        let assist = if rider_off { Assist { roll_stiffness: 0.0, roll_damping: 0.0 } } else { self.setup.assist };
        let pushing_off = matches!(self.left, Some((at, Leaving::SteppedOff)) if self.time_s() - at < STEP_OFF_KICK_S);
        // The rigid rider's stepping foot is an external roll torque on the shell; the articulated
        // rider's right foot presses its own pedal (an internal force pair, below).
        let kick = if pushing_off && ids.articulated.is_none() { STEP_OFF_KICK_NM } else { 0.0 };
        // The roll aid leans wheel and rider together about the tire's contact (`lean_as_one`,
        // below) for every rider held to the wheel by a weld or by its legs. As a bare torque on
        // the shell it also yawed a machine whose mass lies far behind the contact (a rider leaning
        // back in a hard brake), and the heading loop answered that yaw with more lean until the
        // wheel fell. Leaned as one, the aid turns the whole machine about the contact, so it is twice
        // as stiff (for the lateral rider: the shell's spring and the upper body's, `upper_body_aids`):
        // with the single gains a rigid rider's full manual turn rolled in too slowly, overleaned and
        // the firmware cut out. Contact feet hold no body to the shell: their aid stays a torque on
        // the shell, with the upper body's own.
        let as_one = !self.setup.rider.model.has_contact_feet();
        let springs = if as_one { 2.0 } else { 1.0 };
        self.roll_aid_torque =
            -springs * (assist.roll_stiffness * (att.roll - steer.roll) + assist.roll_damping * att.roll_rate);
        self.roll_aid_work_j += self.roll_aid_torque * att.roll_aid_rate * TIMESTEP_S;
        let roll_torque = kick + if as_one { 0.0 } else { self.roll_aid_torque };
        let [hx, hy] = att.heading;
        if let Some(legs) = ids.articulated {
            if legs.pad_pair_count > 0 && !rider_off {
                self.stance(&legs);
            }
            self.drive_legs(&legs, rider_off);
        } else {
            self.data.ctrl_mut()[ids.ankle] = self.ankle_torque;
        }
        if self.ankle_saturated() {
            self.ankle_saturation_s += TIMESTEP_S;
        }
        self.data.qfrc_applied_mut()[ids.axle_dof] = motor_torque_nm;
        if let (Some(s), Some((q, v))) = (&self.setup.wheel.suspension, ids.suspension) {
            let (travel, rate) = (self.data.qpos()[q], self.data.qvel()[v]);
            self.data.qfrc_applied_mut()[v] = -s.axle_force_n(travel, rate);
        }
        // Every external wrench is this step's: MuJoCo keeps an applied wrench until it is set again.
        self.data.xfrc_applied_mut().fill([0.0; 6]);
        self.data.xfrc_applied_mut()[ids.shell] = [0.0, 0.0, 0.0, roll_torque * hx, roll_torque * hy, steer.yaw_nm];
        if let Some(legs) = ids.articulated
            && pushing_off
        {
            self.push_off(&legs);
        }
        let along = if rider_off { 0.0 } else { self.push_n - aero };
        // Contact feet do not hold the rider's body to the shell: the same roll aid holds the
        // upper body at the wheel's roll target, standing in for the hip and ankle roll a person
        // balances with. External, like the shell's aid. The lateral rider's legs hold its body:
        // its roll aid leans the whole machine, and its hips turn the body with the shell.
        let rider_roll = self.rider_roll_aid(att.heading, steer.roll, assist);
        self.data.xfrc_applied_mut()[ids.rider] = [along * hx, along * hy, 0.0, rider_roll * hx, rider_roll * hy, 0.0];
        if as_one {
            self.lean_as_one(att.heading, self.roll_aid_torque);
        }
        // Rolling resistance turns the wheel against the ground about its axle.
        let axis = self.data.xmat()[ids.wheel];
        let rolling = self.rolling_nm;
        self.data.xfrc_applied_mut()[ids.wheel] =
            [0.0, 0.0, 0.0, -rolling * axis[1], -rolling * axis[4], -rolling * axis[7]];
        let before = self.position_m();
        self.data.step2();
        self.rolling_nm = self.rolling_resistance_nm();
        let after = self.position_m();
        self.path_length_m += (after[0] - before[0]).hypot(after[1] - before[1]);
        self.distance_m += att.speed * TIMESTEP_S;
        // Contact loads are consistent only now: this step's contacts with this step's forces.
        self.soles_loaded = self.feet().is_none_or(|feet| feet.iter().any(|foot| foot.in_contact));
        if let Some(normals) = self.weld_normals() {
            let follow = TIMESTEP_S / SOLE_LOAD_TIME_S;
            for (load, normal) in self.sole_normal_n.iter_mut().zip(normals) {
                *load += (normal - *load) * follow;
            }
        }
    }

    /// Each pedal weld's push on its foot along the deck's normal, newtons (left, right), from
    /// this step's constraint forces; `None` without pedal welds. A weld's first three
    /// constraint rows are its force on the foot (its first body) in the world frame. A weld
    /// that is off has no rows and carries nothing.
    fn weld_normals(&self) -> Option<[f64; 2]> {
        let welds = self.ids.articulated?.welds?;
        let shell = self.data.xmat()[self.ids.shell];
        let normal = [shell[2], shell[5], shell[8]];
        let (types, ids, forces) = (self.data.efc_type(), self.data.efc_id(), self.data.efc_force());
        Some(welds.map(|weld| {
            (0..types.len())
                .find(|&row| types[row] == MjtConstraint::mjCNSTR_EQUALITY && ids[row] as usize == weld)
                .map_or(0.0, |row| (0..3).map(|axis| forces[row + axis] * normal[axis]).sum())
        }))
    }

    /// The articulated rider's leg motors for this step: each joint's posture, and for the pitch
    /// joints of the legs on their pedals a share of the lean task's moment (the ankle torque).
    fn drive_legs(&mut self, legs: &ArticulatedIds, rider_off: bool) {
        let config = self.setup.rider.articulation;
        let step = FOOT_LIFT_RATE_RAD_S * TIMESTEP_S;
        for (lift, target) in self.foot_lift_rad.iter_mut().zip(self.foot_lift_target_rad) {
            *lift += (target - *lift).clamp(-step, step);
        }
        let crouch_step = CROUCH_RATE_RAD_S * TIMESTEP_S;
        let brace = if self.absorb.brace { self.setup.rider.control.brace_crouch_rad } else { 0.0 };
        let crouch = self.crouch_target_rad.max(brace);
        self.crouch_rad += (crouch - self.crouch_rad).clamp(-crouch_step, crouch_step);
        self.stance_rad += (self.soft_knee_rad() - self.stance_rad).clamp(-crouch_step, crouch_step);
        let lifted = self.foot_lift_rad.map(|lift| lift > 0.0);
        for (index, joint) in legs.joints().iter().enumerate() {
            let leg = joint.leg;
            let ankle = matches!(joint.kind, JointKind::Ankle | JointKind::AnkleRoll);
            // A rider leans from the ankles with the body straight, as the rigid rider does:
            // hips and knees hold the crouch, and a standing ankle has no posture spring, so
            // the lean task alone sets the ankle angle and the legs lean with the upper body.
            // With a spring at the ankle too, the three joints shared the lean, the legs leaned
            // a third as far as the upper body, and the centre of mass leaned (and the wheel
            // accelerated) about half as much as asked. A lifted foot keeps its spring.
            let stiffness = if ankle && !lifted[leg] {
                0.0
            } else if matches!(joint.kind, JointKind::Hip | JointKind::Knee) {
                config.joint_kp * (1.0 - self.absorb.level * (1.0 - ABSORB_STIFFNESS))
            } else {
                config.joint_kp
            };
            let target = self.posture_target(index, joint);
            // Contact feet damp their ankles in the joint itself (implicit; see mjcf.rs). A foot that
            // rocks off its deck turns on the ankle alone, and this explicit damping on its small
            // inertia (about 0.01 kg·m², above the step's stability limit) made it chatter at the
            // torque cap: the soles hopped on their decks and slid back.
            let damping = if ankle && legs.pairs.is_some() {
                0.0
            } else {
                config.joint_kd * (1.0 - self.absorb.level * (1.0 - ABSORB_DAMPING))
            };
            let posture = stiffness * (target - self.data.qpos()[joint.qpos]) - damping * self.data.qvel()[joint.dof];
            let mut limit = joint.kind.cap(&self.setup.rider);
            if joint.kind == JointKind::AnkleRoll {
                // A real sole rolls its pedal only as far as its centre of pressure can move
                // to the edge of the foot: the weld must not carry more than that.
                limit = limit.min(self.sole_normal_n[leg].max(0.0) * SOLE_HALF_WIDTH_M);
                self.sole_roll_limit_nm[leg] = limit;
            }
            // Upper pitch relative to a supported foot is -(hip + knee + ankle).
            // Its Jacobian transpose applies the same negative task moment at every hinge.
            // Applying upper-body damping only at the ankle can inject energy into the
            // hip/knee modes. The legs that stand on their pedals share the task moment and
            // keep internal reactions; a lifted leg has nothing to push against.
            let share = if !joint.kind.pitch() || lifted[leg] {
                0.0
            } else if lifted[1 - leg] {
                1.0
            } else {
                0.5
            };
            let command = posture - self.ankle_torque * share;
            self.data.ctrl_mut()[joint.motor] = if rider_off { 0.0 } else { command.clamp(-limit, limit) };
        }
        self.ankle_torque = -legs
            .joints()
            .iter()
            .filter(|j| j.kind == JointKind::Ankle)
            .map(|j| self.data.ctrl()[j.motor])
            .sum::<f64>();
    }

    /// The lateral rider's stance against the shell, once a step. While the model settles and
    /// the legs' contact pairs are off, each knee swings out until its leg stands clear of the
    /// shell or the knee has swung as far as it goes; the pairs come on once the knees have bent
    /// to the riding stance and both legs are clear, or have swung out for
    /// [`STANCE_SWING_WAIT_S`] and press into the shell by less than
    /// [`STANCE_PENETRATION_MAX_M`]. Then each knee swings slowly so that its leg rests on the
    /// shell with the preload: out when the shell pushes harder, in when it pushes less. In a
    /// turn the outside leg rests harder, by `turn_press_n_per_rad` per radian of the lean.
    fn stance(&mut self, legs: &ArticulatedIds) {
        let most = SWING_RATE_RAD_S * TIMESTEP_S;
        if self.stance.done && !self.stance.pads_on {
            return;
        }
        if self.stance.pads_on {
            let config = self.setup.rider.articulation;
            let force = self.leg_shell_forces(legs);
            let lean = self.rider.turn_lean();
            for (leg, (swing, force)) in self.stance.swing_rad.iter_mut().zip(force).enumerate() {
                let preload = config.pad_preload_n + config.turn_press_n_per_rad * outside_lean(leg, lean);
                let change = (SWING_PER_N * (force - preload) * TIMESTEP_S).clamp(-most, most);
                *swing = (*swing + change).clamp(0.0, SWING_MAX_RAD);
            }
            return;
        }
        let mut gap = [f64::INFINITY; 2];
        for pad in legs.pad_pairs() {
            let distance = self.data.geom_distance(pad.part, pad.pad, LEG_PAD_GAP_MAX_M, None);
            gap[pad.leg] = gap[pad.leg].min(distance);
        }
        let clear = gap.map(|gap| gap >= STANCE_CLEARANCE_M);
        for (swing, clear) in self.stance.swing_rad.iter_mut().zip(clear) {
            if !clear {
                *swing = (*swing + most).min(SWING_MAX_RAD);
            }
        }
        let full = self.stance.swing_rad.map(|swing| swing >= SWING_MAX_RAD);
        self.stance.wait_s = if full.contains(&true) { self.stance.wait_s + TIMESTEP_S } else { 0.0 };
        let waited = self.stance.wait_s >= STANCE_SWING_WAIT_S;
        let ready = (0..2).all(|leg| clear[leg] || (waited && gap[leg] >= -STANCE_PENETRATION_MAX_M));
        if ready && self.stance_rad >= self.soft_knee_rad() {
            let mut engaged = Box::new(MjModel::clone(self.data.model()));
            for pad in legs.pad_pairs() {
                engaged.pair_margin_mut()[pad.pair] = 0.0;
            }
            self.data.swap_model(engaged);
            self.stance.pads_on = true;
        }
    }

    /// The tire's rolling resistance, N·m against the wheel's spin: each tire contact's normal
    /// force times its pair's rolling coefficient (`crr · r`, which the streaming world scales per
    /// surface), faded out near standstill as the rider's drag is, so it never turns a wheel at
    /// rest. Read after a step, when its contacts and forces belong together.
    fn rolling_resistance_nm(&self) -> f64 {
        let mut torque = 0.0;
        for (index, contact) in self.data.contact().iter().enumerate() {
            if contact.geom[0] as usize == self.ids.tire || contact.geom[1] as usize == self.ids.tire {
                // A degenerate contact can come back without a finite force; it carries none.
                let normal = self.data.contact_force(index)[0].abs();
                if normal.is_finite() {
                    torque += normal * contact.friction[3];
                }
            }
        }
        let speed = self.data.qvel()[self.ids.axle_dof] * self.setup.wheel.tire.radius_m;
        torque * (speed / rider::Drag::ROLLING_FADE_MPS).tanh()
    }

    /// How hard the shell pushes each lateral leg, newtons (left, right): the normal forces of the
    /// leg-on-shell contacts of the last step.
    fn leg_shell_forces(&self, legs: &ArticulatedIds) -> [f64; 2] {
        let mut force = [0.0; 2];
        for (index, contact) in self.data.contact().iter().enumerate() {
            if contact.exclude != 0 {
                continue;
            }
            let geoms = [contact.geom[0] as usize, contact.geom[1] as usize];
            if let Some(pad) = legs.pad_pairs().iter().find(|p| geoms == [p.part, p.pad] || geoms == [p.pad, p.part]) {
                // A degenerate contact can come back without a finite force; it carries none.
                let normal = self.data.contact_force(index)[0].abs();
                if normal.is_finite() {
                    force[pad.leg] += normal;
                }
            }
        }
        force
    }

    /// The articulated rider's stepping foot pushes off: the right foot presses its pedal straight
    /// down and the pedal pushes the foot up, an equal and opposite pair at the pedal's centre, as
    /// strong as the rigid rider's kick about the heading. It tips the riderless wheel as a real
    /// step-off does, without an external force or moment on wheel and rider together, and
    /// without changing the rider's horizontal momentum.
    fn push_off(&mut self, legs: &ArticulatedIds) {
        let point = self.data.geom_xpos()[legs.pedals[1]];
        let lever = self.in_shell(point)[1].abs().max(0.05);
        let down = [0.0, 0.0, -STEP_OFF_KICK_NM / lever];
        for (body, force) in [(self.ids.shell, down), (legs.feet[1], down.map(|f| -f))] {
            let com = self.data.xipos()[body];
            let arm = [point[0] - com[0], point[1] - com[1], point[2] - com[2]];
            let [tx, ty, tz] = cross(arm, force);
            let applied = &mut self.data.xfrc_applied_mut()[body];
            for (axis, value) in force.into_iter().chain([tx, ty, tz]).enumerate() {
                applied[axis] += value;
            }
        }
    }

    /// The welded rider's absorb, once a step: a wheel that slows faster than `absorb_decel_mps2`
    /// starts it in full, and it fades out over `absorb_time_s`. A rider meets a curb on legs
    /// that give: the knees and hips fold and go soft, and the ankles let the body go on over the
    /// feet, so the body's momentum does not turn the pedals over. A sole's load would not do as
    /// the trigger: the sagittal rider's soles take short peaks of over three times their share
    /// in a hard turn.
    fn update_absorb(&mut self, rider_off: bool, speed: f64) {
        let config = self.setup.rider.articulation;
        // The axle's speed across the ground in any direction: in a hard turn the speed along the
        // heading drops by a tenth for a few milliseconds whenever the shell's heading jumps,
        // and that is not a stop. The speed and its fall are each averaged over
        // ABSORB_DECEL_TIME_S.
        let absorb = &mut self.absorb;
        let follow = TIMESTEP_S / ABSORB_DECEL_TIME_S;
        let before = absorb.speed;
        absorb.speed += (speed.abs() - absorb.speed) * follow;
        absorb.decel += ((before - absorb.speed) / TIMESTEP_S - absorb.decel) * follow;
        let welded = self.ids.articulated.is_some_and(|legs| legs.welds.is_some());
        if !welded || rider_off || self.data.eq_active()[self.ids.stand] {
            absorb.level = 0.0;
        } else if absorb.decel > config.absorb_decel_mps2 {
            absorb.level = 1.0;
        } else {
            absorb.level = (absorb.level - TIMESTEP_S / config.absorb_time_s).max(0.0);
        }
    }

    /// The scripted articulated rider looks ahead every [`BRACE_LOOK_STEPS`] steps and braces
    /// while a step up lies within `brace_lead_s` at the speed now: from the ground just in front
    /// of the tire, points at most [`BRACE_SPACING_M`] apart along the heading, and a rise of more
    /// than `brace_step_m` between two neighbours (a step, not a slope: that slope would be steeper
    /// than any road). Once the tire meets the step the ground in front of it is the step's top,
    /// and the brace ends: the absorb takes over. A step down needs no brace. By hand (Free Ride)
    /// a person braces on their own, so the physics never does.
    fn look_ahead(&mut self, att: &Attitude, rider_off: bool) {
        let c = self.setup.rider.control;
        if !((self.time_s() / TIMESTEP_S).round() as u64).is_multiple_of(BRACE_LOOK_STEPS) {
            return;
        }
        let scripted = self.setup.rider.model.is_articulated() && !self.rider.manual() && c.brace_lead_s > 0.0;
        if !scripted || rider_off || self.held() || att.speed < BRACE_SPEED_MIN_MPS {
            self.absorb.brace = false;
            self.rider.set_brace(false);
            return;
        }
        let axle = self.data.xpos()[self.ids.wheel];
        let [hx, hy] = att.heading;
        let front = self.setup.wheel.tire.radius_m;
        let reach = att.speed * c.brace_lead_s;
        let samples = ((reach / BRACE_SPACING_M).ceil() as usize).clamp(1, BRACE_SAMPLES_MAX);
        let mut last = self.ground_ray_m([axle[0] + hx * front, axle[1] + hy * front]);
        let mut brace = false;
        for k in 1..=samples {
            let d = front + reach * k as f64 / samples as f64;
            let height = self.ground_ray_m([axle[0] + hx * d, axle[1] + hy * d]);
            if let (Some(before), Some(now)) = (last, height)
                && now - before > c.brace_step_m
            {
                brace = true;
                break;
            }
            last = height;
        }
        self.absorb.brace = brace;
        self.rider.set_brace(brace);
    }

    /// Simulated seconds since the model was built.
    pub fn time_s(&self) -> f64 {
        self.data.time()
    }

    /// Suspension compression from full extension (metres) and its rate (m/s, compressing
    /// positive); zero without a suspension.
    pub fn suspension(&self) -> (f64, f64) {
        self.ids.suspension.map_or((0.0, 0.0), |(q, v)| (self.data.qpos()[q], self.data.qvel()[v]))
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
        let lean =
            if self.ids.articulated.is_some() { self.lean } else { -att.pitch + self.data.qpos()[self.ids.lean_qpos] };
        BodyState {
            time_s: self.time_s(),
            speed_mps: att.speed,
            distance_m: self.distance_m,
            pitch_rad: att.pitch,
            roll_rad: att.roll,
            rider_lean_rad: lean,
            wheel_rad_s: self.wheel().1,
            suspension_m: self.suspension().0,
            shock_m: self.setup.wheel.suspension.as_ref().map_or(0.0, |s| s.shock_m(self.suspension().0)),
            motor_torque_nm: self.motor_torque,
            ankle_torque_nm: self.ankle_torque,
            held: self.held(),
            fallen: att.pitch.abs() > FALLEN_RAD || att.roll.abs() > FALLEN_RAD,
        }
    }

    /// The bodies a viewer draws, in the order [`Physics::poses`] reports them.
    pub fn bodies(&self) -> Vec<BodyInfo> {
        let model = self.data.model();
        self.body_ids
            .iter()
            .copied()
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

    /// Bounded sagittal crouch request, radians of knee flexion, 0 to 1.2. No pose or velocity
    /// writes. Zero is the riding stance: straight legs, or the lateral rider's soft knees, from
    /// which 1.2 bends the knees to the same full 1.2 rad. The knees bend toward the request at
    /// [`CROUCH_RATE_RAD_S`] and the hips by [`CROUCH_HIP_SHARE`] of it, keeping the centre of
    /// mass over the feet. This estimate is not a human strength model.
    pub fn set_crouch(&mut self, knee_rad: f64) -> Result<(), ParamError> {
        if self.ids.articulated.is_none()
            || self.rider_off()
            || !knee_rad.is_finite()
            || !(0.0..=CROUCH_MAX_RAD).contains(&knee_rad)
        {
            return Err(ParamError("crouch requires an attached articulated rider and 0..1.2 radians".into()));
        }
        self.crouch_target_rad = knee_rad;
        Ok(())
    }

    /// The crouch the legs hold now, radians of knee flexion (0 for the rigid rider).
    pub const fn crouch_rad(&self) -> f64 {
        self.crouch_rad
    }

    /// Lifts one articulated foot off its pedal by bending that leg: `lift_rad` more hip flexion,
    /// twice that at the knee and the ankle keeping the sole level (0.3 rad lifts it about 4 cm at
    /// the default leg length); 0 puts it back down. One foot at a time, with contact feet and an
    /// attached rider. Internal joint torques only: no pose or velocity writes. While a foot is up
    /// the other leg alone carries the lean.
    pub fn set_foot_lift(&mut self, foot: Foot, lift_rad: f64) -> Result<(), ParamError> {
        let contact = self.ids.articulated.is_some_and(|legs| legs.pairs.is_some());
        let other = self.foot_lift_target_rad[1 - foot.index()].max(self.foot_lift_rad[1 - foot.index()]);
        if !contact
            || self.rider_off()
            || !lift_rad.is_finite()
            || !(0.0..=FOOT_LIFT_MAX_RAD).contains(&lift_rad)
            || (lift_rad > 0.0 && other > 0.0)
        {
            return Err(ParamError(format!(
                "a foot lift needs an attached articulated rider with contact feet, 0..{FOOT_LIFT_MAX_RAD} radians, and the other foot down"
            )));
        }
        self.foot_lift_target_rad[foot.index()] = lift_rad;
        Ok(())
    }

    /// The lateral rider's soft knees, radians (zero for the other models: straight legs).
    fn soft_knee_rad(&self) -> f64 {
        if self.setup.rider.model.has_lateral_legs() { self.setup.rider.articulation.soft_knee_rad } else { 0.0 }
    }

    /// The knees' bend the legs hold now, radians: the stance's, deeper in a turn (the lateral
    /// rider's), and the crouch from there to the deepest crouch.
    fn knee_bend_rad(&self) -> f64 {
        let config = self.setup.rider.articulation;
        let turn = if self.setup.rider.model.has_lateral_legs() {
            config.turn_crouch * self.rider.turn_lean().abs()
        } else {
            0.0
        };
        let bend = turn + self.absorb.level * config.absorb_fold_rad;
        let stance = self.stance_rad + bend;
        stance + self.crouch_rad * (1.0 - (self.soft_knee_rad() + bend) / CROUCH_MAX_RAD).max(0.0)
    }

    /// How far leg `leg`'s knee swings out in all (the lateral rider's), radians: the stance's
    /// swing, and the inside knee's spread from its pad in a turn, up to [`SWING_MAX_RAD`].
    fn swing_rad(&self, leg: usize) -> f64 {
        let spread = self.setup.rider.articulation.turn_spread * outside_lean(1 - leg, self.rider.turn_lean());
        (self.stance.swing_rad[leg] + spread).min(SWING_MAX_RAD)
    }

    /// The posture target of an articulated joint, radians: the knee bend, plus the leg's foot
    /// lift, from the rest stance the model was built in. A standing ankle holds no posture (see
    /// `step`); its target is what it would hold with the body upright and the foot level. The
    /// lateral joints hold the rest stance.
    fn posture_target(&self, index: usize, joint: &LegJoint) -> f64 {
        let lift = self.foot_lift_rad[joint.leg];
        let knee = self.knee_bend_rad();
        let swing = self.swing_rad(joint.leg);
        let share = if self.setup.rider.model.has_lateral_legs() { LATERAL_CROUCH_HIP_SHARE } else { CROUCH_HIP_SHARE };
        joint.rest
            + self.stance.offset_rad[index]
            + match joint.kind {
                JointKind::Hip => -knee * share - lift,
                JointKind::Knee => knee + 2.0 * lift,
                JointKind::Ankle => -knee / 2.0 - lift,
                // The knee swings out: the hip turns out, the tibia in, and the hip abducts a little.
                JointKind::HipRotation => -swing,
                JointKind::KneeRotation => swing,
                JointKind::HipAdduction => -ADDUCTION_PER_SWING * swing,
                JointKind::AnkleRoll => 0.0,
            }
    }

    /// Each articulated foot on its pedal (left, then right); `None` for the rigid rider.
    /// Contact loads come from the last step's constraint forces; positions from the kinematics
    /// at its start.
    pub fn feet(&self) -> Option<[FootState; 2]> {
        let legs = self.ids.articulated?;
        let attached = !self.rider_off();
        let shell = self.data.xmat()[self.ids.shell];
        let deck_normal = [shell[2], shell[5], shell[8]];
        Some(std::array::from_fn(|side| {
            let (sole, pedal) = (legs.soles[side], legs.pedals[side]);
            let at = self.in_shell(self.data.geom_xpos()[sole]);
            let home = self.foot_home[side];
            let offset_m = [at[0] - home[0], at[1] - home[1]];
            let (slip_m, lift_m) = (offset_m[0].hypot(offset_m[1]), at[2] - home[2]);
            let name = ["left", "right"][side];
            if legs.pairs.is_none() {
                return FootState {
                    name,
                    in_contact: attached,
                    normal_n: f64::NAN,
                    friction_n: f64::NAN,
                    slip_m,
                    offset_m,
                    lift_m,
                };
            }
            // Sum the pedal's contact forces on this sole in the world.
            let mut force = [0.0; 3];
            for (index, contact) in self.data.contact().iter().enumerate() {
                let pair = [contact.geom[0] as usize, contact.geom[1] as usize];
                if contact.exclude != 0 || !(pair == [sole, pedal] || pair == [pedal, sole]) {
                    continue;
                }
                let f = self.data.contact_force(index);
                let frame = contact.frame;
                for (axis, value) in force.iter_mut().enumerate() {
                    *value += f[0] * frame[axis] + f[1] * frame[3 + axis] + f[2] * frame[6 + axis];
                }
            }
            let normal = force[0] * deck_normal[0] + force[1] * deck_normal[1] + force[2] * deck_normal[2];
            let along: [f64; 3] = std::array::from_fn(|axis| force[axis] - normal * deck_normal[axis]);
            let normal_n = normal.abs();
            FootState {
                name,
                in_contact: attached && normal_n > FOOT_CONTACT_N,
                normal_n,
                friction_n: (along[0] * along[0] + along[1] * along[1] + along[2] * along[2]).sqrt(),
                slip_m,
                offset_m,
                lift_m,
            }
        }))
    }

    /// Actual joint coordinates and bounded applied motor commands, not decorative limb angles.
    pub fn rider_joints(&self) -> Vec<RiderJointState> {
        self.ids.articulated.map_or_else(Vec::new, |legs| {
            legs.joints()
                .iter()
                .enumerate()
                .map(|(index, joint)| RiderJointState {
                    name: joint.name,
                    posture_target_rad: self.posture_target(index, joint),
                    angle_rad: self.data.qpos()[joint.qpos],
                    rate_rad_s: self.data.qvel()[joint.dof],
                    torque_nm: self.data.ctrl()[joint.motor],
                })
                .collect()
        })
    }

    /// Each welded sole on its pedal (left, then right), as of the last step; `None` without
    /// pedal welds (the rigid rider, contact feet). After the rider leaves, the loads are zero.
    pub fn sole_loads(&self) -> Option<[SoleLoad; 2]> {
        let legs = self.ids.articulated?;
        legs.welds?;
        let attached = !self.rider_off();
        Some(std::array::from_fn(|leg| {
            let normal_n = if attached { self.sole_normal_n[leg] } else { 0.0 };
            let roll_nm = legs
                .joints()
                .iter()
                .find(|j| j.leg == leg && j.kind == JointKind::AnkleRoll)
                .map_or(0.0, |j| self.data.ctrl()[j.motor]);
            SoleLoad { name: ["left", "right"][leg], normal_n, roll_nm, roll_limit_nm: self.sole_roll_limit_nm[leg] }
        }))
    }

    /// Each lateral leg against the shell (left, then right); `None` without leg-on-shell contact
    /// (other riders, a wheel without measured pads). Forces come from the last step's contacts;
    /// gaps from the kinematics at its start. Needs `&mut` for MuJoCo's distance query.
    pub fn leg_pads(&mut self) -> Option<[LegPad; 2]> {
        let legs = self.ids.articulated?;
        if legs.pad_pair_count == 0 {
            return None;
        }
        let force = if self.stance.pads_on { self.leg_shell_forces(&legs) } else { [0.0; 2] };
        let mut gap = [f64::INFINITY; 2];
        for pad in legs.pad_pairs() {
            let distance = self.data.geom_distance(pad.part, pad.pad, LEG_PAD_GAP_MAX_M, None);
            gap[pad.leg] = gap[pad.leg].min(distance);
        }
        Some(std::array::from_fn(|leg| LegPad {
            name: ["left", "right"][leg],
            force_n: force[leg],
            gap_m: gap[leg],
            swing_rad: self.swing_rad(leg),
        }))
    }

    /// A welded sole pulls on its pedal now (by more than a few newtons).
    pub fn sole_pulls(&self) -> bool {
        self.sole_loads().is_some_and(|soles| soles.iter().any(|sole| sole.normal_n < -SOLE_PULL_N))
    }

    /// Where each body of [`Physics::bodies`] is, as of the start of the last step.
    pub fn poses(&self) -> Vec<Pose> {
        self.body_ids
            .iter()
            .map(|&(body, _, _)| Pose {
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
        let up = cross(fwd, left);
        let omega: [f64; 3] = std::array::from_fn(|i| vel[3] * fwd[i] + vel[4] * left[i] + vel[5] * up[i]);
        // The free joint's linear velocity is the shell origin's, in the world; the axle moves
        // along the suspension axis from there: add the shell's rotation and the slide.
        let mut axle_vel = [vel[0], vel[1], vel[2]];
        if let (Some(s), Some((q, v))) = (&self.setup.wheel.suspension, self.ids.suspension) {
            let (q, q_rate) = (self.data.qpos()[q], self.data.qvel()[v]);
            let up = cross(fwd, left);
            let to_world = |l: [f64; 3]| std::array::from_fn(|i| l[0] * fwd[i] + l[1] * left[i] + l[2] * up[i]);
            let axis: [f64; 3] = to_world(s.axis());
            let omega: [f64; 3] = to_world([vel[3], vel[4], vel[5]]);
            let spin = cross(omega, axis.map(|a| q * a));
            axle_vel = std::array::from_fn(|i| axle_vel[i] + spin[i] + q_rate * axis[i]);
        }
        Attitude {
            heading,
            pitch: fwd[2].clamp(-1.0, 1.0).asin(),
            roll: left[2].clamp(-1.0, 1.0).asin(),
            // The free joint's angular velocity is in the shell frame: x roll, y pitch (nose down).
            pitch_down_rate: vel[4],
            roll_rate: vel[3],
            roll_aid_rate: omega[0] * heading[0] + omega[1] * heading[1],
            // The vertical component of that rate in the world: the shell's axes' heights weigh it.
            yaw_rate: vel[3] * fwd[2] + vel[4] * left[2] + vel[5] * cross(fwd, left)[2],
            speed: axle_vel[0] * heading[0] + axle_vel[1] * heading[1],
            ground_speed: axle_vel[0].hypot(axle_vel[1]),
        }
    }
}

/// The mass of wheel, shell and rider, kilograms. The world's mocap bodies (the ground window and
/// the box pool) are fixed in space and left out: at their geoms' default density the window alone
/// would add hundreds of tonnes, and the rider would sense no air drag.
fn moving_mass(model: &MjModel) -> f64 {
    model.body_mass().iter().zip(model.body_mocapid()).filter(|&(_, mocap)| *mocap < 0).map(|(mass, _)| mass).sum()
}

/// Every body with mass in the trees of `roots` (the shell's and the rider's free bodies).
fn machine_bodies(model: &MjModel, roots: [usize; 2]) -> Vec<usize> {
    let trees = roots.map(|root| model.body_rootid()[root]);
    (0..model.body_mass().len())
        .filter(|&body| model.body_mass()[body] > 0.0 && trees.contains(&model.body_rootid()[body]))
        .collect()
}

/// How far a turn's `lean` (radians, right side down positive: a right turn) leans away from
/// leg `leg` (0 left, 1 right), radians: the lean's size for the outside leg, zero for the inside
/// leg and straight on.
fn outside_lean(leg: usize, lean: f64) -> f64 {
    if leg == 0 { lean.max(0.0) } else { (-lean).max(0.0) }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

#[cfg(test)]
mod articulated_tests {
    use super::*;

    fn setup() -> Setup {
        let plant: toml::Value = toml::from_str(include_str!("../../../profiles/begode-extreme/plant.toml")).unwrap();
        let wheel: WheelBody = plant["body"].clone().try_into().unwrap();
        let mut setup = Setup::new(wheel);
        setup.rider.model = RiderModel::ArticulatedSagittalV1;
        setup.rider.cda_m2 = 0.0;
        setup.rider.control.steer_max_nm = 0.0;
        setup.assist = Assist { roll_stiffness: 0.0, roll_damping: 0.0 };
        setup
    }

    fn lateral_setup() -> Setup {
        Setup { rider: Rider { model: RiderModel::ArticulatedLateralV1, ..setup().rider }, ..setup() }
    }

    #[test]
    fn articulated_joint_motors_keep_internal_reactions_in_free_flight() {
        for (setup, joints) in [(setup(), 6), (lateral_setup(), 14)] {
            let mut p = Physics::new(setup).unwrap();
            let legs = p.ids.articulated.unwrap();
            assert_eq!(legs.joints().len(), joints);
            for joint in legs.joints() {
                assert_eq!(p.data.model().actuator_trntype()[joint.motor], MjtTrn::mjTRN_JOINT);
                let joint_id = p.data.model().name_to_id(MjtObj::mjOBJ_JOINT, joint.name).unwrap();
                if joint.kind.pitch() {
                    assert_eq!(p.data.model().jnt_axis()[joint_id], [0.0, 1.0, 0.0]);
                }
            }
            for equality in legs.welds.into_iter().flatten() {
                p.data.eq_active_mut()[equality] = false;
            }
            p.data.qpos_mut()[p.ids.rider_qpos + 2] += 3.0;
            p.data.qvel_mut().fill(0.0);
            p.data.forward();
            p.data.subtree_vel();
            let before = p.data.subtree_angmom()[p.ids.rider];
            let angles = p.rider_joints();
            p.set_crouch(0.5).unwrap();
            for _ in 0..100 {
                p.step(0.0);
            }
            p.data.forward();
            p.data.subtree_vel();
            let after = p.data.subtree_angmom()[p.ids.rider];
            assert!(
                angles.iter().zip(p.rider_joints()).any(|(start, end)| (end.angle_rad - start.angle_rad).abs() > 0.02)
            );
            for axis in 0..3 {
                assert!((after[axis] - before[axis]).abs() < 0.01, "angular momentum {before:?} -> {after:?}");
            }
            println!("free-flight rider angular momentum: before={before:?}, after={after:?} N m s");
            assert_eq!(p.data.xfrc_applied()[p.ids.rider], [0.0; 6]);
        }
    }

    #[test]
    fn articulated_fixed_arms_preserve_the_rigid_neutral_silhouette() {
        let mut setup = setup();
        let articulated = MjModel::from_xml_string(&mjcf::mjcf(&setup)).unwrap();
        setup.rider.model = RiderModel::RigidAnkleV1;
        let rigid = MjModel::from_xml_string(&mjcf::mjcf(&setup)).unwrap();
        for name in ["arm_left", "arm_right"] {
            let arm = articulated.name_to_id(MjtObj::mjOBJ_GEOM, name).expect("upper body needs both fixed arms");
            let reference = rigid.name_to_id(MjtObj::mjOBJ_GEOM, name).unwrap();
            let body = articulated.geom_bodyid()[arm] as usize;
            let rigid_body = rigid.geom_bodyid()[reference] as usize;
            assert_eq!(body, articulated.name_to_id(MjtObj::mjOBJ_BODY, names::RIDER).unwrap());
            assert_eq!(articulated.geom_type()[arm], MjtGeom::mjGEOM_CAPSULE);
            assert_eq!(articulated.geom_size()[arm], rigid.geom_size()[reference]);
            assert_eq!(articulated.geom_quat()[arm], rigid.geom_quat()[reference]);
            for axis in 0..3 {
                let actual = articulated.body_pos()[body][axis] + articulated.geom_pos()[arm][axis];
                let expected = rigid.body_pos()[rigid_body][axis] + rigid.geom_pos()[reference][axis];
                assert!((actual - expected).abs() < 1e-10, "{name} axis {axis}: {actual} != {expected}");
            }
        }
    }

    #[test]
    fn articulated_step_off_push_is_an_internal_force_pair() {
        let mut p = Physics::new(setup()).unwrap();
        p.step_off();
        p.step(0.0);
        let legs = p.ids.articulated.unwrap();
        let (shell, foot) = (p.data.xfrc_applied()[p.ids.shell], p.data.xfrc_applied()[legs.feet[1]]);
        // Equal and opposite forces, and their moments about one point cancel: no external
        // force or moment on wheel and rider together (the rigid rider's kick is external).
        let moment = |body: usize, w: [f64; 6]| {
            let com = p.data.xipos()[body];
            let r = cross(com, [w[0], w[1], w[2]]);
            [r[0] + w[3], r[1] + w[4], r[2] + w[5]]
        };
        let (a, b) = (moment(p.ids.shell, shell), moment(legs.feet[1], foot));
        for axis in 0..3 {
            assert!((shell[axis] + foot[axis]).abs() < 1e-9, "{shell:?} {foot:?}");
            assert!((a[axis] + b[axis]).abs() < 1e-9, "{a:?} {b:?}");
        }
        // The shell feels the stepping foot's push about the heading, as the rigid kick.
        assert!((shell[3] - STEP_OFF_KICK_NM).abs() < 1.0, "{shell:?}");
        assert!(p.rider_joints().iter().all(|joint| joint.torque_nm == 0.0));
        for _ in 0..=((STEP_OFF_KICK_S / TIMESTEP_S) as usize) {
            p.step(0.0);
        }
        assert_eq!(p.data.xfrc_applied()[p.ids.shell], [0.0; 6], "the push lasts STEP_OFF_KICK_S");
        assert_eq!(p.data.xfrc_applied()[legs.feet[1]], [0.0; 6]);
    }

    #[test]
    fn articulated_lean_damping_removes_energy_from_internal_stand_motion() {
        let mut p = Physics::new(setup()).unwrap();
        let neutral = p.data.model().qpos0().to_vec();
        p.data.qpos_mut().copy_from_slice(&neutral);
        p.data.qvel_mut().fill(0.0);
        let legs = p.ids.articulated.unwrap();
        // Each leg closes at the fixed foot: upper -0.02, hip +0.04, ankle -0.02 rad/s.
        // An ankle-only upper-body damper does positive work on this internal motion.
        p.data.qvel_mut()[p.ids.rider_dof + 4] = -0.02;
        p.data.qvel_mut()[p.ids.rider_dof] = 2.0 * p.setup.rider.height_m * 0.245 * 0.02;
        for joint in [legs.joints()[0], legs.joints()[3]] {
            p.data.qvel_mut()[joint.dof] = 0.04;
        }
        for joint in [legs.joints()[2], legs.joints()[5]] {
            p.data.qvel_mut()[joint.dof] = -0.02;
        }
        p.data.forward();
        for name in ["left_foot", "right_foot"] {
            let body = p.data.model().name_to_id(MjtObj::mjOBJ_BODY, name).unwrap();
            let (translation, rotation) = p.data.jac_body(true, true, body);
            for row in translation.chunks_exact(p.data.qvel().len()).chain(rotation.chunks_exact(p.data.qvel().len())) {
                let velocity: f64 = row.iter().zip(p.data.qvel()).map(|(j, v)| j * v).sum();
                assert!(velocity.abs() < 1e-10, "the test motion must obey the fixed-foot constraints");
            }
        }
        let before = p.rider_joints();
        p.rider().set_intent(Intent::Lean(0.0));
        p.step(0.0);
        let mut task_power = 0.0;
        let mut ankle_only_power = 0.0;
        for (start, end) in before.iter().zip(p.rider_joints()) {
            let posture = -p.setup.rider.articulation.joint_kp * start.angle_rad
                - p.setup.rider.articulation.joint_kd * start.rate_rad_s;
            task_power += (end.torque_nm - posture) * start.rate_rad_s;
            if start.name.ends_with("ankle") {
                ankle_only_power += (end.torque_nm - posture) * start.rate_rad_s;
            }
        }
        assert!(ankle_only_power > 0.1, "the omitted hip/knee task moments must expose the old injection");
        assert!(task_power < -0.1, "upper-body damping must remove internal motion energy, got {task_power} W");
        println!(
            "upper-body task power on closed-foot motion: ankle-only={ankle_only_power:.9}, full={task_power:.9} W"
        );
    }

    #[test]
    fn articulated_neutral_stand_does_not_self_excite_without_firmware() {
        let mut p = Physics::new(setup()).unwrap();
        p.rider().set_intent(Intent::Lean(0.0));
        for _ in 0..8000 {
            p.step(0.0);
        }
        let state = p.rider_state();
        assert!(state.measured_lean_rad.abs() < 0.005, "neutral held upper body drift: {state:?}");
        assert!(p.rider_joints().iter().all(|joint| joint.rate_rad_s.abs() < 0.01), "{:?}", p.rider_joints());
    }

    #[test]
    fn articulated_initial_mass_com_and_inertias_are_consistent() {
        for setup in [setup(), lateral_setup()] {
            let model = MjModel::from_xml_string(&mjcf::mjcf(&setup)).unwrap();
            let root = model.name_to_id(MjtObj::mjOBJ_BODY, names::RIDER).unwrap();
            let mut data = MjData::new(Box::new(model));
            data.forward();
            let mass: f64 = (root..data.model().body_mass().len()).map(|i| data.model().body_mass()[i]).sum();
            assert!((mass - setup.rider.mass_kg).abs() < 1e-10);
            let pedal_z = setup.wheel.tire.radius_m - mjcf::sag_m(&setup) - setup.wheel.shell.pedal_below_axle_m;
            let com = data.subtree_com()[root];
            assert!(com[0].abs() < 1e-10 && com[1].abs() < 1e-10, "{com:?}");
            assert!((com[2] - pedal_z - setup.rider.com_above_pedals_m).abs() < 1e-10, "{com:?}");
            for inertia in &data.model().body_inertia()[root..] {
                assert!(inertia.iter().all(|value| value.is_finite() && *value > 0.0));
                assert!(inertia[0] + inertia[1] >= inertia[2]);
                assert!(inertia[1] + inertia[2] >= inertia[0]);
                assert!(inertia[2] + inertia[0] >= inertia[1]);
            }
        }
    }

    /// The lateral rider stands on de Leva's segments, its hips at their anatomical width and its
    /// legs leaning in from the pedals; in the rest stance the hips are abducted and the ankles
    /// inverted by that lean, so the feet are flat on the pedals.
    #[test]
    fn lateral_rider_is_built_in_the_rest_stance_on_de_leva_segments() {
        let setup = lateral_setup();
        let model = MjModel::from_xml_string(&mjcf::mjcf(&setup)).unwrap();
        let body = |name: &str| model.name_to_id(MjtObj::mjOBJ_BODY, name).unwrap();
        let h = setup.rider.height_m;
        let m = setup.rider.mass_kg;
        for (name, segment) in [("thigh", anthropometry::THIGH), ("shank", anthropometry::SHANK)] {
            for side in ["left", "right"] {
                let mass = model.body_mass()[body(&format!("{side}_{name}"))];
                assert!((mass - segment.mass * m).abs() < 1e-9, "{side} {name}: {mass} kg");
            }
        }
        let hip = model.body_pos()[body("left_thigh")];
        assert!((hip[1] - anthropometry::HIP_HALF_WIDTH * h).abs() < 1e-9, "{hip:?}");
        assert!((model.body_pos()[body("right_thigh")][1] + hip[1]).abs() < 1e-12);
        let joint = |name: &str| model.name_to_id(MjtObj::mjOBJ_JOINT, name).unwrap();
        let qpos0 = |name: &str| model.qpos0()[model.jnt_qposadr()[joint(name)] as usize];
        // The legs lean in by asin((pedal - hip) / leg) on the profile's pedals.
        let lean = qpos0("left_ankle_roll");
        assert!(lean > 0.05 && lean < 0.15, "{lean}");
        for side in ["left", "right"] {
            assert!((qpos0(&format!("{side}_hip_adduction")) + lean).abs() < 1e-12);
            assert!((qpos0(&format!("{side}_ankle_roll")) - lean).abs() < 1e-12);
            assert_eq!(qpos0(&format!("{side}_hip_rotation")), 0.0);
        }
        assert_eq!(model.nu(), 14);
    }

    /// The lateral rider's roll aid leans wheel and rider as one rigid machine: in free flight
    /// without gravity, with the legs holding their stance, it rolls the machine and moves no
    /// joint further than the stance alone does, where the same torque on the shell alone rolls
    /// the shell away from the body and bends the legs.
    #[test]
    fn the_lateral_roll_aid_leans_the_machine_and_moves_no_joint() {
        let mut moved = Vec::new();
        for (as_one, torque) in [(true, 0.0), (true, 2000.0), (false, 2000.0)] {
            let mut p = Physics::new(lateral_setup()).unwrap();
            let mut weightless = Box::new(MjModel::clone(p.data.model()));
            weightless.opt_mut().gravity = [0.0; 3];
            p.data.swap_model(weightless);
            p.set_held(false);
            for root in [p.ids.root_qpos, p.ids.rider_qpos] {
                p.data.qpos_mut()[root + 2] += 3.0;
            }
            p.data.forward();
            let legs = p.ids.articulated.unwrap();
            let joints = |p: &Physics| legs.joints().iter().map(|j| p.data.qpos()[j.qpos]).collect::<Vec<_>>();
            let start = joints(&p);
            for _ in 0..200 {
                p.data.step1();
                for (joint, &angle) in legs.joints().iter().zip(&start) {
                    let (q, v) = (p.data.qpos()[joint.qpos], p.data.qvel()[joint.dof]);
                    p.data.ctrl_mut()[joint.motor] = 1000.0 * (angle - q) - 60.0 * v;
                }
                p.data.xfrc_applied_mut().fill([0.0; 6]);
                let [hx, hy] = p.attitude().heading;
                if as_one {
                    p.lean_as_one([hx, hy], torque);
                } else {
                    p.data.xfrc_applied_mut()[p.ids.shell] = [0.0, 0.0, 0.0, torque * hx, torque * hy, 0.0];
                }
                p.data.step2();
            }
            let roll = p.attitude().roll;
            let most = joints(&p).iter().zip(&start).map(|(end, start)| (end - start).abs()).fold(0.0, f64::max);
            println!("{torque} N·m, as one {as_one}: roll {roll:.4} rad, largest joint move {most:.4} rad");
            moved.push((roll, most));
        }
        let [(_, stance), (roll, as_one), (_, shell)] = moved[..] else { unreachable!() };
        assert!(roll > 0.05, "{roll} rad");
        assert!(as_one - stance < 0.002, "{moved:?}");
        assert!(shell > 0.2, "{moved:?}");
    }

    #[test]
    fn the_roll_aid_leans_a_rider_far_back_without_yawing_the_wheel() {
        // A hard brake: the rigid rider leans far back (about 29° here, 31° in the full-effort stop),
        // so the machine's mass lies behind and above the tire's contact. In free flight without
        // gravity, 2000 N·m of roll aid leaned as one about the contact rolls the machine and turns
        // it by almost nothing; the same torque on the shell alone also yaws it.
        let mut turned = Vec::new();
        for as_one in [true, false] {
            let mut setup = setup();
            setup.rider.model = RiderModel::RigidAnkleV1;
            setup.rider.control.lean_max_deg = 35.0;
            setup.rider.control.lean_rate_deg_s = 60.0;
            let mut p = Physics::new(setup).unwrap();
            p.rider().set_intent(Intent::Lean(-0.45));
            for _ in 0..(2.0 / TIMESTEP_S).round() as usize {
                p.step(0.0);
            }
            assert!(p.lean < -0.4, "on the stand the rider leans back: {}", p.lean);
            let mut weightless = Box::new(MjModel::clone(p.data.model()));
            weightless.opt_mut().gravity = [0.0; 3];
            p.data.swap_model(weightless);
            p.set_held(false);
            for root in [p.ids.root_qpos, p.ids.rider_qpos] {
                p.data.qpos_mut()[root + 2] += 3.0;
            }
            p.data.forward();
            let lean = p.data.qpos()[p.ids.lean_qpos];
            for _ in 0..200 {
                p.data.step1();
                let (q, v) = (p.data.qpos()[p.ids.lean_qpos], p.data.qvel()[p.ids.lean_dof]);
                p.data.ctrl_mut()[p.ids.ankle] = 6000.0 * (lean - q) - 900.0 * v;
                p.data.xfrc_applied_mut().fill([0.0; 6]);
                let [hx, hy] = p.attitude().heading;
                if as_one {
                    p.lean_as_one([hx, hy], 2000.0);
                } else {
                    p.data.xfrc_applied_mut()[p.ids.shell] = [0.0, 0.0, 0.0, 2000.0 * hx, 2000.0 * hy, 0.0];
                }
                p.data.step2();
            }
            let att = p.attitude();
            println!("as one {as_one}: roll {:.4} rad, yaw rate {:.4} rad/s", att.roll, att.yaw_rate);
            turned.push((att.roll, att.yaw_rate));
        }
        let [(roll, yaw), (_, shell_yaw)] = turned[..] else { unreachable!() };
        assert!(roll > 0.05, "the machine rolls: {roll} rad");
        assert!(shell_yaw.abs() > 0.1, "a bare torque on the shell yaws it: {shell_yaw} rad/s");
        assert!(yaw.abs() < 0.01 * shell_yaw.abs(), "leaned as one it barely yaws: {yaw} rad/s against {shell_yaw}");
    }
}

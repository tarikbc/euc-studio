//! The MJCF model, generated from the parameters.
//!
//! ```text
//! world ── floor (plane, or terrain)
//!   └─ shell (free joint)     measured solids (or generic packs, handle, pedals), IMU site; origin at
//!        │                    the axle (at full extension, with a suspension)
//!        ├─ carriage (slide "suspension")   only with a suspension: stator, clamps, slider blocks;
//!        │    │                             spring, damper and travel limits on the joint
//!        │    └─ wheel (hinge "axle", y)    tire: the measured torus as a convex mesh (or an
//!        │                                  ellipsoid), on the ground through its own pairs
//!        └─ feet (hinge "lean", y)    hinged at the ankles, ankle motor "ankle"
//!   └─ rider (free joint)     the rider's mass and shapes, welded to the feet ("on_pedals") until
//!                             the rider steps off
//! ```
//!
//! Axes: x forward, y left, z up. Both hinges turn about +y, so a positive wheel angle rolls
//! forward and a positive lean tips the rider forward. Only the tire, the shell and the pedals
//! touch the ground alongside rider shapes. The articulated model (the default) uses a free upper
//! body and six internal hip/knee/ankle hinges; each foot welds to its pedal until release, or,
//! for `articulated_contact_v1`, stands on it through its own contact pair (`<side>_sole_on_pedal`).
//! The lateral model (`articulated_lateral_v1`) has the same bodies with six hinges per leg (hip
//! flexion, ab/adduction and rotation, knee, ankle pitch and roll), de Leva's segments and the
//! hips at their anatomical width, the legs leaning in from the pedals to the hips.

use crate::TIMESTEP_S;
use crate::anthropometry::{self, FOOT, SHANK, Segment, THIGH, range};
use crate::params::{Rider, RiderModel, Setup, Solid, Torus};

/// Names the simulator looks up in the compiled model.
pub(crate) mod names {
    pub const SHELL: &str = "shell";
    pub const WHEEL: &str = "wheel";
    pub const TIRE: &str = "tire";
    pub const CARRIAGE: &str = "carriage";
    pub const SUSPENSION: &str = "suspension";
    pub const RIDER: &str = "rider";
    pub const ROOT: &str = "root";
    pub const AXLE: &str = "axle";
    pub const LEAN: &str = "lean";
    pub const ANKLE: &str = "ankle";
    pub const IMU_ACCEL: &str = "imu_accel";
    pub const IMU_GYRO: &str = "imu_gyro";
    pub const STAND: &str = "stand";
    pub const FEET: &str = "feet";
    pub const RIDER_FREE: &str = "rider_free";
    pub const ON_PEDALS: &str = "on_pedals";
}

/// Largest lean of the rider relative to the pedals, radians (the joint limit).
pub(crate) const LEAN_LIMIT_RAD: f64 = 0.7;

/// The articulated rider's sole-on-pedal contact pairs are named `<side>` + this.
pub(crate) const SOLE_PAIR_SUFFIX: &str = "_sole_on_pedal";

/// Half the width of an articulated sole, metres: how far its centre of pressure can move
/// sideways from the middle of the foot.
pub(crate) const SOLE_HALF_WIDTH_M: f64 = 0.045;

/// The lateral rider's leg-on-shell contact pairs are named `<side>_<leg part>_on_<shell part>`.
pub(crate) const PAD_PAIR_INFIX: &str = "_on_";

/// The parts of a lateral leg that touch the shell: the knee, the shin (with its guard) and the
/// calf (`<side>_knee`, `<side>_shin`, `<side>_calf`).
pub(crate) const LEG_PAD_PARTS: [&str; 3] = ["knee", "shin", "calf"];

/// How a leg gives against the shell (MuJoCo `solimp`): soft at first touch, firm 2 cm in, for
/// the calf's flesh and the guards' foam (ESTIMATE).
const LEG_IMPEDANCE: &str = "0.5 0.95 0.02";

/// Measured shell solids a lateral knee can also reach, besides its side's pads: the front
/// handle's block and the top (seat) box.
const LEG_SHELL_PARTS: [&str; 2] = ["front", "top"];

/// The side pads the lateral rider's legs touch: the measured pad faces of its wheel (none for
/// the other riders, or without measured shapes).
pub(crate) fn leg_pads(setup: &Setup) -> &[Solid] {
    match &setup.wheel.shapes {
        Some(shapes) if setup.rider.model.has_lateral_legs() => &shapes.pads,
        _ => &[],
    }
}

/// A pad face that starts further ahead of the pedal's centre than this, metres, is the cover's
/// front block, above the calf: only the knee and the shin reach it. The owner's report: riding
/// straight, the calf stays behind that block and the knee guard rests on its top corner.
const CALF_REACH_X_M: f64 = 0.05;

/// What lateral leg `leg` (0 left, 1 right) can touch: (leg part, shell geom) pairs. The knee and
/// the shin reach the pads on its side and the front and top solids; the calf reaches the pads
/// on its side except the front block. None without measured pads.
pub(crate) fn leg_pairs(setup: &Setup, leg: usize) -> Vec<(&'static str, &str)> {
    let pads = leg_pads(setup);
    let side = ["_left", "_right"][leg];
    let mut pairs = Vec::new();
    for pad in pads.iter().filter(|pad| pad.name().ends_with(side)) {
        let front = matches!(pad, Solid::Box { centre, half, .. } if centre[0] - half[0] > CALF_REACH_X_M);
        for part in LEG_PAD_PARTS.into_iter().filter(|part| !(front && *part == "calf")) {
            pairs.push((part, pad.name()));
        }
    }
    if let (false, Some(shapes)) = (pads.is_empty(), &setup.wheel.shapes) {
        for solid in shapes.solids.iter().filter(|solid| LEG_SHELL_PARTS.contains(&solid.name())) {
            pairs.extend(["knee", "shin"].map(|part| (part, solid.name())));
        }
    }
    pairs
}

/// What an articulated leg joint does. Signs: hip flexion and ankle dorsiflexion negative, knee
/// flexion positive (all about +y); hip adduction, hip internal rotation and ankle inversion
/// positive on both legs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JointKind {
    /// Hip flexion and extension.
    Hip,
    /// Hip abduction and adduction (lateral legs).
    HipAdduction,
    /// Hip internal and external rotation, about the thigh (lateral legs).
    HipRotation,
    /// Knee flexion.
    Knee,
    /// The tibia's turn about the shank at a bent knee, internal positive (lateral legs).
    KneeRotation,
    /// Ankle dorsiflexion and plantarflexion.
    Ankle,
    /// Ankle inversion and eversion (lateral legs).
    AnkleRoll,
}

impl JointKind {
    /// The pitch joints, which share the upper body's lean task.
    pub(crate) const fn pitch(self) -> bool {
        matches!(self, Self::Hip | Self::Knee | Self::Ankle)
    }

    /// The lateral rider's sideways and twisting joints that hold its stance (not the ankle's
    /// roll, which the legs' shape sets).
    pub(crate) const fn lateral(self) -> bool {
        matches!(self, Self::HipAdduction | Self::HipRotation | Self::KneeRotation)
    }

    /// The motor's torque cap, N·m.
    pub(crate) const fn cap(self, rider: &Rider) -> f64 {
        let a = &rider.articulation;
        match self {
            Self::Hip | Self::Knee => a.joint_max_nm,
            Self::Ankle => rider.control.ankle_max_nm / 2.0,
            Self::HipAdduction => a.hip_adduction_max_nm,
            Self::HipRotation => a.hip_rotation_max_nm,
            Self::KneeRotation => a.knee_rotation_max_nm,
            Self::AnkleRoll => a.ankle_roll_max_nm,
        }
    }
}

/// One articulated leg joint: its name, what it does and its leg (0 left, 1 right).
pub(crate) type LegJoint = (&'static str, JointKind, usize);

const SAGITTAL_JOINTS: [LegJoint; 6] = [
    ("left_hip", JointKind::Hip, 0),
    ("left_knee", JointKind::Knee, 0),
    ("left_ankle", JointKind::Ankle, 0),
    ("right_hip", JointKind::Hip, 1),
    ("right_knee", JointKind::Knee, 1),
    ("right_ankle", JointKind::Ankle, 1),
];

const LATERAL_JOINTS: [LegJoint; 14] = [
    ("left_hip", JointKind::Hip, 0),
    ("left_hip_adduction", JointKind::HipAdduction, 0),
    ("left_hip_rotation", JointKind::HipRotation, 0),
    ("left_knee", JointKind::Knee, 0),
    ("left_knee_rotation", JointKind::KneeRotation, 0),
    ("left_ankle", JointKind::Ankle, 0),
    ("left_ankle_roll", JointKind::AnkleRoll, 0),
    ("right_hip", JointKind::Hip, 1),
    ("right_hip_adduction", JointKind::HipAdduction, 1),
    ("right_hip_rotation", JointKind::HipRotation, 1),
    ("right_knee", JointKind::Knee, 1),
    ("right_knee_rotation", JointKind::KneeRotation, 1),
    ("right_ankle", JointKind::Ankle, 1),
    ("right_ankle_roll", JointKind::AnkleRoll, 1),
];

/// The leg joints of an articulated `model`, left leg then right, each from the hip down
/// (none for the rigid rider).
pub(crate) const fn leg_joints(model: RiderModel) -> &'static [LegJoint] {
    match model {
        RiderModel::RigidAnkleV1 => &[],
        RiderModel::ArticulatedSagittalV1 | RiderModel::ArticulatedContactV1 => &SAGITTAL_JOINTS,
        RiderModel::ArticulatedLateralV1 => &LATERAL_JOINTS,
    }
}

/// The articulated rider stands on the pedals through contact (no pedal welds).
pub(crate) const fn feet_in_contact(setup: &Setup) -> bool {
    setup.rider.model.has_contact_feet()
}

/// Shell layout as fractions of the tire radius (drawing and fall contacts only; mass and
/// inertia come from the parameters).
struct ShellShape {
    /// Battery pack half sizes and centre height.
    pack: [f64; 3],
    pack_z: f64,
    /// Lateral gap between the tread and a pack.
    pack_gap: f64,
    /// Pedal half sizes.
    pedal: [f64; 3],
}

const SHELL_SHAPE: ShellShape =
    ShellShape { pack: [0.75, 0.03, 1.0], pack_z: 0.5, pack_gap: 0.005, pedal: [0.13, 0.055, 0.008] };

/// Shell landmarks derived from the tire and [`SHELL_SHAPE`].
struct Layout {
    pack: [f64; 3],
    pack_y: f64,
    pack_z: f64,
    pedal_y: f64,
}

impl Layout {
    fn new(setup: &Setup) -> Self {
        let r = setup.wheel.tire.radius_m;
        let pack = [SHELL_SHAPE.pack[0] * r, SHELL_SHAPE.pack[1], SHELL_SHAPE.pack[2] * r];
        let pack_y = setup.wheel.tire.half_width_m + SHELL_SHAPE.pack_gap + pack[1];
        // Feet stand on the measured left pedal when there is one.
        let measured_pedal = setup.wheel.shapes.as_ref().and_then(|shapes| {
            shapes.solids.iter().find_map(|solid| match solid {
                Solid::Box { name, centre, .. } if name == "pedal_left" => Some(centre[1].abs()),
                _ => None,
            })
        });
        let pedal_y = measured_pedal.unwrap_or(pack_y + pack[1] + SHELL_SHAPE.pedal[1]);
        Self { pack, pack_y, pack_z: SHELL_SHAPE.pack_z * r, pedal_y }
    }
}

/// Radius of the stator for the carriage's inertia, metres (ESTIMATE; the carriage cannot turn in
/// the shell, so its inertia only adds to the shell's).
const STATOR_RADIUS_M: f64 = 0.08;

/// Time constant of the suspension's end stops (MuJoCo `solreflimit`), seconds: a firm bump stop.
const STOP_TIME_S: f64 = 0.004;

/// The suspension's compression at rest with the rider on the pedals, metres (0 without one).
pub(crate) fn sag_m(setup: &Setup) -> f64 {
    let sprung = setup.wheel.shell.mass_kg + setup.rider.mass_kg;
    setup.wheel.suspension.as_ref().map_or(0.0, |s| s.sag_m(sprung, setup.gravity))
}

/// The torus tire's mesh: the outer half of the tube (the tread side), sampled this many times
/// around the axle and across the tube. MuJoCo collides with its convex hull, which stays within
/// 9 µm of the torus around and 0.5 mm across: the tread rolls smoothly and keeps the measured
/// crown. (A ring of 48 capsules sat 0.36 mm low between capsules and handed its contact from one
/// capsule to the next: the shell shook at 48 times the wheel's turn rate and the tire rolled
/// with about twice the drag; docs/models/body.md.) The hull is built with the model: about
/// 60 ms here, against 160 ms for 25 points across, which roll the same; 240 points around shake
/// a wheel without a suspension half as much again.
const TIRE_MESH_AROUND: usize = 360;
const TIRE_MESH_ACROSS: usize = 13;
/// Capsules that draw the torus tire for viewers (they do not collide).
const TIRE_DRAWN_SEGMENTS: usize = 48;

/// How the tire gives against the ground (MuJoCo `solimp` of its pairs): a constant impedance of
/// 0.5 from the first touch, so `tire.contact_time_s` alone sets its stiffness. At 0.022 s the
/// loaded Extreme sinks 9.5 mm, about 125 kN/m: a 3.00-12 tire at 25 psi (docs/models/body.md).
const TIRE_IMPEDANCE: &str = "0.5 0.5 0.001";
/// Time constant of the tire's grip (MuJoCo `solreffriction` of its pairs), seconds. A soft normal
/// contact would let the tread creep under drive and brake torque (MuJoCo softens friction with
/// the normal); a stiff grip holds it. It needs elliptic friction cones, which the model uses.
const TIRE_GRIP_TIME_S: f64 = 0.002;

/// The tire's spin (torsional) friction, metres, in its pairs' friction for completeness: condim 3
/// leaves it out (see `tire_pairs`).
const TIRE_SPIN_M: f64 = 0.01;

/// The ground geoms the tire can meet: the streaming world's window and pool, a height map's
/// floor, or the strip's floor and its curbs.
fn ground_geoms(setup: &Setup) -> Vec<String> {
    if setup.world.is_some() {
        crate::world::WorldGround::geom_names()
    } else if setup.height_map.is_some() {
        vec!["floor".to_owned()]
    } else {
        setup.terrain.geom_names()
    }
}

/// The tire's contact with each ground geom (`<pair>`): soft along the normal (`contact_time_s`
/// with [`TIRE_IMPEDANCE`]: the tire's give), stiff along the ground ([`TIRE_GRIP_TIME_S`]: no
/// creep), with the tire's grip, spin friction and rolling coefficient (`crr · r`, a length).
/// Condim 3, sliding only. Rolling resistance is a torque `Physics::step` applies from each
/// contact's rolling coefficient (the streaming world scales the pairs' friction per surface).
/// Spin friction stays out: MuJoCo's resists a twist as a pivot would, so a rolling wheel turning
/// at 0.4 rad/s needed an 11.6 N·m twist aid (5 N·m allowed); a rolling tire's patch renews as it
/// turns.
fn tire_pairs(x: &mut Xml, setup: &Setup) {
    let tire = setup.wheel.tire;
    let (mu, rolling) = (tire.friction, tire.crr * tire.radius_m);
    for ground in ground_geoms(setup) {
        x.line(format!(
            r#"    <pair geom1="{}" geom2="{ground}" condim="3" friction="{mu} {mu} {TIRE_SPIN_M} {rolling} {rolling}" solref="{} 1" solimp="{TIRE_IMPEDANCE}" solreffriction="{TIRE_GRIP_TIME_S} 1"/>"#,
            names::TIRE,
            tire.contact_time_s
        ));
    }
}

/// The tire's mesh asset for the measured torus, in the wheel frame (the axle on y): its outer
/// radius is the free tire's (0.2296 m on the Extreme); the soft contact lets it sink to its loaded
/// height.
fn tire_mesh(torus: Torus) -> String {
    use std::f64::consts::{FRAC_PI_2, PI, TAU};
    let (major, minor) = (torus.major_m, torus.minor_m);
    let mut vertices = Vec::with_capacity(TIRE_MESH_AROUND * TIRE_MESH_ACROSS);
    for i in 0..TIRE_MESH_AROUND {
        let around = TAU * i as f64 / TIRE_MESH_AROUND as f64;
        for j in 0..TIRE_MESH_ACROSS {
            let across = -FRAC_PI_2 + PI * j as f64 / (TIRE_MESH_ACROSS - 1) as f64;
            let rho = major + minor * across.cos();
            vertices.push(format!("{:.7} {:.7} {:.7}", rho * around.cos(), minor * across.sin(), rho * around.sin()));
        }
    }
    format!(r#"  <asset><mesh name="tire" vertex="{}"/></asset>"#, vertices.join(" "))
}

/// MJCF text, one element per line.
#[derive(Default)]
struct Xml(String);

impl Xml {
    fn line(&mut self, text: impl AsRef<str>) {
        self.0.push_str(text.as_ref());
        self.0.push('\n');
    }
}

/// The MJCF text for `setup`.
pub(crate) fn mjcf(setup: &Setup) -> String {
    let layout = Layout::new(setup);
    let mut x = Xml::default();
    x.line(r#"<mujoco model="euc">"#);
    x.line(r#"  <compiler angle="radian" autolimits="true"/>"#);
    x.line(format!(
        r#"  <option timestep="{TIMESTEP_S}" gravity="0 0 {}" integrator="implicitfast" cone="elliptic"/>"#,
        -setup.gravity
    ));
    if let Some(torus) = setup.wheel.shapes.as_ref().and_then(|s| s.tire) {
        x.line(tire_mesh(torus));
    }
    if setup.world.is_some() {
        x.line(crate::world::WorldGround::asset());
    } else if let Some(map) = &setup.height_map {
        x.line(map.asset().expect("Setup validates the height map before MJCF generation"));
    } else if let Some(asset) = setup.terrain.asset() {
        x.line(asset);
    }
    x.line("  <worldbody>");
    if setup.world.is_some() {
        // The window and the box pool: mocap bodies the run moves between steps.
        x.line(crate::world::WorldGround::bodies());
    } else {
        x.line(setup.height_map.as_ref().map_or_else(
            || setup.terrain.ground(),
            |map| map.ground().expect("Setup validates the height map before MJCF generation"),
        ));
    }
    // The shell starts where the rider's weight holds it: the axle on the tire, the suspension at
    // its sag.
    x.line(format!(r#"    <body name="{}" pos="0 0 {}">"#, names::SHELL, setup.wheel.tire.radius_m - sag_m(setup)));
    x.line(format!(r#"      <freejoint name="{}"/>"#, names::ROOT));
    shell(&mut x, setup, &layout);
    wheel(&mut x, setup);
    if setup.rider.model == RiderModel::RigidAnkleV1 {
        feet(&mut x, setup);
    }
    x.line("    </body>");
    if setup.rider.model == RiderModel::RigidAnkleV1 {
        rider(&mut x, setup, &layout);
    } else if setup.rider.model.has_lateral_legs() {
        lateral(&mut x, setup, &layout);
    } else {
        articulated(&mut x, setup, &layout);
    }
    x.line("  </worldbody>");
    // The tire runs inside the shell (MuJoCo only skips parent-child pairs, and the carriage sits
    // between them with a suspension).
    x.line("  <contact>");
    x.line(format!(r#"    <exclude body1="{}" body2="{}"/>"#, names::SHELL, names::WHEEL));
    if feet_in_contact(setup) {
        // Each sole meets its own pedal and nothing else of the shell: explicit pairs, no mask
        // change, so torso, legs and wheel still never touch the shell.
        let a = setup.rider.articulation;
        let mu = a.sole_friction;
        for side in ["left", "right"] {
            x.line(format!(
                r#"    <pair name="{side}{SOLE_PAIR_SUFFIX}" geom1="{side}_sole" geom2="pedal_{side}" condim="3" friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{} 1"/>"#,
                a.sole_contact_time_s
            ));
        }
    }
    // Each lateral leg's knee, shin and calf meet the pads on its own side and the front and top
    // solids. The pairs start off (far below any penetration): they come on once the legs stand
    // clear of the pads (`Physics::new`).
    let a = setup.rider.articulation;
    let (mu, off) = (a.pad_friction, crate::RELEASED_MARGIN_M);
    for (leg, side) in ["left", "right"].into_iter().enumerate() {
        for (part, target) in leg_pairs(setup, leg) {
            x.line(format!(
                r#"    <pair name="{side}_{part}{PAD_PAIR_INFIX}{target}" geom1="{side}_{part}" geom2="{target}" condim="3" friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{} 1" solimp="{LEG_IMPEDANCE}" margin="{off}"/>"#,
                a.pad_contact_time_s
            ));
        }
    }
    tire_pairs(&mut x, setup);
    x.line("  </contact>");
    x.line("  <actuator>");
    let max = setup.rider.control.ankle_max_nm;
    if setup.rider.model == RiderModel::RigidAnkleV1 {
        x.line(format!(r#"    <motor name="{}" joint="{}" ctrlrange="-{max} {max}"/>"#, names::ANKLE, names::LEAN));
    } else {
        for (joint, kind, _) in leg_joints(setup.rider.model) {
            let limit = kind.cap(&setup.rider);
            let limited = limit > 0.0;
            x.line(format!(r#"    <motor name="{joint}_motor" joint="{joint}" ctrlrange="-{limit} {limit}" ctrllimited="{limited}"/>"#));
        }
    }
    x.line("  </actuator>");
    x.line("  <sensor>");
    x.line(format!(r#"    <accelerometer name="{}" site="imu"/>"#, names::IMU_ACCEL));
    x.line(format!(r#"    <gyro name="{}" site="imu"/>"#, names::IMU_GYRO));
    x.line("  </sensor>");
    x.line("  <equality>");
    x.line(format!(r#"    <weld name="{}" body1="{}" solref="0.004 1"/>"#, names::STAND, names::SHELL));
    if setup.rider.model == RiderModel::RigidAnkleV1 {
        x.line(format!(
            r#"    <weld name="{}" body1="{}" body2="{}" solref="0.002 1"/>"#,
            names::ON_PEDALS,
            names::RIDER,
            names::FEET
        ));
    } else if !feet_in_contact(setup) {
        for side in ["left", "right"] {
            x.line(format!(r#"    <weld name="on_{side}_pedal" body1="{side}_foot" body2="shell" solref="0.002 1"/>"#));
        }
    }
    x.line("  </equality>");
    x.line("</mujoco>");
    x.0
}

/// The shell's mass, shapes and IMU site (inside the shell body).
fn shell(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let shell = setup.wheel.shell;
    let [ixx, iyy, izz] = shell.inertia_kgm2;
    x.line(format!(
        r#"      <inertial pos="0 0 {}" mass="{}" diaginertia="{ixx} {iyy} {izz}"/>"#,
        shell.com_above_axle_m, shell.mass_kg
    ));
    match setup.wheel.shapes.as_ref().filter(|s| !s.solids.is_empty()) {
        Some(shapes) => {
            for solid in &shapes.solids {
                x.line(match solid {
                    Solid::Box { name, centre: [cx, cy, cz], half: [hx, hy, hz] } => {
                        format!(r#"      <geom name="{name}" type="box" size="{hx} {hy} {hz}" pos="{cx} {cy} {cz}"/>"#)
                    }
                    Solid::Capsule { name, from: [ax, ay, az], to: [bx, by, bz], radius } => format!(
                        r#"      <geom name="{name}" type="capsule" fromto="{ax} {ay} {az} {bx} {by} {bz}" size="{radius}"/>"#
                    ),
                });
            }
        }
        None => generic_shell(x, setup, layout),
    }
    // The side pads touch nothing but the lateral rider's legs, through explicit pairs.
    for pad in leg_pads(setup) {
        if let Solid::Box { name, centre: [cx, cy, cz], half: [hx, hy, hz] } = pad {
            x.line(format!(
                r#"      <geom name="{name}" type="box" size="{hx} {hy} {hz}" pos="{cx} {cy} {cz}" contype="0" conaffinity="0"/>"#
            ));
        }
    }
    x.line(format!(r#"      <site name="imu" pos="0 0 {}" size="0.01"/>"#, shell.imu_above_axle_m));
}

/// Packs, handle and pedals sized from the tire, for a wheel without measured shapes.
fn generic_shell(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let shell = setup.wheel.shell;
    let Layout { pack, pack_y, pack_z, pedal_y } = *layout;
    let pedal = SHELL_SHAPE.pedal;
    for (side, sign) in [("left", 1.0), ("right", -1.0)] {
        x.line(format!(
            r#"      <geom name="pack_{side}" type="box" size="{} {} {}" pos="0 {} {pack_z}"/>"#,
            pack[0],
            pack[1],
            pack[2],
            sign * pack_y
        ));
        x.line(format!(
            r#"      <geom name="pedal_{side}" type="box" size="{} {} {}" pos="0 {} {}"/>"#,
            pedal[0],
            pedal[1],
            pedal[2],
            sign * pedal_y,
            -shell.pedal_below_axle_m - pedal[2]
        ));
    }
    x.line(format!(
        r#"      <geom name="handle" type="box" size="0.06 {} 0.02" pos="0 0 {}"/>"#,
        pack_y + pack[1],
        pack_z + pack[2] + 0.02
    ));
}

/// The spinning wheel and its tire.
fn wheel(x: &mut Xml, setup: &Setup) {
    let tire = setup.wheel.tire;
    let wheel = setup.wheel.wheel;
    let (r, hw) = (tire.radius_m, tire.half_width_m);
    let side = 0.5 * wheel.spin_inertia_kgm2;
    if let Some(s) = &setup.wheel.suspension {
        // The joint counts compression from full extension and the model is built at the sag
        // (`ref`). Spring and damper act through the linkage, whose leverage changes along the
        // travel, so `Physics::step` applies their force; the joint keeps the end stops.
        let sag = sag_m(setup);
        let [ax, ay, az] = s.axis();
        let m = s.unsprung_mass_kg.max(0.01);
        let (flat, axial) = (0.25 * m * STATOR_RADIUS_M * STATOR_RADIUS_M, 0.5 * m * STATOR_RADIUS_M * STATOR_RADIUS_M);
        x.line(format!(r#"      <body name="{}" pos="{} {} {}">"#, names::CARRIAGE, sag * ax, sag * ay, sag * az));
        x.line(format!(
            r#"        <joint name="{}" type="slide" axis="{ax} {ay} {az}" range="0 {}" ref="{sag}" solreflimit="{STOP_TIME_S} 1"/>"#,
            names::SUSPENSION,
            s.travel_m(),
        ));
        x.line(format!(r#"        <inertial pos="0 0 0" mass="{m}" diaginertia="{flat} {axial} {flat}"/>"#));
    }
    x.line(format!(r#"      <body name="{}">"#, names::WHEEL));
    x.line(format!(r#"        <joint name="{}" type="hinge" axis="0 1 0"/>"#, names::AXLE));
    x.line(format!(
        r#"        <inertial pos="0 0 0" mass="{}" diaginertia="{side} {} {side}"/>"#,
        wheel.mass_kg, wheel.spin_inertia_kgm2
    ));
    // The tire meets the ground only through its pairs (`tire_pairs`), and nothing else.
    let alone = r#"contype="0" conaffinity="0""#;
    match setup.wheel.shapes.as_ref().and_then(|s| s.tire) {
        Some(torus) => {
            // The torus as one convex mesh (`tire_mesh`, in the assets): what touches the ground.
            x.line(format!(r#"        <geom name="{}" type="mesh" mesh="tire" {alone}/>"#, names::TIRE));
            // For viewers (`Physics::bodies`, which has no mesh shape): the same torus as a ring of
            // capsules along the tube's centre circle. They never collide; the wheel's inertia is
            // set directly, so they add no mass.
            let major = torus.major_m;
            let point = |k: usize| {
                let angle = std::f64::consts::TAU * k as f64 / TIRE_DRAWN_SEGMENTS as f64;
                (major * angle.cos(), major * angle.sin())
            };
            for k in 0..TIRE_DRAWN_SEGMENTS {
                let ((ax, az), (bx, bz)) = (point(k), point(k + 1));
                x.line(format!(
                    r#"        <geom name="tread_{k}" type="capsule" fromto="{ax} 0 {az} {bx} 0 {bz}" size="{}" contype="0" conaffinity="0"/>"#,
                    torus.minor_m
                ));
            }
        }
        None => {
            x.line(format!(r#"        <geom name="{}" type="ellipsoid" size="{r} {hw} {r}" {alone}/>"#, names::TIRE));
        }
    }
    x.line("      </body>");
    if setup.wheel.suspension.is_some() {
        x.line("      </body>");
    }
}

/// The rigid rider, hinged at the ankles; its shapes are for drawing and never collide.
/// Mass of the feet: the part of the rider that stays on the ankle hinge when the rest steps off.
const FEET_KG: f64 = 2.0;

/// Height of the ankle joint above the axle.
fn ankle_above_axle(setup: &Setup) -> f64 {
    -setup.wheel.shell.pedal_below_axle_m + setup.rider.ankle_above_pedals_m
}

/// The feet on the ankle hinge (inside the shell body): light, they carry the ankle motor; the
/// rider's mass hangs on them through the `on_pedals` weld.
fn feet(x: &mut Xml, setup: &Setup) {
    x.line(format!(r#"      <body name="{}" pos="0 0 {}">"#, names::FEET, ankle_above_axle(setup)));
    x.line(format!(
        r#"        <joint name="{}" type="hinge" axis="0 1 0" range="-{LEAN_LIMIT_RAD} {LEAN_LIMIT_RAD}"/>"#,
        names::LEAN
    ));
    // Enough inertia that the ankle motor turns the feet together with the welded body, not ahead
    // of it (0.5 kg·m², against the rider's 70 or so about the ankles).
    x.line(format!(r#"        <inertial pos="0 0 0" mass="{FEET_KG}" diaginertia="0.5 0.5 0.5"/>"#));
    x.line("      </body>");
}

/// The legacy rider's single body, welded to the ankle proxy until release.
/// Its complete shape set contacts ground, including while attached; it excludes the wheel.
fn rider(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let rider = setup.rider;
    let ankle = rider.ankle_above_pedals_m;
    let mass = rider.mass_kg - FEET_KG;
    let pitch = mass * rider.gyration_m * rider.gyration_m;
    let yaw = mass * 0.15 * 0.15;
    // Body landmarks as fractions of standing height, measured from the ankle joint.
    let h = rider.height_m;
    let (hip, shoulder, head) = (0.53 * h - ankle, 0.82 * h - ankle, 0.93 * h - ankle);
    // Every part of the rider touches the ground (group 2 meets the floor only, never the wheel or
    // the shell): once off the pedals the rider stands, or lies, beside the wheel, and after a fall
    // no limb sinks into the ground.
    let ground_only = r#"contype="2" conaffinity="2" solref="0.005 1""#;
    x.line(format!(
        r#"    <body name="{}" pos="0 0 {}">"#,
        names::RIDER,
        setup.wheel.tire.radius_m - sag_m(setup) + ankle_above_axle(setup)
    ));
    x.line(format!(r#"      <freejoint name="{}"/>"#, names::RIDER_FREE));
    x.line(format!(
        r#"        <inertial pos="0 0 {}" mass="{mass}" diaginertia="{pitch} {pitch} {yaw}"/>"#,
        rider.com_above_pedals_m - ankle,
    ));
    for (side, y) in [("left", layout.pedal_y), ("right", -layout.pedal_y)] {
        x.line(format!(
            r#"        <geom name="foot_{side}" type="box" size="0.12 0.045 0.03" pos="0.03 {y} {}" {ground_only}/>"#,
            0.03 - ankle
        ));
        x.line(format!(
            r#"        <geom name="leg_{side}" type="capsule" fromto="0 {y} 0 0 {} {hip}" size="0.07" {ground_only}/>"#,
            0.6 * y
        ));
        let arm = 1.6 * y;
        x.line(format!(
            r#"        <geom name="arm_{side}" type="capsule" fromto="0 {arm} {shoulder} 0 {arm} {hip}" size="0.045" {ground_only}/>"#
        ));
    }
    x.line(format!(
        r#"        <geom name="torso" type="capsule" fromto="0 0 {hip} 0 0 {shoulder}" size="0.15" {ground_only}/>"#
    ));
    x.line(format!(r#"        <geom name="head" type="sphere" size="0.1" pos="0 0 {head}" {ground_only}/>"#));
    x.line("    </body>");
}

/// Seven real rigid bodies. Each foot welds to its pedal, or for `articulated_contact_v1` stands on
/// it through a friction contact pair. Fractions and dimensions are generic estimates. All rider
/// mass travels with the released tree.
fn articulated(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let r = setup.rider;
    let length = r.height_m * 0.245;
    let ankle_z = setup.wheel.tire.radius_m - sag_m(setup) + ankle_above_axle(setup);
    let pelvis_z = ankle_z + 2.0 * length;
    let upper_mass = r.mass_kg * 0.66;
    // Each foot/shank/thigh uses 2/5/10 percent. Solve the upper COM to preserve total COM.
    let leg_moment = 2.0
        * r.mass_kg
        * (0.02 * 0.03
            + 0.05 * (r.ankle_above_pedals_m + length * 0.5)
            + 0.10 * (r.ankle_above_pedals_m + length * 1.5));
    let upper_com =
        (r.mass_kg * r.com_above_pedals_m - leg_moment) / upper_mass - r.ankle_above_pedals_m - 2.0 * length;
    let upper_x = -2.0 * r.mass_kg * 0.02 * 0.03 / upper_mass;
    let inertia = upper_mass * r.gyration_m.powi(2);
    let ground_only = r#"contype="2" conaffinity="0" solref="0.005 1""#;
    x.line(format!(r#"    <body name="rider" pos="0 0 {pelvis_z}">"#));
    x.line(r#"      <freejoint name="rider_free"/>"#);
    x.line(format!(
        r#"      <inertial mass="{upper_mass}" pos="{upper_x} 0 {upper_com}" diaginertia="{inertia} {inertia} {}"/>"#,
        inertia * 0.5
    ));
    x.line(format!(
        r#"      <geom name="trunk" type="capsule" fromto="0 0 0 0 0 {}" size="0.13" {ground_only}/>"#,
        r.height_m * 0.32
    ));
    x.line(format!(
        r#"      <geom name="head" type="sphere" pos="0 0 {}" size="0.10" {ground_only}/>"#,
        r.height_m - r.ankle_above_pedals_m - 2.0 * length - 0.1
    ));
    for (side, y) in [("left", layout.pedal_y), ("right", -layout.pedal_y)] {
        // Fixed arms retain the legacy silhouette. The explicit upper inertial already includes
        // their mass; these shapes add no bodies, elbow joints, or inferred inertia.
        let arm = 1.6 * y;
        let hip = 0.53 * r.height_m - r.ankle_above_pedals_m - 2.0 * length;
        let shoulder = 0.82 * r.height_m - r.ankle_above_pedals_m - 2.0 * length;
        x.line(format!(
            r#"      <geom name="arm_{side}" type="capsule" fromto="0 {arm} {shoulder} 0 {arm} {hip}" size="0.045" mass="0" {ground_only}/>"#
        ));
        x.line(format!(r#"      <body name="{side}_thigh" pos="0 {y} 0">"#));
        x.line(format!(r#"        <joint name="{side}_hip" axis="0 1 0" range="-1.2 0.8" damping="0.5"/>"#));
        limb(x, side, "thigh", r.mass_kg * 0.10, length, 0.06, ground_only);
        x.line(format!(r#"        <body name="{side}_shank" pos="0 0 -{length}">"#));
        x.line(format!(r#"          <joint name="{side}_knee" axis="0 1 0" range="-0.03 2.2" damping="0.5"/>"#));
        limb(x, side, "shank", r.mass_kg * 0.05, length, 0.045, ground_only);
        x.line(format!(r#"          <body name="{side}_foot" pos="0 0 -{length}">"#));
        // Contact feet: the ankle's posture damping lives in the joint, where the implicit
        // integrator keeps it stable on a foot off its deck (lib.rs leaves it out of the motor).
        let ankle_damping = if feet_in_contact(setup) { 0.5 + setup.rider.articulation.joint_kd } else { 0.5 };
        x.line(format!(
            r#"            <joint name="{side}_ankle" axis="0 1 0" range="-0.9 0.7" damping="{ankle_damping}"/>"#
        ));
        let mass = r.mass_kg * 0.02;
        let z = 0.03 - r.ankle_above_pedals_m;
        let (ix, iy, iz) = (
            mass * (0.045_f64.powi(2) + 0.03_f64.powi(2)) / 3.0,
            mass * (0.12_f64.powi(2) + 0.03_f64.powi(2)) / 3.0,
            mass * (0.12_f64.powi(2) + 0.045_f64.powi(2)) / 3.0,
        );
        x.line(format!(r#"            <inertial mass="{mass}" pos="0.03 0 {z}" diaginertia="{ix} {iy} {iz}"/>"#));
        x.line(format!(
            r#"            <geom name="{side}_sole" type="box" pos="0.03 0 {z}" size="0.12 0.045 0.03" {ground_only}/>"#
        ));
        x.line(
            "          </body>
        </body>
      </body>",
        );
    }
    x.line("    </body>");
}

/// The lateral rider: the articulated rider's seven bodies with de Leva's segments, the hips at
/// their anatomical width and seven hinges per leg. The model is built in the rest stance: legs
/// straight, each leaning in from its foot to its hip, feet flat on the pedals. The joints'
/// `ref` makes their angles anatomical: in the rest stance each hip is abducted and each ankle
/// inverted by the legs' lean. The posture loop bends the knees to the soft riding stance.
fn lateral(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let rider = setup.rider;
    let (height, mass) = (rider.height_m, rider.mass_kg);
    let (thigh, shank) = (THIGH.length * height, SHANK.length * height);
    let hip_y = anthropometry::HIP_HALF_WIDTH * height;
    // Each straight leg runs from its hip down and out to its ankle, over the pedal or just
    // outside its centre.
    let foot_y = layout.pedal_y + rider.articulation.foot_offset_m;
    let tilt = ((foot_y - hip_y) / (thigh + shank)).asin();
    let cos = tilt.cos();
    let ankle_z = setup.wheel.tire.radius_m - sag_m(setup) + ankle_above_axle(setup);
    // The hips' height above the pedals.
    let pelvis = rider.ankle_above_pedals_m + (thigh + shank) * cos;
    let (thigh_kg, shank_kg, foot_kg) = (THIGH.mass * mass, SHANK.mass * mass, FOOT.mass * mass);
    let upper_mass = mass - 2.0 * (thigh_kg + shank_kg + foot_kg);
    // Heights above the pedals of each leg's segment centres; the upper body's centre of mass
    // puts the whole rider's at `com_above_pedals_m`, over the middle of the pedals.
    let leg_moment = 2.0
        * (foot_kg * FOOT_COM_M[1]
            + shank_kg * (rider.ankle_above_pedals_m + (1.0 - SHANK.com) * shank * cos)
            + thigh_kg * (rider.ankle_above_pedals_m + (shank + (1.0 - THIGH.com) * thigh) * cos));
    let upper_com = (mass * rider.com_above_pedals_m - leg_moment) / upper_mass - pelvis;
    let upper_x = -2.0 * foot_kg * FOOT_COM_M[0] / upper_mass;
    let inertia = upper_mass * rider.gyration_m.powi(2);
    let ground_only = r#"contype="2" conaffinity="0" solref="0.005 1""#;
    x.line(format!(r#"    <body name="rider" pos="0 0 {}">"#, ankle_z + (thigh + shank) * cos));
    x.line(r#"      <freejoint name="rider_free"/>"#);
    x.line(format!(
        r#"      <inertial mass="{upper_mass}" pos="{upper_x} 0 {upper_com}" diaginertia="{inertia} {inertia} {}"/>"#,
        inertia * 0.5
    ));
    x.line(format!(
        r#"      <geom name="trunk" type="capsule" fromto="0 0 0 0 0 {}" size="0.13" {ground_only}/>"#,
        height * 0.32
    ));
    x.line(format!(
        r#"      <geom name="head" type="sphere" pos="0 0 {}" size="0.10" {ground_only}/>"#,
        height - pelvis - 0.1
    ));
    let (hip, shoulder) = (0.53 * height - pelvis, 0.82 * height - pelvis);
    for (side, sign) in [("left", 1.0), ("right", -1.0)] {
        // Fixed arms for the silhouette; the upper body's inertial already holds their mass.
        let arm = sign * anthropometry::SHOULDER_HALF_WIDTH * height;
        x.line(format!(
            r#"      <geom name="arm_{side}" type="capsule" fromto="0 {arm} {shoulder} 0 {arm} {hip}" size="0.045" mass="0" {ground_only}/>"#
        ));
        let leg = Leg { side, sign, hip_y, thigh, shank, tilt, masses: [thigh_kg, shank_kg, foot_kg], height };
        lateral_leg(x, &leg, rider.ankle_above_pedals_m, ground_only);
    }
    x.line("    </body>");
}

/// Where a lateral rider's foot carries its mass, from the ankle: forward, and up from the pedal,
/// metres (as the articulated rider's).
const FOOT_COM_M: [f64; 2] = [0.03, 0.03];

/// Where a lateral leg touches the shell, ESTIMATES for an adult man with knee and shin guards
/// (the owner rides with them): each part's span down the shank from the knee (fractions of the
/// shank), how far forward of the knee-to-ankle line it sits, and its half width, metres.
/// The shin with its guard runs from below the knee cap down the front of the upper shank, the
/// calf over the middle of the shank behind the shin bone, the knee is a ball at the joint.
struct LegPart {
    span: [f64; 2],
    forward_m: f64,
    radius_m: f64,
}

const SHIN: LegPart = LegPart { span: [0.05, 0.28], forward_m: 0.01, radius_m: 0.035 };
const CALF: LegPart = LegPart { span: [0.3, 0.75], forward_m: -0.02, radius_m: 0.05 };
const KNEE_RADIUS_M: f64 = 0.05;

/// One lateral leg as built: straight, leaning out from its hip by `tilt` (radians) to its pedal.
struct Leg {
    side: &'static str,
    /// +1 for the left leg, −1 for the right.
    sign: f64,
    /// The hip's distance from the midline, metres.
    hip_y: f64,
    /// Thigh and shank lengths, metres.
    thigh: f64,
    shank: f64,
    tilt: f64,
    /// Thigh, shank and foot masses, kilograms.
    masses: [f64; 3],
    /// The rider's height, metres.
    height: f64,
}

/// A lateral leg's thigh, shank and foot with their seven hinges, inside the upper body.
fn lateral_leg(x: &mut Xml, leg: &Leg, ankle_above_pedals: f64, ground_only: &str) {
    let Leg { side, sign, hip_y, thigh, shank, tilt, masses: [thigh_kg, shank_kg, foot_kg], height } = *leg;
    let (sin, cos) = tilt.sin_cos();
    // Down the leg from hip to knee to ankle, leaning out by `tilt`.
    let along = |length: f64| [0.0, sign * length * sin, -length * cos];
    let [_, knee_y, knee_z] = along(thigh);
    let [_, ankle_y, ankle_z] = along(shank);
    // The segments' principal axes run along the leaning leg: a turn of `sign·tilt` about x.
    let quat = format!("{} {} 0 0", (sign * tilt / 2.0).cos(), (sign * tilt / 2.0).sin());
    let segment = |x: &mut Xml, indent: &str, name: &str, seg: Segment, mass: f64, length: f64, radius: f64| {
        let [ix, iy, iz] = seg.gyration.map(|g| mass * (g * length).powi(2));
        let [_, cy, cz] = along(seg.com * length);
        x.line(format!(
            r#"{indent}<inertial mass="{mass}" pos="0 {cy} {cz}" quat="{quat}" diaginertia="{ix} {iy} {iz}"/>"#
        ));
        let [_, ey, ez] = along(length);
        x.line(format!(
            r#"{indent}<geom name="{side}_{name}_shape" type="capsule" fromto="0 0 0 0 {ey} {ez}" size="{radius}" {ground_only}/>"#
        ));
    };
    // Adduction and inversion turn the foot toward the midline and internal rotation turns the
    // toes in, on either leg; `ref` holds the rest stance's angles.
    let inward = format!("{} 0 0", -sign);
    x.line(format!(r#"      <body name="{side}_thigh" pos="0 {} 0">"#, sign * hip_y));
    x.line(format!(
        r#"        <joint name="{side}_hip" axis="0 1 0" range="-{} {}" damping="0.5"/>"#,
        range::HIP_FLEXION,
        range::HIP_EXTENSION
    ));
    x.line(format!(
        r#"        <joint name="{side}_hip_adduction" axis="{inward}" range="-{} {}" ref="{}" damping="0.5"/>"#,
        range::HIP_ABDUCTION,
        range::HIP_ADDUCTION,
        -tilt
    ));
    let (along_z, rotation) = (-sign * cos, range::HIP_ROTATION);
    x.line(format!(
        r#"        <joint name="{side}_hip_rotation" axis="0 {sin} {along_z}" range="-{rotation} {rotation}" damping="0.5"/>"#
    ));
    segment(x, "        ", "thigh", THIGH, thigh_kg, thigh, 0.065);
    x.line(format!(r#"        <body name="{side}_shank" pos="0 {knee_y} {knee_z}">"#));
    x.line(format!(
        r#"          <joint name="{side}_knee" axis="0 1 0" range="-0.03 {}" damping="0.5"/>"#,
        range::KNEE_FLEXION
    ));
    // The tibia turns about the shank, internal rotation positive on either leg.
    x.line(format!(
        r#"          <joint name="{side}_knee_rotation" axis="0 {sin} {along_z}" range="-{} {}" damping="0.5"/>"#,
        range::KNEE_EXTERNAL_ROTATION,
        range::KNEE_INTERNAL_ROTATION
    ));
    segment(x, "          ", "shank", SHANK, shank_kg, shank, 0.045);
    // What touches the shell: the knee, the shin and the calf. They meet nothing else.
    x.line(format!(r#"          <geom name="{side}_knee" type="sphere" size="{KNEE_RADIUS_M}" contype="0" conaffinity="0" mass="0"/>"#));
    for (name, part) in [("shin", SHIN), ("calf", CALF)] {
        let [_, top_y, top_z] = along(part.span[0] * shank);
        let [_, low_y, low_z] = along(part.span[1] * shank);
        let (forward, radius) = (part.forward_m, part.radius_m);
        x.line(format!(
            r#"          <geom name="{side}_{name}" type="capsule" fromto="{forward} {top_y} {top_z} {forward} {low_y} {low_z}" size="{radius}" contype="0" conaffinity="0" mass="0"/>"#
        ));
    }
    x.line(format!(r#"          <body name="{side}_foot" pos="0 {ankle_y} {ankle_z}">"#));
    // Weight-bearing dorsiflexion goes well past the AAOS 20° (a crouch needs it): the
    // articulated rider's ankle range stays.
    x.line(format!(r#"            <joint name="{side}_ankle" axis="0 1 0" range="-0.9 0.7" damping="0.5"/>"#));
    x.line(format!(
        r#"            <joint name="{side}_ankle_roll" axis="{inward}" range="-{} {}" ref="{tilt}" damping="0.5"/>"#,
        range::ANKLE_EVERSION,
        range::ANKLE_INVERSION
    ));
    let [forward, up] = FOOT_COM_M;
    let sole_z = up - ankle_above_pedals;
    let foot_length = FOOT.length * height;
    let [ix, iy, iz] =
        [FOOT.gyration[2], FOOT.gyration[1], FOOT.gyration[0]].map(|g| foot_kg * (g * foot_length).powi(2));
    x.line(format!(
        r#"            <inertial mass="{foot_kg}" pos="{forward} 0 {sole_z}" diaginertia="{ix} {iy} {iz}"/>"#
    ));
    x.line(format!(
        r#"            <geom name="{side}_sole" type="box" pos="{forward} 0 {sole_z}" size="0.12 {SOLE_HALF_WIDTH_M} 0.03" {ground_only}/>"#
    ));
    x.line(
        "          </body>
        </body>
      </body>",
    );
}

fn limb(x: &mut Xml, side: &str, name: &str, mass: f64, length: f64, radius: f64, contact_attrs: &str) {
    let transverse = mass * (length * length / 12.0 + radius * radius / 4.0);
    let axial = mass * radius * radius / 2.0;
    x.line(format!(
        r#"          <inertial mass="{mass}" pos="0 0 {}" diaginertia="{transverse} {transverse} {axial}"/>"#,
        -length / 2.0
    ));
    x.line(format!(r#"          <geom name="{side}_{name}_shape" type="capsule" fromto="0 0 0 0 0 -{length}" size="{radius}" {contact_attrs}/>"#));
}

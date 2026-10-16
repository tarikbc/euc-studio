//! The MJCF model, generated from the parameters.
//!
//! ```text
//! world ── floor (plane)
//!   └─ shell (free joint)     battery packs, handle, pedals, IMU site; origin at the axle
//!        ├─ wheel (hinge "axle", y)   tire: an ellipsoid round in the wheel plane
//!        └─ rider (hinge "lean", y)   rigid body hinged at the ankles, ankle motor "ankle"
//! ```
//!
//! Axes: x forward, y left, z up. Both hinges turn about +y, so a positive wheel angle rolls
//! forward and a positive lean tips the rider forward. Only the tire, the shell and the pedals
//! touch the ground; the rider has no collision shapes.

use crate::TIMESTEP_S;
use crate::params::Setup;

/// Names the simulator looks up in the compiled model.
pub(crate) mod names {
    pub const SHELL: &str = "shell";
    pub const WHEEL: &str = "wheel";
    pub const RIDER: &str = "rider";
    pub const ROOT: &str = "root";
    pub const AXLE: &str = "axle";
    pub const LEAN: &str = "lean";
    pub const ANKLE: &str = "ankle";
    pub const IMU_ACCEL: &str = "imu_accel";
    pub const IMU_GYRO: &str = "imu_gyro";
    pub const STAND: &str = "stand";
}

/// Largest lean of the rider relative to the pedals, radians (the joint limit).
pub(crate) const LEAN_LIMIT_RAD: f64 = 0.7;

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
        Self { pack, pack_y, pack_z: SHELL_SHAPE.pack_z * r, pedal_y: pack_y + pack[1] + SHELL_SHAPE.pedal[1] }
    }
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
        r#"  <option timestep="{TIMESTEP_S}" gravity="0 0 {}" integrator="implicitfast" cone="pyramidal"/>"#,
        -setup.gravity
    ));
    x.line("  <worldbody>");
    x.line(r#"    <geom name="floor" type="plane" size="0 0 1" friction="0.5 0 0" condim="3"/>"#);
    x.line(format!(r#"    <body name="{}" pos="0 0 {}">"#, names::SHELL, setup.wheel.tire.radius_m));
    x.line(format!(r#"      <freejoint name="{}"/>"#, names::ROOT));
    shell(&mut x, setup, &layout);
    wheel(&mut x, setup);
    rider(&mut x, setup, &layout);
    x.line("    </body>");
    x.line("  </worldbody>");
    x.line("  <actuator>");
    let max = setup.rider.control.ankle_max_nm;
    x.line(format!(r#"    <motor name="{}" joint="{}" ctrlrange="-{max} {max}"/>"#, names::ANKLE, names::LEAN));
    x.line("  </actuator>");
    x.line("  <sensor>");
    x.line(format!(r#"    <accelerometer name="{}" site="imu"/>"#, names::IMU_ACCEL));
    x.line(format!(r#"    <gyro name="{}" site="imu"/>"#, names::IMU_GYRO));
    x.line("  </sensor>");
    x.line("  <equality>");
    x.line(format!(r#"    <weld name="{}" body1="{}" solref="0.004 1"/>"#, names::STAND, names::SHELL));
    x.line("  </equality>");
    x.line("</mujoco>");
    x.0
}

/// The shell's mass, shapes and IMU site (inside the shell body).
fn shell(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let shell = setup.wheel.shell;
    let [ixx, iyy, izz] = shell.inertia_kgm2;
    let Layout { pack, pack_y, pack_z, pedal_y } = *layout;
    let pedal = SHELL_SHAPE.pedal;
    x.line(format!(
        r#"      <inertial pos="0 0 {}" mass="{}" diaginertia="{ixx} {iyy} {izz}"/>"#,
        shell.com_above_axle_m, shell.mass_kg
    ));
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
    x.line(format!(r#"      <site name="imu" pos="0 0 {}" size="0.01"/>"#, shell.imu_above_axle_m));
}

/// The spinning wheel and its tire.
fn wheel(x: &mut Xml, setup: &Setup) {
    let tire = setup.wheel.tire;
    let wheel = setup.wheel.wheel;
    let (r, hw) = (tire.radius_m, tire.half_width_m);
    let side = 0.5 * wheel.spin_inertia_kgm2;
    x.line(format!(r#"      <body name="{}">"#, names::WHEEL));
    x.line(format!(r#"        <joint name="{}" type="hinge" axis="0 1 0"/>"#, names::AXLE));
    x.line(format!(
        r#"        <inertial pos="0 0 0" mass="{}" diaginertia="{side} {} {side}"/>"#,
        wheel.mass_kg, wheel.spin_inertia_kgm2
    ));
    // Rolling friction is a length: resisting torque over normal load, crr times the radius.
    x.line(format!(
        r#"        <geom name="tire" type="ellipsoid" size="{r} {hw} {r}" condim="6" friction="{} 0.01 {}" solref="{} 1"/>"#,
        tire.friction,
        tire.crr * r,
        tire.contact_time_s
    ));
    x.line("      </body>");
}

/// The rigid rider, hinged at the ankles; its shapes are for drawing and never collide.
fn rider(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let rider = setup.rider;
    let ankle = rider.ankle_above_pedals_m;
    let pitch = rider.mass_kg * rider.gyration_m * rider.gyration_m;
    let yaw = rider.mass_kg * 0.15 * 0.15;
    // Body landmarks as fractions of standing height, measured from the ankle joint.
    let h = rider.height_m;
    let (hip, shoulder, head) = (0.53 * h - ankle, 0.82 * h - ankle, 0.93 * h - ankle);
    let ghost = r#"contype="0" conaffinity="0""#;
    x.line(format!(
        r#"      <body name="{}" pos="0 0 {}">"#,
        names::RIDER,
        -setup.wheel.shell.pedal_below_axle_m + ankle
    ));
    x.line(format!(
        r#"        <joint name="{}" type="hinge" axis="0 1 0" range="-{LEAN_LIMIT_RAD} {LEAN_LIMIT_RAD}"/>"#,
        names::LEAN
    ));
    x.line(format!(
        r#"        <inertial pos="0 0 {}" mass="{}" diaginertia="{pitch} {pitch} {yaw}"/>"#,
        rider.com_above_pedals_m - ankle,
        rider.mass_kg
    ));
    for (side, y) in [("left", layout.pedal_y), ("right", -layout.pedal_y)] {
        x.line(format!(
            r#"        <geom name="foot_{side}" type="box" size="0.12 0.045 0.03" pos="0.03 {y} {}" {ghost}/>"#,
            0.03 - ankle
        ));
        x.line(format!(
            r#"        <geom name="leg_{side}" type="capsule" fromto="0 {y} 0 0 {} {hip}" size="0.07" {ghost}/>"#,
            0.6 * y
        ));
        let arm = 1.6 * y;
        x.line(format!(
            r#"        <geom name="arm_{side}" type="capsule" fromto="0 {arm} {shoulder} 0 {arm} {hip}" size="0.045" {ghost}/>"#
        ));
    }
    x.line(format!(
        r#"        <geom name="torso" type="capsule" fromto="0 0 {hip} 0 0 {shoulder}" size="0.15" {ghost}/>"#
    ));
    x.line(format!(r#"        <geom name="head" type="sphere" size="0.1" pos="0 0 {head}" {ghost}/>"#));
    x.line("      </body>");
}

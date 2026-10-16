//! The MJCF model, generated from the parameters.
//!
//! ```text
//! world ── floor (plane, or terrain)
//!   └─ shell (free joint)     measured solids (or generic packs, handle, pedals), IMU site; origin at the axle
//!        ├─ wheel (hinge "axle", y)   tire: a torus as a ring of capsules (or an ellipsoid)
//!        └─ feet (hinge "lean", y)    hinged at the ankles, ankle motor "ankle"
//!   └─ rider (free joint)     the rider's mass and shapes, welded to the feet ("on_pedals") until
//!                             the rider steps off
//! ```
//!
//! Axes: x forward, y left, z up. Both hinges turn about +y, so a positive wheel angle rolls
//! forward and a positive lean tips the rider forward. Only the tire, the shell and the pedals
//! touch the ground; the rider has no collision shapes.

use crate::TIMESTEP_S;
use crate::params::{Setup, Solid};

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
    pub const FEET: &str = "feet";
    pub const RIDER_FREE: &str = "rider_free";
    pub const ON_PEDALS: &str = "on_pedals";
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

/// Capsules in the ring that stands in for a torus: 48 keeps the radius within 0.4 mm.
const TORUS_SEGMENTS: usize = 48;

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
    if let Some(asset) = setup.terrain.asset() {
        x.line(asset);
    }
    x.line("  <worldbody>");
    x.line(setup.terrain.ground());
    x.line(format!(r#"    <body name="{}" pos="0 0 {}">"#, names::SHELL, setup.wheel.tire.radius_m));
    x.line(format!(r#"      <freejoint name="{}"/>"#, names::ROOT));
    shell(&mut x, setup, &layout);
    wheel(&mut x, setup);
    feet(&mut x, setup);
    x.line("    </body>");
    rider(&mut x, setup, &layout);
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
    x.line(format!(
        r#"    <weld name="{}" body1="{}" body2="{}" solref="0.002 1"/>"#,
        names::ON_PEDALS,
        names::RIDER,
        names::FEET
    ));
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
    x.line(format!(r#"      <body name="{}">"#, names::WHEEL));
    x.line(format!(r#"        <joint name="{}" type="hinge" axis="0 1 0"/>"#, names::AXLE));
    x.line(format!(
        r#"        <inertial pos="0 0 0" mass="{}" diaginertia="{side} {} {side}"/>"#,
        wheel.mass_kg, wheel.spin_inertia_kgm2
    ));
    // Rolling friction is a length: resisting torque over normal load, crr times the radius.
    let contact =
        format!(r#"condim="6" friction="{} 0.01 {}" solref="{} 1""#, tire.friction, tire.crr * r, tire.contact_time_s);
    match setup.wheel.shapes.as_ref().and_then(|s| s.tire) {
        Some(torus) => {
            // A ring of capsules along the tube's centre circle in the wheel plane, sized so the
            // tire stands on its loaded rolling radius (MuJoCo's contact barely sinks in).
            let major = r - torus.minor_m;
            let point = |k: usize| {
                let angle = std::f64::consts::TAU * k as f64 / TORUS_SEGMENTS as f64;
                (major * angle.cos(), major * angle.sin())
            };
            for k in 0..TORUS_SEGMENTS {
                let ((ax, az), (bx, bz)) = (point(k), point(k + 1));
                x.line(format!(
                    r#"        <geom name="tire_{k}" type="capsule" fromto="{ax} 0 {az} {bx} 0 {bz}" size="{}" {contact}/>"#,
                    torus.minor_m
                ));
            }
        }
        None => {
            x.line(format!(r#"        <geom name="tire" type="ellipsoid" size="{r} {hw} {r}" {contact}/>"#));
        }
    }
    x.line("      </body>");
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

/// The rider's body (in the world, welded to the feet): mass, inertia and shapes. Only the torso
/// touches the ground, and only once the rider has stepped off and falls.
fn rider(x: &mut Xml, setup: &Setup, layout: &Layout) {
    let rider = setup.rider;
    let ankle = rider.ankle_above_pedals_m;
    let mass = rider.mass_kg - FEET_KG;
    let pitch = mass * rider.gyration_m * rider.gyration_m;
    let yaw = mass * 0.15 * 0.15;
    // Body landmarks as fractions of standing height, measured from the ankle joint.
    let h = rider.height_m;
    let (hip, shoulder, head) = (0.53 * h - ankle, 0.82 * h - ankle, 0.93 * h - ankle);
    let ghost = r#"contype="0" conaffinity="0""#;
    // The feet and the torso touch the ground (group 2 meets the floor only, never the wheel or the
    // shell): once off the pedals the rider stands, or lies, beside the wheel.
    let ground_only = r#"contype="2" conaffinity="2""#;
    x.line(format!(
        r#"    <body name="{}" pos="0 0 {}">"#,
        names::RIDER,
        setup.wheel.tire.radius_m + ankle_above_axle(setup)
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
            r#"        <geom name="leg_{side}" type="capsule" fromto="0 {y} 0 0 {} {hip}" size="0.07" {ghost}/>"#,
            0.6 * y
        ));
        let arm = 1.6 * y;
        x.line(format!(
            r#"        <geom name="arm_{side}" type="capsule" fromto="0 {arm} {shoulder} 0 {arm} {hip}" size="0.045" {ghost}/>"#
        ));
    }
    x.line(format!(
        r#"        <geom name="torso" type="capsule" fromto="0 0 {hip} 0 0 {shoulder}" size="0.15" {ground_only}/>"#
    ));
    x.line(format!(r#"        <geom name="head" type="sphere" size="0.1" pos="0 0 {head}" {ghost}/>"#));
    x.line("    </body>");
}

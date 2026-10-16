//! The model on its own, balanced by a plain PD law on the motor torque instead of firmware.

use euc_physics::{
    Intent, Leaving, Physics, Setup, Shapes, Shell, Suspension, TIMESTEP_S, Tire, Torus, Wheel, WheelBody,
};

/// The Begode Extreme's `[body]` values without its suspension (the axle fixed in the shell, the
/// stator's mass in the shell), with the scanned tire torus.
fn extreme() -> WheelBody {
    WheelBody {
        tire: Tire { radius_m: 0.2251, half_width_m: 0.0407, friction: 0.9, crr: 0.012, contact_time_s: 0.02 },
        wheel: Wheel { mass_kg: 16.0, spin_inertia_kgm2: 0.487 },
        shell: Shell {
            mass_kg: 26.0,
            com_above_axle_m: 0.10,
            inertia_kgm2: [0.85, 0.92, 0.46],
            pedal_below_axle_m: -0.012,
            imu_above_axle_m: 0.15,
        },
        suspension: None,
        shapes: Some(Shapes { tire: Some(Torus { major_m: 0.1716, minor_m: 0.058 }), solids: Vec::new() }),
    }
}

/// The Begode Extreme with its rear suspension (as in profiles/begode-extreme/plant.toml): the
/// stator rides on the carriage.
fn extreme_suspended() -> WheelBody {
    let rigid = extreme();
    WheelBody {
        shell: Shell { mass_kg: 20.0, com_above_axle_m: 0.13, inertia_kgm2: [0.76, 0.82, 0.45], ..rigid.shell },
        suspension: Some(Suspension {
            unsprung_mass_kg: 6.0,
            axis_forward_deg: -0.13,
            spring_rate_n_per_mm: 131.3,
            preload_mm: 2.0,
            stroke_mm: 55.0,
            eye_to_eye_mm: 200.0,
            linkage_ratio: 1.2,
            ratio_curve: Vec::new(),
            damping_n_s_per_m: 2200.0,
        }),
        ..rigid
    }
}

/// A balance law in place of firmware: torque against pitch and pitch rate (nose down drives
/// forward), as a self-balancing controller does.
fn balance(physics: &Physics) -> f64 {
    let state = physics.state();
    let gyro_pitch_down = physics.imu().gyro_rad_s[1];
    (-2500.0 * state.pitch_rad + 250.0 * gyro_pitch_down).clamp(-300.0, 300.0)
}

fn run(physics: &mut Physics, seconds: f64) {
    for _ in 0..(seconds / TIMESTEP_S).round() as usize {
        let torque = if physics.held() { 0.0 } else { balance(physics) };
        physics.step(torque);
    }
}

#[test]
fn stands_on_the_stand_with_the_imu_reading_one_g_up() {
    let mut physics = Physics::new(Setup::new(extreme())).unwrap();
    let start = physics.imu();
    assert!((start.accel_mps2[2] - 9.81).abs() < 1e-6, "{start:?}");
    run(&mut physics, 1.0);
    let state = physics.state();
    assert!(state.held && !state.fallen);
    assert!(state.pitch_rad.abs() < 1e-3 && state.speed_mps.abs() < 1e-3, "{state:?}");
    let imu = physics.imu();
    assert!(imu.accel_mps2[0].abs() < 0.05 && (imu.accel_mps2[2] - 9.81).abs() < 0.05, "{imu:?}");
    assert_eq!(physics.bodies().len(), physics.poses().len());
}

#[test]
fn a_balanced_wheel_carries_the_rider_to_speed_and_back() {
    let mut physics = Physics::new(Setup::new(extreme())).unwrap();
    run(&mut physics, 0.5);
    physics.set_held(false);
    run(&mut physics, 5.0);
    let standing = physics.state();
    assert!(!standing.fallen && standing.pitch_rad.abs() < 0.05, "stands: {standing:?}");
    assert!(standing.speed_mps.abs() < 0.3, "stays put: {standing:?}");

    physics.rider().set_intent(Intent::Speed(20.0 / 3.6));
    run(&mut physics, 12.0);
    let cruising = physics.state();
    assert!(!cruising.fallen, "{cruising:?}");
    assert!((cruising.speed_mps * 3.6 - 20.0).abs() < 1.0, "reaches 20 km/h: {cruising:?}");
    assert!(cruising.rider_lean_rad > 0.0, "leans into the drag: {cruising:?}");
    // Nose-down specific force while it pushes against drag: the IMU's x reads the frame's pitch
    // and acceleration only, so it stays small at a steady speed.
    assert!(physics.imu().accel_mps2[0].abs() < 1.5, "{:?}", physics.imu());

    physics.rider().set_intent(Intent::Speed(0.0));
    run(&mut physics, 10.0);
    let stopped = physics.state();
    assert!(!stopped.fallen && stopped.speed_mps.abs() < 0.3, "stops: {stopped:?}");
    assert!(stopped.distance_m > 30.0, "went somewhere: {stopped:?}");
    assert!(stopped.roll_rad.abs() < 0.01, "the roll spring keeps it upright: {stopped:?}");
    assert!(!physics.rider_off(), "a rider on a wheel that holds never lets go");
}

#[test]
fn a_rider_who_steps_off_lands_beside_the_wheel() {
    // How upright the rider's body is: the z component of its z axis (1 standing, 0 lying).
    let upright = |physics: &Physics| {
        let [_, x, y, _] = physics.poses()[2].quat;
        1.0 - 2.0 * (x * x + y * y)
    };
    let mut physics = Physics::new(Setup::new(extreme())).unwrap();
    run(&mut physics, 0.5);
    physics.set_held(false);
    run(&mut physics, 2.0);
    assert!(upright(&physics) > 0.99 && !physics.rider_off(), "the rider stands on the pedals");
    physics.step_off();
    // No motor torque: the wheel is left to itself.
    for _ in 0..(3.0 / TIMESTEP_S) as usize {
        physics.step(0.0);
    }
    assert!(physics.rider_off());
    assert_eq!(physics.state().ankle_torque_nm, 0.0);
    // The ankles drop from the pedals (0.31 m) to just above the ground: the rider stands on it.
    let ankles = physics.poses()[2].pos[2];
    assert!((0.05..0.15).contains(&ankles), "the rider stands on the ground: ankles {ankles} m up");
}

#[test]
fn the_suspension_sits_at_its_sag_and_carries_the_rider() {
    let setup = Setup::new(extreme_suspended());
    let suspension = setup.wheel.suspension.clone().unwrap();
    let sag = suspension.sag_m(setup.wheel.shell.mass_kg + setup.rider.mass_kg, setup.gravity);
    assert!(sag > 0.005 && sag < 0.5 * suspension.travel_m(), "sag {sag} m");
    let mut physics = Physics::new(setup).unwrap();
    // On the stand the shell is held at the sag over an unloaded tire; the spring presses the tire
    // into the ground (the tire's own give), so the suspension extends a little until released.
    run(&mut physics, 0.5);
    let (held, _) = physics.suspension();
    assert!(held > 0.5 * sag && held <= sag + 1e-4, "held at {held} m, sag {sag} m");
    physics.set_held(false);
    run(&mut physics, 5.0);
    let standing = physics.state();
    assert!(!standing.fallen && standing.speed_mps.abs() < 0.3, "stands: {standing:?}");
    assert!((standing.suspension_m - sag).abs() < 0.002, "rests at the sag: {standing:?}");
    let imu = physics.imu();
    assert!((imu.accel_mps2[2] - 9.81).abs() < 0.1, "the IMU feels 1 g: {imu:?}");

    physics.rider().set_intent(Intent::Speed(20.0 / 3.6));
    run(&mut physics, 12.0);
    let cruising = physics.state();
    assert!(!cruising.fallen, "{cruising:?}");
    assert!((cruising.speed_mps * 3.6 - 20.0).abs() < 1.0, "reaches 20 km/h: {cruising:?}");
    assert!(
        cruising.suspension_m > 0.0 && cruising.suspension_m < suspension.travel_m(),
        "within the travel: {cruising:?}"
    );
    assert_eq!(physics.bodies().len(), physics.poses().len());
}

/// `vector` turned by the unit quaternion `quat` (`[w, x, y, z]`).
fn turn(quat: [f64; 4], vector: [f64; 3]) -> [f64; 3] {
    let [w, axis @ ..] = quat;
    let twice = cross(axis, vector).map(|c| 2.0 * c);
    let second = cross(axis, twice);
    std::array::from_fn(|i| vector[i] + w * twice[i] + second[i])
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn product(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// The lowest point of the rider's shapes, metres above flat ground.
fn rider_lowest_m(physics: &Physics) -> f64 {
    use euc_telemetry::GeomKind;
    let pose = &physics.poses()[2];
    let (pos, quat) = (pose.pos.map(f64::from), pose.quat.map(f64::from));
    physics.bodies()[2]
        .geoms
        .iter()
        .map(|g| {
            let centre = turn(quat, g.pos.map(f64::from));
            let q = product(quat, g.quat.map(f64::from));
            let [a, b, c] = g.size.map(f64::from);
            let low = match g.kind {
                GeomKind::Sphere => a,
                GeomKind::Capsule => turn(q, [0.0, 0.0, b])[2].abs() + a,
                GeomKind::Box => {
                    (0..3).map(|i| turn(q, std::array::from_fn(|j| [a, b, c][j] * f64::from(i == j)))[2].abs()).sum()
                }
                _ => 0.0,
            };
            pos[2] + centre[2] - low
        })
        .fold(f64::INFINITY, f64::min)
}

/// Where the rider's body sits in the shell's frame, metres: fixed while the rider is on the pedals.
fn rider_in_shell(physics: &Physics) -> [f64; 3] {
    let poses = physics.poses();
    let (shell, rider) = (&poses[1], &poses[2]);
    let [w, x, y, z] = shell.quat.map(f64::from);
    turn([w, -x, -y, -z], std::array::from_fn(|i| f64::from(rider.pos[i] - shell.pos[i])))
}

#[test]
fn the_rider_stays_on_the_pedals_and_never_sinks_into_the_ground() {
    let mut physics = Physics::new(Setup::new(extreme_suspended())).unwrap();
    run(&mut physics, 0.5);
    physics.set_held(false);
    let on_pedals = rider_in_shell(&physics);
    physics.rider().set_intent(Intent::Speed(15.0 / 3.6));
    for _ in 0..(10.0 / TIMESTEP_S) as usize {
        physics.step(balance(&physics));
        let at = rider_in_shell(&physics);
        let drift = (0..3).map(|i| (at[i] - on_pedals[i]).abs()).fold(0.0, f64::max);
        assert!(drift < 0.002, "the weld holds the rider on the pedals: moved {drift} m");
    }
    assert!(!physics.state().fallen && physics.state().speed_mps > 3.5, "{:?}", physics.state());
    assert!(rider_lowest_m(&physics) > 0.15, "the feet ride on the pedals");
    // The motor stops at 15 km/h: wheel and rider tumble. The rider's shapes may press into the soft
    // ground for a moment at an impact, but none stays in it.
    let (mut lowest, mut pressed_s) = (f64::INFINITY, 0.0);
    for _ in 0..(4.0 / TIMESTEP_S) as usize {
        physics.step(0.0);
        let low = rider_lowest_m(&physics);
        lowest = lowest.min(low);
        if low < -0.01 {
            pressed_s += TIMESTEP_S;
        }
    }
    assert!(physics.state().fallen, "{:?}", physics.state());
    let resting = rider_lowest_m(&physics);
    assert!(resting > -0.01, "a fallen rider lies on the ground, not in it: {resting} m");
    assert!(lowest > -0.05 && pressed_s < 0.15, "impacts press in {lowest} m, for {pressed_s} s in all");
}

#[test]
fn the_rider_lets_go_when_the_drive_is_cut_and_the_wheel_tips() {
    let mut physics = Physics::new(Setup::new(extreme_suspended())).unwrap();
    run(&mut physics, 0.5);
    physics.set_held(false);
    physics.rider().set_intent(Intent::Speed(15.0 / 3.6));
    run(&mut physics, 8.0);
    assert!(!physics.rider_off() && physics.state().speed_mps > 3.5, "{:?}", physics.state());
    // The firmware cuts the drive: no torque, and the session says so.
    let cut_at = physics.time_s();
    physics.set_drive_cut(true);
    for _ in 0..(3.0 / TIMESTEP_S) as usize {
        physics.step(0.0);
    }
    let Some((at, how)) = physics.left_pedals() else { panic!("still on the pedals: {:?}", physics.state()) };
    assert_eq!(how, Leaving::LetGoAfterCut);
    // With no torque at all the wheel tips past 20° about 0.24 s after the cut; the rider reacts 0.1 s later.
    assert!(at - cut_at < 0.4, "lets go {} s after the cut", at - cut_at);
    assert_eq!(physics.state().ankle_torque_nm, 0.0);
    let resting = rider_lowest_m(&physics);
    assert!(resting > -0.01, "the rider lies on the ground, not in it: {resting} m");
}

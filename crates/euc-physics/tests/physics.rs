//! The model on its own, balanced by a plain PD law on the motor torque instead of firmware.

use euc_physics::{Intent, Physics, Setup, Shell, TIMESTEP_S, Tire, Wheel, WheelBody};

/// The Begode Extreme's `[body]` values (as in profiles/begode-extreme/plant.toml).
fn extreme() -> WheelBody {
    WheelBody {
        tire: Tire { radius_m: 0.209, half_width_m: 0.038, friction: 0.9, crr: 0.012, contact_time_s: 0.02 },
        wheel: Wheel { mass_kg: 16.0, spin_inertia_kgm2: 0.42 },
        shell: Shell {
            mass_kg: 26.0,
            com_above_axle_m: 0.10,
            inertia_kgm2: [0.85, 0.92, 0.46],
            pedal_below_axle_m: 0.03,
            imu_above_axle_m: 0.15,
        },
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
}

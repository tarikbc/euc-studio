//! The torus tire as rubber at 25 psi: one convex mesh, so the shell does not shake at a multiple
//! of the wheel's turn rate (the ring of 48 capsules it replaced shook the IMU at 48 times the turn
//! rate); it sinks into the ground as the owner's tire does and rolls with `crr · N · r`
//! (docs/models/body.md, "Tire"). Balanced by the plain PD law of tests/physics.rs instead of
//! firmware; no firmware image or hardware.

use euc_physics::{Intent, Physics, Setup, Shapes, TIMESTEP_S, Torus, WheelBody};

/// The Begode Extreme's `[body]` from its profile with the scanned tire torus; `suspension` keeps
/// the profile's suspension, else the axle sits in the shell (as tests/turning.rs).
fn extreme(suspension: bool) -> Setup {
    let plant: toml::Value = toml::from_str(include_str!("../../../profiles/begode-extreme/plant.toml")).unwrap();
    let mut body: WheelBody = plant["body"].clone().try_into().unwrap();
    body.shapes = Some(Shapes { tire: Some(Torus { major_m: 0.1716, minor_m: 0.058 }), ..Shapes::default() });
    if !suspension {
        body.suspension = None;
    }
    Setup::new(body)
}

/// Motor torque that keeps the pedals level (nothing while the stand holds the wheel).
fn balance(physics: &Physics) -> f64 {
    if physics.held() {
        return 0.0;
    }
    let state = physics.state();
    (-2500.0 * state.pitch_rad + 250.0 * physics.imu().gyro_rad_s[1]).clamp(-300.0, 300.0)
}

fn sd(values: &[f64]) -> f64 {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    (values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64).sqrt()
}

/// A steady ride at `kmh` on flat ground: 0.5 s on the stand, then 12.5 s riding. Over the last
/// 5 s: the spread of the IMU's accelerometer x and z (m/s², standard deviation over every step),
/// the mean motor torque that holds the speed (N·m) and the mean axle height (m).
fn steady(setup: Setup, kmh: f64) -> ([f64; 2], f64, f64) {
    let mut physics = Physics::new(setup).unwrap();
    let release = (0.5 / TIMESTEP_S).round() as usize;
    let steps = (13.0 / TIMESTEP_S).round() as usize;
    let from = steps - (5.0 / TIMESTEP_S).round() as usize;
    let (mut x, mut z, mut torque, mut axle) = (Vec::new(), Vec::new(), 0.0, 0.0);
    for step in 0..steps {
        if step == release {
            physics.set_held(false);
            physics.rider().set_intent(Intent::Speed(kmh / 3.6));
        }
        let drive = balance(&physics);
        physics.step(drive);
        if step >= from {
            let imu = physics.imu();
            x.push(imu.accel_mps2[0]);
            z.push(imu.accel_mps2[2]);
            torque += drive;
            axle += f64::from(physics.poses()[0].pos[2]);
        }
    }
    let state = physics.state();
    assert!(!state.fallen && (state.speed_mps * 3.6 - kmh).abs() < 1.0, "{state:?}");
    let n = (steps - from) as f64;
    ([sd(&x), sd(&z)], torque / n, axle / n)
}

/// At 15 km/h the ring shook the IMU at 145 Hz: accel x 0.43 and z 0.16 m/s² with the profile's
/// suspension, z 9.8 m/s² without one (the pedal welds then swung by ±250 N). On the mesh:
/// x 0.07 and z 0.02 with the suspension.
#[test]
fn the_tire_rolls_without_shaking_the_shell() {
    let ([x, z], _, _) = steady(extreme(true), 15.0);
    println!("15 km/h with the suspension: accel x sd {x:.3}, z sd {z:.3} m/s²");
    assert!(x < 0.2 && z < 0.1, "x {x}, z {z}");
    let ([x, z], _, _) = steady(extreme(false), 15.0);
    println!("15 km/h without a suspension: accel x sd {x:.3}, z sd {z:.3} m/s²");
    assert!(x < 0.3 && z < 3.0, "x {x}, z {z}");
}

/// A 3.00-12 tire at the owner's 25 psi is about 125 kN/m (`2π · p · √(R · r_crown)`): the loaded
/// Extreme sinks about 9.6 mm from the free radius, 0.2296 m.
#[test]
fn the_tire_sinks_as_a_tire_at_25_psi() {
    let (_, _, axle) = steady(extreme(true), 5.0);
    let sink = 0.2296 - axle;
    println!("axle {axle:.4} m: the tire sinks {:.1} mm", sink * 1e3);
    assert!((0.008..0.011).contains(&sink), "sink {sink} m");
}

/// Rolling resistance is `crr · N · r` against the spin, either way: at 5 km/h on flat ground it is
/// almost all of the holding torque (the air adds about 0.1 N·m), and riding backwards it turns.
#[test]
fn the_tire_rolls_with_its_rolling_resistance() {
    let setup = extreme(true);
    let tire = setup.wheel.tire;
    let weight = Physics::new(setup.clone()).unwrap().total_mass() * setup.gravity;
    let rolling = tire.crr * weight * tire.radius_m;
    let (_, forward, _) = steady(setup.clone(), 5.0);
    let (_, backward, _) = steady(setup, -5.0);
    println!("holding torque {forward:.2} N·m at 5 km/h, {backward:.2} at -5 km/h; crr · N · r = {rolling:.2} N·m");
    assert!(forward > rolling && forward < rolling + 0.4, "{forward} N·m against {rolling} N·m");
    assert!(backward < -rolling && backward > -rolling - 0.4, "{backward} N·m against {rolling} N·m");
}

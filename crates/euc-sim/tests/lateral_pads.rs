//! The lateral rider's legs on the Begode Extreme's measured side pads (`articulated_lateral_v1`,
//! stages 2 and 3): the pads come from the profile's `[pad_covers]` contact faces. Balanced by the
//! plain PD law of the physics tests instead of firmware.

use euc_physics::{Intent, LegPad, Physics, RiderModel, Setup, TIMESTEP_S};
use euc_profile::ProfileSource;
use euc_sim::WheelFiles;

fn physics(model: RiderModel) -> Physics {
    let files = WheelFiles::load(&ProfileSource::Builtin, "begode-extreme").unwrap();
    let mut setup = Setup::new(files.body);
    setup.rider.model = model;
    Physics::new(setup).unwrap()
}

fn balance(physics: &Physics) -> f64 {
    let state = physics.state();
    if physics.held() {
        0.0
    } else {
        (-2500.0 * state.pitch_rad + 250.0 * physics.imu().gyro_rad_s[1]).clamp(-300.0, 300.0)
    }
}

/// Rides `seconds` and returns the largest leg-on-shell force on either leg, newtons.
fn ride(physics: &mut Physics, seconds: f64) -> f64 {
    let mut peak = 0.0_f64;
    for _ in 0..(seconds / TIMESTEP_S).round() as usize {
        let torque = balance(physics);
        physics.step(torque);
        assert!(!physics.state().fallen && !physics.rider_off(), "{:?}", physics.state());
        if let Some(legs) = physics.leg_pads() {
            peak = legs.iter().map(|leg| leg.force_n).fold(peak, f64::max);
        }
    }
    peak
}

fn legs(physics: &mut Physics) -> [LegPad; 2] {
    physics.leg_pads().expect("the Extreme has measured pads")
}

/// Rides `seconds` and returns each leg's mean force on its pad and each sole's mean load on
/// its pedal, newtons (left, right).
fn hold(physics: &mut Physics, seconds: f64) -> ([f64; 2], [f64; 2]) {
    let n = (seconds / TIMESTEP_S).round() as usize;
    let (mut pads, mut soles) = ([0.0; 2], [0.0; 2]);
    for _ in 0..n {
        let torque = balance(physics);
        physics.step(torque);
        assert!(!physics.state().fallen && !physics.rider_off(), "{:?}", physics.state());
        for (sum, leg) in pads.iter_mut().zip(legs(physics)) {
            *sum += leg.force_n / n as f64;
        }
        for (sum, sole) in soles.iter_mut().zip(physics.sole_loads().expect("welded soles")) {
            *sum += sole.normal_n / n as f64;
        }
    }
    (pads, soles)
}

fn knees(physics: &Physics) -> [f64; 2] {
    let joints = physics.rider_joints();
    ["left_knee", "right_knee"].map(|name| joints.iter().find(|joint| joint.name == name).unwrap().angle_rad)
}

/// At rest each leg's calf rests on its pad's upper arm with about the preload, the knees swung
/// out a little; nothing presses in.
#[test]
fn at_rest_the_legs_rest_on_the_pads_with_the_preload() {
    let mut physics = physics(RiderModel::ArticulatedLateralV1);
    ride(&mut physics, 1.0);
    let preload = physics.setup().rider.articulation.pad_preload_n;
    for leg in legs(&mut physics) {
        println!("rest: {leg:?}");
        assert!((leg.force_n - preload).abs() < 10.0, "{leg:?}");
        assert!(leg.gap_m > -0.005, "{leg:?}");
        assert!((0.0..0.45).contains(&leg.swing_rad), "{leg:?}");
    }
    // The other riders have no leg contact.
    assert!(self::physics(RiderModel::ArticulatedSagittalV1).leg_pads().is_none());
}

/// Riding straight the pads do not hold the rider back: the speed builds up as the sagittal
/// rider's, and the legs stay near the preload.
#[test]
fn riding_straight_the_pads_do_not_hold_the_rider_back() {
    let mut speeds = Vec::new();
    for model in [RiderModel::ArticulatedSagittalV1, RiderModel::ArticulatedLateralV1] {
        let mut physics = physics(model);
        ride(&mut physics, 0.5);
        physics.set_held(false);
        physics.rider().set_intent(Intent::Speed(15.0 / 3.6));
        let peak = ride(&mut physics, 10.0);
        speeds.push(physics.state().speed_mps);
        println!("{model:?}: {:.3} m/s after 10 s, largest leg load {peak:.1} N", physics.state().speed_mps);
        if model == RiderModel::ArticulatedLateralV1 {
            assert!(peak < 100.0, "{peak} N");
        }
    }
    assert!((speeds[0] - speeds[1]).abs() < 0.2, "{speeds:?}");
}

/// In a 0.4 rad/s left turn at 15 km/h (about 10° of lean; the external aids still steer) the
/// outside leg rests on its pad harder, by the turn's press, and the inside knee spreads off
/// its pad: a short load as the wheel rolls in, then the outside leg alone.
#[test]
fn in_a_moderate_turn_the_outside_leg_rests_harder_and_the_inside_knee_spreads() {
    let mut physics = physics(RiderModel::ArticulatedLateralV1);
    ride(&mut physics, 0.5);
    physics.set_held(false);
    physics.rider().set_intent(Intent::Speed(15.0 / 3.6));
    ride(&mut physics, 10.0);
    let before = physics.rider_state();
    physics.rider().set_turn_rate(0.4);
    let entry = ride(&mut physics, 2.0);
    let (pads, soles) = hold(&mut physics, 2.0);
    let after = physics.rider_state();
    let config = physics.setup().rider.articulation;
    let press = config.pad_preload_n + config.turn_press_n_per_rad * after.turn_lean_rad.abs();
    let [inside, outside] = legs(&mut physics);
    println!(
        "turn: entry peak {entry:.0} N, pads {:.1} / {:.1} N (press {press:.1} N), soles {:.0} / {:.0} N, roll {:.1}°; aid work in 4 s: roll {:.1} J, yaw {:.1} J",
        pads[0],
        pads[1],
        soles[0],
        soles[1],
        physics.state().roll_rad.to_degrees(),
        after.roll_aid_work_j - before.roll_aid_work_j,
        after.yaw_aid_work_j - before.yaw_aid_work_j
    );
    assert!(physics.state().roll_rad < -0.1);
    assert!(entry < 600.0, "{entry} N");
    assert!((pads[1] - press).abs() < 10.0, "{pads:?}");
    assert!(pads[0] < 5.0 && inside.gap_m > 0.0, "{inside:?}");
    assert!(outside.gap_m < 0.001, "{outside:?}");
    assert!(soles[0] > soles[1], "{soles:?}");
}

/// A hard turn (the full manual turn rate at 5 m/s, about 35° of lean) as the owner rides one:
/// the knees bend deeper, the inside knee spreads off its pad, the outside leg rests on its pad
/// and the inside sole carries more of the rider. The legs hold the body, so the aids lean the
/// whole machine and turn the shell only: their roll work stays near the sagittal rider's, whose
/// welded legs hold its body by construction, and their yaw work stays small.
#[test]
fn in_a_hard_turn_the_knees_bend_the_inside_knee_spreads_and_the_inside_sole_carries_more() {
    let mut sagittal = (0.0, 0.0);
    for (model, rate) in [
        (RiderModel::ArticulatedSagittalV1, 10.0),
        (RiderModel::ArticulatedLateralV1, 10.0),
        (RiderModel::ArticulatedLateralV1, -10.0),
    ] {
        let mut physics = physics(model);
        ride(&mut physics, 0.5);
        physics.set_held(false);
        physics.rider().set_manual(true);
        physics.rider().set_intent(Intent::Speed(5.0));
        ride(&mut physics, 10.0);
        let (straight, before) = (knees(&physics), physics.rider_state());
        physics.rider().set_turn_rate(rate);
        ride(&mut physics, 3.0);
        let lateral = model == RiderModel::ArticulatedLateralV1;
        let (pads, soles) = if lateral {
            hold(&mut physics, 1.0)
        } else {
            ride(&mut physics, 1.0);
            ([0.0; 2], [0.0; 2])
        };
        let after = physics.rider_state();
        let work = (after.roll_aid_work_j - before.roll_aid_work_j, after.yaw_aid_work_j - before.yaw_aid_work_j);
        let roll = physics.state().roll_rad;
        println!(
            "{model:?} at {rate} rad/s: roll {:.1}° (target {:.1}°), aid work roll {:.1} J, yaw {:.1} J; pads {pads:.0?} N, soles {soles:.0?} N, knees {:.0?}° from {:.0?}°",
            roll.to_degrees(),
            after.turn_lean_rad.to_degrees(),
            work.0,
            work.1,
            knees(&physics).map(f64::to_degrees),
            straight.map(f64::to_degrees)
        );
        assert!((roll - after.turn_lean_rad).abs() < 0.02 && roll.abs() > 0.5, "{roll} rad");
        if !lateral {
            sagittal = work;
            continue;
        }
        assert!(work.0 < 1.4 * sagittal.0, "{work:?} against {sagittal:?}");
        // The upper body's yaw aid is gone: in stage 2 it made this turn's yaw work about 200 J.
        // The shell's own twist aid does a few joules to tens of joules, as the tire's spin
        // friction asks of it, for either rider.
        assert!(work.1.abs() < 30.0, "{work:?} against {sagittal:?}");
        // A left turn (positive rate) has the left leg (0) inside.
        let (inside, outside) = if rate > 0.0 { (0, 1) } else { (1, 0) };
        let bent = knees(&physics);
        assert!((0..2).all(|leg| bent[leg] - straight[leg] > 0.25), "{bent:?} from {straight:?}");
        let legs = legs(&mut physics);
        assert!(pads[inside] < 5.0 && legs[inside].gap_m > 0.035, "{:?}", legs[inside]);
        assert!(pads[outside] > physics.setup().rider.articulation.pad_preload_n, "{pads:?}");
        assert!(soles[inside] > soles[outside] + 80.0, "{soles:?}");
    }
}

/// Stepping off switches the legs' pairs off: the legs pass the pads.
#[test]
fn stepping_off_releases_the_legs_from_the_pads() {
    let mut physics = physics(RiderModel::ArticulatedLateralV1);
    physics.step_off();
    for _ in 0..(1.0 / TIMESTEP_S) as usize {
        physics.step(0.0);
    }
    assert!(legs(&mut physics).iter().all(|leg| leg.force_n == 0.0));
}

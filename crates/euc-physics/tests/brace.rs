//! The scripted rider's brace for a step up it sees ahead (task 61, part K2). Balanced by the
//! plain PD law of the physics tests instead of firmware.
use euc_physics::{Feature, Intent, Physics, RiderModel, Setup, TIMESTEP_S, Terrain};

fn setup(model: RiderModel, height_m: f64) -> Setup {
    let plant: toml::Value = toml::from_str(include_str!("../../../profiles/begode-extreme/plant.toml")).unwrap();
    let mut setup = Setup::new(plant["body"].clone().try_into().unwrap());
    setup.rider.model = model;
    setup.terrain = Terrain {
        length_m: 40.0,
        features: vec![Feature::Curb { at_m: 12.0, height_m, yaw_deg: 0.0 }],
        ..Terrain::flat()
    };
    setup
}

fn step(physics: &mut Physics) {
    let state = physics.state();
    let torque = if physics.held() {
        0.0
    } else {
        (-2500.0 * state.pitch_rad + 250.0 * physics.imu().gyro_rad_s[1]).clamp(-300.0, 300.0)
    };
    physics.step(torque);
}

/// What a ride at 10 km/h towards the curb shows: when the brace starts and ends, the most the
/// knees bend in it, and when the absorb starts (the tire meets the curb).
struct Seen {
    brace: Option<(f64, f64)>,
    knees: f64,
    absorb: Option<f64>,
}

fn ride(setup: Setup, manual: bool) -> Seen {
    let mut physics = Physics::new(setup).unwrap();
    for _ in 0..1000 {
        step(&mut physics);
    }
    physics.set_held(false);
    physics.rider().set_manual(manual);
    physics.rider().set_intent(Intent::Speed(10.0 / 3.6));
    let mut seen = Seen { brace: None, knees: 0.0, absorb: None };
    let knee = |physics: &Physics| physics.rider_joints().iter().find(|j| j.name == "left_knee").unwrap().angle_rad;
    let straight = knee(&physics);
    for _ in 0..(8.0 / TIMESTEP_S) as usize {
        step(&mut physics);
        let (state, t) = (physics.rider_state(), physics.time_s());
        if state.brace_lean_rad > 0.0 {
            let start = seen.brace.map_or(t, |(start, _)| start);
            seen.brace = Some((start, t));
            seen.knees = seen.knees.max(knee(&physics) - straight);
        }
        if state.absorb > 0.0 && seen.absorb.is_none() {
            seen.absorb = Some(t);
        }
        if seen.absorb.is_some_and(|at| t > at + 0.5) || physics.rider_off() || physics.state().fallen {
            break;
        }
    }
    seen
}

/// Riding at 10 km/h towards a 15 cm curb, the scripted rider braces from about `brace_lead_s`
/// before the tire meets it until it does: it leans back and crouches. The absorb takes over.
#[test]
fn a_scripted_rider_braces_for_a_curb_it_sees() {
    for model in [RiderModel::ArticulatedSagittalV1, RiderModel::ArticulatedLateralV1] {
        let setup = setup(model, 0.15);
        let (lead, crouch) = (setup.rider.control.brace_lead_s, setup.rider.control.brace_crouch_rad);
        let seen = ride(setup, false);
        let ((start, end), absorb) = (seen.brace.expect("a brace"), seen.absorb.expect("the curb stops the wheel"));
        println!(
            "{model:?}: brace {start:.3}..{end:.3} s ({:.2} s), knees +{:.2} rad, absorb from {absorb:.3} s",
            end - start,
            seen.knees
        );
        assert!((end - start - lead).abs() < 0.1, "{model:?}: braced {} s", end - start);
        assert!((absorb - end).abs() < 0.05, "{model:?}: the brace ends as the tire meets the curb");
        assert!(seen.knees > 0.5 * crouch, "{model:?}: the knees bend {} rad", seen.knees);
    }
}

/// No brace for a 5 cm curb (below `brace_step_m`), up a 20 % slope (a slope is not a step), by
/// hand (a person braces on their own) or with `brace_lead_s = 0`.
#[test]
fn no_brace_for_a_low_curb_by_hand_or_when_turned_off() {
    let model = RiderModel::ArticulatedSagittalV1;
    assert!(ride(setup(model, 0.05), false).brace.is_none());
    let mut slope = setup(model, 0.15);
    slope.terrain =
        Terrain { slope_pct: 20.0, slope_start_m: 8.0, slope_ramp_m: 1.0, length_m: 40.0, ..Terrain::flat() };
    assert!(ride(slope, false).brace.is_none());
    assert!(ride(setup(model, 0.15), true).brace.is_none());
    let mut off = setup(model, 0.15);
    off.rider.control.brace_lead_s = 0.0;
    assert!(ride(off, false).brace.is_none());
}

/// The brace's parameters are checked before the model is built.
#[test]
fn brace_parameters_are_checked() {
    for (lead, step, lean, crouch, ok) in [
        (0.55, 0.07, 5.7, 0.3, true),
        (-0.1, 0.07, 5.7, 0.3, false),
        (0.55, -0.01, 5.7, 0.3, false),
        (0.55, 0.07, f64::NAN, 0.3, false),
        (0.55, 0.07, 5.7, 1.5, false),
    ] {
        let mut setup = setup(RiderModel::ArticulatedSagittalV1, 0.15);
        let control = &mut setup.rider.control;
        (control.brace_lead_s, control.brace_step_m, control.brace_lean_deg, control.brace_crouch_rad) =
            (lead, step, lean, crouch);
        assert_eq!(Physics::new(setup).is_ok(), ok, "{lead} {step} {lean} {crouch}");
    }
}

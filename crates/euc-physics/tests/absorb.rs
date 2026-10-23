//! The welded riders' absorb: legs that give way when something stops the wheel (task 61, part K1).
//! Balanced by the plain PD law of the physics tests instead of firmware.
use euc_physics::{
    Feature, Intent, Physics, RiderModel, Setup, Shapes, Shell, TIMESTEP_S, Terrain, Tire, Torus, Wheel, WheelBody,
};

const WELDED: [RiderModel; 2] = [RiderModel::ArticulatedSagittalV1, RiderModel::ArticulatedLateralV1];

fn setup(model: RiderModel, terrain: Terrain) -> Setup {
    let plant: toml::Value = toml::from_str(include_str!("../../../profiles/begode-extreme/plant.toml")).unwrap();
    let mut setup = Setup::new(plant["body"].clone().try_into().unwrap());
    setup.rider.model = model;
    setup.terrain = terrain;
    setup
}

/// The Extreme without its suspension, with the scanned tire torus (as tests/turning.rs): the
/// wheel the full manual turn is tested on.
fn turning_setup(model: RiderModel) -> Setup {
    let mut setup = Setup::new(WheelBody {
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
        shapes: Some(Shapes { tire: Some(Torus { major_m: 0.1716, minor_m: 0.058 }), ..Shapes::default() }),
    });
    setup.rider.model = model;
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

/// Rides `seconds` and returns the largest absorb level met.
fn ride(physics: &mut Physics, seconds: f64) -> f64 {
    let mut most = 0.0_f64;
    for _ in 0..(seconds / TIMESTEP_S).round() as usize {
        step(physics);
        most = most.max(physics.rider_state().absorb);
    }
    most
}

fn knee(physics: &Physics) -> f64 {
    physics.rider_joints().iter().find(|joint| joint.name == "left_knee").unwrap().angle_rad
}

/// Riding up to 15 km/h and braking hard to a stop (the Extreme with its suspension), and a full
/// manual turn at 5 m/s (the turning tests' wheel), slow the wheel smoothly: no absorb.
#[test]
fn riding_braking_hard_and_turning_hard_do_not_start_an_absorb() {
    for model in WELDED {
        let mut physics = Physics::new(setup(model, Terrain::flat())).unwrap();
        ride(&mut physics, 0.5);
        physics.set_held(false);
        physics.rider().set_intent(Intent::Speed(15.0 / 3.6));
        let riding = ride(&mut physics, 10.0);
        physics.rider().set_intent(Intent::Speed(0.0));
        let braking = ride(&mut physics, 6.0);
        let mut physics = Physics::new(turning_setup(model)).unwrap();
        ride(&mut physics, 0.5);
        physics.set_held(false);
        physics.rider().set_manual(true);
        physics.rider().set_intent(Intent::Speed(5.0));
        ride(&mut physics, 8.0);
        physics.rider().set_turn_rate(10.0);
        let turning = ride(&mut physics, 4.0);
        let roll = physics.state().roll_rad.to_degrees();
        println!("{model:?}: absorb riding {riding}, braking {braking}, turning {turning} (roll {roll:.1}°)");
        assert_eq!((riding, braking, turning), (0.0, 0.0, 0.0));
        assert!(roll < -30.0, "{model:?}: the turn leans the wheel: {roll}°");
    }
}

/// A 10 cm curb met at 10 km/h stops the wheel: the absorb starts in full, the knees fold, and it
/// fades out over `absorb_time_s` once the wheel no longer slows hard. With
/// `absorb_decel_mps2 = inf` it never starts.
#[test]
fn a_curb_starts_an_absorb_that_folds_the_knees_and_fades() {
    let curb = Terrain {
        length_m: 40.0,
        features: vec![Feature::Curb { at_m: 12.0, height_m: 0.10, yaw_deg: 0.0 }],
        ..Terrain::flat()
    };
    for model in WELDED {
        for off in [false, true] {
            let mut setup = setup(model, curb.clone());
            if off {
                setup.rider.articulation.absorb_decel_mps2 = f64::INFINITY;
            }
            let fade = setup.rider.articulation.absorb_time_s;
            let mut physics = Physics::new(setup).unwrap();
            ride(&mut physics, 0.5);
            physics.set_held(false);
            physics.rider().set_intent(Intent::Speed(10.0 / 3.6));
            let (mut start, mut last_full, mut folded) = (None, None, 0.0_f64);
            let straight = knee(&physics);
            for _ in 0..(8.0 / TIMESTEP_S) as usize {
                step(&mut physics);
                let absorb = physics.rider_state().absorb;
                if absorb >= 1.0 {
                    start.get_or_insert(physics.time_s());
                    last_full = Some(physics.time_s());
                }
                if start.is_some() {
                    folded = folded.max(knee(&physics) - straight);
                }
                if let Some(last) = last_full
                    && physics.time_s() > last + fade + 0.01
                {
                    assert_eq!(absorb, 0.0, "{model:?}: the absorb fades out over absorb_time_s");
                }
                if physics.rider_off() || physics.state().fallen {
                    break;
                }
            }
            println!(
                "{model:?}, absorb off {off}: started {start:?}, last full {last_full:?}, knees folded {folded:.2} rad, x {:.2} m",
                physics.position_m()[0]
            );
            if off {
                assert!(start.is_none());
            } else {
                assert!(start.is_some(), "{model:?}: the curb must start an absorb");
                assert!(folded > 0.3, "{model:?}: the knees fold by {folded} rad");
            }
        }
    }
}

/// The absorb's parameters are checked before the model is built.
#[test]
fn absorb_parameters_are_checked() {
    for (decel, time, fold, ok) in [
        (30.0, 0.3, 0.6, true),
        (f64::INFINITY, 0.3, 0.6, true),
        (0.0, 0.3, 0.6, false),
        (f64::NAN, 0.3, 0.6, false),
        (30.0, 0.0, 0.6, false),
        (30.0, 0.3, -0.1, false),
        (30.0, 0.3, 1.5, false),
    ] {
        let mut setup = setup(RiderModel::ArticulatedSagittalV1, Terrain::flat());
        let articulation = &mut setup.rider.articulation;
        (articulation.absorb_decel_mps2, articulation.absorb_time_s, articulation.absorb_fold_rad) =
            (decel, time, fold);
        assert_eq!(Physics::new(setup).is_ok(), ok, "{decel} {time} {fold}");
    }
}

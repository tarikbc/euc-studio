//! The Begode Extreme plant (`profiles/begode-extreme/plant.toml`) under a current controller that
//! uses the stock firmware's conventions, as `docs/models/electrical.md` lists them: phase
//! currents from ADC1 ch4 (inverted) and ADC2 ch5, the ST library's Clarke/Park/inverse Park with
//! its sign of β, the hall sector table with sector-centre angles, ST's SVPWM, and phase A on CCR1,
//! B on CCR3, C on CCR2. If the plant's conventions match the firmware's, a positive q-current
//! command makes forward torque of the size `1.5 · p · ψ · Iq`.

use euc_plant::inverter::LegDrive;
use euc_plant::rig::WheelRig;
use euc_plant::sensors::Phase;
use euc_plant::{ElectricalPlant, PlantParams};

const HCLK: u64 = 72_000_000;
/// 12 kHz current-mode PWM: ARR 3000, centre-aligned.
const ARR: i64 = 3000;
const PWM_HZ: u64 = 12_000;
/// Q15 counts per ampere of the firmware's current scale (32768 = 437.5 A).
const COUNTS_PER_A: f64 = 32768.0 / 437.5;

fn params() -> PlantParams {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../profiles/begode-extreme/plant.toml");
    PlantParams::from_toml(&std::fs::read_to_string(path).expect("plant.toml")).expect("valid plant.toml")
}

/// The firmware's sector-centre angle for a hall code, radians.
fn hall_angle(code: u8) -> f64 {
    let degrees: f64 = match code {
        1 => -150.0,
        3 => -90.0,
        2 => -30.0,
        6 => 30.0,
        4 => 90.0,
        5 => 150.0,
        _ => 0.0,
    };
    degrees.to_radians()
}

/// ST SVPWM (`SVPWM_IcsCalcDutyCycles`): compare values A, B, C for Q15 `(Vα, Vβ)`.
// The library's variable names and C's truncating division, so the code reads like motor.md §5.
#[allow(clippy::many_single_char_names, clippy::manual_midpoint)]
fn svpwm(v_alpha: i64, v_beta: i64) -> (i64, i64, i64) {
    let t = 4 * ARR;
    let t_sqrt3 = (t as f64 * 3f64.sqrt()) as i64;
    let (u_alpha, u_beta) = (v_alpha * t_sqrt3, -v_beta * t);
    let (x, y, z) = (u_beta, (u_beta + u_alpha) / 2, (u_beta - u_alpha) / 2);
    let sector = if y < 0 {
        if z < 0 {
            5
        } else if x <= 0 {
            4
        } else {
            3
        }
    } else if z >= 0 {
        2
    } else if x <= 0 {
        6
    } else {
        1
    };
    let q = 131_072;
    match sector {
        1 | 4 => {
            let a = t / 8 + ((t + x - z) / 2) / q;
            let b = a + z / q;
            (a, b, b - x / q)
        }
        2 | 5 => {
            let a = t / 8 + ((t + y - z) / 2) / q;
            (a, a + z / q, a - y / q)
        }
        _ => {
            let a = t / 8 + ((t - x + y) / 2) / q;
            let c = a - y / q;
            (a, c + x / q, c)
        }
    }
}

/// A q-current loop with the firmware's frame conventions and roughly its PI gains.
struct Foc {
    iq_ref: f64,
    integral_q: f64,
    integral_d: f64,
}

impl Foc {
    /// One control period: read the sensors, write the three legs. Returns the measured Iq (Q15).
    fn tick(&mut self, plant: &mut ElectricalPlant, now: u64) -> f64 {
        let raw = |adc, ch| f64::from(plant.adc_raw(adc, ch).expect("wired"));
        let ia = -(16.0 * raw(1, 4) - 32768.0);
        let ib = 16.0 * raw(2, 5) - 32768.0;
        let alpha = ia;
        let beta = -(ia + 2.0 * ib) / 3f64.sqrt();
        let (sin, cos) = hall_angle(plant.halls()).sin_cos();
        let iq = alpha * cos - beta * sin;
        let id = alpha * sin + beta * cos;
        self.integral_q = (self.integral_q + 0.003_66 * (self.iq_ref - iq)).clamp(-32000.0, 32000.0);
        self.integral_d = (self.integral_d + 0.014_65 * -id).clamp(-32000.0, 32000.0);
        let vq = (2.93 * (self.iq_ref - iq) + self.integral_q).clamp(-32000.0, 32000.0);
        let vd = (2.148 * -id + self.integral_d).clamp(-32000.0, 32000.0);
        let v_alpha = vq * cos + vd * sin;
        let v_beta = vd * cos - vq * sin;
        let (a, b, c) = svpwm(v_alpha as i64, v_beta as i64);
        let period = 1.0 / PWM_HZ as f64;
        let leg = |ccr: i64| LegDrive::Pwm { duty: (ccr as f64 / ARR as f64).clamp(0.0, 1.0), period_s: period };
        // The board: phase A on CCR1, phase B on CCR3, phase C on CCR2 (named by the firmware's order).
        plant.set_leg(now, Phase::A, leg(a));
        plant.set_leg(now, Phase::B, leg(b));
        plant.set_leg(now, Phase::C, leg(c));
        iq
    }
}

/// Runs `seconds` with `iq_ref_a` amperes commanded; returns (mean torque of the last 10 ms,
/// final measured Iq in amperes, the wheel, the hall codes seen in order).
fn run(iq_ref_a: f64, mut wheel: WheelRig, seconds: f64) -> (f64, f64, WheelRig, Vec<u8>) {
    let mut plant = ElectricalPlant::new(params(), HCLK).unwrap();
    let mut foc = Foc { iq_ref: iq_ref_a * COUNTS_PER_A, integral_q: 0.0, integral_d: 0.0 };
    let period = HCLK / PWM_HZ;
    let ticks = (seconds * PWM_HZ as f64) as u64;
    let (mut torque_tail, mut iq) = (0.0, 0.0);
    let mut codes = vec![plant.halls()];
    for n in 1..=ticks {
        let now = n * period;
        plant.advance(now);
        let torque = plant.mean_torque_nm();
        wheel.step(torque, 1.0 / PWM_HZ as f64);
        plant.set_wheel(now, wheel.angle(), wheel.speed());
        iq = foc.tick(&mut plant, now) / COUNTS_PER_A;
        if n > ticks - PWM_HZ / 100 {
            torque_tail += torque / (PWM_HZ / 100) as f64;
        }
        if codes.last() != Some(&plant.halls()) {
            codes.push(plant.halls());
        }
    }
    (torque_tail, iq, wheel, codes)
}

/// `1.5 · p · ψ` of the plant, newton-metre per ampere.
fn torque_constant() -> f64 {
    let m = params().motor;
    1.5 * f64::from(m.pole_pairs) * m.flux_wb
}

#[test]
fn held_wheel_positive_command_pushes_forward_with_the_right_size() {
    for angle in [0.0, 0.004, 0.011, 0.019] {
        let mut wheel = WheelRig::new(1.0, 0.0, 0.0, true);
        // Hold the wheel at a few rotor positions inside a hall sector.
        wheel.held = false;
        wheel.step(angle * 1.0e6, 1e-3);
        wheel.step(-angle * 1.0e6, 1e-3);
        wheel.held = true;
        let (torque, iq, _, _) = run(40.0, wheel, 0.2);
        assert!((iq - 40.0).abs() < 1.0, "the loop reaches its reference: {iq} A");
        let full = torque_constant() * 40.0;
        // Sector-centre angles are within ±30° of the rotor: torque between cos 30° and 1 of full.
        assert!(torque > 0.85 * full && torque < 1.01 * full, "angle {angle}: {torque} N·m of {full}");
    }
}

#[test]
fn held_wheel_negative_command_pushes_backward() {
    let (torque, iq, _, _) = run(-40.0, WheelRig::new(1.0, 0.0, 0.0, true), 0.2);
    assert!((iq + 40.0).abs() < 1.0);
    assert!(torque < -0.85 * torque_constant() * 40.0, "{torque}");
}

#[test]
fn free_wheel_spins_up_forward_with_the_firmware_negative_hall_order() {
    let (_, _, wheel, codes) = run(30.0, WheelRig::new(2.0, 0.05, 0.5, false), 1.0);
    // About 23 N·m on 2 kg·m² for 1 s, less friction.
    assert!(wheel.speed() > 8.0 && wheel.speed() < 12.0, "{} rad/s", wheel.speed());
    let order: Vec<u8> = codes.windows(2).map(|w| next_forward(w[0])).collect();
    assert_eq!(order, codes[1..], "every edge goes 4 → 6 → 2 → 3 → 1 → 5");
}

#[test]
fn free_wheel_spins_up_backward_with_the_firmware_positive_hall_order() {
    let (_, _, wheel, codes) = run(-30.0, WheelRig::new(2.0, 0.05, 0.5, false), 1.0);
    assert!(wheel.speed() < -8.0, "{} rad/s", wheel.speed());
    assert!(codes.windows(2).all(|w| next_forward(w[1]) == w[0]), "5 → 1 → 3 → 2 → 6 → 4: {codes:?}");
}

#[test]
fn spin_up_is_smooth() {
    // No reversal and no stall on the way up: the speed rises every 10 ms.
    let mut last = 0.0;
    for tenth in 1..=5 {
        let (_, _, wheel, _) = run(30.0, WheelRig::new(2.0, 0.05, 0.5, false), 0.1 * f64::from(tenth));
        assert!(wheel.speed() > last, "{} after {} s", wheel.speed(), 0.1 * f64::from(tenth));
        last = wheel.speed();
    }
}

/// The code after `code` when the wheel rolls forward (the firmware's negative direction).
fn next_forward(code: u8) -> u8 {
    match code {
        4 => 6,
        6 => 2,
        2 => 3,
        3 => 1,
        1 => 5,
        _ => 4,
    }
}

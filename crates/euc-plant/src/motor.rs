//! Permanent-magnet synchronous motor in the rotor frame (dq).
//!
//! Conventions (the "standard frame" of this crate): the motor's phases a, b, c sit at electrical
//! angles 0°, +120°, −120°; the electrical angle θ is the direction of the rotor flux (the d axis);
//! the q axis leads it by 90°; transforms are amplitude-invariant, so a phase current of peak I is
//! `|i_dq| = I` and power is `1.5 · (vd·id + vq·iq)`.

use std::f64::consts::TAU;

use serde::Deserialize;

/// Electrical angles of phases a, b, c.
const PHASE_ANGLE: [f64; 3] = [0.0, TAU / 3.0, -TAU / 3.0];

/// A quantity in the rotor frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Dq {
    /// Along the rotor flux.
    pub d: f64,
    /// 90° electrical ahead of the flux: the torque-producing axis.
    pub q: f64,
}

/// The rotor-frame components of three phase values at electrical angle `theta`.
pub fn park(abc: [f64; 3], theta: f64) -> Dq {
    let (mut d, mut q) = (0.0, 0.0);
    for (x, phase) in abc.iter().zip(PHASE_ANGLE) {
        let (sin, cos) = (theta - phase).sin_cos();
        d += x * cos;
        q -= x * sin;
    }
    Dq { d: d * 2.0 / 3.0, q: q * 2.0 / 3.0 }
}

/// The phase values of a rotor-frame quantity at electrical angle `theta` (no zero sequence).
pub fn inverse_park(dq: Dq, theta: f64) -> [f64; 3] {
    PHASE_ANGLE.map(|phase| {
        let (sin, cos) = (theta - phase).sin_cos();
        dq.d * cos - dq.q * sin
    })
}

/// Motor parameters (`[motor]` in `plant.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorParams {
    /// Pole pairs: electrical revolutions per mechanical revolution.
    pub pole_pairs: u32,
    /// Phase resistance at 25 °C, ohm.
    pub resistance_ohm: f64,
    /// d-axis inductance, henry.
    pub ld_h: f64,
    /// q-axis inductance, henry.
    pub lq_h: f64,
    /// Peak flux linkage of the magnets per phase, weber (back-EMF peak = `ωe · ψ`).
    pub flux_wb: f64,
    /// `+1` or `−1`: the sign of the electrical angle change when the wheel rolls forward.
    pub direction: i8,
    /// Electrical angle at wheel angle 0, degrees.
    #[serde(default)]
    pub angle_offset_deg: f64,
}

impl MotorParams {
    /// The electrical angle (radians, not wrapped) at wheel angle `wheel_rad`.
    pub fn electrical_angle(&self, wheel_rad: f64) -> f64 {
        f64::from(self.direction) * f64::from(self.pole_pairs) * wheel_rad + self.angle_offset_deg.to_radians()
    }

    /// Electrical speed (rad/s) at wheel speed `wheel_rad_s`.
    pub fn electrical_speed(&self, wheel_rad_s: f64) -> f64 {
        f64::from(self.direction) * f64::from(self.pole_pairs) * wheel_rad_s
    }

    /// Electromagnetic torque in the standard frame (positive towards increasing electrical angle)
    /// for rotor-frame currents `i`: `1.5 · p · (ψ·iq + (Ld − Lq)·id·iq)`.
    pub fn torque(&self, i: Dq) -> f64 {
        1.5 * f64::from(self.pole_pairs) * (self.flux_wb * i.q + (self.ld_h - self.lq_h) * i.d * i.q)
    }

    /// Torque on the wheel, positive forward, for rotor-frame currents `i`.
    pub fn wheel_torque(&self, i: Dq) -> f64 {
        f64::from(self.direction) * self.torque(i)
    }

    /// Back-EMF of the three phases at electrical angle `theta` and speed `omega_e`.
    pub fn back_emf(&self, theta: f64, omega_e: f64) -> [f64; 3] {
        PHASE_ANGLE.map(|phase| -omega_e * self.flux_wb * (theta - phase).sin())
    }

    /// Energy stored in the inductances, joule.
    pub fn magnetic_energy(&self, i: Dq) -> f64 {
        0.75 * (self.ld_h * i.d * i.d + self.lq_h * i.q * i.q)
    }
}

/// The motor's electrical state: its rotor-frame currents.
#[derive(Debug, Clone)]
pub struct Pmsm {
    params: MotorParams,
    current: Dq,
}

impl Pmsm {
    /// A motor with no current.
    pub fn new(params: MotorParams) -> Self {
        Self { params, current: Dq::default() }
    }

    /// The parameters.
    pub const fn params(&self) -> &MotorParams {
        &self.params
    }

    /// Rotor-frame currents.
    pub const fn current(&self) -> Dq {
        self.current
    }

    /// Replaces the currents (when another solver, such as the open-bridge one, moved them).
    pub fn set_current(&mut self, current: Dq) {
        self.current = current;
    }

    /// One step of `h` seconds with rotor-frame voltages `v` at electrical speed `omega_e`, with
    /// `resistance` ohm in series per phase (motor plus switch).
    ///
    /// Backward Euler on the whole linear system (resistance, inductance, speed cross-coupling),
    /// with the voltages and the speed held over the step: unconditionally stable, so the 5 µs
    /// grid is safe however small the inductance or high the speed.
    pub fn step(&mut self, v: Dq, omega_e: f64, resistance: f64, h: f64) {
        let MotorParams { ld_h: ld, lq_h: lq, flux_wb: psi, .. } = self.params;
        let Dq { d: id, q: iq } = self.current;
        let (a11, a12) = (ld / h + resistance, -omega_e * lq);
        let (a21, a22) = (omega_e * ld, lq / h + resistance);
        let b1 = v.d + ld / h * id;
        let b2 = v.q - omega_e * psi + lq / h * iq;
        let det = a11 * a22 - a12 * a21;
        self.current = Dq { d: (b1 * a22 - a12 * b2) / det, q: (a11 * b2 - a21 * b1) / det };
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const MOTOR: MotorParams = MotorParams {
        pole_pairs: 26,
        resistance_ohm: 0.08,
        ld_h: 0.000_2,
        lq_h: 0.000_2,
        flux_wb: 0.0205,
        direction: -1,
        angle_offset_deg: 0.0,
    };

    #[test]
    fn park_round_trip_and_phase_a_alignment() {
        let theta = 0.7;
        let dq = Dq { d: 3.0, q: -5.0 };
        let abc = inverse_park(dq, theta);
        assert!(abc.iter().sum::<f64>().abs() < 1e-12, "no zero sequence");
        let back = park(abc, theta);
        assert!((back.d - 3.0).abs() < 1e-12 && (back.q + 5.0).abs() < 1e-12);
        // A current of peak 1 along phase a, seen at θ = 0, is pure d.
        let along_a = park([1.0, -0.5, -0.5], 0.0);
        assert!((along_a.d - 1.0).abs() < 1e-12 && along_a.q.abs() < 1e-12);
        // Zero-sequence voltages (a common offset on all poles) do not reach the rotor frame.
        let shifted = park([1.0 + 7.0, -0.5 + 7.0, -0.5 + 7.0], 0.0);
        assert!((shifted.d - 1.0).abs() < 1e-12);
    }

    #[test]
    fn back_emf_is_pure_q() {
        let e = MOTOR.back_emf(1.1, 300.0);
        let dq = park(e, 1.1);
        assert!(dq.d.abs() < 1e-9);
        assert!((dq.q - 300.0 * MOTOR.flux_wb).abs() < 1e-9);
    }

    #[test]
    fn torque_sign_and_direction() {
        let i = Dq { d: 0.0, q: 10.0 };
        let torque = MOTOR.torque(i);
        assert!((torque - 1.5 * 26.0 * 0.0205 * 10.0).abs() < 1e-12);
        assert_eq!(MOTOR.wheel_torque(i), -torque, "direction -1: +iq pushes backwards");
        assert_eq!(MOTOR.electrical_speed(10.0), -260.0);
        // Reluctance torque with Ld < Lq and negative id adds to the magnet torque.
        let salient = MotorParams { ld_h: 0.000_1, ..MOTOR };
        assert!(salient.torque(Dq { d: -10.0, q: 10.0 }) > salient.torque(Dq { d: 0.0, q: 10.0 }));
    }

    #[test]
    fn steady_state_matches_the_dc_solution() {
        // Locked rotor: iq settles at vq / R.
        let mut motor = Pmsm::new(MOTOR);
        for _ in 0..20_000 {
            motor.step(Dq { d: 0.0, q: 1.6 }, 0.0, MOTOR.resistance_ohm, 5e-6);
        }
        assert!((motor.current().q - 20.0).abs() < 1e-6);
        // Spinning with vq = back-EMF and vd = 0: no current flows.
        let omega_e = 2000.0;
        let mut motor = Pmsm::new(MOTOR);
        for _ in 0..1000 {
            motor.step(Dq { d: 0.0, q: omega_e * MOTOR.flux_wb }, omega_e, MOTOR.resistance_ohm, 5e-6);
        }
        assert!(motor.current().d.abs() < 1e-9 && motor.current().q.abs() < 1e-9);
    }

    #[test]
    fn energy_balance_over_a_transient() {
        // Electrical input = copper loss + mechanical output + stored magnetic energy.
        let params = MotorParams { ld_h: 0.000_15, lq_h: 0.000_25, ..MOTOR };
        let mut motor = Pmsm::new(params);
        let (h, omega_e) = (1e-7, 800.0);
        let (mut input, mut loss, mut mech) = (0.0, 0.0, 0.0);
        for n in 0..20_000 {
            let v = Dq { d: -3.0 * (f64::from(n) * 1e-4).sin(), q: 20.0 };
            let before = motor.current();
            motor.step(v, omega_e, params.resistance_ohm, h);
            let after = motor.current();
            let mid = Dq { d: f64::midpoint(before.d, after.d), q: f64::midpoint(before.q, after.q) };
            input += 1.5 * (v.d * mid.d + v.q * mid.q) * h;
            loss += 1.5 * params.resistance_ohm * (mid.d * mid.d + mid.q * mid.q) * h;
            mech += params.torque(mid) * omega_e / f64::from(params.pole_pairs) * h;
        }
        let stored = params.magnetic_energy(motor.current());
        let balance = input - loss - mech - stored;
        assert!(balance.abs() < 1e-3 * input.abs(), "imbalance {balance} of {input} J");
    }
}

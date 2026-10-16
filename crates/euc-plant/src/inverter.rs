//! Three-phase two-level inverter, averaged over the PWM period.

use serde::Deserialize;

/// Current below which the dead-time error fades linearly to zero, ampere. Avoids chattering on
/// the sign of a current that hovers around zero.
const DEAD_TIME_BAND_A: f64 = 0.5;

/// Current below which a leg whose switches are off counts as not conducting, ampere.
const OPEN_EPSILON_A: f64 = 1e-9;

/// The series circuit of one phase for the open-leg solver.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Circuit {
    /// Phase inductance, henry.
    pub(crate) inductance: f64,
    /// Phase resistance including the switch, ohm.
    pub(crate) resistance: f64,
    /// Step, seconds.
    pub(crate) dt: f64,
}

/// What the gate driver does with one leg.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LegDrive {
    /// Complementary switching: the high side is on for `duty` of each period of `period_s`.
    Pwm {
        /// High-side on fraction, 0..=1. 0 holds the low side on (short-circuit brake).
        duty: f64,
        /// PWM period, seconds (for the dead-time error).
        period_s: f64,
    },
    /// Both switches off (main output enable cleared, or the channel disabled): only the body
    /// diodes conduct.
    Off,
}

impl LegDrive {
    /// The drive of a timer output carrying a PWM wave of `period` ticks, `high` of them high, at
    /// `tick_hz` (the timer models' `Signal::Pwm`).
    pub fn from_wave(tick_hz: u32, period: u32, high: u32) -> Self {
        let period = period.max(1);
        Self::Pwm {
            duty: (f64::from(high.min(period)) / f64::from(period)).clamp(0.0, 1.0),
            period_s: f64::from(period) / f64::from(tick_hz.max(1)),
        }
    }
}

/// Inverter parameters (`[inverter]` in `plant.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InverterParams {
    /// Effective dead time per switching period, seconds (the larger of the timer's and the gate
    /// driver's).
    pub dead_time_s: f64,
    /// On-resistance of one switch position (parallel MOSFETs), ohm.
    pub switch_resistance_ohm: f64,
    /// Forward drop of a body diode, volt.
    pub diode_drop_v: f64,
    /// Rise plus fall time of one switching transition, seconds (switching loss).
    pub switching_time_s: f64,
}

impl InverterParams {
    /// Average pole voltage (to the negative rail) of an active leg with `duty` at `period_s`,
    /// carrying `current` (positive into the motor): `d·Vbus − sign(i)·td/T·Vbus` while it switches.
    pub fn pole_voltage(&self, duty: f64, period_s: f64, current: f64, vbus: f64) -> f64 {
        let switching = duty > 0.0 && duty < 1.0;
        let dead = if switching {
            (current / DEAD_TIME_BAND_A).clamp(-1.0, 1.0) * self.dead_time_s / period_s * vbus
        } else {
            0.0
        };
        duty * vbus - dead
    }

    /// Steps the phase currents by one `circuit.dt` when at least one leg is off, in the
    /// stationary frame (mean inductance); `emf` is the phase back-EMF and `active` the pole
    /// voltages of the switching legs.
    ///
    /// An off leg conducts through its low diode (pole at −Vf) while its current flows into the
    /// motor, through its high diode (pole at Vbus + Vf) while it flows out, and is open with no
    /// current otherwise. An open leg starts conducting when its terminal voltage passes a rail;
    /// with every leg open, a pair starts when the line back-EMF exceeds Vbus plus two diode
    /// drops (rectification into the battery). A current that would reverse stops at zero. A
    /// `cut` phase (its motor wire open) never conducts, whatever its leg does.
    pub(crate) fn step_with_open_legs(
        &self,
        current: &mut [f64; 3],
        active: [Option<f64>; 3],
        emf: [f64; 3],
        vbus: f64,
        circuit: Circuit,
        cut: [bool; 3],
    ) {
        let vf = self.diode_drop_v;
        let active: [Option<f64>; 3] = std::array::from_fn(|leg| active[leg].filter(|_| !cut[leg]));
        // Pole voltage of each conducting leg; None = open.
        let mut pole: [Option<f64>; 3] = std::array::from_fn(|leg| match active[leg] {
            _ if cut[leg] => None,
            Some(voltage) => Some(voltage),
            None if current[leg] > OPEN_EPSILON_A => Some(-vf),
            None if current[leg] < -OPEN_EPSILON_A => Some(vbus + vf),
            None => None,
        });
        // Star-point voltage that makes the conducting currents sum to zero (needs two legs).
        let neutral = |pole: &[Option<f64>; 3]| -> Option<f64> {
            let drives: Vec<f64> = (0..3).filter_map(|leg| pole[leg].map(|v| v - emf[leg])).collect();
            (drives.len() >= 2).then(|| drives.iter().sum::<f64>() / drives.len() as f64)
        };
        // Open legs whose terminal voltage passes a rail start conducting.
        for _ in 0..3 {
            let Some(star) = neutral(&pole) else { break };
            let mut changed = false;
            for leg in 0..3 {
                if pole[leg].is_none() && active[leg].is_none() && !cut[leg] {
                    let terminal = star + emf[leg];
                    if terminal > vbus + vf {
                        pole[leg] = Some(vbus + vf);
                        changed = true;
                    } else if terminal < -vf {
                        pole[leg] = Some(-vf);
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }
        if neutral(&pole).is_none() {
            // Every leg open: the pair with the largest back-EMF difference may rectify.
            let wired = || (0..3).filter(|&leg| !cut[leg]);
            let high = wired().max_by(|&a, &b| emf[a].total_cmp(&emf[b])).unwrap_or(0);
            let low = wired().min_by(|&a, &b| emf[a].total_cmp(&emf[b])).unwrap_or(0);
            if wired().count() < 2 || emf[high] - emf[low] <= vbus + 2.0 * vf || active.iter().any(Option::is_some) {
                *current = [0.0; 3];
                return;
            }
            pole[high] = Some(vbus + vf);
            pole[low] = Some(-vf);
        }
        let star = neutral(&pole).unwrap_or(0.0);
        let Circuit { inductance, resistance, dt } = circuit;
        let mut next = [0.0; 3];
        for leg in 0..3 {
            if let Some(voltage) = pole[leg] {
                let drive = voltage - star - emf[leg];
                next[leg] = (current[leg] + dt / inductance * drive) / (1.0 + dt * resistance / inductance);
                // A diode does not conduct backwards.
                let diode_low = active[leg].is_none() && voltage < 0.0;
                let diode_high = active[leg].is_none() && voltage > vbus;
                if (diode_low && next[leg] < 0.0) || (diode_high && next[leg] > 0.0) {
                    next[leg] = 0.0;
                }
            }
        }
        // The star point carries no current: spread any residue over the conducting legs.
        let conducting: Vec<usize> = (0..3).filter(|&leg| next[leg] != 0.0 || active[leg].is_some()).collect();
        let residue: f64 = next.iter().sum();
        if conducting.len() >= 2 {
            for &leg in &conducting {
                next[leg] -= residue / conducting.len() as f64;
            }
        } else {
            next = [0.0; 3];
        }
        *current = next;
    }
}

/// Share of the time each leg connects its phase to the positive rail: the duty of a switching
/// leg, 1 for an off leg conducting through its high diode, 0 otherwise. The battery current is
/// `Σ share·i`.
pub(crate) fn high_side_share(legs: &[LegDrive; 3], i: &[f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| match legs[k] {
        LegDrive::Pwm { duty, .. } => duty,
        LegDrive::Off => f64::from(u8::from(i[k] < 0.0)),
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const INVERTER: InverterParams =
        InverterParams { dead_time_s: 5e-7, switch_resistance_ohm: 0.002, diode_drop_v: 0.7, switching_time_s: 1e-7 };

    #[test]
    fn waves_become_duties() {
        // TIM8 centre-aligned at ARR 3000 with CCR 1500: 50 % at 12 kHz.
        let LegDrive::Pwm { duty, period_s } = LegDrive::from_wave(72_000_000, 6000, 3000) else { panic!() };
        assert!((duty - 0.5).abs() < 1e-12 && (period_s - 1.0 / 12_000.0).abs() < 1e-15);
        assert_eq!(LegDrive::from_wave(72_000_000, 6000, 0), LegDrive::Pwm { duty: 0.0, period_s: 6000.0 / 72e6 });
        assert_eq!(LegDrive::from_wave(1, 0, 5), LegDrive::Pwm { duty: 1.0, period_s: 1.0 });
    }

    #[test]
    fn dead_time_error_opposes_the_current() {
        let (vbus, period) = (130.0, 1.0 / 12_000.0);
        let ideal = 0.4 * vbus;
        let error = 5e-7 / period * vbus;
        assert!((INVERTER.pole_voltage(0.4, period, 20.0, vbus) - (ideal - error)).abs() < 1e-9);
        assert!((INVERTER.pole_voltage(0.4, period, -20.0, vbus) - (ideal + error)).abs() < 1e-9);
        assert!((INVERTER.pole_voltage(0.4, period, 0.0, vbus) - ideal).abs() < 1e-9);
        assert_eq!(INVERTER.pole_voltage(0.0, period, 20.0, vbus), 0.0, "no switching, no dead time");
    }

    #[test]
    fn freewheeling_currents_decay_through_the_diodes_and_stop() {
        let mut i = [20.0, -10.0, -10.0];
        let circuit = Circuit { inductance: 2e-4, resistance: 0.08, dt: 5e-6 };
        let mut steps = 0;
        while i.iter().any(|&x| x != 0.0) {
            INVERTER.step_with_open_legs(&mut i, [None; 3], [0.0; 3], 130.0, circuit, [false; 3]);
            assert!(i.iter().sum::<f64>().abs() < 1e-9);
            assert!(i[0] >= 0.0 && i[1] <= 0.0 && i[2] <= 0.0, "no current reverses: {i:?}");
            steps += 1;
            assert!(steps < 1000, "decays within 5 ms");
        }
        // Back-EMF below the bus: the open bridge stays open.
        INVERTER.step_with_open_legs(&mut i, [None; 3], [50.0, -25.0, -25.0], 130.0, circuit, [false; 3]);
        assert_eq!(i, [0.0; 3]);
    }

    #[test]
    fn overspeed_rectifies_into_the_bus() {
        // Line back-EMF 160 V across a 130 V bus: current flows out of the high phase.
        let mut i = [0.0; 3];
        let e = [80.0, -80.0, 0.0];
        for _ in 0..200 {
            let circuit = Circuit { inductance: 2e-4, resistance: 0.08, dt: 5e-6 };
            INVERTER.step_with_open_legs(&mut i, [None; 3], e, 130.0, circuit, [false; 3]);
        }
        assert!(i[0] < -1.0 && i[1] > 1.0, "{i:?}");
        let share = high_side_share(&[LegDrive::Off; 3], &i);
        let battery: f64 = (0..3).map(|k| share[k] * i[k]).sum();
        assert!(battery < 0.0, "the battery is charged");
    }

    #[test]
    fn a_cut_phase_carries_nothing_and_the_others_share_the_current() {
        let circuit = Circuit { inductance: 2e-4, resistance: 0.08, dt: 5e-6 };
        let mut i = [0.0; 3];
        // All legs switching, phase c cut: a and b carry equal and opposite current.
        for _ in 0..8000 {
            INVERTER.step_with_open_legs(
                &mut i,
                [Some(70.0), Some(60.0), Some(65.0)],
                [0.0; 3],
                130.0,
                circuit,
                [false, false, true],
            );
        }
        assert_eq!(i[2], 0.0);
        assert!((i[0] + i[1]).abs() < 1e-9 && (i[0] - 10.0 / 0.16).abs() < 0.1, "{i:?}");
        // Two phases cut: nothing flows.
        let mut i = [5.0, -5.0, 0.0];
        INVERTER.step_with_open_legs(
            &mut i,
            [Some(70.0), Some(60.0), None],
            [0.0; 3],
            130.0,
            circuit,
            [false, true, true],
        );
        assert_eq!(i, [0.0; 3]);
    }
}

//! The rider's controller: target speed to lean, lean to ankle torque.
//!
//! An EUC rider rides by leaning: lean forward and the wheel speeds up to stay under you, lean
//! back and it slows. The controller turns a target speed into a lean from vertical, ramped at
//! a human rate, and the ankles hold that lean with a stiff spring and damper. The ankle torque
//! pushes on the pedals with the opposite sign; that is what the firmware's balance loop feels.

use crate::params::RiderControl;

/// Speed error inside which the rider trims with the integral term, m/s.
const INTEGRAL_BAND_MPS: f64 = 1.0;

/// What the rider wants.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Intent {
    /// Ride at this ground speed, m/s, forward positive.
    Speed(f64),
    /// Hold this lean from vertical, radians, forward positive.
    Lean(f64),
}

/// Rolling resistance and air drag, as decelerations.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Drag {
    /// Rolling resistance, m/s².
    pub rolling: f64,
    /// Air drag per (m/s)², 1/m.
    pub aero: f64,
}

impl Drag {
    /// Speed below which rolling resistance fades out, m/s, so it never pushes a wheel at rest.
    pub const ROLLING_FADE_MPS: f64 = 0.3;

    /// Deceleration at `speed` (m/s), with the sign of the speed.
    pub fn decel(self, speed: f64) -> f64 {
        self.rolling * (speed / Self::ROLLING_FADE_MPS).tanh() + self.aero * speed * speed.abs()
    }
}

/// The speed the rider aims for: `wanted`, unless the pedals tip back while the rider wants to go
/// forward and is not already slowing down. Then the rider eases off, the more the steeper the
/// tiltback, as a rider does when the wheel pushes the toes up: down to a stop, never backwards,
/// and a stopped rider waits for the pedals to level before setting off again.
fn eased(wanted: f64, s: &Sense, c: &RiderControl) -> f64 {
    let tilt = (s.pitch_up - c.tiltback_feel_deg.to_radians()).max(0.0).to_degrees();
    if tilt <= 0.0 || c.tiltback_ease_kmh_per_deg <= 0.0 || wanted <= 0.0 || wanted < s.speed {
        return wanted;
    }
    wanted.min(s.speed - c.tiltback_ease_kmh_per_deg * tilt / 3.6).max(0.0)
}

/// What the rider senses in one step.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Sense {
    /// Step, seconds.
    pub dt: f64,
    /// Ground speed, m/s.
    pub speed: f64,
    /// Lean of the rider from vertical, radians, forward positive.
    pub lean: f64,
    /// Its rate, rad/s.
    pub lean_rate: f64,
    /// What resists motion.
    pub drag: Drag,
    /// Gravity, m/s².
    pub gravity: f64,
    /// The stand holds the wheel: the rider stands still and does not wind up.
    pub held: bool,
    /// The pedals' pitch, radians, nose up positive (tiltback lifts the nose).
    pub pitch_up: f64,
}

/// The rider's controller.
#[derive(Debug, Clone, PartialEq)]
pub struct RiderController {
    control: RiderControl,
    intent: Intent,
    lean_command: f64,
    integral: f64,
}

impl RiderController {
    /// A rider who wants to stand still.
    pub const fn new(control: RiderControl) -> Self {
        Self { control, intent: Intent::Speed(0.0), lean_command: 0.0, integral: 0.0 }
    }

    /// Changes what the rider wants, from the next step on.
    pub const fn set_intent(&mut self, intent: Intent) {
        self.intent = intent;
    }

    /// What the rider wants.
    pub const fn intent(&self) -> Intent {
        self.intent
    }

    /// The lean the ankles are holding, radians from vertical, forward positive.
    pub const fn lean_command(&self) -> f64 {
        self.lean_command
    }

    /// The ankle torque for this step, N·m about the lean hinge (positive tips the rider
    /// forward).
    pub(crate) fn ankle_torque(&mut self, s: &Sense) -> f64 {
        let c = self.control;
        let max_lean = c.lean_max_deg.to_radians();
        let target = match self.intent {
            Intent::Lean(lean) => lean,
            Intent::Speed(wanted) => {
                let speed = eased(wanted, s, &c);
                let brake_max = if speed < wanted { c.brake_max.min(c.tiltback_ease_brake) } else { c.brake_max };
                let error = speed - s.speed;
                let accel = c.speed_kp * error + c.speed_ki * self.integral;
                let limited = accel.clamp(-brake_max, c.accel_max);
                let saturated = accel > c.accel_max || accel < -brake_max;
                // Integrate only near the target and while the request is not saturated, so the
                // integral trims the steady speed and never winds up on the way there.
                if !s.held && !saturated && error.abs() < INTEGRAL_BAND_MPS {
                    self.integral += error * s.dt;
                }
                // Lean also for the drag the target speed will meet, so the rider holds it without
                // a steady error.
                ((limited + s.drag.decel(speed)) / s.gravity).atan()
            }
        };
        let target = target.clamp(-max_lean, max_lean);
        let step = c.lean_rate_deg_s.to_radians() * s.dt;
        self.lean_command += (target - self.lean_command).clamp(-step, step);
        let torque = c.lean_kp * (self.lean_command - s.lean) - c.lean_kd * s.lean_rate;
        torque.clamp(-c.ankle_max_nm, c.ankle_max_nm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sense(speed: f64) -> Sense {
        Sense {
            dt: 0.001,
            speed,
            lean: 0.0,
            lean_rate: 0.0,
            drag: Drag::default(),
            gravity: 9.81,
            held: false,
            pitch_up: 0.0,
        }
    }

    #[test]
    fn leans_forward_to_speed_up_at_a_human_rate() {
        let mut rider = RiderController::new(RiderControl::default());
        rider.set_intent(Intent::Speed(5.0));
        let torque = rider.ankle_torque(&sense(0.0));
        assert!(torque > 0.0, "tips forward: {torque}");
        let rate = RiderControl::default().lean_rate_deg_s.to_radians() * 0.001;
        assert!((rider.lean_command() - rate).abs() < 1e-12, "ramped: {}", rider.lean_command());
        for _ in 0..5000 {
            rider.ankle_torque(&sense(0.0));
        }
        // a = 1.5 m/s² at most: atan(1.5 / 9.81) = 8.7°.
        assert!((rider.lean_command().to_degrees() - 8.69).abs() < 0.05, "{}", rider.lean_command().to_degrees());
    }

    #[test]
    fn leans_back_to_brake_and_holds_a_lean_on_request() {
        let mut rider = RiderController::new(RiderControl::default());
        for _ in 0..5000 {
            rider.ankle_torque(&sense(5.0));
        }
        assert!(rider.lean_command() < -0.2, "leans back: {}", rider.lean_command());
        rider.set_intent(Intent::Lean(0.05));
        for _ in 0..5000 {
            rider.ankle_torque(&sense(5.0));
        }
        assert!((rider.lean_command() - 0.05).abs() < 1e-9);
    }

    #[test]
    fn eases_off_when_the_pedals_tip_back() {
        let c = RiderControl::default();
        let riding = |pitch_up_deg: f64| Sense { pitch_up: f64::to_radians(pitch_up_deg), ..sense(10.0) };
        assert_eq!(eased(12.0, &riding(1.0), &c), 12.0, "a little tilt is fine");
        let eased_speed = eased(12.0, &riding(6.0), &c);
        assert!((eased_speed - (10.0 - 1.5 * 4.0 / 3.6)).abs() < 1e-9, "4° beyond the feel: {eased_speed}");
        assert_eq!(eased(5.0, &riding(6.0), &c), 5.0, "already braking: no change");
        assert_eq!(eased(12.0, &riding(30.0), &c), 0.0, "a steep tiltback stops the rider, never reverses");
        let stopped = Sense { pitch_up: f64::to_radians(6.0), ..sense(-0.1) };
        assert_eq!(eased(12.0, &stopped, &c), 0.0, "stopped: waits for the pedals to level");
        assert_eq!(eased(-2.0, &stopped, &c), -2.0, "riding backwards: no change");
        let none = RiderControl { tiltback_ease_kmh_per_deg: 0.0, ..c };
        assert_eq!(eased(12.0, &riding(6.0), &none), 12.0, "ignoring it rides through");
    }

    #[test]
    fn holds_still_without_winding_up_on_the_stand() {
        let mut rider = RiderController::new(RiderControl::default());
        rider.set_intent(Intent::Speed(1.0));
        let held = Sense { held: true, ..sense(0.0) };
        for _ in 0..10_000 {
            rider.ankle_torque(&held);
        }
        assert_eq!(rider.integral, 0.0);
    }
}

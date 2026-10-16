//! A one-degree-of-freedom wheel for testing the plant without the body physics.

/// A wheel on a test stand: held still, or spinning freely against friction.
///
/// The simulator's body physics replaces it on a real run. Use it to check the drive
/// on its own: hold the wheel and look at the torque a current command makes, or let it spin and
/// watch it accelerate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelRig {
    /// Moment of inertia about the axle (wheel, tire and rotor), kg·m².
    pub inertia: f64,
    /// Viscous friction, newton-metre per rad/s.
    pub viscous: f64,
    /// Dry friction, newton-metre.
    pub coulomb: f64,
    /// Held still: the angle and speed stay put whatever the torque.
    pub held: bool,
    angle: f64,
    speed: f64,
}

impl WheelRig {
    /// A wheel at rest at angle 0.
    pub const fn new(inertia: f64, viscous: f64, coulomb: f64, held: bool) -> Self {
        Self { inertia, viscous, coulomb, held, angle: 0.0, speed: 0.0 }
    }

    /// Angle, radians (forward positive).
    pub const fn angle(&self) -> f64 {
        self.angle
    }

    /// Speed, rad/s (forward positive).
    pub const fn speed(&self) -> f64 {
        self.speed
    }

    /// Advances `h` seconds under the drive `torque` (semi-implicit Euler).
    pub fn step(&mut self, torque: f64, h: f64) {
        if self.held {
            return;
        }
        let friction = self.viscous * self.speed + self.coulomb * (self.speed / 0.1).clamp(-1.0, 1.0);
        self.speed += (torque - friction) / self.inertia * h;
        self.angle += self.speed * h;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spins_up_and_coasts_down() {
        let mut wheel = WheelRig::new(0.5, 0.01, 0.0, false);
        for _ in 0..1000 {
            wheel.step(5.0, 1e-3);
        }
        assert!((wheel.speed() - 10.0).abs() < 0.2, "{}", wheel.speed());
        let mut held = WheelRig { held: true, ..wheel };
        held.step(100.0, 1.0);
        assert_eq!(held.speed(), wheel.speed());
        for _ in 0..1_000_000 {
            wheel.step(0.0, 1e-3);
        }
        assert!(wheel.speed().abs() < 0.01);
    }
}

//! A small deterministic random source for injected noise: the same seed gives the same run.

/// `SplitMix64`, with Box-Muller for Gaussian samples.
#[derive(Debug, Clone)]
pub(crate) struct Rng {
    state: u64,
}

impl Rng {
    pub(crate) const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1].
    pub(crate) fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 / (1_u64 << 53) as f64
    }

    /// A normal sample with standard deviation `sigma` (0 when `sigma` is 0, without drawing).
    pub(crate) fn gaussian(&mut self, sigma: f64) -> f64 {
        if sigma == 0.0 {
            return 0.0;
        }
        let (u1, u2) = (self.unit(), self.unit());
        sigma * (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

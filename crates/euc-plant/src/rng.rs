//! A small deterministic random source for sensor noise: the same seed gives the same run.

/// `SplitMix64`, with Box-Muller for Gaussian samples.
#[derive(Debug, Clone)]
pub(crate) struct Rng {
    state: u64,
}

impl Rng {
    pub(crate) const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1].
    fn unit(&mut self) -> f64 {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_and_roughly_normal() {
        let mut a = Rng::new(7);
        let xs: Vec<f64> = (0..20_000).map(|_| a.gaussian(2.0)).collect();
        let mut b = Rng::new(7);
        assert_eq!([b.gaussian(2.0), b.gaussian(2.0)], [xs[0], xs[1]], "same seed, same stream");
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        let var = xs.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.05 && (var.sqrt() - 2.0).abs() < 0.05, "mean {mean}, sd {}", var.sqrt());
        assert_eq!(Rng::new(1).gaussian(0.0), 0.0);
    }
}

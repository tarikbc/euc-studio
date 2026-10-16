//! Three digital hall sensors that report the rotor's electrical sector.

use serde::Deserialize;

use crate::rng::Rng;

/// Hall code of each 60° sector of the angle the firmware assumes, starting at 0°. Bit 0 is
/// sensor a, bit 1 sensor b, bit 2 sensor c.
///
/// This is the stock Begode firmware's table (sector centres 6: +30°, 4: +90°, 5: +150°,
/// 1: −150°, 3: −90°, 2: −30°), so an increasing angle gives the order 6, 4, 5, 1, 3, 2: the
/// firmware's positive direction 5 → 1 → 3 → 2 → 6 → 4.
pub const SECTOR_CODES: [u8; 6] = [6, 4, 5, 1, 3, 2];

/// Hall parameters (`[halls]` in `plant.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HallParams {
    /// Angle of the sensors' sector pattern relative to the rotor flux, electrical degrees: the
    /// pattern reads sector `(θ + offset) / 60°`. A misplaced sensor board is a different offset.
    pub offset_deg: f64,
    /// Standard deviation of the noise on the sensed angle at each reading, electrical degrees:
    /// edges jitter, and near an edge a line can chatter.
    #[serde(default)]
    pub noise_deg: f64,
    /// Lines stuck at a level whatever the rotor does (a broken sensor or wire).
    #[serde(default)]
    pub stuck: StuckLines,
}

/// Hall lines held at a fixed level: `true` high, `false` low, absent = working.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StuckLines {
    /// Sensor a (code bit 0).
    pub a: Option<bool>,
    /// Sensor b (code bit 1).
    pub b: Option<bool>,
    /// Sensor c (code bit 2).
    pub c: Option<bool>,
}

impl HallParams {
    /// The code the sensors give at electrical angle `theta` (radians), with `rng` for noise.
    pub(crate) fn code(&self, theta: f64, rng: &mut Rng) -> u8 {
        let degrees = theta.to_degrees() + self.offset_deg + rng.gaussian(self.noise_deg);
        let mut code = sector_code(degrees);
        for (bit, stuck) in [self.stuck.a, self.stuck.b, self.stuck.c].into_iter().enumerate() {
            match stuck {
                Some(true) => code |= 1 << bit,
                Some(false) => code &= !(1 << bit),
                None => {}
            }
        }
        code
    }
}

/// The code of the sector that holds `degrees` of the firmware's angle.
pub fn sector_code(degrees: f64) -> u8 {
    SECTOR_CODES[(degrees.rem_euclid(360.0) / 60.0) as usize % 6]
}

#[cfg(test)]
mod tests {
    use super::*;

    const HALLS: HallParams =
        HallParams { offset_deg: -90.0, noise_deg: 0.0, stuck: StuckLines { a: None, b: None, c: None } };

    #[test]
    fn sector_centres_match_the_firmware_table() {
        for (centre, code) in [(30.0, 6), (90.0, 4), (150.0, 5), (-150.0, 1), (-90.0, 3), (-30.0, 2)] {
            assert_eq!(sector_code(centre), code, "{centre}°");
        }
        assert_eq!((sector_code(359.9), sector_code(360.0), sector_code(-0.1)), (2, 6, 2));
    }

    #[test]
    fn rolling_forward_runs_the_firmware_negative_order() {
        // With direction −1 the electrical angle falls as the wheel rolls forward: the codes then
        // run 4 → 6 → 2 → 3 → 1 → 5, the reverse of the firmware's positive order.
        let mut rng = Rng::new(1);
        let mut seen = Vec::new();
        for step in 0..720 {
            let theta = -f64::from(step).to_radians() + 0.25;
            let code = HALLS.code(theta, &mut rng);
            if seen.last() != Some(&code) {
                seen.push(code);
            }
        }
        let start = seen.iter().position(|&c| c == 4).unwrap();
        assert_eq!(seen[start..start + 6], [4, 6, 2, 3, 1, 5]);
    }

    #[test]
    fn offset_and_stuck_lines() {
        let mut rng = Rng::new(1);
        // Electrical 120° with the −90° offset reads firmware 30°: code 6.
        assert_eq!(HALLS.code(120f64.to_radians(), &mut rng), 6);
        let shifted = HallParams { offset_deg: -30.0, ..HALLS };
        assert_eq!(shifted.code(120f64.to_radians(), &mut rng), 4);
        let broken = HallParams { stuck: StuckLines { a: Some(false), b: None, c: Some(true) }, ..HALLS };
        assert_eq!(broken.code(120f64.to_radians(), &mut rng), 6, "c already high, a already low");
        assert_eq!(broken.code(240f64.to_radians(), &mut rng), 4, "5 with a held low");
    }

    #[test]
    fn noise_makes_edges_chatter() {
        let noisy = HallParams { noise_deg: 2.0, ..HALLS };
        let mut rng = Rng::new(9);
        // 1° from an edge, 2° of noise: both codes show up.
        let theta = (90.0 + 1.0f64).to_radians();
        let codes: std::collections::BTreeSet<u8> = (0..200).map(|_| noisy.code(theta, &mut rng)).collect();
        assert_eq!(codes.len(), 2, "{codes:?}");
    }
}

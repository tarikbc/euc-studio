//! The ground the wheel rides on: flat, a constant slope, bumps and potholes.
//!
//! A scenario's `[terrain]` table describes a profile along the path, as height against world x
//! (the wheel starts at x = 0 heading +x):
//!
//! ```toml
//! [terrain]
//! slope_pct = 8.0        # grade, + = uphill along the path
//! slope_start_m = 40.0   # where it starts (default 0: from the start)
//! slope_ramp_m = 10.0    # the grade builds up over this distance (default 10)
//! length_m = 300.0       # how far the ground goes (default 300)
//!
//! [[terrain.feature]]
//! kind = "bump"          # or "pothole"
//! at_m = 60.0            # start along the path, metres
//! height_m = 0.06        # depth_m for a pothole
//! length_m = 0.5
//! ```
//!
//! Flat ground is MuJoCo's infinite floor plane, as before. Anything else is a heightfield strip:
//! the profile sampled every 2 cm along x and extruded 1.5 m to each side, from 5 m behind the
//! start to `length_m`. Beyond its ends there is no ground. A bump is a smooth cosine hump; a
//! pothole has walls as steep as the 2 cm sampling allows.

use serde::Deserialize;

/// Heightfield column spacing, metres.
const COLUMN_M: f64 = 0.02;
/// Ground behind the start, metres.
const BEHIND_M: f64 = 5.0;
/// Half width of the strip, metres.
const HALF_WIDTH_M: f64 = 1.5;
/// Thickness of the heightfield below its lowest point, metres.
const BASE_M: f64 = 0.5;
/// The floor of flat ground (the MJCF line every model had before terrain).
const FLOOR_PLANE: &str =
    r#"    <geom name="floor" type="plane" size="0 0 1" friction="0.5 0 0" condim="3" conaffinity="3"/>"#;

/// The ground (`[terrain]` in a scenario).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Terrain {
    /// Grade of the slope, percent: rise per 100 m along the path, + uphill.
    pub slope_pct: f64,
    /// Where the slope starts, metres along the path.
    pub slope_start_m: f64,
    /// Distance over which the grade builds up from 0, metres: a road's vertical curve, so the
    /// wheel meets no kink.
    pub slope_ramp_m: f64,
    /// How far the ground goes, metres along the path.
    pub length_m: f64,
    /// Bumps and potholes.
    #[serde(rename = "feature")]
    pub features: Vec<Feature>,
}

impl Default for Terrain {
    /// Flat ground.
    fn default() -> Self {
        Self { slope_pct: 0.0, slope_start_m: 0.0, slope_ramp_m: 10.0, length_m: 300.0, features: Vec::new() }
    }
}

/// Something on the ground (`[[terrain.feature]]`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Feature {
    /// A smooth hump: `height_m · (1 − cos)/2` over its length (a speed bump, a root).
    Bump {
        /// Where it starts, metres along the path.
        at_m: f64,
        /// Its height, metres.
        height_m: f64,
        /// Its length along the path, metres.
        length_m: f64,
    },
    /// A hole with steep walls.
    Pothole {
        /// Where it starts, metres along the path.
        at_m: f64,
        /// Its depth, metres.
        depth_m: f64,
        /// Its length along the path, metres.
        length_m: f64,
    },
}

impl Feature {
    /// Where it starts and its length, metres.
    const fn span(&self) -> (f64, f64) {
        match *self {
            Self::Bump { at_m, length_m, .. } | Self::Pothole { at_m, length_m, .. } => (at_m, length_m),
        }
    }

    /// Height it adds to the ground at `x_m`, metres.
    fn height(&self, x_m: f64) -> f64 {
        let (at, length) = self.span();
        let along = (x_m - at) / length;
        if !(0.0..=1.0).contains(&along) {
            return 0.0;
        }
        match *self {
            Self::Bump { height_m, .. } => height_m * (1.0 - (std::f64::consts::TAU * along).cos()) / 2.0,
            Self::Pothole { depth_m, .. } => -depth_m,
        }
    }
}

impl Terrain {
    /// Flat ground.
    pub fn flat() -> Self {
        Self::default()
    }

    /// No slope and nothing on the ground.
    pub fn is_flat(&self) -> bool {
        self.slope_pct == 0.0 && self.features.is_empty()
    }

    /// Ground height at `x_m` along the path, metres (0 at the start).
    pub fn height(&self, x_m: f64) -> f64 {
        let (grade, along, ramp) = (self.slope_pct / 100.0, (x_m - self.slope_start_m).max(0.0), self.slope_ramp_m);
        // A parabola over the ramp, then the full grade: the slope changes without a kink.
        let slope = if along < ramp { grade * along * along / (2.0 * ramp) } else { grade * (along - ramp / 2.0) };
        slope + self.features.iter().map(|f| f.height(x_m)).sum::<f64>()
    }

    /// Checks ranges that the types cannot.
    pub fn validate(&self) -> Result<(), String> {
        let finite =
            [self.slope_pct, self.slope_start_m, self.slope_ramp_m, self.length_m].iter().all(|v| v.is_finite());
        if !finite
            || self.slope_pct.abs() > 50.0
            || self.slope_start_m < 0.0
            || self.slope_ramp_m < 0.0
            || self.length_m < 1.0
        {
            return Err(
                "terrain: slope_pct within ±50, slope_start_m and slope_ramp_m not negative, length_m at least 1"
                    .into(),
            );
        }
        for feature in &self.features {
            let (at, length) = feature.span();
            let size = match *feature {
                Feature::Bump { height_m, .. } => height_m,
                Feature::Pothole { depth_m, .. } => depth_m,
            };
            if !(at >= 1.0 && at + length <= self.length_m && length >= 4.0 * COLUMN_M && size > 0.0 && size < 1.0) {
                return Err(format!(
                    "terrain feature at {at} m: needs 1 m <= at_m, at_m + length_m <= {} m, length_m >= {} m and a \
                     height or depth between 0 and 1 m",
                    self.length_m,
                    4.0 * COLUMN_M
                ));
            }
        }
        Ok(())
    }

    /// The heightfield's MJCF `<asset>` element, or `None` for flat ground.
    pub(crate) fn asset(&self) -> Option<String> {
        if self.is_flat() {
            return None;
        }
        let grid = self.grid();
        let row: Vec<String> = grid.heights.iter().map(|z| format!("{:.5}", z - grid.low)).collect();
        let row = row.join(" ");
        Some(format!(
            r#"  <asset><hfield name="terrain" nrow="2" ncol="{}" size="{} {HALF_WIDTH_M} {} {BASE_M}" elevation="{row} {row}"/></asset>"#,
            grid.heights.len(),
            grid.half_length,
            (grid.high - grid.low).max(1e-6),
        ))
    }

    /// The MJCF ground geom.
    pub(crate) fn ground(&self) -> String {
        if self.is_flat() {
            return FLOOR_PLANE.to_owned();
        }
        let grid = self.grid();
        format!(
            r#"    <geom name="floor" type="hfield" hfield="terrain" pos="{} 0 {}" friction="0.5 0 0" condim="3" conaffinity="3"/>"#,
            grid.centre, grid.low
        )
    }

    /// The profile sampled for the heightfield.
    fn grid(&self) -> Grid {
        let columns = ((self.length_m + BEHIND_M) / COLUMN_M).ceil() as usize + 1;
        let step = (self.length_m + BEHIND_M) / (columns - 1) as f64;
        let heights: Vec<f64> = (0..columns).map(|c| self.height(-BEHIND_M + c as f64 * step)).collect();
        let low = heights.iter().copied().fold(0.0, f64::min);
        let high = heights.iter().copied().fold(0.0, f64::max);
        Grid {
            heights,
            low,
            high,
            half_length: f64::midpoint(self.length_m, BEHIND_M),
            centre: (self.length_m - BEHIND_M) / 2.0,
        }
    }
}

/// The profile at the heightfield's columns.
struct Grid {
    heights: Vec<f64>,
    low: f64,
    high: f64,
    half_length: f64,
    centre: f64,
}

#[cfg(test)]
mod tests {
    use mujoco_rs::prelude::*;

    use super::*;

    fn pothole_and_bump() -> Terrain {
        Terrain {
            features: vec![
                Feature::Pothole { at_m: 2.0, depth_m: 0.05, length_m: 0.6 },
                Feature::Bump { at_m: 4.0, height_m: 0.06, length_m: 0.5 },
            ],
            length_m: 10.0,
            ..Terrain::flat()
        }
    }

    #[test]
    fn profile_and_validation() {
        let t = pothole_and_bump();
        assert_eq!((t.height(1.9), t.height(2.3), t.height(3.0)), (0.0, -0.05, 0.0));
        assert!((t.height(4.25) - 0.06).abs() < 1e-12, "the bump peaks in its middle");
        let hill = Terrain { slope_pct: 8.0, slope_start_m: 10.0, ..Terrain::flat() };
        assert_eq!(hill.height(5.0), 0.0);
        assert!((hill.height(15.0) - 0.08 * 25.0 / 20.0).abs() < 1e-12, "half way up the ramp: a parabola");
        assert!((hill.height(60.0) - 0.08 * 45.0).abs() < 1e-12, "then the full grade, 5 m behind a kinked slope");
        assert!(t.validate().is_ok() && hill.validate().is_ok());
        let late = Terrain { length_m: 4.2, ..pothole_and_bump() };
        assert!(late.validate().unwrap_err().contains("at_m + length_m <= 4.2 m"));
        let parsed: Terrain = toml::from_str(
            "slope_pct = -5.0\n[[feature]]\nkind = \"pothole\"\nat_m = 30.0\ndepth_m = 0.04\nlength_m = 0.5\n",
        )
        .unwrap();
        assert_eq!(parsed.features[0], Feature::Pothole { at_m: 30.0, depth_m: 0.04, length_m: 0.5 });
        assert!(toml::from_str::<Terrain>("[[feature]]\nkind = \"crater\"\nat_m = 3.0").is_err());
    }

    #[test]
    fn flat_ground_is_the_old_plane() {
        assert_eq!(Terrain::flat().asset(), None);
        assert_eq!(Terrain::flat().ground(), FLOOR_PLANE);
    }

    /// Drops a box of half size 0.1 m at `x` onto `terrain` and returns where it settles.
    fn settle(terrain: &Terrain, x: f64) -> f64 {
        let xml = format!(
            "<mujoco>\n{}\n  <worldbody>\n{}\n    <body name=\"b\" pos=\"{x} 0 0.6\"><freejoint/>\
             <geom type=\"box\" size=\"0.1 0.1 0.1\" mass=\"1\" friction=\"1 0 0\"/></body>\n  </worldbody>\n</mujoco>",
            terrain.asset().unwrap_or_default(),
            terrain.ground()
        );
        let model = MjModel::from_xml_string(&xml).unwrap();
        let body = model.name_to_id(MjtObj::mjOBJ_BODY, "b").unwrap();
        let mut data = MjData::new(Box::new(model));
        for _ in 0..3000 {
            data.step();
        }
        data.xpos()[body][2]
    }

    /// The ground's height at `x` as MuJoCo sees it: a ray cast straight down onto the floor geom.
    fn surface(terrain: &Terrain, x: f64) -> f64 {
        let xml = format!(
            "<mujoco>\n{}\n  <worldbody>\n{}\n  </worldbody>\n</mujoco>",
            terrain.asset().unwrap_or_default(),
            terrain.ground()
        );
        let mut data = MjData::new(Box::new(MjModel::from_xml_string(&xml).unwrap()));
        data.forward();
        let (hit, distance) = data.ray(&[x, 0.0, 100.0], &[0.0, 0.0, -1.0], None, true, None, None);
        assert!(hit.is_some(), "the ray at {x} m meets the ground");
        100.0 - distance
    }

    #[test]
    fn the_heightfield_surface_is_the_profile_within_a_millimetre() {
        let hill = Terrain { slope_pct: 20.0, slope_start_m: 40.0, length_m: 300.0, ..Terrain::flat() };
        let downhill = Terrain { slope_pct: -2.0, slope_start_m: 25.0, length_m: 250.0, ..Terrain::flat() };
        let pothole = Terrain {
            features: vec![Feature::Pothole { at_m: 50.0, depth_m: 0.02, length_m: 0.5 }],
            length_m: 120.0,
            ..Terrain::flat()
        };
        let cases: [(&Terrain, &[f64]); 4] = [
            (&hill, &[-3.0, 0.0, 30.0, 45.0, 60.0, 150.0, 290.0]),
            (&downhill, &[-3.0, 10.0, 40.0, 100.0, 240.0]),
            (&pothole, &[-3.0, 20.0, 50.25, 80.0]),
            (&pothole_and_bump(), &[1.0, 2.3, 4.25, 7.0]),
        ];
        for (terrain, xs) in cases {
            for &x in xs {
                let (ground, profile) = (surface(terrain, x), terrain.height(x));
                assert!((ground - profile).abs() < 0.001, "at {x} m: MuJoCo {ground} m, profile {profile} m");
            }
        }
    }

    #[test]
    fn mujoco_puts_the_ground_where_the_profile_says() {
        let t = pothole_and_bump();
        // A 0.2 m box in a 0.6 m pothole sits on its bottom; on the bump's crest it sits 6 cm up.
        assert!((settle(&t, 2.3) - (0.1 - 0.05)).abs() < 0.005, "{}", settle(&t, 2.3));
        assert!((settle(&t, 4.25) - (0.1 + 0.06)).abs() < 0.01, "{}", settle(&t, 4.25));
        assert!((settle(&t, 7.0) - 0.1).abs() < 0.005);
        // 10 % uphill from 1 m: at 6 m the ground is 0.5 m up (the box's centre sits 0.1 m above
        // the slope, normal to it).
        let hill =
            Terrain { slope_pct: 10.0, slope_start_m: 1.0, slope_ramp_m: 0.0, length_m: 20.0, ..Terrain::flat() };
        assert!((settle(&hill, 6.0) - (0.5 + 0.1 * 1.01_f64.sqrt())).abs() < 0.03, "{}", settle(&hill, 6.0));
    }
}

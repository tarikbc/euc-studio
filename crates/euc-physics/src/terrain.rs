//! The ground the wheel rides on: flat, a constant slope, bumps, potholes and curbs.
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
//!
//! [[terrain.feature]]
//! kind = "curb"          # a step up onto a sidewalk
//! at_m = 80.0            # where its face crosses the path, metres
//! height_m = 0.12
//! yaw_deg = 30.0         # the face's angle to square on (default 0)
//! ```
//!
//! Flat ground is MuJoCo's infinite floor plane, as before. Anything else is a heightfield strip:
//! the profile sampled every 2 cm along x and extruded 1.5 m to each side, from 5 m behind the
//! start to `length_m`. Beyond its ends there is no ground. A bump is a smooth cosine hump; a
//! pothole has walls as steep as the 2 cm sampling allows. A curb is a box on the strip, with a
//! sharp edge at any angle: 4 m deep, wide enough that the path leaves it through its back, and
//! reaching 15 cm below the road as the world's curb stones do.

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
    r#"    <geom name="floor" type="plane" size="0 0 1" friction="0.5 0 0" condim="3" conaffinity="3" group="3"/>"#;

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
    /// Bumps, potholes and curbs.
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
    /// A step up onto a sidewalk: a box [`CURB_DEPTH_M`] deep whose face crosses the path at
    /// `at_m`, turned about the vertical through that point. It is wide enough that the path
    /// leaves it through its back at any yaw ([`CURB_WIDTH_M`] plus what the yaw sweeps).
    Curb {
        /// Where its face crosses the path, metres along the path.
        at_m: f64,
        /// Its height above the road, metres.
        height_m: f64,
        /// The face's angle to square on, degrees, left positive (0: met square on).
        #[serde(default)]
        yaw_deg: f64,
    },
}

/// Depth of a curb's box behind its face, metres.
const CURB_DEPTH_M: f64 = 4.0;
/// Width of a square-on curb's box along its face, metres.
const CURB_WIDTH_M: f64 = 6.0;
/// How far a curb's box reaches below the road, metres (as the world's curb stones).
const CURB_BELOW_M: f64 = 0.15;
/// Largest angle between a curb's face and square on, degrees.
const CURB_MAX_YAW_DEG: f64 = 80.0;

impl Feature {
    /// Where it starts and its length along the path, metres (a turned curb's box covers more of
    /// the path than its depth).
    fn span(&self) -> (f64, f64) {
        match *self {
            Self::Bump { at_m, length_m, .. } | Self::Pothole { at_m, length_m, .. } => (at_m, length_m),
            Self::Curb { at_m, yaw_deg, .. } => (at_m, CURB_DEPTH_M / yaw_deg.to_radians().cos()),
        }
    }

    /// A box on the ground (a curb), not part of the heightfield.
    const fn is_solid(&self) -> bool {
        matches!(self, Self::Curb { .. })
    }

    /// Height it adds to the ground at `x_m`, metres. A curb's counts along the path's centre line
    /// (for drawing; its box is what the wheel meets).
    fn height(&self, x_m: f64) -> f64 {
        let (at, length) = self.span();
        let along = (x_m - at) / length;
        if !(0.0..=1.0).contains(&along) {
            return 0.0;
        }
        match *self {
            Self::Bump { height_m, .. } => height_m * (1.0 - (std::f64::consts::TAU * along).cos()) / 2.0,
            Self::Pothole { depth_m, .. } => -depth_m,
            Self::Curb { height_m, .. } => height_m,
        }
    }

    /// A curb's box as an MJCF geom (`curb_<index>`), with the floor's contact settings, so the
    /// tire's friction decides as on the road; `None` for the heightfield's features.
    fn solid(&self, index: usize) -> Option<String> {
        let Self::Curb { at_m, height_m, yaw_deg } = *self else { return None };
        let yaw = yaw_deg.to_radians();
        let (half_depth, half_height) = (CURB_DEPTH_M / 2.0, f64::midpoint(height_m, CURB_BELOW_M));
        // The path reaches the back 4·tan(yaw) to the side of where it crossed the face.
        let half_width = CURB_WIDTH_M / 2.0 + CURB_DEPTH_M * yaw.abs().tan();
        // A quaternion, not Euler angles: the line reads the same whatever the model's angle unit.
        Some(format!(
            r#"    <geom name="curb_{index}" type="box" size="{half_depth} {half_width} {half_height}" pos="{} {} {}" quat="{} 0 0 {}" friction="0.5 0 0" condim="3" conaffinity="3" group="3"/>"#,
            at_m + half_depth * yaw.cos(),
            half_depth * yaw.sin(),
            height_m - half_height,
            (yaw / 2.0).cos(),
            (yaw / 2.0).sin(),
        ))
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

    /// Ground height at `x_m` along the path, metres (0 at the start). Curbs count along the path's
    /// centre line.
    pub fn height(&self, x_m: f64) -> f64 {
        self.road(x_m) + self.features.iter().filter(|f| f.is_solid()).map(|f| f.height(x_m)).sum::<f64>()
    }

    /// The heightfield's height at `x_m`, metres: the slope, bumps and potholes, not the curbs.
    fn road(&self, x_m: f64) -> f64 {
        let (grade, along, ramp) = (self.slope_pct / 100.0, (x_m - self.slope_start_m).max(0.0), self.slope_ramp_m);
        // A parabola over the ramp, then the full grade: the slope changes without a kink.
        let slope = if along < ramp { grade * along * along / (2.0 * ramp) } else { grade * (along - ramp / 2.0) };
        slope + self.features.iter().filter(|f| !f.is_solid()).map(|f| f.height(x_m)).sum::<f64>()
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
                Feature::Bump { height_m, .. } | Feature::Curb { height_m, .. } => height_m,
                Feature::Pothole { depth_m, .. } => depth_m,
            };
            // A turned curb's face crosses the strip's edges ahead of and behind `at_m`: it must stay
            // ahead of the start across the whole strip.
            if let Feature::Curb { yaw_deg, .. } = *feature
                && !(yaw_deg.is_finite()
                    && yaw_deg.abs() <= CURB_MAX_YAW_DEG
                    && at - HALF_WIDTH_M * yaw_deg.to_radians().abs().tan() >= 1.0)
            {
                return Err(format!(
                    "terrain curb at {at} m: yaw_deg within ±{CURB_MAX_YAW_DEG}, and its face 1 m or more ahead \
                     of the start across the strip (at_m >= 1 + {HALF_WIDTH_M} tan|yaw|)"
                ));
            }
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

    /// The MJCF ground geoms: the floor, then a box per curb.
    pub(crate) fn ground(&self) -> String {
        if self.is_flat() {
            return FLOOR_PLANE.to_owned();
        }
        let grid = self.grid();
        let mut ground = format!(
            r#"    <geom name="floor" type="hfield" hfield="terrain" pos="{} 0 {}" friction="0.5 0 0" condim="3" conaffinity="3" group="3"/>"#,
            grid.centre, grid.low
        );
        for solid in self.features.iter().enumerate().filter_map(|(index, f)| f.solid(index)) {
            ground.push('\n');
            ground.push_str(&solid);
        }
        ground
    }

    /// The names of the ground geoms (`ground`): the floor, then a box per curb.
    pub(crate) fn geom_names(&self) -> Vec<String> {
        let curbs =
            self.features.iter().enumerate().filter(|(_, f)| f.is_solid()).map(|(index, _)| format!("curb_{index}"));
        std::iter::once("floor".to_owned()).chain(curbs).collect()
    }

    /// The road's profile (`road`) sampled for the heightfield.
    fn grid(&self) -> Grid {
        let columns = ((self.length_m + BEHIND_M) / COLUMN_M).ceil() as usize + 1;
        let step = (self.length_m + BEHIND_M) / (columns - 1) as f64;
        let heights: Vec<f64> = (0..columns).map(|c| self.road(-BEHIND_M + c as f64 * step)).collect();
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
        surface_at(terrain, x, 0.0)
    }

    /// The ground's height at (`x`, `y`) as MuJoCo sees it: a ray cast straight down onto the ground
    /// geoms.
    fn surface_at(terrain: &Terrain, x: f64, y: f64) -> f64 {
        let xml = format!(
            "<mujoco>\n{}\n  <worldbody>\n{}\n  </worldbody>\n</mujoco>",
            terrain.asset().unwrap_or_default(),
            terrain.ground()
        );
        let mut data = MjData::new(Box::new(MjModel::from_xml_string(&xml).unwrap()));
        data.forward();
        let (hit, distance) = data.ray(&[x, y, 100.0], &[0.0, 0.0, -1.0], None, true, None, None);
        assert!(hit.is_some(), "the ray at ({x}, {y}) m meets the ground");
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

    /// A 12 cm curb whose face crosses the path at `at_m`, on a strip `length_m` long.
    fn curb_at(at_m: f64, yaw_deg: f64, length_m: f64) -> Terrain {
        Terrain { features: vec![Feature::Curb { at_m, height_m: 0.12, yaw_deg }], length_m, ..Terrain::flat() }
    }

    /// A 12 cm curb whose face crosses the path at 3 m.
    fn curb(yaw_deg: f64) -> Terrain {
        curb_at(3.0, yaw_deg, 10.0)
    }

    #[test]
    fn a_curb_is_a_box_on_the_road_with_its_face_at_its_angle() {
        let parsed: Terrain =
            toml::from_str("length_m = 10.0\n[[feature]]\nkind = \"curb\"\nat_m = 3.0\nheight_m = 0.12\n").unwrap();
        assert_eq!(parsed, curb(0.0), "square on unless a yaw is given");
        assert!(curb(0.0).validate().is_ok() && curb_at(10.0, -80.0, 40.0).validate().is_ok());
        assert!(curb(85.0).validate().unwrap_err().contains("yaw_deg within ±80"));
        // At 60° the face crosses the strip's right edge 1.5·tan 60° = 2.6 m before 3 m: too near
        // the start.
        assert!(curb(60.0).validate().unwrap_err().contains("ahead of the start"));
        let short = Terrain { length_m: 6.0, ..curb(0.0) };
        assert!(
            short.validate().unwrap_err().contains("at_m + length_m <= 6 m"),
            "the box reaches 4 m behind its face"
        );
        let t = curb(0.0);
        assert_eq!((t.height(2.9), t.height(3.1), t.height(7.5)), (0.0, 0.12, 0.0), "the drawn profile");
        // The heightfield stays at the road; the box is the step and the sidewalk behind it.
        assert!(surface(&t, 2.95).abs() < 0.001, "{}", surface(&t, 2.95));
        assert!((surface(&t, 3.05) - 0.12).abs() < 0.001, "{}", surface(&t, 3.05));
        assert!(surface(&t, 7.2).abs() < 0.001, "the road again behind the box");
        // Turned 30° about the point where it crosses the path: 1 m to the left the face lies
        // tan 30° m nearer, at 2.42 m.
        let turned = curb(30.0);
        assert!((surface_at(&turned, 3.05, 0.0) - 0.12).abs() < 0.001);
        assert!(surface_at(&turned, 2.37, 1.0).abs() < 0.001, "{}", surface_at(&turned, 2.37, 1.0));
        assert!((surface_at(&turned, 2.47, 1.0) - 0.12).abs() < 0.001, "{}", surface_at(&turned, 2.47, 1.0));
        // At 60° the path rides the box for 4 m / cos 60° = 8 m and leaves it through its back,
        // not its side; the drawn profile says the same.
        let steep = curb_at(6.0, 60.0, 20.0);
        assert!(steep.validate().is_ok());
        assert!((surface(&steep, 13.8) - 0.12).abs() < 0.001, "{}", surface(&steep, 13.8));
        assert!(surface(&steep, 14.2).abs() < 0.001, "{}", surface(&steep, 14.2));
        assert_eq!((steep.height(13.8), steep.height(14.2)), (0.12, 0.0));
        // A 0.2 m box set down in front of the face sits on the road, one behind it on the curb.
        assert!((settle(&t, 2.85) - 0.1).abs() < 0.005, "{}", settle(&t, 2.85));
        assert!((settle(&t, 3.5) - 0.22).abs() < 0.005, "{}", settle(&t, 3.5));
        assert!((settle(&turned, 3.5) - 0.22).abs() < 0.005, "{}", settle(&turned, 3.5));
    }
}

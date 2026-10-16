//! The tail light: where each LED of the WS2812 strip sits, and what the stock firmware shows.
//!
//! A wheel profile's `lights.toml` ([`Lights::parse`]) describes the strip, the LED rings it is
//! wired into (place, size, which way they shine, and how the LED index runs around each ring),
//! the translucent body the light shines through, other lights on it, and the states the stock
//! firmware shows, as data for people and for our own firmware. A renderer takes the strip's
//! colours (one `[r, g, b]` per LED, as `euc_devices::ws2812::decode` returns them) and draws LED
//! `n` at [`Lights::leds`]`[n - 1]`.

use std::collections::BTreeSet;

use serde::Deserialize;

use crate::{Evidence, ParseError};

/// A parsed `lights.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lights {
    /// Schema version, 1.
    pub schema: u32,
    /// The LED strip.
    pub strip: Strip,
    /// The bar that holds the rings, if known.
    #[serde(default)]
    pub bar: Option<Bar>,
    /// The translucent roller the rings shine through, if the light is one.
    #[serde(default)]
    pub roller: Option<Roller>,
    /// Lights on the same part that the strip does not drive.
    #[serde(default, rename = "indicator")]
    pub indicators: Vec<Indicator>,
    /// Parts of the light's body that do not light (for drawing only).
    #[serde(default, rename = "speaker")]
    pub speakers: Vec<Speaker>,
    /// The LED rings, in strip order.
    #[serde(rename = "ring")]
    pub rings: Vec<Ring>,
    /// What the stock firmware shows, in priority order.
    #[serde(rename = "state")]
    pub states: Vec<State>,
}

/// The LED strip.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Strip {
    /// LED chip.
    pub chip: String,
    /// Where the data comes from.
    pub data: String,
    /// LED frames the firmware sends.
    pub slots: u8,
    /// Byte order on the wire.
    pub byte_order: String,
    /// The whole strip is resent this often, ms.
    pub frame_ms: f64,
    /// LEDs (1-based) the firmware never writes: always off.
    #[serde(default)]
    pub dark: Vec<u8>,
}

/// The bar the rings sit in, as a box in the wheel frame (x forward, y left, z up, m).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bar {
    /// Lower corner.
    pub min_m: [f64; 3],
    /// Upper corner.
    pub max_m: [f64; 3],
    /// How it is known.
    pub evidence: String,
}

/// A translucent cylinder that glows where the rings shine: two halves either side of an
/// opaque centre band, dark end caps beyond.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Roller {
    /// A point on the axis, at the middle of the centre band, wheel frame, m.
    pub centre_m: [f64; 3],
    /// Unit axis.
    pub axis: [f64; 3],
    /// Outer radius, m.
    pub radius_m: f64,
    /// The opaque centre band covers the axis from -this to +this, m.
    pub band_half_width_m: f64,
    /// The glowing halves run from the band to -this and +this along the axis, m.
    pub glow_half_length_m: f64,
    /// What the light passes through, in words.
    pub sleeve: String,
    /// Fraction of red, green and blue light the diffuser lets through.
    pub diffuser_tint: [f64; 3],
    /// Colour of the unlit halves under room light, `#RRGGBB`.
    pub unlit: String,
    /// Colour of the centre band and end caps, `#RRGGBB`.
    pub band: String,
    /// How it is known.
    pub evidence: String,
}

/// A light on the same part that the strip does not drive.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Indicator {
    /// Name.
    pub id: String,
    /// Colour, `#RRGGBB`.
    pub colour: String,
    /// Where it is, wheel frame, m.
    pub position_m: [f64; 3],
    /// What drives it.
    pub source: String,
    /// When it is on, and how it blinks.
    pub behaviour: String,
    /// How it is known.
    pub evidence: String,
}

/// A speaker in an end cap of the light: not a light, and not driven by the controller.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Speaker {
    /// Name.
    pub id: String,
    /// Which side of the wheel it is on.
    pub side: Side,
    /// Centre of its outer face, wheel frame, m.
    pub centre_m: [f64; 3],
    /// Unit normal of the face, pointing out.
    pub normal: [f64; 3],
    /// Outer radius of the cap, m.
    pub radius_m: f64,
    /// The grille: an annulus from this inner radius ...
    pub grille_inner_m: f64,
    /// ... to this outer radius, m.
    pub grille_outer_m: f64,
    /// What drives it.
    pub source: String,
    /// How it is known.
    pub evidence: String,
}

/// Which way the LEDs of a ring shine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Facing {
    /// Along the ring's normal, out of a flat face.
    Normal,
    /// Straight out from the ring's centre, around its axis (a ring inside a tube).
    Radial,
}

/// A side of the wheel, for a rider facing forward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    /// The rider's left.
    Left,
    /// The rider's right.
    Right,
}

/// One ring of LEDs.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ring {
    /// Name (`A`, `B`).
    pub id: String,
    /// First LED (1-based strip index).
    pub first: u8,
    /// LEDs in the ring, consecutive from `first`.
    pub count: u8,
    /// Which side of the wheel it is on.
    pub side: Side,
    /// How the side is known.
    pub side_evidence: Evidence,
    /// Centre of the ring in the wheel frame, m.
    pub centre_m: [f64; 3],
    /// Unit normal of the ring's plane, pointing away from the light's middle (for a radial ring,
    /// its axis).
    pub normal: [f64; 3],
    /// Which way the LEDs shine.
    pub facing: Facing,
    /// Unit vector in the face that points to 12 o'clock.
    pub up: [f64; 3],
    /// How the centre, normal and size are known.
    pub geometry_evidence: String,
    /// Diameter of the circle the LEDs are drawn on, m (for a radial ring inside a diffuser, where
    /// their hot spots show).
    pub diameter_m: f64,
    /// Angle of the ring's first LED, degrees clockwise from 12 o'clock, seen facing the face.
    pub first_angle_deg: f64,
    /// The LED index grows clockwise (seen facing the face).
    pub clockwise: bool,
    /// How the angles are known.
    pub layout_evidence: Evidence,
    /// The firmware's animation position `p` (1..=count) lights LED
    /// `first + (offset + direction * (p - 1)) mod count`.
    pub position_offset: u8,
    /// `1` or `-1`, see `position_offset`.
    pub position_direction: i8,
}

impl Ring {
    /// The strip LED (1-based) that the firmware's animation position `p` (1-based) lights.
    pub fn led_at_position(&self, p: u8) -> u8 {
        let n = i32::from(self.count);
        let k =
            (i32::from(self.position_offset) + i32::from(self.position_direction) * (i32::from(p) - 1)).rem_euclid(n);
        self.first + k as u8
    }
}

/// A state of the tail light, as the stock firmware shows it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    /// Name.
    pub id: String,
    /// When it shows.
    pub trigger: String,
    /// What it shows.
    pub pattern: String,
    /// How it is known.
    pub evidence: Evidence,
}

/// Where one LED is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Led {
    /// Strip index, 1-based.
    pub index: u8,
    /// Index of its ring in [`Lights::rings`].
    pub ring: usize,
    /// In the ring's face, seen facing it: x to the viewer's right, y to 12 o'clock, m from the
    /// ring centre.
    pub face_m: [f64; 2],
    /// In the wheel frame (x forward, y left, z up), m.
    pub wheel_m: [f64; 3],
    /// Unit direction it shines in, wheel frame.
    pub direction: [f64; 3],
}

impl Lights {
    /// Parses and checks a `lights.toml`: known schema, rings inside the strip without overlap,
    /// a position map that visits every LED of its ring once.
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let lights: Self = toml::from_str(text)?;
        lights.validate().map_err(ParseError::Invalid)?;
        Ok(lights)
    }

    /// Every ring LED, in strip order (LED 1 first). Dark slots are left out.
    pub fn leds(&self) -> Vec<Led> {
        let mut leds = Vec::new();
        for (r, ring) in self.rings.iter().enumerate() {
            let radius = ring.diameter_m / 2.0;
            let step = 360.0 / f64::from(ring.count);
            for k in 0..ring.count {
                let turn = if ring.clockwise { 1.0 } else { -1.0 };
                let angle = (ring.first_angle_deg + turn * step * f64::from(k)).to_radians();
                let (x, y) = (radius * angle.sin(), radius * angle.cos());
                // A viewer facing the face looks along -normal; their right is -normal x up.
                let right = cross(neg(ring.normal), ring.up);
                let wheel_m = std::array::from_fn(|i| ring.centre_m[i] + x * right[i] + y * ring.up[i]);
                let direction = match ring.facing {
                    Facing::Normal => ring.normal,
                    Facing::Radial => std::array::from_fn(|i| (x * right[i] + y * ring.up[i]) / radius),
                };
                leds.push(Led { index: ring.first + k, ring: r, face_m: [x, y], wheel_m, direction });
            }
        }
        leds
    }

    fn validate(&self) -> Result<(), String> {
        if self.schema != 1 {
            return Err(format!("schema {} is not supported (1 is)", self.schema));
        }
        let mut used = BTreeSet::new();
        for ring in &self.rings {
            if ring.count == 0
                || ring.first == 0
                || u16::from(ring.first) + u16::from(ring.count) - 1 > u16::from(self.strip.slots)
            {
                return Err(format!(
                    "ring {}: LEDs {}+{} do not fit the {} slots",
                    ring.id, ring.first, ring.count, self.strip.slots
                ));
            }
            for led in ring.first..ring.first + ring.count {
                if !used.insert(led) || self.strip.dark.contains(&led) {
                    return Err(format!("ring {}: LED {led} is used twice or marked dark", ring.id));
                }
            }
            if ring.position_direction.abs() != 1 || ring.position_offset >= ring.count {
                return Err(format!(
                    "ring {}: position_direction must be 1 or -1 and position_offset < count",
                    ring.id
                ));
            }
            if ring.diameter_m <= 0.0 {
                return Err(format!("ring {}: diameter_m must be positive", ring.id));
            }
            let norm = |v: [f64; 3]| dot(v, v).sqrt();
            if (norm(ring.normal) - 1.0).abs() > 1e-6
                || (norm(ring.up) - 1.0).abs() > 1e-6
                || dot(ring.normal, ring.up).abs() > 1e-6
            {
                return Err(format!("ring {}: normal and up must be unit vectors at right angles", ring.id));
            }
        }
        Ok(())
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn neg(a: [f64; 3]) -> [f64; 3] {
    a.map(|v| -v)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPEC: &str = include_str!("../../../profiles/begode-extreme/lights.toml");

    #[test]
    fn the_profile_parses() {
        let lights = Lights::parse(SPEC).unwrap();
        assert_eq!(lights.rings.len(), 2);
        let leds = lights.leds();
        assert_eq!(leds.len(), 24);
        assert!(leds.iter().enumerate().all(|(i, led)| usize::from(led.index) == i + 1));
        assert_eq!(lights.strip.dark, (25..=32).collect::<Vec<u8>>());
    }

    #[test]
    fn the_position_maps_match_the_firmware() {
        // Positions 1..12 light LEDs 8-12 then 1-7 on ring A, and 24 down to 13 on ring B.
        let lights = Lights::parse(SPEC).unwrap();
        let a: Vec<u8> = (1..=12).map(|p| lights.rings[0].led_at_position(p)).collect();
        let b: Vec<u8> = (1..=12).map(|p| lights.rings[1].led_at_position(p)).collect();
        assert_eq!(a, [8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(b, [24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13]);
    }

    #[test]
    fn leds_sit_on_their_ring_in_the_face_plane() {
        let lights = Lights::parse(SPEC).unwrap();
        for led in lights.leds() {
            let ring = &lights.rings[led.ring];
            let offset: [f64; 3] = std::array::from_fn(|i| led.wheel_m[i] - ring.centre_m[i]);
            assert!((dot(offset, offset).sqrt() - ring.diameter_m / 2.0).abs() < 1e-9);
            assert!(dot(offset, ring.normal).abs() < 1e-12, "in the face plane");
        }
        // The first LED of each ring is at 12 o'clock: straight up from its centre.
        let leds = lights.leds();
        assert!((leds[0].wheel_m[2] - lights.rings[0].centre_m[2] - lights.rings[0].diameter_m / 2.0).abs() < 1e-12);
    }

    #[test]
    fn the_rings_shine_out_of_the_roller() {
        let lights = Lights::parse(SPEC).unwrap();
        let roller = lights.roller.as_ref().unwrap();
        let a = &lights.rings[0];
        assert_eq!((a.side, a.normal, a.facing), (Side::Right, [0.0, -1.0, 0.0], Facing::Radial));
        for led in lights.leds() {
            let ring = &lights.rings[led.ring];
            assert!((dot(led.direction, led.direction) - 1.0).abs() < 1e-9);
            assert!(dot(led.direction, ring.normal).abs() < 1e-12, "radial: at right angles to the axis");
            // Each ring sits in its own glowing half, beside the centre band.
            let along = dot(std::array::from_fn(|i| led.wheel_m[i] - roller.centre_m[i]), roller.axis).abs();
            assert!(along > roller.band_half_width_m && along < roller.glow_half_length_m);
            assert!(ring.diameter_m / 2.0 <= roller.radius_m);
        }
        // Facing the right ring from outside (along +y), the viewer's right is the wheel's front.
        assert!(lights.leds()[3].wheel_m[0] > a.centre_m[0]);
    }

    #[test]
    fn the_end_caps_are_speakers_and_the_blue_light_is_the_ble_module() {
        let lights = Lights::parse(SPEC).unwrap();
        let roller = lights.roller.as_ref().unwrap();
        assert_eq!(lights.speakers.len(), 2);
        for speaker in &lights.speakers {
            // Outside the glowing halves, on the roller's axis.
            let along = dot(std::array::from_fn(|i| speaker.centre_m[i] - roller.centre_m[i]), roller.axis).abs();
            assert!(along > roller.glow_half_length_m);
            assert!(speaker.grille_inner_m < speaker.grille_outer_m && speaker.grille_outer_m < speaker.radius_m);
        }
        assert_eq!(lights.indicators.iter().map(|i| i.id.as_str()).collect::<Vec<_>>(), ["ble"]);
    }

    #[test]
    fn rejects_overlapping_rings() {
        let bad = SPEC.replacen("first = 13", "first = 12", 1);
        assert!(matches!(Lights::parse(&bad), Err(ParseError::Invalid(_))));
    }
}

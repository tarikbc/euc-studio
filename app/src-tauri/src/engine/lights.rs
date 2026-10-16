//! The tail light as the web view draws it, from the profile's `lights.toml`: the roller (a
//! translucent tube whose two halves glow), where each LED of the WS2812 strip sits and shines,
//! the BLE module's status light and the speaker caps at the ends. Sent once per run; the LED
//! colours come with the board outputs.

use euc_display::lights::{Facing, Lights, Side};
use serde::Serialize;

const fn side(s: Side) -> &'static str {
    match s {
        Side::Left => "left",
        Side::Right => "right",
    }
}

/// The light's tube. Wheel frame (x forward, y left, z up), metres.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RollerInfo {
    pub centre_m: [f64; 3],
    /// Unit vector along the tube.
    pub axis: [f64; 3],
    pub radius_m: f64,
    /// The dark band round the middle reaches this far from the centre.
    pub band_half_width_m: f64,
    /// The glowing halves reach this far from the centre.
    pub glow_half_length_m: f64,
    /// What the diffuser lets through of each colour channel, 0..1.
    pub diffuser_tint: [f64; 3],
    /// The halves with the LEDs off, `#RRGGBB`.
    pub unlit: String,
    /// The centre band, `#RRGGBB`.
    pub band: String,
}

/// One ring of LEDs.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RingInfo {
    pub id: String,
    /// `left` or `right`, for a rider facing forward.
    pub side: &'static str,
    pub centre_m: [f64; 3],
    /// Unit normal (along the tube for a radial ring), pointing out of the light.
    pub normal: [f64; 3],
    /// Unit vector toward the ring's 12 o'clock.
    pub up: [f64; 3],
    /// `normal` (a flat face) or `radial` (shining out of a tube).
    pub facing: &'static str,
    pub diameter_m: f64,
}

/// One LED.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LedInfo {
    /// Strip index, 1-based: its colour is `outputs.leds[index - 1]`.
    pub index: u8,
    /// Index into `rings`.
    pub ring: usize,
    /// In its ring seen along the normal from outside: x to the viewer's right, y up, m.
    pub face_m: [f64; 2],
    /// Where its light shows, wheel frame, m.
    pub wheel_m: [f64; 3],
    /// Unit direction it shines in, wheel frame.
    pub direction: [f64; 3],
}

/// A light that is not on the strip (the BLE module's status LED).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct IndicatorInfo {
    pub id: String,
    pub colour: String,
    pub position_m: [f64; 3],
}

/// A dark speaker cap at one end of the tube.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SpeakerInfo {
    pub id: String,
    pub side: &'static str,
    pub centre_m: [f64; 3],
    pub normal: [f64; 3],
    pub radius_m: f64,
    pub grille_inner_m: f64,
    pub grille_outer_m: f64,
}

/// The tail light.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LightsLayout {
    /// LED slots the firmware sends.
    pub slots: u8,
    pub roller: Option<RollerInfo>,
    pub rings: Vec<RingInfo>,
    pub leds: Vec<LedInfo>,
    pub indicators: Vec<IndicatorInfo>,
    pub speakers: Vec<SpeakerInfo>,
    /// The bar as a box in the wheel frame, `[min, max]` corners, m; absent while unknown.
    pub bar: Option<[[f64; 3]; 2]>,
}

impl LightsLayout {
    /// The layout of `lights.toml`.
    pub fn parse(text: &str) -> Result<Self, String> {
        let lights = Lights::parse(text).map_err(|e| format!("lights.toml: {e}"))?;
        Ok(Self {
            slots: lights.strip.slots,
            roller: lights.roller.as_ref().map(|r| RollerInfo {
                centre_m: r.centre_m,
                axis: r.axis,
                radius_m: r.radius_m,
                band_half_width_m: r.band_half_width_m,
                glow_half_length_m: r.glow_half_length_m,
                diffuser_tint: r.diffuser_tint,
                unlit: r.unlit.clone(),
                band: r.band.clone(),
            }),
            rings: lights
                .rings
                .iter()
                .map(|r| RingInfo {
                    id: r.id.clone(),
                    side: side(r.side),
                    centre_m: r.centre_m,
                    normal: r.normal,
                    up: r.up,
                    facing: match r.facing {
                        Facing::Normal => "normal",
                        Facing::Radial => "radial",
                    },
                    diameter_m: r.diameter_m,
                })
                .collect(),
            leds: lights
                .leds()
                .into_iter()
                .map(|l| LedInfo {
                    index: l.index,
                    ring: l.ring,
                    face_m: l.face_m,
                    wheel_m: l.wheel_m,
                    direction: l.direction,
                })
                .collect(),
            indicators: lights
                .indicators
                .iter()
                .map(|i| IndicatorInfo { id: i.id.clone(), colour: i.colour.clone(), position_m: i.position_m })
                .collect(),
            speakers: lights
                .speakers
                .iter()
                .map(|s| SpeakerInfo {
                    id: s.id.clone(),
                    side: side(s.side),
                    centre_m: s.centre_m,
                    normal: s.normal,
                    radius_m: s.radius_m,
                    grille_inner_m: s.grille_inner_m,
                    grille_outer_m: s.grille_outer_m,
                })
                .collect(),
            bar: lights.bar.as_ref().map(|b| [b.min_m, b.max_m]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_extreme_tail_light_is_a_roller_with_two_rings_of_twelve() {
        let layout = LightsLayout::parse(include_str!("../../../../profiles/begode-extreme/lights.toml")).unwrap();
        assert_eq!(layout.slots, 32);
        assert!(layout.roller.is_some(), "the light is the roller");
        assert_eq!(layout.rings.len(), 2);
        assert_eq!(layout.leds.len(), 24);
        assert!(layout.leds.iter().all(|l| (1..=layout.slots).contains(&l.index)));
    }
}

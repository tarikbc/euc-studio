//! The wheel's front display: which LEDs the firmware's driver bytes light, and where they are.
//!
//! A wheel profile's `display.toml` ([`Display::parse`]) describes the display board, the LED
//! driver's bit layout, its 7-segment styles and every element (digit, bar, dot, symbol) with
//! the grid bytes and bits that light it. Given the driver's grid memory (the 16 bytes a TM1640
//! holds, in address order, as the simulator's TM1640 model returns them):
//!
//! - [`lit_elements`] lists the elements that are on, and which of their segments;
//! - [`Display::parts`] gives every drawable piece with its outline in board millimetres, and
//!   [`Part::is_lit`] says whether a grid lights it (a renderer draws all parts, lit or dim);
//! - [`Display::text`] reads a group of elements back as text: `"000"`, `"25.3°C"`, `"01468.6"`.
//!
//! ```
//! let spec = euc_display::Display::parse(include_str!("../../../profiles/begode-extreme/display.toml"))?;
//! // Grid bytes the stock firmware wrote at standstill, 130 V, 25.3 °C, odometer 1468.6 km.
//! let grid = [0x61, 0xFD, 0xF3, 0xB7, 0xDB, 0xF3, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xBE, 0xFF, 0xBE, 0x66];
//! assert_eq!(spec.text(&grid, "speed"), "000");
//! assert_eq!(spec.text(&grid, "odometer"), "01468.6");
//! # Ok::<(), euc_display::ParseError>(())
//! ```
//!
//! The [`lights`] module does the same for the tail light's LED strip (`lights.toml`).
//!
//! Pure data and geometry: no I/O and no engine dependencies.

mod geometry;
pub mod lights;

use std::collections::{BTreeMap, BTreeSet, HashSet};

use serde::Deserialize;
use thiserror::Error;

pub use geometry::{Ellipse, Outline, Part};

/// Grid registers of the driver (bytes per refresh).
pub const GRIDS: usize = 16;

/// One driver bit: `(grid index, bit)`.
pub type Source = (usize, u8);

/// A parsed `display.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Display {
    /// Schema version, 1.
    pub schema: u32,
    /// The display board.
    pub board: Board,
    /// The LED panel's window; every element lies inside it.
    pub window: Rect,
    /// The LED driver.
    pub driver: Driver,
    /// Colours for a replica.
    pub appearance: Appearance,
    /// How the board sits in the wheel.
    pub mount: Mount,
    /// 7-segment styles by name.
    #[serde(rename = "style")]
    pub styles: BTreeMap<String, Seg7Style>,
    /// Every element, in file order.
    #[serde(rename = "element")]
    pub elements: Vec<Element>,
}

/// The display board.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Board {
    /// Board name as printed.
    pub name: String,
    /// Date printed on the board.
    pub date: String,
    /// Size printed on the board, mm.
    pub silkscreen_mm: [f64; 2],
    /// Approximate outline, mm, clockwise.
    pub outline: Vec<[f64; 2]>,
}

/// An axis-aligned rectangle in mm.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rect {
    /// Left edge.
    pub x: f64,
    /// Top edge.
    pub y: f64,
    /// Width.
    pub w: f64,
    /// Height.
    pub h: f64,
}

impl Rect {
    /// Whether `[x, y]` lies inside (edges included).
    pub fn contains(&self, [x, y]: [f64; 2]) -> bool {
        x >= self.x && x <= self.x + self.w && y >= self.y && y <= self.y + self.h
    }
}

/// The LED driver and how the firmware uses it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Driver {
    /// Chip.
    pub chip: String,
    /// Grid registers; must be [`GRIDS`].
    pub grids: usize,
    /// Refreshes per second.
    pub refresh_hz: f64,
    /// Brightness level the firmware sets, 0..=7.
    pub brightness: u8,
    /// Which bit of a grid byte drives each segment line.
    pub segment_bits: SegmentBits,
    /// Bits that light nothing.
    #[serde(default)]
    pub unused: Vec<Source>,
}

/// Which bit of a grid byte drives each segment line.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SegmentBits {
    /// Top.
    pub a: u8,
    /// Upper right.
    pub b: u8,
    /// Lower right.
    pub c: u8,
    /// Bottom.
    pub d: u8,
    /// Lower left.
    pub e: u8,
    /// Upper left.
    pub f: u8,
    /// Middle.
    pub g: u8,
    /// Decimal point line (bars, dots and symbols use it on this board).
    pub dp: u8,
}

impl SegmentBits {
    /// The bit that drives `segment`.
    pub const fn bit(&self, segment: Segment) -> u8 {
        match segment {
            Segment::A => self.a,
            Segment::B => self.b,
            Segment::C => self.c,
            Segment::D => self.d,
            Segment::E => self.e,
            Segment::F => self.f,
            Segment::G => self.g,
        }
    }
}

/// Colours for a replica, as `#RRGGBB`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Appearance {
    /// A lit LED.
    pub lit: String,
    /// An unlit segment.
    pub unlit: String,
    /// The panel around the segments.
    pub panel: String,
}

/// How the board sits in the wheel, for placing it on a 3D shell.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mount {
    /// Where on the shell, in words.
    pub surface: String,
    /// The way the board's top edge (y = 0, the top of the speed digits) points on the wheel.
    pub board_up: Toward,
    /// The way the board's +x points on the wheel.
    pub board_right: Toward,
    /// Rotation of the panel as a rider facing forward sees it: 0 upright, 180 upside down.
    pub rider_view_deg: f64,
    /// How the placement is known.
    pub evidence: Evidence,
    /// What the evidence is.
    pub note: String,
}

/// A direction on the wheel, for a rider facing forward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Toward {
    /// Direction of travel.
    Front,
    /// Toward the tail light.
    Rear,
    /// The rider's left.
    Left,
    /// The rider's right.
    Right,
}

/// A 7-segment digit style: upright or slanted, 45-degree miters, pointed middle ends.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Seg7Style {
    /// Digit box width, mm.
    pub width: f64,
    /// Digit box height, mm.
    pub height: f64,
    /// Segment thickness, mm.
    pub stroke: f64,
    /// How far each segment end is set back from its joint's diagonal, along the axis, mm.
    pub miter: f64,
    /// Outer acute corners are cut flat this far from the digit box edge, mm.
    pub corner: f64,
    /// Slant, degrees; positive leans the top to the right.
    pub skew_deg: f64,
}

/// How a mapping is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Evidence {
    /// Seen in the stock firmware's output in the simulator.
    Observed,
    /// From the firmware code or the board layout only.
    Inferred,
}

/// One display element.
#[derive(Debug, Clone, Deserialize)]
pub struct Element {
    /// Stable id, `group.name` (`speed.tens`, `battery.3`).
    pub id: String,
    /// What it shows.
    pub meaning: String,
    /// Unit of the value, if any.
    #[serde(default)]
    pub unit: Option<String>,
    /// How the bit mapping is known.
    pub mapping: Evidence,
    /// How it blinks, if it does (the firmware blinks it by rewriting the bits).
    #[serde(default)]
    pub blink: Option<String>,
    /// What the owner's photos of the lit display show for it, if anything.
    #[serde(default)]
    pub photo: Option<String>,
    /// Kind, geometry and sources.
    #[serde(flatten)]
    pub shape: Shape,
}

/// An element's kind, geometry (mm) and driver bits.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Shape {
    /// A 7-segment digit; segment `s` is lit when its bit is set in any of `grids`.
    Digit7 {
        /// Name of its [`Seg7Style`].
        style: String,
        /// Left edge of the digit box.
        x: f64,
        /// Top edge of the digit box.
        y: f64,
        /// Grid bytes that drive it (the firmware writes the same byte to each).
        grids: Vec<usize>,
    },
    /// A rectangular bar.
    Bar {
        /// Left edge.
        x: f64,
        /// Top edge.
        y: f64,
        /// Width.
        w: f64,
        /// Height.
        h: f64,
        /// Bits that light it (any).
        sources: Vec<Source>,
    },
    /// A round dot.
    Dot {
        /// Centre x.
        cx: f64,
        /// Centre y.
        cy: f64,
        /// Diameter.
        d: f64,
        /// Bits that light it (any).
        sources: Vec<Source>,
    },
    /// A stroked symbol: a ring and an open path.
    Symbol {
        /// What it reads as, e.g. `"°C"`.
        glyph: String,
        /// The ring.
        ring: Ellipse,
        /// The open path.
        path: Vec<[f64; 2]>,
        /// Stroke width.
        stroke: f64,
        /// Bits that light it (any).
        sources: Vec<Source>,
    },
}

impl Element {
    /// The group, the id before the first dot (`speed` for `speed.tens`).
    pub fn group(&self) -> &str {
        self.id.split_once('.').map_or(self.id.as_str(), |(group, _)| group)
    }

    /// Every driver bit this element uses.
    pub fn sources(&self, bits: &SegmentBits) -> Vec<Source> {
        match &self.shape {
            Shape::Digit7 { grids, .. } => {
                grids.iter().flat_map(|&g| Segment::ALL.iter().map(move |&s| (g, bits.bit(s)))).collect()
            }
            Shape::Bar { sources, .. } | Shape::Dot { sources, .. } | Shape::Symbol { sources, .. } => sources.clone(),
        }
    }

    /// What `grid` lights: for a digit, bit `i` set = segment `i` lit (a = bit 0 ... g = bit 6);
    /// for a bar, dot or symbol, 1 when lit. 0 = dark.
    pub fn mask(&self, grid: &[u8; GRIDS], bits: &SegmentBits) -> u8 {
        let on = |(g, b): Source| grid[g] >> b & 1 == 1;
        match &self.shape {
            Shape::Digit7 { grids, .. } => Segment::ALL
                .iter()
                .enumerate()
                .filter(|&(_, &s)| grids.iter().any(|&g| on((g, bits.bit(s)))))
                .fold(0, |mask, (i, _)| mask | 1 << i),
            Shape::Bar { sources, .. } | Shape::Dot { sources, .. } | Shape::Symbol { sources, .. } => {
                u8::from(sources.iter().any(|&s| on(s)))
            }
        }
    }

    /// Left edge, mm (to order a group left to right).
    fn left(&self) -> f64 {
        match &self.shape {
            Shape::Digit7 { x, .. } | Shape::Bar { x, .. } => *x,
            Shape::Dot { cx, d, .. } => cx - d / 2.0,
            Shape::Symbol { ring, .. } => ring.cx - ring.rx,
        }
    }
}

/// A segment of a 7-segment digit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Segment {
    /// Top.
    A,
    /// Upper right.
    B,
    /// Lower right.
    C,
    /// Bottom.
    D,
    /// Lower left.
    E,
    /// Upper left.
    F,
    /// Middle.
    G,
}

impl Segment {
    /// a..g, in mask bit order.
    pub const ALL: [Self; 7] = [Self::A, Self::B, Self::C, Self::D, Self::E, Self::F, Self::G];

    /// Lowercase letter.
    pub const fn letter(self) -> char {
        match self {
            Self::A => 'a',
            Self::B => 'b',
            Self::C => 'c',
            Self::D => 'd',
            Self::E => 'e',
            Self::F => 'f',
            Self::G => 'g',
        }
    }
}

/// Digit masks (a = bit 0 ... g = bit 6) for 0-9, as the stock firmware draws them.
const DIGITS: [u8; 10] = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F];

/// The character a digit mask shows: `' '` when dark, `'0'..='9'`, `'?'` for anything else.
pub fn glyph(mask: u8) -> char {
    if mask == 0 {
        return ' ';
    }
    DIGITS.iter().position(|&d| d == mask).map_or('?', |n| char::from(b'0' + n as u8))
}

/// An element that is on, with what is lit (see [`Element::mask`]).
#[derive(Debug, Clone, Copy)]
pub struct LitElement<'a> {
    /// The element.
    pub element: &'a Element,
    /// Lit segments (digits) or 1 (bars, dots, symbols).
    pub mask: u8,
}

/// The elements `grid` lights, in file order; dark elements are left out.
pub fn lit_elements<'a>(grid: &[u8; GRIDS], spec: &'a Display) -> Vec<LitElement<'a>> {
    spec.elements
        .iter()
        .map(|element| LitElement { element, mask: element.mask(grid, &spec.driver.segment_bits) })
        .filter(|lit| lit.mask != 0)
        .collect()
}

/// Why a `display.toml` was rejected.
#[derive(Debug, Error)]
pub enum ParseError {
    /// Not valid TOML, or not the schema.
    #[error(transparent)]
    Toml(#[from] toml::de::Error),
    /// Valid TOML that breaks a rule.
    #[error("display.toml: {0}")]
    Invalid(String),
}

impl Display {
    /// Parses and checks a `display.toml`: known schema, 16 grids, distinct segment bits, known
    /// styles with room for their segments, unique ids, and every one of the 128 driver bits
    /// used by exactly one element or listed as unused.
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let display: Self = toml::from_str(text)?;
        display.validate().map_err(ParseError::Invalid)?;
        Ok(display)
    }

    /// The element with this id.
    pub fn element(&self, id: &str) -> Option<&Element> {
        self.elements.iter().find(|e| e.id == id)
    }

    /// A group's elements read left to right as text: digits as `0-9` (`' '` dark, `'?'`
    /// unknown), a lit dot as `.`, a lit symbol as its glyph, a bar as `|` lit or `-` dark.
    pub fn text(&self, grid: &[u8; GRIDS], group: &str) -> String {
        let mut members: Vec<&Element> = self.elements.iter().filter(|e| e.group() == group).collect();
        members.sort_by(|a, b| a.left().total_cmp(&b.left()));
        let bits = &self.driver.segment_bits;
        let mut text = String::new();
        for element in members {
            let mask = element.mask(grid, bits);
            match &element.shape {
                Shape::Digit7 { .. } => text.push(glyph(mask)),
                Shape::Bar { .. } => text.push(if mask == 0 { '-' } else { '|' }),
                Shape::Dot { .. } if mask != 0 => text.push('.'),
                Shape::Symbol { glyph, .. } if mask != 0 => text.push_str(glyph),
                Shape::Dot { .. } | Shape::Symbol { .. } => {}
            }
        }
        text
    }

    fn validate(&self) -> Result<(), String> {
        if self.schema != 1 {
            return Err(format!("schema {} is not supported (1 is)", self.schema));
        }
        if self.driver.grids != GRIDS {
            return Err(format!("driver.grids is {}, the decoder handles {GRIDS}", self.driver.grids));
        }
        let b = &self.driver.segment_bits;
        let lines = [b.a, b.b, b.c, b.d, b.e, b.f, b.g, b.dp];
        if lines.iter().any(|&bit| bit > 7) || lines.iter().collect::<HashSet<_>>().len() != 8 {
            return Err("driver.segment_bits must be 8 distinct bits 0..=7".to_owned());
        }
        for (name, style) in &self.styles {
            style.check().map_err(|why| format!("style.{name}: {why}"))?;
        }
        let mut ids = HashSet::new();
        let mut used: BTreeSet<Source> = BTreeSet::new();
        let mut claim = |source: Source, id: &str| -> Result<(), String> {
            let (g, bit) = source;
            if g >= GRIDS || bit > 7 {
                return Err(format!("{id}: bit ({g}, {bit}) is outside the {GRIDS} x 8 grid"));
            }
            if !used.insert(source) {
                return Err(format!("{id}: bit ({g}, {bit}) is used twice"));
            }
            Ok(())
        };
        for &source in &self.driver.unused {
            claim(source, "driver.unused")?;
        }
        for element in &self.elements {
            if !ids.insert(element.id.as_str()) {
                return Err(format!("element id {} appears twice", element.id));
            }
            if let Shape::Digit7 { style, grids, .. } = &element.shape {
                if !self.styles.contains_key(style) {
                    return Err(format!("{}: unknown style {style}", element.id));
                }
                if grids.is_empty() {
                    return Err(format!("{}: a digit needs at least one grid", element.id));
                }
            }
            for source in element.sources(b) {
                claim(source, &element.id)?;
            }
        }
        if used.len() != GRIDS * 8 {
            let missing: Vec<String> = (0..GRIDS)
                .flat_map(|g| (0..8).map(move |bit| (g, bit)))
                .filter(|s| !used.contains(s))
                .map(|(g, bit)| format!("({g}, {bit})"))
                .collect();
            return Err(format!("no element or unused entry for bits {}", missing.join(", ")));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPEC: &str = include_str!("../../../profiles/begode-extreme/display.toml");

    fn spec() -> Display {
        Display::parse(SPEC).unwrap()
    }

    /// Grid bytes the stock firmware (GW1825007) wrote in the simulator. Firmware output, not
    /// firmware code: each line names the plant inputs and settings of the run.
    mod grids {
        /// Power-on defaults: 130 V, 25 °C board, empty settings (medium pedals, km, odometer 0).
        pub const BOOT: [u8; 16] =
            [0xFD, 0xFD, 0xF3, 0xB7, 0xDB, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        /// The maintainer's settings seeded in flash: odometer 1,468,670 m, hard pedals, km.
        pub const MAINTAINER: [u8; 16] =
            [0x61, 0xFD, 0xF3, 0xB7, 0xDB, 0xF3, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xBE, 0xFF, 0xBE, 0x66];
        /// The same in miles mode.
        pub const MILES: [u8; 16] =
            [0xFD, 0xFD, 0xF3, 0xB7, 0xDB, 0xF3, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xB6, 0xFD, 0x60, 0xF6];
        /// Odometer 98,765,432 m, soft pedals.
        pub const BIG_ODOMETER: [u8; 16] =
            [0xFF, 0xF7, 0xF3, 0xB7, 0xDB, 0x61, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0x66, 0xB7, 0xBE, 0xE0];
        /// Odometer 4,000,000,000 m: clamped.
        pub const ODOMETER_CLAMP: [u8; 16] =
            [0xF7, 0xF7, 0xF3, 0xB7, 0xDB, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xF6, 0xF7, 0xF6, 0xF6];
        /// 98 V: below every battery level, bar 1 blinking; its dark and lit phases.
        pub const LOW_DARK: [u8; 16] =
            [0xFC, 0xFC, 0xF3, 0xB7, 0xDA, 0xDA, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFC, 0xFC];
        pub const LOW_LIT: [u8; 16] =
            [0xFC, 0xFC, 0xF3, 0xB7, 0xDA, 0xDA, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        /// 109.5 V, 116 V and 121.5 V.
        pub const BARS_2: [u8; 16] =
            [0xFC, 0xFC, 0xF3, 0xB7, 0xDA, 0xDA, 0xFC, 0xFC, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        pub const BARS_3: [u8; 16] =
            [0xFC, 0xFC, 0xF3, 0xB7, 0xDA, 0xDA, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        pub const BARS_4: [u8; 16] =
            [0xFC, 0xFC, 0xF3, 0xB7, 0xDB, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        /// Riding scenario ride-20kmh-stop with odometer 558,300 m and medium pedals seeded, as
        /// the TM1640 model decoded it from the bus: speeding up (007) and holding 20 km/h.
        pub const RIDE_7: [u8; 16] =
            [0xFD, 0xFD, 0x67, 0xB7, 0xDB, 0xDB, 0xE1, 0xE1, 0xFD, 0xFD, 0xFD, 0xFD, 0xF2, 0xFF, 0xB6, 0xB6];
        pub const RIDE_19: [u8; 16] =
            [0xFD, 0xFD, 0xB7, 0xB7, 0xDB, 0xDB, 0xF7, 0xF7, 0x61, 0x61, 0xFD, 0xFD, 0xF2, 0xFF, 0xB6, 0xB6];
        pub const RIDE_21: [u8; 16] =
            [0xFD, 0xFD, 0xB7, 0xB7, 0xDB, 0xDB, 0x61, 0x61, 0xDB, 0xDB, 0xFD, 0xFD, 0xF2, 0xFF, 0xB6, 0xB6];
        /// Board at 46 °C, above 99.9 °C, and below 0 °C.
        pub const TEMP_46: [u8; 16] =
            [0xFD, 0xFD, 0xFD, 0xBF, 0x67, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        pub const TEMP_HOT: [u8; 16] =
            [0xFD, 0xFD, 0xF7, 0xF7, 0xF7, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
        pub const TEMP_FROST: [u8; 16] =
            [0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xDB, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFC, 0xFD, 0xFC, 0xFC];
    }

    fn read(grid: &[u8; GRIDS]) -> [String; 5] {
        let s = spec();
        ["speed", "battery", "pedal", "temp", "odometer"].map(|group| s.text(grid, group))
    }

    #[test]
    fn the_profile_parses_and_covers_every_driver_bit() {
        let s = spec();
        // 3 speed digits, 5 bars, the pedal digit, 3 temperature digits, a point and the sign,
        // 6 odometer digits and a point.
        assert_eq!(s.elements.len(), 21);
        assert_eq!(s.driver.unused, [(12, 0), (14, 0), (15, 0)]);
    }

    #[test]
    fn boot_screen() {
        assert_eq!(read(&grids::BOOT), ["000", "|||||", "2", "25.3°C", "00000.0"]);
    }

    #[test]
    fn odometer_pedal_mode_and_units() {
        assert_eq!(read(&grids::MAINTAINER), ["000", "|||||", "3", "25.3°C", "01468.6"]);
        assert_eq!(read(&grids::MILES)[4], "00910.5");
        assert_eq!(read(&grids::BIG_ODOMETER)[2..], ["1", "25.3°C", "98765.4"]);
        assert_eq!(read(&grids::ODOMETER_CLAMP)[4], "99999.9");
    }

    #[test]
    fn riding() {
        // The owner's photos read 000 parked and 021 riding, 2 on the pedal digit, 00558.3.
        assert_eq!(read(&grids::RIDE_7), ["007", "|||||", "2", "25.4°C", "00558.3"]);
        assert_eq!(read(&grids::RIDE_19)[0], "019");
        assert_eq!(read(&grids::RIDE_21), ["021", "|||||", "2", "25.5°C", "00558.3"]);
    }

    #[test]
    fn the_mount_and_photo_notes_parse() {
        let s = spec();
        assert_eq!(
            (s.mount.board_up, s.mount.board_right, s.mount.evidence),
            (Toward::Front, Toward::Right, Evidence::Observed)
        );
        assert_eq!(s.element("pedal.mode").and_then(|e| e.photo.as_deref()), Some("2 (medium) in both photos"));
        let inferred: Vec<&str> =
            s.elements.iter().filter(|e| e.mapping == Evidence::Inferred).map(|e| e.id.as_str()).collect();
        assert_eq!(inferred, ["temp.unit"]);
    }

    #[test]
    fn battery_levels_fill_from_bar_one() {
        assert_eq!(read(&grids::LOW_DARK)[1], "-----");
        assert_eq!(read(&grids::LOW_LIT)[1], "|----");
        assert_eq!(read(&grids::BARS_2)[1], "||---");
        assert_eq!(read(&grids::BARS_3)[1], "|||--");
        assert_eq!(read(&grids::BARS_4)[1], "||||-");
    }

    #[test]
    fn temperatures() {
        assert_eq!(read(&grids::TEMP_46)[3], "46.0°C");
        assert_eq!(read(&grids::TEMP_HOT)[3], "99.9°C");
        assert_eq!(read(&grids::TEMP_FROST)[3], "00.0°C");
    }

    #[test]
    fn a_blank_driver_lights_nothing() {
        let s = spec();
        assert!(lit_elements(&[0; GRIDS], &s).is_empty());
        assert_eq!(s.text(&[0; GRIDS], "speed"), "   ");
        assert_eq!(s.text(&[0; GRIDS], "temp"), "   ");
    }

    #[test]
    fn lit_elements_name_segments() {
        let s = spec();
        let lit = lit_elements(&grids::LOW_DARK, &s);
        let ids: Vec<&str> = lit.iter().map(|l| l.element.id.as_str()).collect();
        assert!(!ids.iter().any(|id| id.starts_with("battery.")));
        let hundreds = lit.iter().find(|l| l.element.id == "speed.hundreds").unwrap();
        assert_eq!((hundreds.mask, glyph(hundreds.mask)), (0x3F, '0'));
        assert_eq!(lit.len(), s.elements.len() - 5);
    }

    #[test]
    fn either_grid_of_a_big_digit_lights_the_segment() {
        let s = spec();
        let mut grid = [0; GRIDS];
        grid[10] = 0x60; // b, c on one of the two hundreds grids only
        assert_eq!(s.text(&grid, "speed"), "1  ");
    }

    #[test]
    fn digit_glyphs_match_the_firmware_patterns() {
        // The firmware's patterns (a = bit 7 ... g = bit 1) reach the same digits.
        let bits = spec().driver.segment_bits;
        let digit = Element {
            id: "t.d".to_owned(),
            meaning: String::new(),
            unit: None,
            mapping: Evidence::Observed,
            blink: None,
            photo: None,
            shape: Shape::Digit7 { style: "small".to_owned(), x: 0.0, y: 0.0, grids: vec![0] },
        };
        for (n, byte) in [0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6].into_iter().enumerate() {
            let mut grid = [0; GRIDS];
            grid[0] = byte;
            assert_eq!(glyph(digit.mask(&grid, &bits)), char::from(b'0' + n as u8));
        }
        assert_eq!(glyph(0x01 | 0x40), '?');
    }

    #[test]
    fn rejects_a_bit_used_twice_and_a_bit_left_out() {
        let twice =
            SPEC.replace("unused = [[12, 0], [14, 0], [15, 0]]", "unused = [[12, 0], [14, 0], [15, 0], [3, 0]]");
        assert!(
            matches!(Display::parse(&twice), Err(ParseError::Invalid(why)) if why.contains("(3, 0) is used twice"))
        );
        let missing = SPEC.replace("unused = [[12, 0], [14, 0], [15, 0]]", "unused = [[12, 0], [14, 0]]");
        assert!(matches!(Display::parse(&missing), Err(ParseError::Invalid(why)) if why.contains("(15, 0)")));
    }
}

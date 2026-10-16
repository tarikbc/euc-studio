//! Outlines of the drawable pieces, in board millimetres (origin top-left, y down).

use serde::Deserialize;

use crate::{Display, Element, GRIDS, Seg7Style, Segment, Shape, Source};

/// An axis-aligned ellipse, mm.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ellipse {
    /// Centre x.
    pub cx: f64,
    /// Centre y.
    pub cy: f64,
    /// Horizontal radius (to the stroke's centre line).
    pub rx: f64,
    /// Vertical radius (to the stroke's centre line).
    pub ry: f64,
}

/// The outline of one drawable piece.
#[derive(Debug, Clone, PartialEq)]
pub enum Outline {
    /// A filled polygon, clockwise on screen.
    Polygon(Vec<[f64; 2]>),
    /// A filled circle.
    Circle {
        /// Centre x.
        cx: f64,
        /// Centre y.
        cy: f64,
        /// Radius.
        r: f64,
    },
    /// A stroked ellipse.
    Ring {
        /// The stroke's centre line.
        ellipse: Ellipse,
        /// Stroke width.
        width: f64,
    },
    /// A stroked open path.
    Path {
        /// Vertices.
        points: Vec<[f64; 2]>,
        /// Stroke width.
        width: f64,
    },
}

impl Outline {
    /// Bounding box as `[min, max]` corners, strokes included.
    pub fn bounds(&self) -> [[f64; 2]; 2] {
        let span = |points: &[[f64; 2]], pad: f64| {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for p in points {
                for i in 0..2 {
                    lo[i] = lo[i].min(p[i] - pad);
                    hi[i] = hi[i].max(p[i] + pad);
                }
            }
            [lo, hi]
        };
        match self {
            Self::Polygon(points) => span(points, 0.0),
            Self::Circle { cx, cy, r } => [[cx - r, cy - r], [cx + r, cy + r]],
            Self::Ring { ellipse: e, width } => {
                let (rx, ry) = (e.rx + width / 2.0, e.ry + width / 2.0);
                [[e.cx - rx, e.cy - ry], [e.cx + rx, e.cy + ry]]
            }
            Self::Path { points, width } => span(points, width / 2.0),
        }
    }
}

/// One drawable piece of an element: a digit segment, a bar, a dot or a symbol stroke.
#[derive(Debug, Clone)]
pub struct Part<'a> {
    /// The element it belongs to.
    pub element: &'a Element,
    /// Which segment, for a digit.
    pub segment: Option<Segment>,
    /// Where it is.
    pub outline: Outline,
    /// Driver bits that light it (any one is enough).
    pub sources: Vec<Source>,
}

impl Part<'_> {
    /// Whether `grid` lights this piece.
    pub fn is_lit(&self, grid: &[u8; GRIDS]) -> bool {
        self.sources.iter().any(|&(g, bit)| grid[g] >> bit & 1 == 1)
    }
}

impl Seg7Style {
    /// The outline of `segment`, relative to the digit box's top-left corner.
    ///
    /// Each segment is a bar of `stroke` thickness whose ends are cut by the 45-degree
    /// diagonals through its joints, set back by `miter`; the outer acute corners are cut flat
    /// at `corner` from the box edge (when `corner > miter`); slant shears x by the height above
    /// the bottom edge.
    #[allow(clippy::many_single_char_names)] // w, h, t, m, c read like the formulas they are
    pub fn polygon(&self, segment: Segment) -> Vec<[f64; 2]> {
        let (w, h, t, m, c) = (self.width, self.height, self.stroke, self.miter, self.corner);
        let mid = h / 2.0;
        let cut = c > m;
        // The top segment and the upper-left one; the other four corners' segments are mirrors.
        let mut a = if cut { vec![[c, 0.0], [w - c, 0.0], [w - c, c - m]] } else { vec![[m, 0.0], [w - m, 0.0]] };
        a.extend([[w - t - m, t], [t + m, t]]);
        if cut {
            a.push([c, c - m]);
        }
        let mut f = if cut { vec![[0.0, c], [c - m, c]] } else { vec![[0.0, m]] };
        f.extend([[t, t + m], [t, mid - m - t / 2.0], [t / 2.0, mid - m], [0.0, mid - m - t / 2.0]]);
        let g = vec![
            [t / 2.0 + m, mid],
            [t + m, mid - t / 2.0],
            [w - t - m, mid - t / 2.0],
            [w - t / 2.0 - m, mid],
            [w - t - m, mid + t / 2.0],
            [t + m, mid + t / 2.0],
        ];
        // A mirror reverses the winding; reversing the vertex order restores it.
        let mirror_x = |p: &[[f64; 2]]| p.iter().rev().map(|&[x, y]| [w - x, y]).collect::<Vec<_>>();
        let mirror_y = |p: &[[f64; 2]]| p.iter().rev().map(|&[x, y]| [x, h - y]).collect::<Vec<_>>();
        let points = match segment {
            Segment::A => a,
            Segment::B => mirror_x(&f),
            Segment::C => mirror_x(&mirror_y(&f)),
            Segment::D => mirror_y(&a),
            Segment::E => mirror_y(&f),
            Segment::F => f,
            Segment::G => g,
        };
        let shear = self.skew_deg.to_radians().tan();
        points.into_iter().map(|[x, y]| [x + (h - y) * shear, y]).collect()
    }

    /// Positive sizes, and room for every segment between its miters.
    pub(crate) fn check(&self) -> Result<(), String> {
        let (w, h, t, m) = (self.width, self.height, self.stroke, self.miter);
        if [w, h, t].iter().any(|&v| v <= 0.0) || m < 0.0 || self.corner < 0.0 {
            return Err("width, height and stroke must be positive, miter and corner not negative".to_owned());
        }
        if w - 2.0 * (t + m) <= 0.0 || h / 2.0 - t / 2.0 - 2.0 * m - t <= 0.0 {
            return Err("the stroke and miter leave no room for the segments".to_owned());
        }
        if self.skew_deg.abs() >= 45.0 {
            return Err("skew_deg must be within +-45".to_owned());
        }
        Ok(())
    }
}

impl Display {
    /// Every drawable piece, element by element in file order: a digit's seven segments (a..g),
    /// a bar, a dot, a symbol's ring then path.
    ///
    /// # Panics
    ///
    /// Never for a [`Display`] from [`Display::parse`], which checks that every digit's style
    /// exists.
    pub fn parts(&self) -> Vec<Part<'_>> {
        let bits = &self.driver.segment_bits;
        let mut parts = Vec::new();
        for element in &self.elements {
            let part = |segment, outline, sources| Part { element, segment, outline, sources };
            match &element.shape {
                Shape::Digit7 { style, x, y, grids } => {
                    let style = &self.styles[style];
                    for segment in Segment::ALL {
                        let outline = Outline::Polygon(
                            style.polygon(segment).into_iter().map(|[px, py]| [x + px, y + py]).collect(),
                        );
                        let sources = grids.iter().map(|&g| (g, bits.bit(segment))).collect();
                        parts.push(part(Some(segment), outline, sources));
                    }
                }
                Shape::Bar { x, y, w, h, sources } => {
                    let outline = Outline::Polygon(vec![[*x, *y], [x + w, *y], [x + w, y + h], [*x, y + h]]);
                    parts.push(part(None, outline, sources.clone()));
                }
                Shape::Dot { cx, cy, d, sources } => {
                    parts.push(part(None, Outline::Circle { cx: *cx, cy: *cy, r: d / 2.0 }, sources.clone()));
                }
                Shape::Symbol { ring, path, stroke, sources, .. } => {
                    parts.push(part(None, Outline::Ring { ellipse: *ring, width: *stroke }, sources.clone()));
                    let outline = Outline::Path { points: path.clone(), width: *stroke };
                    parts.push(part(None, outline, sources.clone()));
                }
            }
        }
        parts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPEC: &str = include_str!("../../../profiles/begode-extreme/display.toml");

    fn close(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
        a.len() == b.len() && a.iter().zip(b).all(|(p, q)| (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9)
    }

    fn style() -> Seg7Style {
        // The big digit in photo pixels, where its corners were measured.
        Seg7Style { width: 124.0, height: 246.0, stroke: 28.5, miter: 4.75, corner: 9.0, skew_deg: 0.0 }
    }

    #[test]
    fn segments_match_the_measured_corners() {
        let s = style();
        let a = [[9.0, 0.0], [115.0, 0.0], [115.0, 4.25], [90.75, 28.5], [33.25, 28.5], [9.0, 4.25]];
        assert!(close(&s.polygon(Segment::A), &a));
        let f = [[0.0, 9.0], [4.25, 9.0], [28.5, 33.25], [28.5, 104.0], [14.25, 118.25], [0.0, 104.0]];
        assert!(close(&s.polygon(Segment::F), &f));
        let g = s.polygon(Segment::G);
        assert!(close(&g[..1], &[[19.0, 123.0]]) && close(&g[3..4], &[[105.0, 123.0]]));
    }

    #[test]
    fn mirrored_segments_keep_the_clockwise_winding() {
        // Shoelace sum with y down: positive = clockwise on screen.
        let area = |p: &[[f64; 2]]| {
            (0..p.len()).map(|i| p[i][0] * p[(i + 1) % p.len()][1] - p[(i + 1) % p.len()][0] * p[i][1]).sum::<f64>()
        };
        let s = style();
        let a = area(&s.polygon(Segment::A));
        for segment in Segment::ALL {
            let polygon = s.polygon(segment);
            assert!(area(&polygon) > 0.0, "{segment:?}");
            assert!(polygon.iter().all(|&[x, y]| (0.0..=124.0).contains(&x) && (0.0..=246.0).contains(&y)));
        }
        assert!((area(&s.polygon(Segment::D)) - a).abs() < 1e-6);
    }

    #[test]
    fn skew_leans_the_top_right() {
        let s = Seg7Style { skew_deg: 10.0, ..style() };
        let at = |segment, [x, y]: [f64; 2]| {
            s.polygon(segment).iter().any(|p| (p[0] - x).abs() < 1e-9 && (p[1] - y).abs() < 1e-9)
        };
        // The top edge moves right by height * tan(skew); the bottom edge stays.
        assert!(at(Segment::A, [9.0 + 246.0 * 10f64.to_radians().tan(), 0.0]));
        assert!(at(Segment::D, [9.0, 246.0]));
    }

    #[test]
    fn every_part_lies_in_the_window_and_the_counts_add_up() {
        let display = Display::parse(SPEC).unwrap();
        let parts = display.parts();
        // 13 digits x 7 segments, 5 bars, 2 dots, a ring and a path.
        assert_eq!(parts.len(), 13 * 7 + 5 + 2 + 2);
        for part in &parts {
            let [lo, hi] = part.outline.bounds();
            assert!(display.window.contains(lo) && display.window.contains(hi), "{} {lo:?} {hi:?}", part.element.id);
        }
        let all = [0xFF; GRIDS];
        assert!(parts.iter().all(|p| p.is_lit(&all)));
        assert!(parts.iter().all(|p| !p.is_lit(&[0; GRIDS])));
    }

    #[test]
    fn bad_styles_are_rejected() {
        let tight = Seg7Style { stroke: 70.0, ..style() };
        assert!(tight.check().is_err());
        assert!(style().check().is_ok());
    }
}

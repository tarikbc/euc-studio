//! The wheel's front display as the web view draws it: every LED piece of the replica, in board
//! millimetres, with the driver bits that light it. Sent once per run; the lit state then comes
//! from the TM1640 grid bytes in the board outputs.

use euc_display::{Display, Outline, Toward};
use serde::Serialize;

/// One drawable LED piece.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Piece {
    /// A filled polygon.
    Polygon { points: Vec<[f64; 2]>, sources: Vec<[u8; 2]> },
    /// A filled circle.
    Circle { cx: f64, cy: f64, r: f64, sources: Vec<[u8; 2]> },
    /// A stroked ellipse.
    Ring { cx: f64, cy: f64, rx: f64, ry: f64, width: f64, sources: Vec<[u8; 2]> },
    /// A stroked open path.
    Path { points: Vec<[f64; 2]>, width: f64, sources: Vec<[u8; 2]> },
}

/// The display replica: the LED window, its colours, how it sits on the shell, and its pieces.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DisplayLayout {
    /// The board's name ("T16-007").
    pub board: String,
    /// The LED window, mm: `[x, y, w, h]`; every piece lies inside it.
    pub window: [f64; 4],
    /// A lit LED, `#RRGGBB`.
    pub lit: String,
    /// An unlit segment (the faint ghost behind the smoked window).
    pub unlit: String,
    /// The panel around the segments.
    pub panel: String,
    /// Where the board's top edge points on the wheel (`front`, `rear`, `left`, `right`).
    pub board_up: &'static str,
    /// Where the board's +x points on the wheel.
    pub board_right: &'static str,
    /// Pieces in file order; a piece is lit when any of its `[grid, bit]` sources is set.
    pub pieces: Vec<Piece>,
}

const fn toward(t: Toward) -> &'static str {
    match t {
        Toward::Front => "front",
        Toward::Rear => "rear",
        Toward::Left => "left",
        Toward::Right => "right",
    }
}

impl DisplayLayout {
    /// The replica of `display.toml`.
    pub fn parse(text: &str) -> Result<Self, String> {
        let spec = Display::parse(text).map_err(|e| format!("display.toml: {e}"))?;
        let pieces = spec
            .parts()
            .into_iter()
            .map(|part| {
                let sources: Vec<[u8; 2]> = part.sources.iter().map(|&(g, bit)| [g as u8, bit]).collect();
                match part.outline {
                    Outline::Polygon(points) => Piece::Polygon { points, sources },
                    Outline::Circle { cx, cy, r } => Piece::Circle { cx, cy, r, sources },
                    Outline::Ring { ellipse: e, width } => {
                        Piece::Ring { cx: e.cx, cy: e.cy, rx: e.rx, ry: e.ry, width, sources }
                    }
                    Outline::Path { points, width } => Piece::Path { points, width, sources },
                }
            })
            .collect();
        let w = spec.window;
        Ok(Self {
            board: spec.board.name.clone(),
            window: [w.x, w.y, w.w, w.h],
            lit: spec.appearance.lit.clone(),
            unlit: spec.appearance.unlit.clone(),
            panel: spec.appearance.panel.clone(),
            board_up: toward(spec.mount.board_up),
            board_right: toward(spec.mount.board_right),
            pieces,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_extreme_display_has_three_big_digits_and_fits_its_window() {
        let layout = DisplayLayout::parse(include_str!("../../../../profiles/begode-extreme/display.toml")).unwrap();
        assert_eq!(layout.board, "T16-007");
        assert_eq!((layout.board_up, layout.board_right), ("front", "right"));
        let polygons = layout.pieces.iter().filter(|p| matches!(p, Piece::Polygon { .. })).count();
        assert!(polygons >= 3 * 7, "three 7-segment digits at least, got {polygons} polygons");
        let [x, y, w, h] = layout.window;
        for piece in &layout.pieces {
            let (Piece::Polygon { points, sources } | Piece::Path { points, sources, .. }) = piece else { continue };
            assert!(!sources.is_empty());
            for &[px, py] in points {
                assert!(px >= x - 0.5 && px <= x + w + 0.5 && py >= y - 0.5 && py <= y + h + 0.5, "{px},{py}");
            }
        }
    }
}

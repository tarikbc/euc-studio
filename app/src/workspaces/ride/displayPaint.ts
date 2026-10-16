import type { DisplayLayout, DisplayPiece } from "../../engine/types.ts";

// The wheel's front display, drawn from its profile layout (board millimetres) and the TM1640
// grid bytes the firmware wrote. One painter serves the HUD card and the decal on the 3D shell,
// so both show exactly the same LEDs.

/** Whether the grid lights a piece (any of its driver bits is enough). */
export function isLit(piece: DisplayPiece, grid: readonly number[]): boolean {
  return piece.sources.some(([g, bit]) => (((grid[g] ?? 0) >> bit) & 1) === 1);
}

const paths = new WeakMap<DisplayLayout, Path2D[]>();

function piecePath(p: DisplayPiece): Path2D {
  const path = new Path2D();
  switch (p.kind) {
    case "polygon":
    case "path":
      p.points.forEach(([x, y], i) => (i === 0 ? path.moveTo(x, y) : path.lineTo(x, y)));
      if (p.kind === "polygon") path.closePath();
      break;
    case "circle":
      path.arc(p.cx, p.cy, p.r, 0, Math.PI * 2);
      break;
    case "ring":
      path.ellipse(p.cx, p.cy, p.rx, p.ry, 0, 0, Math.PI * 2);
      break;
  }
  return path;
}

function draw(ctx: CanvasRenderingContext2D, piece: DisplayPiece, path: Path2D) {
  if (piece.kind === "ring" || piece.kind === "path") {
    ctx.lineWidth = piece.width;
    ctx.stroke(path);
  } else {
    ctx.fill(path);
  }
}

export interface DisplayState {
  grid: readonly number[];
  /** The driver's display-on flag; off leaves every LED dark. */
  on: boolean;
  /** Driver brightness, 0..7. */
  brightness: number;
}

/**
 * Paints the display window into a `width` × `height` pixel canvas: the panel colour, every
 * unlit piece as a faint ghost, every lit piece in the lit colour with a glow. The window keeps
 * its aspect and is centred.
 */
export function paintDisplay(
  ctx: CanvasRenderingContext2D,
  layout: DisplayLayout,
  state: DisplayState,
  width: number,
  height: number,
) {
  let cache = paths.get(layout);
  if (!cache) {
    cache = layout.pieces.map(piecePath);
    paths.set(layout, cache);
  }
  const [wx, wy, ww, wh] = layout.window;
  const scale = Math.min(width / ww, height / wh);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.shadowBlur = 0;
  ctx.fillStyle = layout.panel;
  ctx.fillRect(0, 0, width, height);
  ctx.setTransform(
    scale,
    0,
    0,
    scale,
    (width - ww * scale) / 2 - wx * scale,
    (height - wh * scale) / 2 - wy * scale,
  );
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  const lit = layout.pieces.map((p) => state.on && isLit(p, state.grid));
  ctx.fillStyle = ctx.strokeStyle = layout.unlit;
  layout.pieces.forEach((p, i) => {
    if (!lit[i]) draw(ctx, p, cache[i] as Path2D);
  });
  // The firmware never dims the panel, but the driver could: scale the light with it.
  ctx.globalAlpha = 0.55 + 0.45 * (Math.min(7, Math.max(0, state.brightness)) / 7);
  ctx.fillStyle = ctx.strokeStyle = layout.lit;
  ctx.shadowColor = layout.lit;
  ctx.shadowBlur = Math.max(2, 1.2 * scale);
  layout.pieces.forEach((p, i) => {
    if (lit[i]) draw(ctx, p, cache[i] as Path2D);
  });
  ctx.globalAlpha = 1;
  ctx.shadowBlur = 0;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
}

/** The window's aspect ratio, width over height. */
export const displayAspect = (layout: DisplayLayout) => layout.window[2] / layout.window[3];

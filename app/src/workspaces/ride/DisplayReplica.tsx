import { useEffect, useRef } from "react";
import type { DisplayLayout } from "../../engine/types.ts";
import type { Outputs } from "../../engine/telemetry.gen.ts";
import { displayAspect, paintDisplay } from "./displayPaint.ts";

/** The wheel's display at `height` CSS pixels, repainted whenever the board outputs change. */
export function DisplayReplica({
  layout,
  outputs,
  height,
}: {
  layout: DisplayLayout;
  outputs: Outputs;
  height: number;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const width = Math.round(height * displayAspect(layout));
  useEffect(() => {
    const c = canvas.current;
    const ctx = c?.getContext("2d");
    if (!c || !ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const [w, h] = [Math.round(width * dpr), Math.round(height * dpr)];
    if (c.width !== w || c.height !== h) [c.width, c.height] = [w, h];
    const state = {
      grid: outputs.display_grid,
      on: outputs.display_on,
      brightness: outputs.display_brightness,
    };
    paintDisplay(ctx, layout, state, w, h);
  }, [layout, outputs, width, height]);
  return (
    <canvas
      ref={canvas}
      style={{ width, height }}
      className="rounded-8"
      role="img"
      aria-label={`Wheel display (${layout.board} board), ${outputs.display_on ? "lit" : "dark"}`}
      data-testid="display-replica"
    />
  );
}

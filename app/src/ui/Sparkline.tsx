import { useEffect, useRef } from "react";
import { onFrame } from "../lib/frameloop.ts";
import type { RingBuffer } from "../lib/ring.ts";
import { resolveColor, withAlpha } from "../lib/canvas.ts";
import { cx } from "../lib/cx.ts";

export interface SparklineProps {
  /** Live source: drawn from a ring buffer on the shared frame loop (never re-renders React). */
  ring?: RingBuffer;
  /** Static data (gallery, previews). */
  data?: ArrayLike<number>;
  /** How many of the newest samples to show. */
  window?: number;
  color: string;
  /** Fixed scale; otherwise it fits the visible data with a little headroom. */
  min?: number;
  max?: number;
  height?: number;
  className?: string;
  /** Redraw rate for live sources. */
  fps?: number;
}

function draw(
  canvas: HTMLCanvasElement,
  values: ArrayLike<number>,
  n: number,
  color: string,
  min?: number,
  max?: number,
) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w === 0 || h === 0) return;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (n < 2) return;

  let lo = min ?? Infinity;
  let hi = max ?? -Infinity;
  if (min === undefined || max === undefined) {
    for (let i = 0; i < n; i++) {
      const v = values[i] as number;
      if (min === undefined && v < lo) lo = v;
      if (max === undefined && v > hi) hi = v;
    }
    const pad = (hi - lo) * 0.15 || 1;
    if (min === undefined) lo -= pad;
    if (max === undefined) hi += pad;
  }
  const pad = 3; // keeps the end dot and stroke inside the canvas
  const x = (i: number) => (i / (n - 1)) * (w - pad * 2) + pad;
  const y = (v: number) => h - pad - ((v - lo) / (hi - lo || 1)) * (h - pad * 2);

  ctx.beginPath();
  for (let i = 0; i < n; i++) {
    const px = x(i);
    const py = y(values[i] as number);
    if (i === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  }
  ctx.lineWidth = 1.5;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.strokeStyle = color;
  ctx.stroke();

  // Soft fill under the line, fading to transparent.
  ctx.lineTo(x(n - 1), h);
  ctx.lineTo(x(0), h);
  ctx.closePath();
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, withAlpha(color, 0.22));
  grad.addColorStop(1, withAlpha(color, 0));
  ctx.fillStyle = grad;
  ctx.fill();

  // End dot.
  const ex = x(n - 1);
  const ey = y(values[n - 1] as number);
  ctx.beginPath();
  ctx.arc(ex, ey, 2.5, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

/** Small canvas line chart. */
export function Sparkline({
  ring,
  data,
  window: win = 300,
  color,
  min,
  max,
  height = 32,
  className,
  fps = 30,
}: SparklineProps) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const resolved = resolveColor(canvas, color);
    if (data) {
      const render = () => draw(canvas, data, data.length, resolved, min, max);
      render();
      const ro = new ResizeObserver(render);
      ro.observe(canvas);
      return () => ro.disconnect();
    }
    if (!ring) return;
    const buf = new Float64Array(win);
    let last = 0;
    let drawn = -1;
    return onFrame((now) => {
      if (ring.version === drawn || now - last < 1000 / fps) return;
      drawn = ring.version;
      last = now;
      const n = ring.copyTo(buf, win);
      draw(canvas, buf, n, resolved, min, max);
    });
  }, [ring, data, win, color, min, max, fps]);

  return (
    <canvas
      ref={ref}
      className={cx("block w-full", className)}
      style={{ height }}
      aria-hidden="true"
    />
  );
}

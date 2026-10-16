import { useEffect, useRef } from "react";
import uPlot from "uplot";
import { font } from "../../design/tokens.ts";
import { resolveColor, withAlpha } from "../../lib/canvas.ts";
import { onFrame } from "../../lib/frameloop.ts";
import { isLowPower } from "../../lib/power.ts";
import type { RingBuffer } from "../../lib/ring.ts";
import { SAMPLE_RATE } from "../../engine/types.ts";
import { scopeRange } from "./scopeRange.ts";

export interface ScopeProps {
  time: RingBuffer;
  values: RingBuffer;
  color: string;
  min: number;
  max: number;
  /** Visible window in seconds. */
  seconds: number;
  /** Shows the time axis (only on the bottom lane). */
  showTime?: boolean;
  unit: string;
  syncKey?: string;
}

const AXIS_FONT = `500 10px ${font.mono}`;

/**
 * One uPlot lane fed straight from the ring buffers on the shared frame loop: copy the window
 * into preallocated arrays, then setData. React renders this component once.
 */
export function Scope({
  time,
  values,
  color,
  min,
  max,
  seconds,
  showTime,
  unit,
  syncKey = "telemetry",
}: ScopeProps) {
  const host = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const el = host.current;
    if (!el) return;
    const stroke = resolveColor(el, color);
    const fg2 = resolveColor(el, "var(--color-fg-2)");
    const grid = "rgba(255, 255, 255, 0.06)";
    const cap = Math.ceil(seconds * SAMPLE_RATE) + 2;
    const xs = new Float64Array(cap);
    const ys = new Float64Array(cap);
    // The y range starts at the channel's scale and grows past the session's extremes
    // (scopeRange.ts); a run that starts over (sim time going back) resets it.
    let [lo, hi] = [min, max];
    let seenLo = Infinity;
    let seenHi = -Infinity;
    let lastNow = -Infinity;

    const opts: uPlot.Options = {
      width: Math.max(10, el.clientWidth),
      height: Math.max(10, el.clientHeight),
      pxAlign: false,
      padding: [8, 8, showTime ? 2 : 8, 0],
      legend: { show: false },
      cursor: {
        sync: { key: syncKey, setSeries: false },
        drag: { x: false, y: false, setScale: false },
        points: { size: 6, width: 0, fill: stroke },
        y: false,
      },
      select: { show: false, left: 0, top: 0, width: 0, height: 0 },
      scales: {
        x: { time: false, range: () => [-seconds, 0] },
        y: { range: () => [lo, hi] },
      },
      axes: [
        {
          // Every lane draws the time grid; only the bottom lane labels it.
          stroke: fg2,
          font: AXIS_FONT,
          size: showTime ? 22 : 0,
          gap: 4,
          ticks: { show: false },
          grid: { stroke: grid, width: 1 },
          // Skip the left edge: its label would collide with the lowest value label.
          splits: () => Array.from({ length: 5 }, (_, i) => -seconds + ((i + 1) * seconds) / 5),
          values: (_u, splits) =>
            splits.map((v) => (!showTime ? "" : Math.abs(v) < 1e-9 ? "now" : `${Math.round(v)} s`)),
        },
        {
          stroke: fg2,
          font: AXIS_FONT,
          size: 38,
          gap: 4,
          ticks: { show: false },
          grid: { stroke: grid, width: 1 },
          splits: () => [lo, (lo + hi) / 2, hi],
          values: (_u, splits) => splits.map((v) => `${Math.round(v)}`),
        },
      ],
      series: [
        {},
        {
          label: unit,
          stroke,
          width: 1.5,
          points: { show: false },
          fill: (u: uPlot) => {
            const g = u.ctx.createLinearGradient(0, u.bbox.top, 0, u.bbox.top + u.bbox.height);
            g.addColorStop(0, withAlpha(stroke, 0.18));
            g.addColorStop(1, withAlpha(stroke, 0));
            return g;
          },
        },
      ],
    };

    const plot = new uPlot(opts, [[], []], el);
    const ro = new ResizeObserver(() => {
      plot.setSize({ width: Math.max(10, el.clientWidth), height: Math.max(10, el.clientHeight) });
    });
    ro.observe(el);

    // Redraw only when new samples arrived (a paused run costs nothing), and at most 30 times a
    // second when the machine renders without a GPU.
    let drawn = -1;
    let lastAt = 0;
    const off = onFrame((frameAt) => {
      if (time.version === drawn) return;
      if (isLowPower() && frameAt - lastAt < 1000 / 30) return;
      drawn = time.version;
      lastAt = frameAt;
      const n = Math.min(time.copyTo(xs, cap), values.copyTo(ys, cap));
      const now = n > 0 ? (xs[n - 1] as number) : 0;
      if (now < lastNow) {
        seenLo = Infinity;
        seenHi = -Infinity;
      }
      lastNow = now;
      for (let i = 0; i < n; i++) {
        xs[i] = (xs[i] as number) - now;
        const v = ys[i] as number;
        if (v < seenLo) seenLo = v;
        if (v > seenHi) seenHi = v;
      }
      [lo, hi] = scopeRange(min, max, seenLo, seenHi);
      plot.setData([xs.subarray(0, n), ys.subarray(0, n)], true);
    });

    return () => {
      off();
      ro.disconnect();
      plot.destroy();
    };
  }, [time, values, color, min, max, seconds, showTime, unit, syncKey]);

  return <div ref={host} className="h-full w-full" />;
}

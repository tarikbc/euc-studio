import type { ReactNode } from "react";
import { cx } from "../lib/cx.ts";
import { RollingNumber } from "./RollingNumber.tsx";

export interface GaugeZone {
  from: number;
  tone: "caution" | "danger";
}

export interface GaugeProps {
  value: number;
  min?: number;
  max?: number;
  label: ReactNode;
  unit?: string;
  decimals?: number;
  /** Diameter in px. */
  size?: number;
  /** Thresholds where the arc turns caution or danger. */
  zones?: GaugeZone[];
  /** Arc color when no zone applies (CSS color). Defaults to the accent. */
  color?: string;
  className?: string;
}

const SWEEP = 270; // degrees, open at the bottom
const START = 135; // degrees from +x, clockwise (SVG)

function polar(cx: number, cy: number, r: number, deg: number) {
  const a = (deg * Math.PI) / 180;
  return [cx + r * Math.cos(a), cy + r * Math.sin(a)] as const;
}

function arcPath(cx: number, cy: number, r: number, from: number, to: number) {
  const [x0, y0] = polar(cx, cy, r, from);
  const [x1, y1] = polar(cx, cy, r, to);
  const large = to - from > 180 ? 1 : 0;
  return `M ${x0} ${y0} A ${r} ${r} 0 ${large} 1 ${x1} ${y1}`;
}

const toneColor = { caution: "var(--color-caution)", danger: "var(--color-danger)" };

/**
 * Arc gauge. The value arc moves with the data token (linear, 0.25 s) and changes color when it
 * enters a caution or danger zone; zone thresholds are marked with ticks on the track.
 */
export function Gauge({
  value,
  min = 0,
  max = 100,
  label,
  unit,
  decimals = 0,
  size = 96,
  zones = [],
  color = "var(--color-accent)",
  className,
}: GaugeProps) {
  const stroke = Math.max(4, Math.round(size / 14));
  const r = size / 2 - stroke / 2 - 1;
  const c = size / 2;
  // An unmodelled value (NaN) shows an empty arc.
  const frac = Number.isFinite(value) ? Math.min(1, Math.max(0, (value - min) / (max - min))) : 0;
  const length = (Math.PI * 2 * r * SWEEP) / 360;
  const active = [...zones].sort((a, b) => b.from - a.from).find((z) => value >= z.from);
  const arcColor = active ? toneColor[active.tone] : color;
  const track = arcPath(c, c, r, START, START + SWEEP);

  return (
    <div
      className={cx("relative inline-grid place-items-center", className)}
      style={{ width: size, height: size }}
      role="meter"
      aria-valuemin={min}
      aria-valuemax={max}
      aria-valuenow={Number.isFinite(value) ? Number(value.toFixed(decimals)) : undefined}
      aria-label={typeof label === "string" ? label : undefined}
    >
      <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`} className="absolute inset-0">
        <path
          d={track}
          fill="none"
          stroke="var(--color-track)"
          strokeWidth={stroke}
          strokeLinecap="round"
        />
        <path
          d={track}
          fill="none"
          stroke={arcColor}
          strokeWidth={stroke}
          strokeLinecap="round"
          strokeDasharray={`${length} ${length}`}
          strokeDashoffset={length * (1 - frac)}
          style={{
            transition:
              "stroke-dashoffset var(--dur-data) var(--ease-data), stroke var(--dur-hover) var(--ease-hover)",
          }}
        />
        {zones.map((z) => {
          const deg = START + SWEEP * ((z.from - min) / (max - min));
          const [x0, y0] = polar(c, c, r - stroke / 2 - 2, deg);
          const [x1, y1] = polar(c, c, r - stroke / 2 - 5, deg);
          return (
            <line
              key={z.from}
              x1={x0}
              y1={y0}
              x2={x1}
              y2={y1}
              stroke={toneColor[z.tone]}
              strokeWidth={1.5}
              strokeLinecap="round"
            />
          );
        })}
      </svg>
      <div className="relative flex flex-col items-center leading-none">
        <span className="flex items-baseline gap-2">
          <RollingNumber
            value={value}
            decimals={decimals}
            className={cx("font-medium text-fg", size >= 96 ? "text-20" : "text-15")}
          />
          {unit && <span className="text-11 font-semibold text-fg-2">{unit}</span>}
        </span>
        <span className="label mt-4 text-[10px] tracking-[0.8px]">{label}</span>
      </div>
    </div>
  );
}

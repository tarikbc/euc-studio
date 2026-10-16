import { cx } from "../../lib/cx.ts";
import { RollingNumber } from "../../ui/RollingNumber.tsx";

export interface PitchProtractorProps {
  /** Degrees, nose up positive. */
  pitch: number;
  /** Full scale either side (degrees). */
  range?: number;
  /** Where tiltback starts and where the firmware cuts out (degrees, both sides). */
  caution?: number;
  danger?: number;
  width?: number;
  className?: string;
}

// The dial spans 120 degrees of arc for ±range degrees of pitch (magnified so small leans read).
const ARC = 60;

function point(cx: number, cy: number, r: number, deg: number) {
  const a = ((deg - 90) * Math.PI) / 180;
  return [cx + r * Math.cos(a), cy + r * Math.sin(a)] as const;
}

function arc(cx: number, cy: number, r: number, from: number, to: number) {
  const [x0, y0] = point(cx, cy, r, from);
  const [x1, y1] = point(cx, cy, r, to);
  return `M ${x0} ${y0} A ${r} ${r} 0 0 1 ${x1} ${y1}`;
}

/** Pitch protractor: ticks every degree, labels every 5, a needle that moves with the data token. */
export function PitchProtractor({
  pitch,
  range = 15,
  caution = 8,
  danger = 12,
  width = 208,
  className,
}: PitchProtractorProps) {
  const h = Math.round(width * 0.42);
  const c = width / 2;
  const cy = h + 6;
  const r = width / 2 - 14;
  const k = ARC / range;
  // An unmodelled pitch (NaN) parks the needle at level.
  const clamped = Number.isFinite(pitch) ? Math.max(-range, Math.min(range, pitch)) : 0;
  const tone =
    Math.abs(pitch) >= danger
      ? "var(--color-danger)"
      : Math.abs(pitch) >= caution
        ? "var(--color-caution)"
        : "var(--color-fg)";

  const ticks = [];
  for (let d = -range; d <= range; d++) {
    const major = d % 5 === 0;
    const [x0, y0] = point(c, cy, r, d * k);
    const [x1, y1] = point(c, cy, r - (major ? 9 : 5), d * k);
    ticks.push(
      <line
        key={d}
        x1={x0}
        y1={y0}
        x2={x1}
        y2={y1}
        stroke={major ? "var(--color-fg-2)" : "var(--color-fg-3)"}
        strokeWidth={major ? 1.5 : 1}
        strokeLinecap="round"
      />,
    );
    if (major && d !== 0) {
      const [lx, ly] = point(c, cy, r - 20, d * k);
      ticks.push(
        <text
          key={`l${d}`}
          x={lx}
          y={ly}
          textAnchor="middle"
          dominantBaseline="middle"
          className="num fill-fg-2 text-[10px]"
        >
          {Math.abs(d)}
        </text>,
      );
    }
  }

  return (
    <div
      className={cx("flex flex-col items-center", className)}
      role="img"
      aria-label={
        Number.isFinite(pitch) ? `Pitch ${pitch.toFixed(1)} degrees` : "Pitch not modelled"
      }
    >
      <svg
        width={width}
        height={h + 10}
        viewBox={`0 0 ${width} ${h + 10}`}
        className="overflow-visible"
      >
        {/* Zones: caution then danger, both sides. */}
        <path
          d={arc(c, cy, r + 4, caution * k, danger * k)}
          stroke="var(--color-caution)"
          strokeWidth={2}
          fill="none"
          strokeLinecap="round"
          opacity={0.7}
        />
        <path
          d={arc(c, cy, r + 4, -danger * k, -caution * k)}
          stroke="var(--color-caution)"
          strokeWidth={2}
          fill="none"
          strokeLinecap="round"
          opacity={0.7}
        />
        <path
          d={arc(c, cy, r + 4, danger * k, range * k)}
          stroke="var(--color-danger)"
          strokeWidth={2}
          fill="none"
          strokeLinecap="round"
        />
        <path
          d={arc(c, cy, r + 4, -range * k, -danger * k)}
          stroke="var(--color-danger)"
          strokeWidth={2}
          fill="none"
          strokeLinecap="round"
        />
        {ticks}
        {/* Needle. */}
        <g
          style={{
            transform: `rotate(${clamped * k}deg)`,
            transformOrigin: `${c}px ${cy}px`,
            transition: "transform var(--dur-data) var(--ease-data)",
          }}
        >
          <line
            x1={c}
            y1={cy}
            x2={c}
            y2={cy - r + 2}
            stroke={tone}
            strokeWidth={2}
            strokeLinecap="round"
            style={{ transition: "stroke var(--dur-hover)" }}
          />
        </g>
        <circle
          cx={c}
          cy={cy}
          r={4}
          fill="var(--color-card)"
          stroke="var(--color-fg)"
          strokeWidth={1.5}
        />
      </svg>
      <div className="mt-4 flex items-baseline gap-2 text-15 font-medium text-fg">
        <RollingNumber value={pitch} decimals={1} signed />
        <span className="text-12 text-fg-2">°</span>
      </div>
    </div>
  );
}

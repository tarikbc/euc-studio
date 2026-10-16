import type { Outputs } from "../../engine/telemetry.gen.ts";
import type { LightsLayout } from "../../engine/types.ts";
import { ledCss, litCount } from "./ledColor.ts";

// The tail light: the bar across the back of the seat with a round cap at each end, each cap's
// ring of LEDs drawn face-on in the colour the firmware sent. LED places come from the profile's
// lights.toml; the caps are drawn larger than life so single LEDs stay readable. The rider's left
// cap is on the left.

/** Drawn radius of a ring, px. */
const RING_R = 19;
const LED_R = 3.2;
const CAP_R = RING_R + 7;

export function TailLight({
  layout,
  outputs,
  width,
}: {
  layout: LightsLayout;
  outputs: Outputs;
  width: number;
}) {
  // The wheel's +y is the rider's left: larger y goes further left.
  const lateral = layout.rings.map((r) => -r.centre_m[1]);
  const lo = Math.min(...lateral);
  const span = Math.max(...lateral) - lo || 1;
  const x = (v: number) => CAP_R + 2 + ((v - lo) / span) * (width - 2 * (CAP_R + 2));
  const cy = CAP_R + 2;
  const height = 2 * CAP_R + 18;
  const leds = outputs.leds as [number, number, number][];
  return (
    <svg
      width={width}
      height={height}
      viewBox={`0 0 ${width} ${height}`}
      role="img"
      aria-label={`Tail light, ${litCount(layout, leds)} of ${layout.leds.length} LEDs lit`}
      data-testid="tail-light"
    >
      <defs>
        <filter id="tail-glow" x="-100%" y="-100%" width="300%" height="300%">
          <feGaussianBlur stdDeviation="1.8" />
        </filter>
      </defs>
      {/* The bar between the caps. */}
      <rect
        x={x(lo)}
        y={cy - CAP_R * 0.62}
        width={x(lo + span) - x(lo)}
        height={CAP_R * 1.24}
        fill="#0E0E10"
        stroke="var(--color-hairline-strong)"
      />
      {layout.rings.map((r) => (
        <g key={r.id}>
          <circle
            cx={x(-r.centre_m[1])}
            cy={cy}
            r={CAP_R}
            fill="#111114"
            stroke="var(--color-hairline-strong)"
          />
          <text
            x={x(-r.centre_m[1])}
            y={height - 2}
            textAnchor="middle"
            className="fill-fg-3 text-[10px]"
          >
            {r.side === "left" ? "Left" : "Right"}
          </text>
        </g>
      ))}
      {layout.leds.map((l) => {
        const ring = layout.rings[l.ring];
        if (!ring) return null;
        const scale = RING_R / (ring.diameter_m / 2);
        const px = x(-ring.centre_m[1]) + l.face_m[0] * scale;
        const py = cy - l.face_m[1] * scale;
        const c = leds[l.index - 1];
        const on = c !== undefined && c[0] + c[1] + c[2] > 0;
        const fill = on ? ledCss(c) : "#1E1E22";
        return (
          <g key={l.index}>
            {on && <circle cx={px} cy={py} r={LED_R * 1.6} fill={fill} filter="url(#tail-glow)" />}
            <circle cx={px} cy={py} r={LED_R} fill={fill} />
          </g>
        );
      })}
    </svg>
  );
}

// The wheel's display, generic version: each TM1640 grid byte drawn as a 7-segment digit
// (bit 0..6 = segments a..g, bit 7 = decimal point), lit segments white with a soft glow, unlit
// ones a faint ghost. The exact board layout (big digits, bars, icons) replaces this when
// profiles/begode-extreme/display.toml lands.

import { useState } from "react";
import { cx } from "../../lib/cx.ts";

/** Segment polygons in a 40 × 64 cell, slanted like the real display. */
const SEGMENTS: [string, string][] = [
  ["a", "10,4 32,4 28,9 14,9"],
  ["b", "33,5 35,7 33,30 29,27 31,10"],
  ["c", "32,34 35,37 32,58 28,55 30,37"],
  ["d", "6,60 28,60 32,64 2,64"],
  ["e", "4,37 8,34 6,55 2,58"],
  ["f", "6,7 9,5 11,10 9,27 5,30"],
  ["g", "9,30 29,30 31,32 29,34 9,34 7,32"],
];

function Digit({ byte, size }: { byte: number; size: number }) {
  return (
    <svg width={(size * 40) / 64} height={size} viewBox="0 0 40 68" aria-hidden="true">
      {SEGMENTS.map(([name, points], bit) => {
        const on = (byte >> bit) & 1;
        return (
          <polygon
            key={name}
            points={points}
            className={cx(
              "transition-[fill,filter] duration-75",
              on
                ? "fill-fg [filter:drop-shadow(0_0_3px_rgba(242,242,247,0.75))]"
                : "fill-fg/[0.07]",
            )}
          />
        );
      })}
      <circle
        cx="37"
        cy="62"
        r="2.6"
        className={
          (byte >> 7) & 1
            ? "fill-fg [filter:drop-shadow(0_0_3px_rgba(242,242,247,0.75))]"
            : "fill-fg/[0.07]"
        }
      />
    </svg>
  );
}

export interface SevenSegmentProps {
  grid: readonly number[];
  on: boolean;
  /** 0..7, scales the glow. */
  brightness: number;
  size?: number;
  className?: string;
}

/** Shows every grid the firmware has written so far (grids it never used stay hidden). */
export function SevenSegment({ grid, on, brightness, size = 28, className }: SevenSegmentProps) {
  // Remember the highest grid ever written, so digits do not come and go as the value changes.
  const [lastUsed, setLastUsed] = useState(-1);
  const lastNow = grid.findLastIndex((b) => b !== 0);
  if (lastNow > lastUsed) setLastUsed(lastNow);
  const last = Math.max(lastUsed, lastNow);
  const shown = last < 0 ? [] : grid.slice(0, last + 1);
  return (
    <div
      className={cx("flex items-end gap-2", className)}
      style={{ opacity: on ? 0.55 + (brightness / 7) * 0.45 : 0.25 }}
      role="img"
      aria-label={on ? "Wheel display" : "Wheel display (off)"}
    >
      {shown.length === 0 ? (
        <span className="text-11 text-fg-3">Display blank</span>
      ) : (
        shown.map((b, i) => <Digit key={i} byte={on ? b : 0} size={size} />)
      )}
    </div>
  );
}

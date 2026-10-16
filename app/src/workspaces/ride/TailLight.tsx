import type { Outputs } from "../../engine/telemetry.gen.ts";
import type { LightsLayout } from "../../engine/types.ts";
import { type Rgb, linearCss, litCount, luma, ringLight, throughDiffuser } from "./ledColor.ts";

// The tail light seen from behind, to scale: the roller across the back of the seat, its two
// translucent halves lit by the LED rings inside, the dark band in the middle with the BLE
// module's blue light, and the speaker caps at the ends. Every colour comes from the frame the
// firmware sent and the diffuser in lights.toml; nothing here animates.

const PAD = 4;

export function TailLight({
  layout,
  outputs,
  width,
}: {
  layout: LightsLayout;
  outputs: Outputs;
  width: number;
}) {
  const roller = layout.roller;
  if (!roller) return null;
  const leds = outputs.leds as Rgb[];
  const tint = roller.diffuser_tint;
  const capR = Math.max(roller.radius_m, ...layout.speakers.map((s) => s.radius_m));
  const reach = Math.max(
    roller.glow_half_length_m,
    ...layout.speakers.map((s) => Math.abs(s.centre_m[1] - roller.centre_m[1])),
  );
  const scale = (width - 2 * PAD) / (2 * reach);
  const height = 2 * capR * scale + 2 * PAD;
  // Seen from behind: the rider's right (the wheel's -y) on the right, up is up.
  const sx = (y: number) => width / 2 - (y - roller.centre_m[1]) * scale;
  const sy = (z: number) => height / 2 - (z - roller.centre_m[2]) * scale;
  const r = roller.radius_m * scale;
  const band = roller.band_half_width_m * scale;
  const glow = roller.glow_half_length_m * scale;
  const mid = width / 2;
  // The right half (ring on the -y side) is drawn right of the band.
  const halves = layout.rings.map((ring, i) => {
    const right = ring.centre_m[1] < roller.centre_m[1];
    const light = ringLight(layout, leds, i, tint);
    // Perceived glow: a square root keeps a few comet LEDs visible and a full ring strong.
    return { id: ring.id, right, light, strength: Math.min(1, Math.sqrt(luma(light)) * 2.2) };
  });

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
        <filter id="tail-spot" x="-200%" y="-200%" width="500%" height="500%">
          <feGaussianBlur stdDeviation="2.2" />
        </filter>
        {/* The tube's roundness: darker toward the top and bottom edges. */}
        <linearGradient id="tail-round" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#000" stopOpacity="0.45" />
          <stop offset="0.45" stopColor="#000" stopOpacity="0" />
          <stop offset="1" stopColor="#000" stopOpacity="0.55" />
        </linearGradient>
        {halves.map((h) => (
          <linearGradient
            key={h.id}
            id={`tail-glow-${h.id}`}
            x1={h.right ? "0" : "1"}
            x2={h.right ? "1" : "0"}
            y1="0"
            y2="0"
          >
            <stop
              offset="0"
              stopColor={linearCss(h.light.map((v) => v * 3) as Rgb)}
              stopOpacity={h.strength}
            />
            <stop
              offset="1"
              stopColor={linearCss(h.light.map((v) => v * 3) as Rgb)}
              stopOpacity={h.strength * 0.25}
            />
          </linearGradient>
        ))}
      </defs>

      {/* Speaker caps at the ends. */}
      {layout.speakers.map((s) => {
        const edge = sx(s.centre_m[1]);
        const inner = s.centre_m[1] > roller.centre_m[1] ? mid - glow : mid + glow;
        const x = Math.min(edge, inner);
        return (
          <rect
            key={s.id}
            x={x - 1}
            y={height / 2 - s.radius_m * scale}
            width={Math.abs(edge - inner) + 2}
            height={2 * s.radius_m * scale}
            rx={3}
            fill="#121214"
            stroke="var(--color-hairline-strong)"
          />
        );
      })}

      {/* The glowing halves: the diffuser's own colour, the light from inside, the tube's shading. */}
      {halves.map((h) => {
        const x = h.right ? mid + band : mid - glow;
        const w = glow - band;
        return (
          <g key={h.id}>
            <rect
              x={x}
              y={height / 2 - r}
              width={w}
              height={2 * r}
              fill={roller.unlit}
              opacity={0.45}
            />
            <rect
              x={x}
              y={height / 2 - r}
              width={w}
              height={2 * r}
              fill={`url(#tail-glow-${h.id})`}
            />
          </g>
        );
      })}

      {/* Hot spots: LEDs facing the viewer show most; the diffuser scatters the rest a little. */}
      {layout.leds.map((l) => {
        const c = leds[l.index - 1];
        if (!c || c[0] + c[1] + c[2] === 0) return null;
        const seen = Math.min(1, Math.max(0, -l.direction[0]) + 0.12);
        return (
          <circle
            key={l.index}
            cx={sx(l.wheel_m[1])}
            cy={sy(l.wheel_m[2])}
            r={3.4}
            fill={linearCss(throughDiffuser(c, tint).map((v) => v * 2.5) as Rgb)}
            opacity={seen}
            filter="url(#tail-spot)"
          />
        );
      })}

      {halves.map((h) => {
        const x = h.right ? mid + band : mid - glow;
        return (
          <rect
            key={h.id}
            x={x}
            y={height / 2 - r}
            width={glow - band}
            height={2 * r}
            fill="url(#tail-round)"
          />
        );
      })}

      {/* The band, then the BLE module's light on it. */}
      <rect
        x={mid - band}
        y={height / 2 - r - 0.5}
        width={2 * band}
        height={2 * r + 1}
        fill={roller.band}
      />
      {layout.indicators.map((i) => (
        <g key={i.id}>
          <circle
            cx={sx(i.position_m[1])}
            cy={sy(i.position_m[2])}
            r={3}
            fill={i.colour}
            filter="url(#tail-spot)"
          />
          <circle cx={sx(i.position_m[1])} cy={sy(i.position_m[2])} r={1.4} fill={i.colour} />
        </g>
      ))}
    </svg>
  );
}

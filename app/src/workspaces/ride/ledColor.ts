import type { LightsLayout } from "../../engine/types.ts";

// WS2812 colours are PWM duty, which is linear light; screens want sRGB. These helpers work in
// linear light (0..1) and convert at the end.

export type Rgb = [number, number, number];

const toSrgb = (v: number) => Math.round(255 * Math.pow(Math.min(1, Math.max(0, v)), 1 / 2.2));

/** A linear colour (0..1 per channel) as CSS. */
export function linearCss([r, g, b]: Rgb): string {
  return `rgb(${toSrgb(r)} ${toSrgb(g)} ${toSrgb(b)})`;
}

/** A WS2812 colour (0..255 duty) as CSS. */
export function ledCss([r, g, b]: Rgb): string {
  return linearCss([r / 255, g / 255, b / 255]);
}

/** A WS2812 colour seen through a diffuser that passes `tint` of each channel, linear 0..1. */
export function throughDiffuser([r, g, b]: Rgb, tint: Rgb): Rgb {
  return [(r / 255) * tint[0], (g / 255) * tint[1], (b / 255) * tint[2]];
}

/** The mean colour of one ring's LEDs, through the diffuser, linear 0..1. */
export function ringLight(layout: LightsLayout, leds: Rgb[], ring: number, tint: Rgb): Rgb {
  const sum: Rgb = [0, 0, 0];
  let n = 0;
  for (const l of layout.leds) {
    if (l.ring !== ring) continue;
    n++;
    const c = leds[l.index - 1];
    if (!c) continue;
    const t = throughDiffuser(c, tint);
    for (let i = 0; i < 3; i++) sum[i] = (sum[i] as number) + (t[i] as number);
  }
  return n ? (sum.map((v) => v / n) as Rgb) : sum;
}

/** Perceived brightness of a linear colour, 0..1. */
export const luma = ([r, g, b]: Rgb) => 0.2126 * r + 0.7152 * g + 0.0722 * b;

/** How many of the layout's LEDs are lit. */
export function litCount(layout: LightsLayout, leds: Rgb[]): number {
  return layout.leds.filter((l) => {
    const c = leds[l.index - 1];
    return c !== undefined && c[0] + c[1] + c[2] > 0;
  }).length;
}

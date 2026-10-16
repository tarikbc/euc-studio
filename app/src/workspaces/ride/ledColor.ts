import type { LightsLayout } from "../../engine/types.ts";

/** A WS2812 colour as CSS. The values are PWM duty, which is linear light; screens want sRGB. */
export function ledCss([r, g, b]: [number, number, number]): string {
  const s = (v: number) => Math.round(255 * Math.pow(Math.min(255, v) / 255, 1 / 2.2));
  return `rgb(${s(r)} ${s(g)} ${s(b)})`;
}

/** How many of the layout's LEDs are lit. */
export function litCount(layout: LightsLayout, leds: [number, number, number][]): number {
  return layout.leds.filter((l) => {
    const c = leds[l.index - 1];
    return c !== undefined && c[0] + c[1] + c[2] > 0;
  }).length;
}

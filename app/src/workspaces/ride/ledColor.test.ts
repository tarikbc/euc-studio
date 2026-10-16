import { describe, expect, it } from "vitest";
import type { LightsLayout } from "../../engine/types.ts";
import { type Rgb, ledCss, litCount, ringLight, throughDiffuser } from "./ledColor.ts";

const layout: LightsLayout = {
  slots: 4,
  roller: null,
  rings: [
    {
      id: "A",
      side: "right",
      centre_m: [0, 0, 0],
      normal: [0, -1, 0],
      up: [0, 0, 1],
      facing: "radial",
      diameter_m: 0.02,
    },
  ],
  leds: [1, 2].map((index) => ({
    index,
    ring: 0,
    face_m: [0, 0] as [number, number],
    wheel_m: [0, 0, 0] as Rgb,
    direction: [-1, 0, 0] as Rgb,
  })),
  indicators: [],
  speakers: [],
  bar: null,
};

describe("ledCss", () => {
  it("turns linear PWM duty into sRGB, so comet tails stay faintly visible", () => {
    expect(ledCss([255, 0, 0])).toBe("rgb(255 0 0)");
    expect(ledCss([100, 78, 0])).toBe("rgb(167 149 0)");
    // A tail value of 1 still shows.
    expect(ledCss([1, 0, 0])).toBe("rgb(21 0 0)");
  });
});

describe("the diffuser", () => {
  const tint: Rgb = [1, 0.41, 0.18];

  it("passes red and lets green and blue through faintly", () => {
    const [r, g, b] = throughDiffuser([255, 255, 255], tint);
    expect(r).toBe(1);
    expect(g).toBeCloseTo(0.41);
    expect(b).toBeCloseTo(0.18);
  });

  it("averages a ring's LEDs", () => {
    const leds: Rgb[] = [
      [255, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
      [9, 9, 9],
    ];
    expect(ringLight(layout, leds, 0, tint)[0]).toBeCloseTo(0.5);
  });
});

describe("litCount", () => {
  it("counts only the layout's LEDs", () => {
    // LED 4 is lit but not part of a ring (a dark slot).
    const leds: Rgb[] = [
      [10, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
      [9, 9, 9],
    ];
    expect(litCount(layout, leds)).toBe(1);
  });
});

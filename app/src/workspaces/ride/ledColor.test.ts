import { describe, expect, it } from "vitest";
import type { LightsLayout } from "../../engine/types.ts";
import { ledCss, litCount } from "./ledColor.ts";

describe("ledCss", () => {
  it("turns linear PWM duty into sRGB, so comet tails stay faintly visible", () => {
    expect(ledCss([255, 0, 0])).toBe("rgb(255 0 0)");
    expect(ledCss([100, 78, 0])).toBe("rgb(167 149 0)");
    // A tail value of 1 still shows.
    expect(ledCss([1, 0, 0])).toBe("rgb(21 0 0)");
  });
});

describe("litCount", () => {
  it("counts only the layout's LEDs", () => {
    const layout: LightsLayout = {
      slots: 4,
      rings: [
        {
          id: "A",
          side: "right",
          centre_m: [0, 0, 0],
          normal: [0, -1, 0],
          up: [0, 0, 1],
          diameter_m: 0.02,
        },
      ],
      leds: [1, 2].map((index) => ({
        index,
        ring: 0,
        face_m: [0, 0] as [number, number],
        wheel_m: [0, 0, 0] as [number, number, number],
      })),
      bar: null,
    };
    // LED 4 is lit but not part of a ring (a dark slot).
    const leds: [number, number, number][] = [
      [10, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
      [9, 9, 9],
    ];
    expect(litCount(layout, leds)).toBe(1);
  });
});

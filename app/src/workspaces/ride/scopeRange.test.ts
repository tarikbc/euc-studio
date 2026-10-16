import { describe, expect, it } from "vitest";
import { niceStep, scopeRange } from "./scopeRange.ts";

describe("scopeRange", () => {
  it("keeps the channel's scale while the data stays inside it", () => {
    expect(scopeRange(0, 40, 0, 20.6)).toEqual([0, 40]);
    expect(scopeRange(-40, 40, -12, 35)).toEqual([-40, 40]);
  });

  it("widens just past a peak, to a round number", () => {
    // 63.6 km/h on a 0-40 lane: 63.6 + 4 headroom, rounded up to the next 10.
    expect(scopeRange(0, 40, 0, 63.6)).toEqual([0, 70]);
    // 237 A on a +-40 A lane: 237 + 8, rounded up to the next 20.
    expect(scopeRange(-40, 40, -40, 237)).toEqual([-40, 260]);
  });

  it("widens below the scale too", () => {
    // Rolling backwards at 6 km/h on a 0-40 lane.
    expect(scopeRange(0, 40, -6, 20)).toEqual([-10, 40]);
  });

  it("always leaves room above the extreme", () => {
    const [, hi] = scopeRange(0, 40, 0, 70);
    expect(hi).toBeGreaterThan(70);
  });
});

describe("niceStep", () => {
  it("rounds up to 1, 2 or 5 times a power of ten", () => {
    expect(niceStep(10)).toBe(10);
    expect(niceStep(20)).toBe(20);
    expect(niceStep(8.75)).toBe(10);
    expect(niceStep(17.5)).toBe(20);
    expect(niceStep(0.3)).toBeCloseTo(0.5);
  });
});

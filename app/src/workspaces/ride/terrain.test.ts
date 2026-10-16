import { describe, expect, it } from "vitest";
import type { TerrainProfile } from "../../engine/types.ts";
import { type Slope, slopeAt, strip } from "./terrain.ts";

/** Every 5 cm from -5 m to 100 m: flat to 25 m, then 2 % down, with a 2 cm pothole at 50-50.5 m. */
function downhill(): TerrainProfile {
  const out: TerrainProfile = [];
  for (let i = 0; i <= 2100; i++) {
    const x = -5 + i * 0.05;
    const slope = x > 25 ? -0.02 * (x - 25) : 0;
    const hole = i >= 1100 && i <= 1110 ? -0.02 : 0;
    out.push([x, slope + hole, slope]);
  }
  return out;
}

const z = (positions: Float32Array, vertex: number) => positions[vertex * 3 + 2] as number;

describe("strip", () => {
  it("draws the ground where the physics has it, below zero too", () => {
    const { positions } = strip(downhill(), 1.5);
    // The last sample, 100 m: 75 m down a 2 % slope. Both edges, nothing held at the flat floor.
    expect(z(positions, 4200)).toBeCloseTo(-1.5, 5);
    expect(z(positions, 4201)).toBeCloseTo(-1.5, 5);
    expect(positions[4200 * 3 + 1]).toBe(1.5);
    expect(positions[4201 * 3 + 1]).toBe(-1.5);
    // In the pothole the strip dips 2 cm below the slope.
    expect(z(positions, 2 * 1105)).toBeCloseTo(-0.02 * 25.25 - 0.02, 5);
  });

  it("marks potholes against the slope, not against zero", () => {
    const profile = downhill();
    const { holes } = strip(profile, 1.5);
    const marked = profile.filter((_, i) => holes[i]).map(([x]) => x);
    expect(marked.length).toBe(11);
    expect(Math.min(...marked)).toBeCloseTo(50, 5);
    expect(Math.max(...marked)).toBeCloseTo(50.5, 5);
  });

  it("lays a line across the strip at each whole metre, on the ground", () => {
    const { marks } = strip(downhill(), 1.5);
    expect(marks.length).toBe(106 * 6); // -5 m to 100 m
    const at80 = (80 + 5) * 6;
    expect(Array.from(marks.slice(at80, at80 + 6))).toEqual(
      [80, 1.5, -1.099, 80, -1.5, -1.099].map((v) => expect.closeTo(v, 5)),
    );
  });
});

describe("slopeAt", () => {
  const out: Slope = { z: Number.NaN, pitch: Number.NaN };

  it("is flat ground at zero without a profile", () => {
    expect(slopeAt(null, 12, out)).toEqual({ z: 0, pitch: 0 });
  });

  it("follows the slope under the wheel, past potholes", () => {
    const profile = downhill();
    slopeAt(profile, 80, out);
    expect(out.z).toBeCloseTo(-1.1, 5);
    expect(out.pitch).toBeCloseTo(Math.atan(-0.02), 6);
    // Over the pothole: the slope's height, not the hole's.
    expect(slopeAt(profile, 50.25, out).z).toBeCloseTo(-0.505, 5);
    // Between samples: interpolated.
    expect(slopeAt(profile, 30.025, out).z).toBeCloseTo(-0.1005, 5);
    // Past the ends: held.
    expect(slopeAt(profile, -20, out)).toEqual({ z: 0, pitch: 0 });
  });
});

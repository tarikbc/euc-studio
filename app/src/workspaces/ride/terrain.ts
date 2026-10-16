import type { TerrainProfile } from "../../engine/types.ts";

// A scenario's ground as the 3D view draws it, in the physics world frame (x forward, y left,
// z up). The road strip follows the ground exactly, potholes and all; what sits on the ground
// under the wheel (contact shadow, light pool) and the camera follow the slope alone, so they do
// not jump at a pothole.

type Sample = TerrainProfile[number];

/** Column `k` of the evenly spaced profile at `x`, linearly interpolated; the ends hold. */
function sampleAt(profile: TerrainProfile, x: number, k: 1 | 2): number {
  const n = profile.length;
  const first = profile[0] as Sample;
  const step = ((profile[n - 1] as Sample)[0] - first[0]) / (n - 1);
  const f = Math.min(Math.max((x - first[0]) / step, 0), n - 1);
  const i = Math.min(Math.floor(f), n - 2);
  const t = f - i;
  return (profile[i] as Sample)[k] * (1 - t) + (profile[i + 1] as Sample)[k] * t;
}

export interface Slope {
  /** Height of the slope, m. */
  z: number;
  /** Its pitch, rad (+ uphill along x). */
  pitch: number;
}

/** Half the base the pitch is measured over, m. */
const HALF_BASE = 0.25;

/** The slope under `x` (bumps and potholes left out); flat ground at z 0 without a profile. */
export function slopeAt(profile: TerrainProfile | null, x: number, out: Slope): Slope {
  if (!profile || profile.length < 2) {
    out.z = 0;
    out.pitch = 0;
    return out;
  }
  out.z = sampleAt(profile, x, 2);
  const rise = sampleAt(profile, x + HALF_BASE, 2) - sampleAt(profile, x - HALF_BASE, 2);
  out.pitch = Math.atan2(rise, 2 * HALF_BASE);
  return out;
}

/** Where the ground is deeper than this below the slope, the strip marks a pothole, m. */
const HOLE_M = 0.001;
/** The metre lines sit this far above the ground, so they do not fight its surface, m. */
const MARK_LIFT_M = 0.001;

export interface Strip {
  /** Two vertices per sample, left edge then right edge: x, y, z. */
  positions: Float32Array;
  /** Per sample: whether the ground there is a pothole (below the slope). */
  holes: boolean[];
  /** A line across the strip at each whole metre, on the ground: pairs of x, y, z. */
  marks: Float32Array;
}

/** The road strip over `profile`, `halfWidth` to each side of the path, at the ground's true height. */
export function strip(profile: TerrainProfile, halfWidth: number): Strip {
  const positions = new Float32Array(profile.length * 2 * 3);
  const holes = profile.map(([x, z, slope], i) => {
    positions.set([x, halfWidth, z, x, -halfWidth, z], i * 6);
    return z < slope - HOLE_M;
  });
  const metres: number[] = [];
  if (profile.length >= 2) {
    const from = Math.ceil((profile[0] as Sample)[0]);
    const to = Math.floor((profile[profile.length - 1] as Sample)[0]);
    for (let m = from; m <= to; m++) {
      const z = sampleAt(profile, m, 1) + MARK_LIFT_M;
      metres.push(m, halfWidth, z, m, -halfWidth, z);
    }
  }
  return { positions, holes, marks: new Float32Array(metres) };
}

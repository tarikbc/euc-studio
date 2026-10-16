import type * as THREE from "three";
import { engine } from "../../engine/index.ts";
import { POSE_FLOATS } from "../../engine/telemetry.gen.ts";

/** The wheel's position in three.js coordinates (y up), or null before the first snapshot. */
export function wheelPosition(out: THREE.Vector3): THREE.Vector3 | null {
  const p = engine.latest.poses;
  const i = engine.bodies.findIndex((b) => b.role === "wheel");
  if (!p || i < 0) return null;
  const o = i * POSE_FLOATS;
  // MuJoCo (x, y, z) → three.js (x, z, -y).
  return out.set(p[o] as number, p[o + 2] as number, -(p[o + 1] as number));
}

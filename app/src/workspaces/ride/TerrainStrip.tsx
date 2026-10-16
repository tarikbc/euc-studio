import { useMemo } from "react";
import * as THREE from "three";

// The scenario's ground along the path, in the physics world frame (x forward, y left, z up):
// a strip of road that follows the slope and bumps. The studio floor stays flat at z 0, so a
// pothole cannot be cut into it; the strip marks it darker instead.

/** Half the strip's width, m. */
const HALF_WIDTH = 0.6;
const ROAD = new THREE.Color("#2E2E33");
const HOLE = new THREE.Color("#000000");

export function TerrainStrip({ profile }: { profile: [number, number][] }) {
  const geometry = useMemo(() => {
    const n = profile.length;
    const positions = new Float32Array(n * 2 * 3);
    const colors = new Float32Array(n * 2 * 3);
    const index: number[] = [];
    profile.forEach(([x, z], i) => {
      // Just above the flat floor where the ground dips below it.
      const h = Math.max(z, 0) + 0.0008;
      const colour = z < -0.001 ? HOLE : ROAD;
      for (const [k, y] of [
        [0, HALF_WIDTH],
        [1, -HALF_WIDTH],
      ] as const) {
        positions.set([x, y, h], (i * 2 + k) * 3);
        colors.set([colour.r, colour.g, colour.b], (i * 2 + k) * 3);
      }
      if (i > 0) {
        const a = (i - 1) * 2;
        index.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
      }
    });
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    g.setIndex(index);
    g.computeVertexNormals();
    return g;
  }, [profile]);

  return (
    <mesh geometry={geometry} receiveShadow>
      {/* Matte: a PBR surface this large catches the environment at grazing angles and turns grey. */}
      <meshLambertMaterial vertexColors side={THREE.DoubleSide} />
    </mesh>
  );
}

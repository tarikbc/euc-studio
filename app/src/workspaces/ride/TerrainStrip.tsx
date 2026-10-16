import { useMemo } from "react";
import * as THREE from "three";
import { surface } from "../../design/tokens.ts";
import type { TerrainProfile } from "../../engine/types.ts";
import { strip } from "./terrain.ts";

// The scenario's ground along the path, in the physics world frame (x forward, y left, z up): a
// strip of road at the ground's true height, so the wheel and rider stand on it on a slope or in
// a pothole. On such a run the strip is the floor (the flat studio grid is not drawn); a line
// across it each metre shows the speed, and potholes are darker.

/** Half the strip's width: as wide as the physics ground, m. */
const HALF_WIDTH = 1.5;
const ROAD = new THREE.Color("#2E2E33");
const HOLE = new THREE.Color("#000000");

export function TerrainStrip({ profile }: { profile: TerrainProfile }) {
  const { road, marks } = useMemo(() => {
    const s = strip(profile, HALF_WIDTH);
    const colors = new Float32Array(s.positions.length);
    const index: number[] = [];
    s.holes.forEach((hole, i) => {
      const c = hole ? HOLE : ROAD;
      colors.set([c.r, c.g, c.b, c.r, c.g, c.b], i * 6);
      if (i > 0) {
        const a = (i - 1) * 2;
        index.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
      }
    });
    const road = new THREE.BufferGeometry();
    road.setAttribute("position", new THREE.BufferAttribute(s.positions, 3));
    road.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    road.setIndex(index);
    road.computeVertexNormals();
    const marks = new THREE.BufferGeometry();
    marks.setAttribute("position", new THREE.BufferAttribute(s.marks, 3));
    return { road, marks };
  }, [profile]);

  return (
    <>
      <mesh geometry={road} receiveShadow>
        {/* Matte: a PBR surface this large catches the environment at grazing angles and turns
            grey. One-sided (it faces up): seen from below, the road never covers the wheel. */}
        <meshLambertMaterial vertexColors />
      </mesh>
      <lineSegments geometry={marks}>
        <lineBasicMaterial color={surface.lift} />
      </lineSegments>
    </>
  );
}

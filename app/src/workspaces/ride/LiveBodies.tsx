import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";
import { engine, useEngineVersion } from "../../engine/index.ts";
import { POSE_FLOATS, type Geom } from "../../engine/telemetry.gen.ts";
import { ShellVisual, WheelVisual } from "./EucModel.tsx";

// The live engine's bodies, posed every frame from the latest snapshot. Everything sits inside
// one group that turns the physics world (MuJoCo: z up) into three.js (y up), so body poses go
// in unchanged. The wheel and the shell get the studio's stylized model; every other body (the
// rider) is drawn from its MJCF geoms.

/** Visuals are modelled y up with the axle on z; a body frame has z up with the axle on y. */
const TO_BODY_FRAME: [number, number, number] = [Math.PI / 2, 0, 0];

const riderMaterial = new THREE.MeshStandardMaterial({
  color: "#C7C7CC",
  roughness: 0.62,
  metalness: 0,
});

function GeomMesh({ geom }: { geom: Geom }) {
  const { kind, size, pos, quat } = geom;
  const geometry = useMemo(() => {
    switch (kind) {
      case "capsule":
        return new THREE.CapsuleGeometry(size[0], size[1] * 2, 6, 16);
      case "cylinder":
        return new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 24);
      case "box":
        return new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2);
      case "sphere":
        return new THREE.SphereGeometry(size[0], 24, 16);
      case "ellipsoid": {
        const g = new THREE.SphereGeometry(1, 24, 16);
        g.scale(size[0], size[1], size[2]);
        return g;
      }
    }
  }, [kind, size]);
  // three.js builds capsules and cylinders along y; MJCF puts them along the geom's z.
  const alongZ = kind === "capsule" || kind === "cylinder";
  return (
    <group position={pos} quaternion={new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0])}>
      <mesh
        geometry={geometry}
        material={riderMaterial}
        rotation={alongZ ? TO_BODY_FRAME : [0, 0, 0]}
        castShadow
        receiveShadow
      />
    </group>
  );
}

export function LiveBodies({ accent }: { accent: string }) {
  useEngineVersion();
  const bodies = engine.bodies;
  const nodes = useRef<(THREE.Group | null)[]>([]);

  useFrame(() => {
    const p = engine.latest.poses;
    if (!p) return;
    for (let i = 0; i < bodies.length; i++) {
      const g = nodes.current[i];
      const o = i * POSE_FLOATS;
      if (!g || o + POSE_FLOATS > p.length) continue;
      g.position.set(p[o] as number, p[o + 1] as number, p[o + 2] as number);
      // Poses are w x y z; three.js quaternions are x y z w.
      g.quaternion.set(
        p[o + 4] as number,
        p[o + 5] as number,
        p[o + 6] as number,
        p[o + 3] as number,
      );
    }
  });

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {bodies.map((b, i) => (
        <group
          key={`${i}-${b.name}`}
          ref={(g) => {
            nodes.current[i] = g;
          }}
        >
          {b.role === "wheel" ? (
            <group rotation={TO_BODY_FRAME}>
              <WheelVisual />
            </group>
          ) : b.role === "shell" ? (
            <group rotation={TO_BODY_FRAME}>
              <ShellVisual accent={accent} />
            </group>
          ) : (
            b.geoms.map((g, j) => <GeomMesh key={j} geom={g} />)
          )}
        </group>
      ))}
    </group>
  );
}

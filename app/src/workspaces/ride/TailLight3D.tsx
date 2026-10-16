import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";
import { engine } from "../../engine/index.ts";
import type { LightsLayout } from "../../engine/types.ts";

// The tail light on the 3D wheel: each LED of lights.toml at its place, just proud of its cap's
// face, lit every frame in the colour the firmware sent, with a soft additive halo. The firmware
// animates every pattern; nothing here does. With `body` the bar and its caps are drawn too (the
// stylized shell has none; a scanned shell brings its own).

type Vec3 = [number, number, number];

/** Wheel frame (x forward, y left, z up) to the shell's visual frame (x forward, y up, z right). */
const toShell = ([x, y, z]: Vec3): THREE.Vector3 => new THREE.Vector3(x, z, -y);

const LED_RADIUS = 0.0022;
/** LEDs sit this far out of the cap's face, m. */
const PROUD = 0.0012;
const OFF = new THREE.Color("#141416");
const BLACK = new THREE.Color(0, 0, 0);
const color = new THREE.Color();
const at = new THREE.Matrix4();
const Y = new THREE.Vector3(0, 1, 0);

export function TailLight3D({ layout, body = true }: { layout: LightsLayout; body?: boolean }) {
  const lenses = useRef<THREE.InstancedMesh>(null);
  const halos = useRef<THREE.InstancedMesh>(null);
  const shown = useRef<{ layout: LightsLayout; leds: unknown } | null>(null);
  const count = layout.leds.length;

  // Caps as discs facing out of the light; the bar as a cylinder between their centres.
  const parts = useMemo(() => {
    const caps = layout.rings.map((r) => {
      const normal = toShell(r.normal);
      return {
        id: r.id,
        position: toShell(r.centre_m),
        quaternion: new THREE.Quaternion().setFromUnitVectors(Y, normal),
        radius: r.diameter_m * 0.75,
      };
    });
    const [a, b] = caps;
    const bar =
      a && b
        ? {
            position: a.position.clone().add(b.position).multiplyScalar(0.5),
            quaternion: new THREE.Quaternion().setFromUnitVectors(
              Y,
              b.position.clone().sub(a.position).normalize(),
            ),
            length: a.position.distanceTo(b.position),
            radius: Math.max(a.radius, b.radius) * 1.02,
          }
        : null;
    const ledAt = layout.leds.map((led) => {
      const ring = layout.rings[led.ring];
      const p = toShell(led.wheel_m);
      return ring ? p.add(toShell(ring.normal).multiplyScalar(PROUD)) : p;
    });
    return { caps, bar, ledAt };
  }, [layout]);

  useFrame(() => {
    const l = lenses.current;
    const h = halos.current;
    if (!l || !h) return;
    const leds = engine.outputs?.leds;
    if (shown.current?.layout === layout && shown.current.leds === leds) return;
    if (shown.current?.layout !== layout) {
      parts.ledAt.forEach((p, i) => {
        at.makeTranslation(p);
        l.setMatrixAt(i, at);
        h.setMatrixAt(i, at);
      });
      l.instanceMatrix.needsUpdate = true;
      h.instanceMatrix.needsUpdate = true;
    }
    shown.current = { layout, leds };
    layout.leds.forEach((led, i) => {
      const c = leds?.[led.index - 1];
      const on = c !== undefined && c[0] + c[1] + c[2] > 0;
      // PWM duty is linear light.
      if (on) color.setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.LinearSRGBColorSpace);
      l.setColorAt(i, on ? color : OFF);
      h.setColorAt(i, on ? color : BLACK);
    });
    if (l.instanceColor) l.instanceColor.needsUpdate = true;
    if (h.instanceColor) h.instanceColor.needsUpdate = true;
  });

  return (
    <group>
      {body && parts.bar && (
        <mesh position={parts.bar.position} quaternion={parts.bar.quaternion} castShadow>
          <cylinderGeometry args={[parts.bar.radius, parts.bar.radius, parts.bar.length, 32]} />
          <meshStandardMaterial color="#1A1A1D" roughness={0.6} metalness={0.1} />
        </mesh>
      )}
      {body &&
        parts.caps.map((c) => (
          <mesh key={c.id} position={c.position} quaternion={c.quaternion}>
            <cylinderGeometry args={[c.radius, c.radius, 0.002, 32]} />
            <meshStandardMaterial color="#0E0E10" roughness={0.3} metalness={0.2} />
          </mesh>
        ))}
      <instancedMesh ref={lenses} args={[undefined, undefined, count]}>
        <sphereGeometry args={[LED_RADIUS, 10, 8]} />
        <meshBasicMaterial toneMapped={false} />
      </instancedMesh>
      <instancedMesh ref={halos} args={[undefined, undefined, count]}>
        <sphereGeometry args={[LED_RADIUS * 2.2, 10, 8]} />
        <meshBasicMaterial
          toneMapped={false}
          transparent
          opacity={0.35}
          blending={THREE.AdditiveBlending}
          depthWrite={false}
        />
      </instancedMesh>
    </group>
  );
}

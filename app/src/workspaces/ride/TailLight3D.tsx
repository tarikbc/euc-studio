import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";
import { engine } from "../../engine/index.ts";
import type { LightsLayout } from "../../engine/types.ts";
import { type Rgb, ringLight, throughDiffuser } from "./ledColor.ts";

// The tail light on the 3D wheel, from lights.toml: the roller across the back of the seat. Each
// translucent half shows the diffuser's own colour and glows with the mean light of its LED ring,
// fading from the dark centre band toward the speaker cap; every LED adds a hot spot where it
// shines through. The BLE module's blue status light sits on the band. On a scanned shell the
// scan's own sleeves (roller_sleeve_right for ring A's side, roller_sleeve_left for the other)
// and ble_led glow instead of drawn ones. The firmware animates every pattern; this only shows
// each frame. The studio's phone is connected while the engine runs, so the BLE light is steady
// (the module blinks it only while advertising).

/** Wheel frame (x forward, y left, z up) to the shell's visual frame (x forward, y up, z right). */
const toShell = ([x, y, z]: Rgb): THREE.Vector3 => new THREE.Vector3(x, z, -y);

/** Each half is drawn as this many rings, so its glow can fade toward the end. */
const SEGMENTS = 4;
const SPOT_RADIUS = 0.0035;
const Y = new THREE.Vector3(0, 1, 0);
const glowColor = new THREE.Color();
const spotColor = new THREE.Color();
const BLACK = new THREE.Color(0, 0, 0);
const at = new THREE.Matrix4();

interface Segment {
  ring: number;
  /** 0 next to the band .. 1 at the end. */
  along: number;
  position: THREE.Vector3;
  length: number;
}

/** A scanned shell's light meshes, each with its own material to light. */
interface ScanLights {
  /** Sleeve materials by ring index. */
  sleeves: Map<number, THREE.MeshStandardMaterial[]>;
  ble: THREE.MeshStandardMaterial[];
}

/** Gives each mesh under `node` its own copy of its material, and returns the copies. */
function ownMaterials(node: THREE.Object3D | undefined): THREE.MeshStandardMaterial[] {
  const out: THREE.MeshStandardMaterial[] = [];
  node?.traverse((o) => {
    if (o instanceof THREE.Mesh && o.material instanceof THREE.MeshStandardMaterial) {
      const m = o.material.clone();
      o.material = m;
      out.push(m);
    }
  });
  return out;
}

export function TailLight3D({ layout, scan }: { layout: LightsLayout; scan?: THREE.Object3D }) {
  const scanLights = useRef<{ root: THREE.Object3D; lights: ScanLights } | null>(null);
  const spots = useRef<THREE.InstancedMesh>(null);
  const status = useRef<THREE.Group>(null);
  const glows = useRef<(THREE.MeshStandardMaterial | null)[]>([]);
  const shown = useRef<{ layout: LightsLayout; leds: unknown } | null>(null);
  const roller = layout.roller;

  const parts = useMemo(() => {
    if (!roller) return null;
    const centre = toShell(roller.centre_m);
    const axis = toShell(roller.axis).normalize();
    const along = (d: number) => centre.clone().addScaledVector(axis, d);
    const span = roller.glow_half_length_m - roller.band_half_width_m;
    const segments: Segment[] = [];
    layout.rings.forEach((ring, i) => {
      // Which half the ring lights: the side of the centre it sits on.
      const sign = Math.sign(toShell(ring.centre_m).sub(centre).dot(axis)) || 1;
      for (let k = 0; k < SEGMENTS; k++) {
        const from = roller.band_half_width_m + (span * k) / SEGMENTS;
        segments.push({
          ring: i,
          along: k / (SEGMENTS - 1),
          position: along(sign * (from + span / SEGMENTS / 2)),
          length: span / SEGMENTS,
        });
      }
    });
    const turn = new THREE.Quaternion().setFromUnitVectors(Y, axis);
    // Hot spots on the sleeve, along each LED's direction from where its light shows.
    const spotAt = layout.leds.map((l) => {
      const ring = layout.rings[l.ring];
      const out = toShell(l.direction);
      const p = toShell(l.wheel_m);
      const lift = ring?.facing === "radial" ? roller.radius_m - ring.diameter_m / 2 : 0;
      return p.addScaledVector(out, lift + 0.0004);
    });
    const caps = layout.speakers.map((s) => {
      const c = toShell(s.centre_m);
      const out = toShell(s.normal).normalize();
      const length = Math.abs(c.clone().sub(centre).dot(axis)) - roller.glow_half_length_m;
      return {
        id: s.id,
        body: c.clone().addScaledVector(out, -length / 2),
        face: c.clone().addScaledVector(out, 0.0002),
        length,
        radius: s.radius_m,
        inner: s.grille_inner_m,
        outer: s.grille_outer_m,
        turn: new THREE.Quaternion().setFromUnitVectors(Y, out),
        faceTurn: new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), out),
      };
    });
    return {
      centre,
      turn,
      segments,
      spotAt,
      caps,
      indicators: layout.indicators.map((i) => ({ ...i, at: toShell(i.position_m) })),
    };
  }, [layout, roller]);

  useFrame(() => {
    const s = spots.current;
    if (!s || !parts || !roller) return;
    const root = scan;
    if (root && scanLights.current?.root !== root) {
      const sleeves = new Map<number, THREE.MeshStandardMaterial[]>();
      layout.rings.forEach((ring, i) => {
        const name = ring.side === "right" ? "roller_sleeve_right" : "roller_sleeve_left";
        sleeves.set(i, ownMaterials(root.getObjectByName(name)));
      });
      scanLights.current = {
        root,
        lights: { sleeves, ble: ownMaterials(root.getObjectByName("ble_led")) },
      };
      shown.current = null;
    }
    const lit = scanLights.current?.lights;
    const on = engine.outputs !== null;
    if (status.current) status.current.visible = on && !lit;
    for (const m of lit?.ble ?? [])
      m.emissive.set(on ? (layout.indicators[0]?.colour ?? "#3050FF") : "#000000");
    const leds = engine.outputs?.leds as Rgb[] | undefined;
    if (shown.current?.layout === layout && shown.current.leds === leds) return;
    if (shown.current?.layout !== layout) {
      parts.spotAt.forEach((p, i) => s.setMatrixAt(i, at.makeTranslation(p)));
      s.instanceMatrix.needsUpdate = true;
    }
    shown.current = { layout, leds };
    const tint = roller.diffuser_tint;
    lit?.sleeves.forEach((materials, ring) => {
      const light = leds ? ringLight(layout, leds, ring, tint) : ([0, 0, 0] as Rgb);
      glowColor.setRGB(light[0] * 2.5, light[1] * 2.5, light[2] * 2.5, THREE.LinearSRGBColorSpace);
      for (const m of materials) m.emissive.copy(glowColor);
    });
    parts.segments.forEach((seg, i) => {
      const m = glows.current[i];
      if (!m) return;
      const light = leds ? ringLight(layout, leds, seg.ring, tint) : ([0, 0, 0] as Rgb);
      // Brightest next to the band, a quarter of that at the end cap.
      const fade = 1 - 0.75 * seg.along;
      glowColor.setRGB(
        light[0] * fade * 3,
        light[1] * fade * 3,
        light[2] * fade * 3,
        THREE.LinearSRGBColorSpace,
      );
      m.emissive.copy(glowColor);
    });
    layout.leds.forEach((l, i) => {
      const c = leds?.[l.index - 1];
      const on = c !== undefined && c[0] + c[1] + c[2] > 0;
      if (on) {
        const [r, g, b] = throughDiffuser(c, tint);
        spotColor.setRGB(r * 2.5, g * 2.5, b * 2.5, THREE.LinearSRGBColorSpace);
      }
      s.setColorAt(i, on ? spotColor : BLACK);
    });
    if (s.instanceColor) s.instanceColor.needsUpdate = true;
  });

  if (!roller || !parts) return null;
  return (
    <group>
      {/* The translucent halves, as rings that can glow less toward the ends. */}
      {!scan &&
        parts.segments.map((seg, i) => (
          <mesh key={i} position={seg.position} quaternion={parts.turn}>
            <cylinderGeometry args={[roller.radius_m, roller.radius_m, seg.length, 32, 1, true]} />
            <meshStandardMaterial
              ref={(m) => {
                glows.current[i] = m;
              }}
              color={roller.unlit}
              roughness={0.35}
              transparent
              opacity={0.88}
              side={THREE.DoubleSide}
              toneMapped={false}
            />
          </mesh>
        ))}
      {/* The centre band. */}
      <mesh position={parts.centre} quaternion={parts.turn} visible={!scan}>
        <cylinderGeometry
          args={[roller.radius_m * 1.01, roller.radius_m * 1.01, roller.band_half_width_m * 2, 32]}
        />
        <meshStandardMaterial color={roller.band} roughness={0.5} />
      </mesh>
      {/* Speaker caps: a dark body and a grille ring on the outer face. */}
      {parts.caps.map((c) => (
        <group key={c.id} visible={!scan}>
          <mesh position={c.body} quaternion={c.turn} castShadow>
            <cylinderGeometry args={[c.radius, c.radius, c.length, 32]} />
            <meshStandardMaterial color="#121214" roughness={0.6} />
          </mesh>
          <mesh position={c.face} quaternion={c.faceTurn}>
            <ringGeometry args={[c.inner, c.outer, 32]} />
            <meshStandardMaterial color="#060607" roughness={0.9} />
          </mesh>
        </group>
      ))}
      {/* Hot spots where the LEDs shine through. */}
      <instancedMesh ref={spots} args={[undefined, undefined, layout.leds.length]}>
        <sphereGeometry args={[SPOT_RADIUS, 10, 8]} />
        <meshBasicMaterial
          toneMapped={false}
          transparent
          opacity={0.6}
          blending={THREE.AdditiveBlending}
          depthWrite={false}
        />
      </instancedMesh>
      {/* The BLE module's status light, steady while the wheel is on. */}
      <group ref={status} visible={false}>
        {parts.indicators.map((i) => (
          <mesh key={i.id} position={i.at}>
            <sphereGeometry args={[0.0014, 10, 8]} />
            <meshBasicMaterial color={i.colour} toneMapped={false} />
          </mesh>
        ))}
      </group>
    </group>
  );
}

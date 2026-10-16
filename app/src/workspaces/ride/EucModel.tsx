import { RoundedBox } from "@react-three/drei";
import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";
import { series } from "../../design/tokens.ts";
import { engine } from "../../engine/index.ts";
import { DisplayDecal, DisplayOnScan } from "./DisplayDecal.tsx";
import { TailLight3D } from "./TailLight3D.tsx";
import { type WheelModel, useWheelModel } from "./wheelModel.ts";

// A stylized electric unicycle built from primitives, about the size of a Begode Extreme
// (22 inch tire). Units are meters; the wheel rolls about z and travels along +x. The shell is
// narrower than the tire so the tire shows in front, behind and below, like the real wheel.

const TIRE_R = 0.28;
const TIRE_TUBE = 0.05;
const DEG = Math.PI / 180;
const SHELL_W = 0.4; // fore-aft
const SHELL_Z = 0.092; // side panel offset from the wheel plane

function TreadBlocks() {
  const count = 44;
  const matrices = useMemo(() => {
    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const axis = new THREE.Vector3(0, 0, 1);
    const one = new THREE.Vector3(1, 1, 1);
    return Array.from({ length: count }, (_, i) => {
      const a = (i / count) * Math.PI * 2;
      q.setFromAxisAngle(axis, a);
      return m
        .compose(
          new THREE.Vector3(Math.cos(a) * (TIRE_R - 0.003), Math.sin(a) * (TIRE_R - 0.003), 0),
          q,
          one,
        )
        .clone();
    });
  }, []);
  return (
    <instancedMesh
      ref={(mesh) => {
        if (!mesh) return;
        matrices.forEach((m, i) => mesh.setMatrixAt(i, m));
        mesh.instanceMatrix.needsUpdate = true;
      }}
      args={[undefined, undefined, count]}
      castShadow
    >
      <boxGeometry args={[0.014, 0.02, 0.074]} />
      <meshStandardMaterial color="#18181A" roughness={0.95} />
    </instancedMesh>
  );
}

/** Rounds the corners of a polygon with quadratic curves of radius `r`. */
function roundedShape(points: [number, number][], r: number): THREE.Shape {
  const shape = new THREE.Shape();
  const n = points.length;
  points.forEach(([x, y], i) => {
    const [px, py] = points[(i - 1 + n) % n] as [number, number];
    const [nx, ny] = points[(i + 1) % n] as [number, number];
    const l1 = Math.hypot(px - x, py - y);
    const l2 = Math.hypot(nx - x, ny - y);
    const ax = x + ((px - x) / l1) * r;
    const ay = y + ((py - y) / l1) * r;
    if (i === 0) shape.moveTo(ax, ay);
    else shape.lineTo(ax, ay);
    shape.quadraticCurveTo(x, y, x + ((nx - x) / l2) * r, y + ((ny - y) / l2) * r);
  });
  shape.closePath();
  return shape;
}

/**
 * Side panel profile: taller than wide and tapered toward the top, like a Begode shell. The
 * bevel adds 12 mm all round, so the outline is drawn that much smaller.
 */
function useSideShell() {
  return useMemo(() => {
    const bevel = 0.012;
    const w0 = SHELL_W / 2 - bevel;
    const w1 = w0 - 0.045;
    const shape = roundedShape(
      [
        [-w0, -0.25 + bevel],
        [w0, -0.25 + bevel],
        [w1, 0.25 - bevel],
        [-w1, 0.25 - bevel],
      ],
      0.07,
    );
    const depth = 0.026;
    const geo = new THREE.ExtrudeGeometry(shape, {
      depth,
      bevelEnabled: true,
      bevelThickness: bevel,
      bevelSize: bevel,
      bevelSegments: 5,
      curveSegments: 16,
    });
    geo.translate(0, 0, -depth / 2);
    geo.computeVertexNormals();
    return geo;
  }, []);
}

function ShellMaterial() {
  return (
    <meshPhysicalMaterial
      color="#3A3A3C"
      roughness={0.42}
      metalness={0.05}
      clearcoat={0.7}
      clearcoatRoughness={0.25}
    />
  );
}

/** Tire, tread, rim and hub motor, centred on the axle (axle along local z). */
export function WheelVisual() {
  return (
    <>
      <mesh castShadow receiveShadow>
        <torusGeometry args={[TIRE_R - TIRE_TUBE, TIRE_TUBE, 32, 120]} />
        <meshStandardMaterial color="#111113" roughness={0.85} />
      </mesh>
      <TreadBlocks />
      <mesh>
        <torusGeometry args={[TIRE_R - TIRE_TUBE * 1.95, 0.011, 12, 96]} />
        <meshStandardMaterial color="#AEAEB2" roughness={0.25} metalness={0.95} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.155, 0.155, 0.09, 64]} />
        <meshStandardMaterial color="#2C2C2E" roughness={0.35} metalness={0.85} />
      </mesh>
      {/* Hub caps with a slot so the spin reads even at rest. */}
      {[1, -1].map((s) => (
        <group key={s} position={[0, 0, s * 0.047]}>
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.075, 0.075, 0.006, 48]} />
            <meshStandardMaterial color="#636366" roughness={0.3} metalness={0.95} />
          </mesh>
          <mesh position={[0.04, 0, s * 0.0035]}>
            <boxGeometry args={[0.05, 0.01, 0.002]} />
            <meshStandardMaterial color="#111113" roughness={0.7} />
          </mesh>
        </group>
      ))}
    </>
  );
}

/**
 * Shell, pedals and lights around the axle (local frame: x forward, y up, axle along z). With
 * the live engine the tail light shows the firmware's LED colours; the demo trims the side panels
 * in the UI accent instead.
 */
export function ShellVisual({ accent }: { accent: string }) {
  const led = useMemo(() => new THREE.Color(accent), [accent]);
  const sideShell = useSideShell();
  return (
    <>
      {[1, -1].map((s) => (
        <group key={s} position={[0, 0.16, s * SHELL_Z]}>
          <mesh geometry={sideShell} castShadow receiveShadow>
            <ShellMaterial />
          </mesh>
          {/* Accent trim (demo only; the live wheel shows its real tail light). */}
          {!engine.lights && (
            <mesh position={[0, -0.17, s * 0.0262]}>
              <boxGeometry args={[0.3, 0.014, 0.002]} />
              <meshBasicMaterial color={led} toneMapped={false} />
            </mesh>
          )}
          {/* Vent detail. */}
          {[0, 1, 2, 3].map((i) => (
            <mesh key={i} position={[-0.09 + i * 0.06, 0.14, s * 0.0262]}>
              <boxGeometry args={[0.035, 0.006, 0.002]} />
              <meshStandardMaterial color="#0A0A0B" roughness={1} />
            </mesh>
          ))}
        </group>
      ))}
      {/* Fairing over the tire and the top bridge with the trolley handle. */}
      <RoundedBox
        args={[SHELL_W - 0.04, 0.12, 0.14]}
        radius={0.04}
        smoothness={4}
        position={[0, 0.34, 0]}
        castShadow
      >
        <ShellMaterial />
      </RoundedBox>
      <mesh position={[0, 0.405, 0]} castShadow>
        <torusGeometry args={[0.085, 0.013, 12, 40, Math.PI]} />
        <meshStandardMaterial color="#48484A" roughness={0.35} metalness={0.7} />
      </mesh>
      <DisplayDecal />
      {engine.lights && <TailLight3D layout={engine.lights} />}
      {/* Headlight on the front of the fairing; behind, a stand-in tail light for the demo (the
          live engine draws the real one from lights.toml). */}
      <mesh position={[(SHELL_W - 0.04) / 2 + 0.001, 0.335, 0]}>
        <boxGeometry args={[0.004, 0.036, 0.1]} />
        <meshStandardMaterial
          color="#F2F2F7"
          emissive="#F2F2F7"
          emissiveIntensity={1.4}
          toneMapped={false}
        />
      </mesh>
      {!engine.lights && (
        <mesh position={[-(SHELL_W - 0.04) / 2 - 0.001, 0.335, 0]}>
          <boxGeometry args={[0.004, 0.024, 0.08]} />
          <meshStandardMaterial
            color={series.current}
            emissive={series.current}
            emissiveIntensity={1.1}
            toneMapped={false}
          />
        </mesh>
      )}
      {/* Pedals: grip plate, edge and hinge bracket. */}
      {[1, -1].map((s) => (
        <group key={s} position={[0.01, -0.12, s * 0.17]}>
          <mesh castShadow>
            <boxGeometry args={[0.15, 0.018, 0.105]} />
            <meshStandardMaterial color="#2C2C2E" roughness={0.6} metalness={0.5} />
          </mesh>
          <mesh position={[0, 0.0095, 0]}>
            <boxGeometry args={[0.14, 0.002, 0.095]} />
            <meshStandardMaterial color="#0E0E10" roughness={1} />
          </mesh>
          <mesh position={[0, 0.03, -s * 0.05]} castShadow>
            <boxGeometry args={[0.05, 0.06, 0.016]} />
            <meshStandardMaterial color="#48484A" roughness={0.4} metalness={0.7} />
          </mesh>
        </group>
      ))}
    </>
  );
}

/** Axle height of the scanned tire, unloaded. */
const SCAN_AXLE_Y = 0.2296;
/**
 * Animating the rear shock waits for Tarik's press test (does the mount move with the axle?): until
 * then the shock and its mount stay as modelled, and the travel shows as the wheel moving against
 * the frame and as the Suspension channel. `?shock=animate` turns the motion below on.
 */
const ANIMATE_SHOCK =
  typeof location !== "undefined" &&
  new URLSearchParams(location.search).get("shock") === "animate";

const span = new THREE.Vector3();

/**
 * The GLB's shock parts and where the mount sits at full extension. The contract with the model:
 * "shock" has its origin on the eye that stays with the frame and is built along its local y;
 * "shock_mount" is whatever moves with the axle carriage, with its origin on the other eye.
 */
interface ShockRig {
  root: THREE.Object3D;
  shock: THREE.Object3D;
  mount: THREE.Object3D;
  mountRest: THREE.Vector3;
  /** The shock's local direction toward the mount (+y or -y, whichever the model chose). */
  toMount: THREE.Vector3;
  /** Eye to eye at rest, m. */
  rest: number;
}

function shockRig(root: THREE.Object3D): ShockRig | null {
  const shock = root.getObjectByName("shock");
  const mount = root.getObjectByName("shock_mount");
  if (!shock || !mount) return null;
  const toMount = mount.position.clone().sub(shock.position);
  const rest = toMount.length();
  toMount.applyQuaternion(shock.quaternion.clone().invert()).normalize();
  return { root, shock, mount, mountRest: mount.position.clone(), toMount, rest };
}

/**
 * Moves the mount up with the axle carriage by the axle travel and turns the shock from its fixed
 * eye toward the mount. Its length comes from the engine's shock channel (the linkage is a rocker
 * whose ratio changes along the travel); without it, from the distance to the mount.
 */
function poseShock(rig: ShockRig, travelM: number, shockM: number) {
  rig.mount.position.copy(rig.mountRest);
  rig.mount.position.y += travelM;
  span.subVectors(rig.mount.position, rig.shock.position);
  rig.shock.quaternion.setFromUnitVectors(rig.toMount, span.clone().normalize());
  const length = Number.isFinite(shockM) ? rig.rest - shockM : span.length();
  rig.shock.scale.y = length / rig.rest;
}

/** The spinning part: the scanned tire, hub and rim when the model loaded, else the stylized wheel. */
export function WheelPart({ model }: { model: WheelModel | null }) {
  const scanned = useMemo(() => model?.wheel.clone(true), [model]);
  return scanned ? <primitive object={scanned} /> : <WheelVisual />;
}

/**
 * The part that follows the shell: the scanned body (its own tail-light bar and pedals) with the
 * live display and LEDs on it, else the stylized shell.
 */
export function BodyPart({ model, accent }: { model: WheelModel | null; accent: string }) {
  const scanned = useMemo(() => model?.body.clone(true), [model]);
  // The shock follows the suspension channel (behind ANIMATE_SHOCK; the links stay with the frame).
  const rig = useRef<ShockRig | null>(null);
  useFrame(() => {
    if (!scanned || !ANIMATE_SHOCK) return;
    if (rig.current?.root !== scanned) {
      rig.current = shockRig(scanned);
      if (!rig.current) return;
    }
    // A wheel without suspension in its profile, or a run that does not model it: at rest.
    const l = engine.latest;
    const travel = engine.suspension && Number.isFinite(l.suspension) ? l.suspension : 0;
    const shock = engine.suspension ? l.shock : Number.NaN;
    poseShock(rig.current, travel / 1000, shock / 1000);
  });
  if (!scanned) return <ShellVisual accent={accent} />;
  return (
    <>
      <primitive object={scanned} />
      <DisplayOnScan body={scanned} />
      {engine.lights && <TailLight3D layout={engine.lights} scan={scanned} />}
    </>
  );
}

/** The demo's kinematic wheel: rolls by distance, pitches and rolls with the demo signals. */
export function EucModel({ accent }: { accent: string }) {
  const model = useWheelModel();
  const spin = useRef<THREE.Group>(null);
  const body = useRef<THREE.Group>(null);
  const root = useRef<THREE.Group>(null);

  useFrame(() => {
    const l = engine.latest;
    if (spin.current) spin.current.rotation.z = -l.wheelAngle;
    // Nose up (positive pitch) tilts the shell back around the axle; roll is a slight sway.
    if (body.current) body.current.rotation.z = l.pitch * DEG;
    if (root.current) root.current.rotation.x = l.roll * DEG;
  });

  return (
    <group ref={root} position={[0, model ? SCAN_AXLE_Y : TIRE_R, 0]}>
      <group ref={spin}>
        <WheelPart model={model} />
      </group>
      <group ref={body}>
        <BodyPart model={model} accent={accent} />
      </group>
    </group>
  );
}

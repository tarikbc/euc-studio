import { ContactShadows, Environment, Grid, Lightformer, OrbitControls } from "@react-three/drei";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { useReducedMotion } from "motion/react";
import { type ComponentRef, Suspense, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { accents, surface } from "../../design/tokens.ts";
import { engine } from "../../engine/index.ts";
import type { CameraMode } from "../../state/ui.ts";
import { useUi } from "../../state/ui.ts";
import { setLowPower } from "../../lib/power.ts";
import { EucModel } from "./EucModel.tsx";
import { LiveBodies } from "./LiveBodies.tsx";
import { wheelPosition } from "./poses.ts";
import { type Slope, slopeAt } from "./terrain.ts";
import { TerrainStrip } from "./TerrainStrip.tsx";
import { LOW_FPS, type Quality, chooseQuality, qualityOverride, rendererName } from "./quality.ts";

const TARGET = new THREE.Vector3(0.02, 0.34, 0);
/** Where the camera looks: the axle area of the wheel, wherever the wheel has gone. */
const focus = new THREE.Vector3();
const wheelAt = new THREE.Vector3();
/** The slope under the live wheel (flat ground: z 0). */
const ground: Slope = { z: 0, pitch: 0 };
function focusPoint(out: THREE.Vector3): THREE.Vector3 {
  if (engine.kind === "live" && wheelPosition(wheelAt)) {
    // With a rider on board, frame the whole person: look at the hips, not the axle. Up or
    // down a slope, the view goes with the ground.
    const y = slopeAt(engine.terrain, wheelAt.x, ground).z + (hasRider() ? 0.85 : TARGET.y);
    return out.set(wheelAt.x + TARGET.x, y, wheelAt.z + TARGET.z);
  }
  return out.copy(TARGET);
}

function hasRider(): boolean {
  return engine.bodies.some((b) => b.role === "rider");
}

/**
 * Keeps its children under the live wheel (floor light, contact shadow), laid on the slope there;
 * still for the demo.
 */
function FollowWheel({ children }: { children: React.ReactNode }) {
  const group = useRef<THREE.Group>(null);
  useFrame(() => {
    if (group.current && engine.kind === "live" && wheelPosition(wheelAt)) {
      slopeAt(engine.terrain, wheelAt.x, ground);
      // Physics x is three.js x and physics z is three.js y: the pitch turns about three.js z.
      group.current.position.set(wheelAt.x, ground.z, wheelAt.z);
      group.current.rotation.z = ground.pitch;
    }
  });
  return <group ref={group}>{children}</group>;
}
const FOLLOW_OFFSET = new THREE.Vector3(1.05, 0.52, 1.95);
const sway = new THREE.Vector3();
const shift = new THREE.Vector3();

/**
 * Ground grid that scrolls under the wheel, so speed reads even though the wheel stays put. Only
 * on flat ground (`flat`): on a scenario's terrain the road strip is the floor, and a flat grid
 * would cover the road where it goes downhill.
 */
function Floor({ flat }: { flat: boolean }) {
  const grid = useRef<THREE.Mesh>(null);
  useFrame(() => {
    // The demo wheel stays put and the floor scrolls; the live wheel moves over a still floor.
    if (grid.current && engine.kind === "demo")
      grid.current.position.x = -(engine.latest.distance % 1);
  });
  return (
    <>
      {flat && (
        <>
          <Grid
            ref={grid}
            infiniteGrid
            cellSize={0.25}
            sectionSize={1}
            cellThickness={0.6}
            sectionThickness={1}
            cellColor={surface.raised}
            sectionColor={surface.lift}
            fadeDistance={11}
            fadeStrength={2.2}
            position={[0, 0.0005, 0]}
          />
          <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
            <planeGeometry args={[40, 40]} />
            <shadowMaterial transparent opacity={0.35} />
          </mesh>
        </>
      )}
      <FollowWheel>
        <Spot />
      </FollowWheel>
    </>
  );
}

/** A soft pool of light on the floor that frames the wheel, like a studio sweep. */
function Spot() {
  const texture = useMemo(() => {
    const c = document.createElement("canvas");
    c.width = c.height = 256;
    const g = c.getContext("2d");
    if (g) {
      const grad = g.createRadialGradient(128, 128, 0, 128, 128, 128);
      grad.addColorStop(0, "rgba(255, 255, 255, 0.09)");
      grad.addColorStop(0.55, "rgba(255, 255, 255, 0.03)");
      grad.addColorStop(1, "rgba(255, 255, 255, 0)");
      g.fillStyle = grad;
      g.fillRect(0, 0, 256, 256);
    }
    const t = new THREE.CanvasTexture(c);
    t.colorSpace = THREE.SRGBColorSpace;
    return t;
  }, []);
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.0003, 0]}>
      <planeGeometry args={[4.5, 4.5]} />
      <meshBasicMaterial map={texture} transparent depthWrite={false} />
    </mesh>
  );
}

/**
 * Follow: the camera eases to a three-quarter view and breathes with speed. Orbit: free orbit
 * with damping. With reduced motion, camera moves cut instead of easing. `still` (low quality)
 * drops the idle sway so a paused scene needs no redraws.
 */
function CameraRig({ mode, still }: { mode: CameraMode; still: boolean }) {
  const camera = useThree((s) => s.camera);
  const controls = useRef<ComponentRef<typeof OrbitControls>>(null);
  const reduce = useReducedMotion();
  const desired = useRef(new THREE.Vector3());
  const first = useRef(true);
  /** Whether the orbit distance was set for a rider (null: not set yet). */
  const orbitFramed = useRef<boolean | null>(null);

  useFrame((state, dt) => {
    focusPoint(focus);
    if (mode === "orbit") {
      const c = controls.current;
      if (!c) return;
      const rider = hasRider();
      if (orbitFramed.current !== rider) {
        // First frame: the follow view's direction. Later (a rider arrived or left): the
        // direction the user chose. Either way at the follow view's distance for that framing.
        const distance = FOLLOW_OFFSET.distanceTo(TARGET) * (rider ? 2 : 1);
        if (orbitFramed.current === null) shift.subVectors(FOLLOW_OFFSET, TARGET);
        else shift.subVectors(camera.position, c.target);
        shift.setLength(distance);
        c.target.copy(focus);
        camera.position.copy(focus).add(shift);
        orbitFramed.current = rider;
        return;
      }
      // Orbit around the wheel wherever it rides: carry the camera along with the target.
      shift.subVectors(focus, c.target);
      c.target.add(shift);
      camera.position.add(shift);
      return;
    }
    orbitFramed.current = null;
    const l = engine.latest;
    const t = state.clock.elapsedTime;
    const speed = Number.isFinite(l.speed) ? Math.abs(l.speed) : 0;
    const pull = Math.min(speed / 30, 1) * 0.5; // back off a little at speed
    desired.current
      .copy(FOLLOW_OFFSET)
      .multiplyScalar((1 + pull) * (hasRider() ? 2 : 1))
      .add(
        still
          ? sway.set(0, 0, 0)
          : sway.set(Math.sin(t * 0.17) * 0.06, Math.sin(t * 0.23) * 0.025, 0),
      )
      .add(focus)
      .sub(TARGET);
    if (reduce || still || first.current) {
      camera.position.copy(desired.current);
      first.current = false;
    } else {
      camera.position.lerp(desired.current, 1 - Math.exp(-dt * 2.2));
    }
    camera.lookAt(focus);
  });

  useEffect(() => {
    if (mode === "orbit" && controls.current) {
      controls.current.target.copy(focusPoint(focus));
      controls.current.update();
    }
  }, [mode]);

  return mode === "orbit" ? (
    <OrbitControls
      ref={controls}
      makeDefault
      enableDamping={!reduce}
      dampingFactor={0.08}
      minDistance={1.2}
      maxDistance={12}
      maxPolarAngle={Math.PI / 2 - 0.05}
      target={TARGET}
    />
  ) : null;
}

const KEY_OFFSET = new THREE.Vector3(2.5, 4, 2.5);

function Lights() {
  // The key light (and its shadow) travels with the live wheel.
  const key = useRef<THREE.DirectionalLight>(null);
  useFrame(() => {
    const light = key.current;
    if (!light || engine.kind !== "live" || !wheelPosition(wheelAt)) return;
    light.position.copy(wheelAt).add(KEY_OFFSET);
    light.target.position.copy(wheelAt);
    light.target.updateMatrixWorld();
  });
  return (
    <>
      <hemisphereLight args={["#F2F2F7", "#000000", 0.35]} />
      <directionalLight
        ref={key}
        position={KEY_OFFSET.toArray()}
        intensity={2.8}
        castShadow
        shadow-mapSize={[1024, 1024]}
        shadow-camera-left={-2}
        shadow-camera-right={2}
        shadow-camera-top={2.2}
        shadow-camera-bottom={-2}
        shadow-bias={-0.0004}
      />
      {/* Warm rim from behind separates the dark shell from the black background. */}
      <directionalLight position={[-2.5, 1.6, -2.2]} intensity={2.2} color="#FFB27A" />
      <directionalLight position={[-1, 0.8, 3]} intensity={0.5} color="#CDE8FF" />
      {/* Studio softboxes built in-scene: no HDR download, works offline. */}
      <Environment resolution={256} frames={1}>
        <Lightformer
          form="rect"
          intensity={2.4}
          position={[0, 4, 0]}
          rotation-x={Math.PI / 2}
          scale={[5, 5, 1]}
        />
        <Lightformer
          form="rect"
          intensity={1.4}
          position={[-4, 1.2, 2]}
          rotation-y={Math.PI / 2}
          scale={[4, 1.5, 1]}
        />
        <Lightformer
          form="rect"
          intensity={1}
          position={[4, 1.2, 2]}
          rotation-y={-Math.PI / 2}
          scale={[4, 1.5, 1]}
        />
        <Lightformer form="ring" intensity={0.8} color="#FF7A1A" position={[0, 1, -5]} scale={2} />
      </Environment>
    </>
  );
}

/**
 * In low quality the scene redraws on a fixed clock, and only when the simulation moved. Orbit
 * drags still redraw at once (the controls invalidate on change).
 */
function Ticker({ fps }: { fps: number }) {
  const invalidate = useThree((s) => s.invalidate);
  useEffect(() => {
    let drawnAt = Number.NaN;
    const id = window.setInterval(() => {
      if (engine.latest.t === drawnAt) return;
      drawnAt = engine.latest.t;
      invalidate();
    }, 1000 / fps);
    return () => window.clearInterval(id);
  }, [fps, invalidate]);
  return null;
}

export interface SceneProps {
  active: boolean;
  onReady?: (info: { quality: Quality; renderer: string }) => void;
  /** Called once, after the scene has drawn its first frame (tests wait on it). */
  onFirstFrame?: () => void;
}

/** Reports the first drawn frame (from the frame after it, so the first one has reached the screen). */
function FirstFrame({ onFirstFrame }: { onFirstFrame?: () => void }) {
  const done = useRef(false);
  useFrame(() => {
    if (done.current) return;
    done.current = true;
    requestAnimationFrame(() => onFirstFrame?.());
  });
  return null;
}

export function Scene({ active, onReady, onFirstFrame }: SceneProps) {
  const camera = useUi((s) => s.camera);
  const accent = useUi((s) => s.accent);
  const [override] = useState(() => qualityOverride(location.search));
  const [quality, setQuality] = useState<Quality>(override ?? "high");
  const high = quality === "high";

  return (
    <Canvas
      shadows={high}
      dpr={high ? [1, 2] : 1}
      frameloop={!active ? "never" : high ? "always" : "demand"}
      camera={{ position: FOLLOW_OFFSET.toArray(), fov: 30, near: 0.05, far: 60 }}
      gl={{ antialias: true, powerPreference: "high-performance" }}
      onCreated={({ gl, get }) => {
        // Dev builds: scripts and agents reach the scene (camera, controls) without clicks.
        if (import.meta.env.DEV) Object.assign(window, { __eucScene: get });
        gl.toneMapping = THREE.ACESFilmicToneMapping;
        gl.toneMappingExposure = 1.05;
        const renderer = rendererName(gl.getContext());
        const chosen = chooseQuality(renderer, override);
        setQuality(chosen);
        setLowPower(chosen === "low");
        onReady?.({ quality: chosen, renderer });
      }}
      aria-label="3D view of the wheel"
    >
      <color attach="background" args={[surface.bg]} />
      <fog attach="fog" args={[surface.bg, 5, 13]} />
      <Suspense fallback={null}>
        <Lights />
        <Floor flat={!engine.terrain} />
        {engine.terrain && (
          // The physics world (z up) in three.js (y up), as the live bodies.
          <group rotation={[-Math.PI / 2, 0, 0]}>
            <TerrainStrip profile={engine.terrain} />
          </group>
        )}
        {engine.kind === "live" ? (
          <LiveBodies accent={accents[accent].accent} />
        ) : (
          <EucModel accent={accents[accent].accent} />
        )}
        <FollowWheel>
          <ContactShadows
            // The key: a new component when quality changes, so `frames` applies from the start.
            key={quality}
            position={[0, 0.001, 0]}
            opacity={0.75}
            scale={3}
            blur={2.2}
            far={0.9}
            resolution={high ? 512 : 256}
            frames={high ? Infinity : 1}
          />
        </FollowWheel>
      </Suspense>
      <CameraRig mode={camera} still={!high} />
      <FirstFrame onFirstFrame={onFirstFrame} />
      {active && !high && <Ticker fps={LOW_FPS} />}
    </Canvas>
  );
}

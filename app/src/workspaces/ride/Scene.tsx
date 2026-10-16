import { ContactShadows, Environment, Grid, Lightformer, OrbitControls } from "@react-three/drei";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { useReducedMotion } from "motion/react";
import { type ComponentRef, Suspense, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { accents, surface } from "../../design/tokens.ts";
import { engine } from "../../sim/engine.ts";
import type { CameraMode } from "../../state/ui.ts";
import { useUi } from "../../state/ui.ts";
import { setLowPower } from "../../lib/power.ts";
import { EucModel } from "./EucModel.tsx";
import { LOW_FPS, type Quality, chooseQuality, qualityOverride, rendererName } from "./quality.ts";

const TARGET = new THREE.Vector3(0.02, 0.34, 0);
const FOLLOW_OFFSET = new THREE.Vector3(1.05, 0.52, 1.95);
const sway = new THREE.Vector3();

/** Ground grid that scrolls under the wheel, so speed reads even though the wheel stays put. */
function Floor() {
  const grid = useRef<THREE.Mesh>(null);
  useFrame(() => {
    if (grid.current) grid.current.position.x = -(engine.latest.distance % 1);
  });
  return (
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
      <Spot />
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

  useFrame((state, dt) => {
    if (mode !== "follow") return;
    const l = engine.latest;
    const t = state.clock.elapsedTime;
    const pull = Math.min(l.speed / 30, 1) * 0.5; // back off a little at speed
    desired.current
      .copy(FOLLOW_OFFSET)
      .multiplyScalar(1 + pull)
      .add(
        still
          ? sway.set(0, 0, 0)
          : sway.set(Math.sin(t * 0.17) * 0.06, Math.sin(t * 0.23) * 0.025, 0),
      );
    if (reduce || still || first.current) {
      camera.position.copy(desired.current);
      first.current = false;
    } else {
      camera.position.lerp(desired.current, 1 - Math.exp(-dt * 2.2));
    }
    camera.lookAt(TARGET);
  });

  useEffect(() => {
    if (mode === "orbit" && controls.current) {
      controls.current.target.copy(TARGET);
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
      maxDistance={7}
      maxPolarAngle={Math.PI / 2 - 0.05}
      target={TARGET}
    />
  ) : null;
}

function Lights() {
  return (
    <>
      <hemisphereLight args={["#F2F2F7", "#000000", 0.35]} />
      <directionalLight
        position={[2.5, 4, 2.5]}
        intensity={2.8}
        castShadow
        shadow-mapSize={[1024, 1024]}
        shadow-camera-left={-1.5}
        shadow-camera-right={1.5}
        shadow-camera-top={1.5}
        shadow-camera-bottom={-1.5}
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
}

export function Scene({ active, onReady }: SceneProps) {
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
      onCreated={({ gl }) => {
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
        <Floor />
        <EucModel accent={accents[accent].accent} />
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
      </Suspense>
      <CameraRig mode={camera} still={!high} />
      {active && !high && <Ticker fps={LOW_FPS} />}
    </Canvas>
  );
}

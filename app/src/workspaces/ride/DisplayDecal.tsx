import { useFrame } from "@react-three/fiber";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { engine, useEngineVersion } from "../../engine/index.ts";
import type { Outputs } from "../../engine/telemetry.gen.ts";
import type { DisplayLayout } from "../../engine/types.ts";
import { displayAspect, paintDisplay } from "./displayPaint.ts";

// The wheel's display on top of the 3D shell: the same replica as the HUD card, painted into a
// canvas texture whenever the firmware changes the display memory, laid out on the shell the way
// the profile's mount says (board top toward the front, board right toward the rider's right).

/** Texture width in pixels; the height follows the window's aspect. */
const TEX_W = 512;
/** Where the stylized fairing carries the panel; its flat top is 60 mm wide, so the 83 mm
 * panel is drawn at 72 % there. */
const STYLIZED: { position: [number, number, number]; fit: number } = {
  position: [0.12, 0.4015, 0],
  fit: 0.72,
};

/** Shell-local directions (x forward, y up, z to the rider's right). */
const TOWARD: Record<DisplayLayout["board_up"], THREE.Vector3> = {
  front: new THREE.Vector3(1, 0, 0),
  rear: new THREE.Vector3(-1, 0, 0),
  right: new THREE.Vector3(0, 0, 1),
  left: new THREE.Vector3(0, 0, -1),
};

/** The plane's rotation: texture right along `board_right`, texture top along `board_up`. */
function mountRotation(layout: DisplayLayout): THREE.Euler {
  const right = TOWARD[layout.board_right];
  const up = TOWARD[layout.board_up];
  const normal = new THREE.Vector3().crossVectors(right, up);
  return new THREE.Euler().setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, up, normal));
}

/** The panel at `position` (its centre, shell frame), at `fit` of its real size. */
export function DisplayDecal({
  position = STYLIZED.position,
  fit = STYLIZED.fit,
}: {
  position?: [number, number, number];
  fit?: number;
}) {
  // The layout arrives with the run's bodies, after the scene mounts.
  useEngineVersion();
  const layout = engine.kind === "live" ? engine.display : null;
  if (!layout) return null;
  return <Decal layout={layout} position={position} fit={fit} />;
}

function Decal({
  layout,
  position,
  fit,
}: {
  layout: DisplayLayout;
  position: [number, number, number];
  fit: number;
}) {
  const material = useRef<THREE.MeshBasicMaterial>(null);
  /** The texture, the layout it was made for and the outputs it shows. */
  const painted = useRef<{
    layout: DisplayLayout;
    texture: THREE.CanvasTexture;
    outputs: Outputs | null | undefined;
  } | null>(null);
  const rotation = useMemo(() => mountRotation(layout), [layout]);

  useFrame(() => {
    const mat = material.current;
    if (!mat) return;
    let p = painted.current;
    if (p?.layout !== layout) {
      p?.texture.dispose();
      const canvas = document.createElement("canvas");
      canvas.width = TEX_W;
      canvas.height = Math.round(TEX_W / displayAspect(layout));
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = 4;
      mat.map = texture;
      mat.needsUpdate = true;
      p = painted.current = { layout, texture, outputs: undefined };
    }
    const o = engine.outputs;
    if (p.outputs === o) return;
    p.outputs = o;
    const canvas = p.texture.image as HTMLCanvasElement;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const state = o
      ? { grid: o.display_grid, on: o.display_on, brightness: o.display_brightness }
      : { grid: [], on: false, brightness: 0 };
    paintDisplay(ctx, layout, state, canvas.width, canvas.height);
    p.texture.needsUpdate = true;
  });
  useEffect(() => () => painted.current?.texture.dispose(), []);

  const [, , w, h] = layout.window;
  return (
    <mesh position={position} rotation={rotation}>
      <planeGeometry args={[(w / 1000) * fit, (h / 1000) * fit]} />
      <meshBasicMaterial ref={material} color={layout.panel} toneMapped={false} />
    </mesh>
  );
}

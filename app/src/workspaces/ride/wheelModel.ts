import { useEffect, useState } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { useLowPower } from "../../lib/power.ts";

// The Begode Extreme as scanned (public/models/begode-extreme/, from the model work): a "wheel"
// group (tire, hub, rim) that spins about z and a "body" group (shell, seat, tail light, handle,
// headlight, display window, shock, pedals) that follows the shell. Both use the stylized model's
// frame: x forward, y up, z to the rider's right, metres, origin on the axle. Without the files
// (or if they fail to load) the studio keeps its stylized model.

export interface WheelModel {
  wheel: THREE.Object3D;
  body: THREE.Object3D;
}

const url = (lod: number) =>
  `${import.meta.env.BASE_URL}models/begode-extreme/begode-extreme-lod${lod}.glb`;

const loads = new Map<number, Promise<WheelModel | null>>();

async function load(lod: number): Promise<WheelModel | null> {
  const res = await fetch(url(lod));
  if (!res.ok) return null;
  const bytes = await res.arrayBuffer();
  // A dev server answers a missing file with the app's HTML; a GLB starts with "glTF".
  if (bytes.byteLength < 4 || new TextDecoder().decode(bytes.slice(0, 4)) !== "glTF") return null;
  const loader = new GLTFLoader();
  loader.setMeshoptDecoder(MeshoptDecoder);
  const gltf = await loader.parseAsync(bytes, "");
  const wheel = gltf.scene.getObjectByName("wheel");
  const body = gltf.scene.getObjectByName("body");
  if (!wheel || !body) return null;
  for (const part of [wheel, body]) {
    part.removeFromParent();
    part.traverse((o) => {
      if (o instanceof THREE.Mesh) {
        o.castShadow = true;
        o.receiveShadow = true;
      }
    });
  }
  return { wheel, body };
}

/** The scanned model (level of detail 1, or 2 on a GPU-less renderer), or null. */
export function useWheelModel(): WheelModel | null {
  const lod = useLowPower() ? 2 : 1;
  const [model, setModel] = useState<{ lod: number; model: WheelModel | null } | null>(null);
  useEffect(() => {
    let alive = true;
    let pending = loads.get(lod);
    if (!pending) {
      pending = load(lod).catch(() => null);
      loads.set(lod, pending);
    }
    void pending.then((m) => alive && setModel({ lod, model: m }));
    return () => {
      alive = false;
    };
  }, [lod]);
  return model?.lod === lod ? model.model : null;
}

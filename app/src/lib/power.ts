// Low-power rendering: set when the 3D view finds no GPU (WebGL on a CPU rasterizer). Then the
// whole page is composited in software too, so live views redraw less often and rolling digits
// cross-fade instead of animating blur and position (blur filters are costly to raster).

import { useSyncExternalStore } from "react";

let lowPower = false;
const listeners = new Set<() => void>();

export function isLowPower(): boolean {
  return lowPower;
}

export function setLowPower(on: boolean): void {
  if (lowPower === on) return;
  lowPower = on;
  for (const fn of listeners) fn();
}

function subscribe(fn: () => void): () => void {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

export function useLowPower(): boolean {
  return useSyncExternalStore(subscribe, isLowPower, isLowPower);
}

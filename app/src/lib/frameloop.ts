// One requestAnimationFrame loop for the whole app. The engine advances first, then every
// subscriber (scopes, sparklines, HUD readouts) reads the rings it needs. react-three-fiber runs
// its own loop and reads the same `latest` object in useFrame. In low-power mode (no GPU) the
// loop runs its subscribers at most 30 times a second: every frame there is composited on the
// CPU, so fewer frames leave the main thread free for input.

import { isLowPower } from "./power.ts";

type FrameFn = (now: number, dt: number) => void;

const LOW_POWER_MS = 1000 / 30;

const early = new Set<FrameFn>();
const subscribers = new Set<FrameFn>();
let raf = 0;
let last = 0;

function tick(now: number) {
  raf = requestAnimationFrame(tick);
  // Skip this display frame when low power asks for fewer (2 ms slack for vsync jitter).
  if (last && isLowPower() && now - last < LOW_POWER_MS - 2) return;
  const dt = last ? (now - last) / 1000 : 0;
  last = now;
  for (const fn of early) fn(now, dt);
  for (const fn of subscribers) fn(now, dt);
}

function ensureRunning() {
  if (!raf && typeof requestAnimationFrame !== "undefined") {
    last = 0;
    raf = requestAnimationFrame(tick);
  }
}

/** Runs before regular subscribers each frame (the engine's `advance`). */
export function onFrameEarly(fn: FrameFn): () => void {
  early.add(fn);
  ensureRunning();
  return () => early.delete(fn);
}

export function onFrame(fn: FrameFn): () => void {
  subscribers.add(fn);
  ensureRunning();
  return () => subscribers.delete(fn);
}

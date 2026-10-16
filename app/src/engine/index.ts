import { useEffect, useEffectEvent, useState, useSyncExternalStore } from "react";
import { onFrame, onFrameEarly } from "../lib/frameloop.ts";
import { DemoEngine } from "../sim/synthetic.ts";
import { LiveEngine } from "./live.ts";
import type { EngineClient, Latest } from "./types.ts";

const params = new URLSearchParams(typeof location !== "undefined" ? location.search : "");
const inTauri = typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;

function create(): EngineClient {
  // The desktop app runs the real simulator; browser builds and tests use the demo.
  // `?engine=demo` forces the demo inside the app too.
  if (inTauri && params.get("engine") !== "demo") return new LiveEngine();
  const demo = new DemoEngine(1);
  const preroll = Number(params.get("preroll"));
  if (preroll > 0) demo.preroll(Math.min(preroll, 25));
  if (params.get("paused") === "1") demo.setPlaying(false);
  return demo;
}

/** The engine the studio talks to. */
export const engine: EngineClient = create();

onFrameEarly((now, dt) => {
  engine.advance(dt, now / 1000);
});

/** Re-renders on coarse engine changes only (play state, scenario, status, new events). */
export function useEngineVersion(): number {
  return useSyncExternalStore(engine.subscribe, engine.getVersion, engine.getVersion);
}

/** Re-renders on phone frames and board outputs (and every coarse change). */
export function useEngineData(): number {
  return useSyncExternalStore(engine.subscribeData, engine.getDataVersion, engine.getDataVersion);
}

const same = (a: number, b: number) => a === b || (Number.isNaN(a) && Number.isNaN(b));

/**
 * Reads a live value at a bounded rate (default 8 Hz, the rolling-digit limit) and only
 * re-renders when the rounded value changes. Unmodelled signals read as NaN.
 */
export function useLive(read: (l: Latest) => number, hz = 8, decimals = 1): number {
  const quantize = (v: number) => Math.round(v * 10 ** decimals) / 10 ** decimals;
  const [value, setValue] = useState(() => quantize(read(engine.latest)));
  const sample = useEffectEvent(() => {
    const q = quantize(read(engine.latest));
    setValue((prev) => (same(prev, q) ? prev : q));
  });
  useEffect(() => {
    let lastEmit = 0;
    return onFrame((now) => {
      if (now - lastEmit < 1000 / hz) return;
      lastEmit = now;
      sample();
    });
  }, [hz]);
  return value;
}

export * from "./types.ts";

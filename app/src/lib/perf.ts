// Frame-time measurement, two ways:
// - work: from the start of a frame (the rAF timestamp) to the end of its rendering update, when
//   a message posted from the frame runs. This is the main thread's cost per frame; above the
//   16.7 ms budget a frame is late.
// - interval: between animation frames, as the user sees them. At 60 Hz a frame that misses
//   vsync shows as a ~33 ms interval (dropped).

/** The 60 Hz frame budget, ms. */
export const BUDGET_MS = 1000 / 60;

export interface Spread {
  p50: number;
  p95: number;
  p99: number;
  max: number;
}

export interface FrameStats {
  frames: number;
  seconds: number;
  /** Main-thread work per frame, ms. */
  work: Spread;
  /** Frame intervals, ms. */
  interval: Spread;
  /** Share of frames whose work went over the budget. */
  over: number;
  /** Share of intervals longer than 1.5 budgets (a missed vsync). */
  dropped: number;
}

function spread(values: number[]): Spread {
  const sorted = [...values].sort((a, b) => a - b);
  const at = (q: number) => sorted[Math.min(sorted.length - 1, Math.floor(q * sorted.length))] ?? 0;
  return { p50: at(0.5), p95: at(0.95), p99: at(0.99), max: sorted.at(-1) ?? 0 };
}

export function frameStats(work: number[], intervals: number[], seconds: number): FrameStats {
  const share = (values: number[], limit: number) =>
    values.length ? values.filter((v) => v > limit).length / values.length : 0;
  return {
    frames: intervals.length,
    seconds,
    work: spread(work),
    interval: spread(intervals),
    over: share(work, BUDGET_MS),
    dropped: share(intervals, BUDGET_MS * 1.5),
  };
}

/** Records frame work and intervals for `seconds` and summarizes them. */
export function measureFrames(seconds: number): Promise<FrameStats> {
  return new Promise((resolve) => {
    const work: number[] = [];
    const intervals: number[] = [];
    // A message posted during a frame runs after that frame's rendering update.
    const channel = new MessageChannel();
    let frameStart = 0;
    channel.port1.onmessage = () => work.push(performance.now() - frameStart);
    let start = 0;
    let last = 0;
    const tick = (now: number) => {
      frameStart = now;
      channel.port2.postMessage(null);
      if (!start) start = now;
      else intervals.push(now - last);
      last = now;
      if (now - start < seconds * 1000) {
        requestAnimationFrame(tick);
        return;
      }
      // Let the last frame's message arrive.
      setTimeout(() => {
        channel.port1.close();
        resolve(frameStats(work, intervals, (now - start) / 1000));
      }, 50);
    };
    requestAnimationFrame(tick);
  });
}

export function formatStats(s: FrameStats): string {
  const ms = (v: number) => `${v.toFixed(1)} ms`;
  const pct = (v: number) => `${(v * 100).toFixed(1)} %`;
  return (
    `${s.frames} frames in ${s.seconds.toFixed(1)} s; ` +
    `work p50 ${ms(s.work.p50)}, p95 ${ms(s.work.p95)}, p99 ${ms(s.work.p99)}, max ${ms(s.work.max)}, ${pct(s.over)} over 16.7 ms; ` +
    `interval p50 ${ms(s.interval.p50)}, p95 ${ms(s.interval.p95)}, ${pct(s.dropped)} dropped`
  );
}

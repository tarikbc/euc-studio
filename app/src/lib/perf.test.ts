import { describe, expect, it } from "vitest";
import { formatStats, frameStats } from "./perf.ts";

describe("frameStats", () => {
  it("summarizes work and intervals", () => {
    const work = [...Array.from({ length: 95 }, () => 6), 10, 12, 17, 20, 30];
    const intervals = [...Array.from({ length: 98 }, () => 16.7), 33.4, 50];
    const s = frameStats(work, intervals, 1.7);
    expect(s.frames).toBe(100);
    expect(s.work.p50).toBe(6);
    expect(s.work.p95).toBe(10);
    expect(s.work.max).toBe(30);
    expect(s.over).toBeCloseTo(0.03);
    expect(s.interval.p50).toBeCloseTo(16.7);
    expect(s.dropped).toBeCloseTo(0.02);
    expect(formatStats(s)).toContain("work p50 6.0 ms, p95 10.0 ms");
  });

  it("handles no frames", () => {
    const s = frameStats([], [], 0);
    expect(s.work.p95).toBe(0);
    expect(s.dropped).toBe(0);
  });
});

import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { BatchError, readBatch } from "./batch.ts";
import { CH, CHANNELS } from "./telemetry.gen.ts";

// Written by the Rust encoder (cargo run -p euc-telemetry --example gen_ts): 3 samples, 2 bodies,
// sample i at t = i/120 with speed_kmh = i + 0.5, battery_v = 130, body 0 at x = i.
function fixture(): ArrayBuffer {
  const bytes = readFileSync(new URL("./fixtures/batch-v1.bin", import.meta.url));
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

describe("readBatch", () => {
  it("reads what the Rust encoder wrote", () => {
    const b = readBatch(fixture());
    expect([b.seq, b.dropped, b.n, b.channels, b.bodies]).toEqual([0, 0, 3, CHANNELS.length, 2]);
    expect(Array.from(b.t)).toEqual([0, 1 / 120, 2 / 120]);
    expect(Array.from(b.column(CH.speed_kmh))).toEqual([0.5, 1.5, 2.5]);
    expect(Array.from(b.column(CH.battery_v))).toEqual([130, 130, 130]);
    expect(Array.from(b.column(CH.pwm_pct)).every(Number.isNaN)).toBe(true);
    // sample 2, body 0: x y z qw qx qy qz
    expect(Array.from(b.poses.subarray(2 * 2 * 7, 2 * 2 * 7 + 7))).toEqual([
      2,
      0,
      Math.fround(0.28),
      1,
      0,
      0,
      0,
    ]);
  });

  it("rejects bad input", () => {
    expect(() => readBatch(new ArrayBuffer(4))).toThrow(BatchError);
    expect(() => readBatch(new ArrayBuffer(32))).toThrow(/not a telemetry batch/);
    const truncated = fixture().slice(0, 60);
    expect(() => readBatch(truncated)).toThrow(/truncated/);
  });
});

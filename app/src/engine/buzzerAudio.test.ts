import { describe, expect, it } from "vitest";
import { nextOffset, toneGain } from "./buzzerAudio.ts";

describe("toneGain", () => {
  it("is loudest at half duty and silent at the ends", () => {
    expect(toneGain(0.5)).toBeGreaterThan(toneGain(0.226));
    expect(toneGain(0)).toBe(0);
    expect(toneGain(1)).toBeCloseTo(0);
    // The stock firmware's default volume (compare 151 of 667).
    expect(toneGain(151 / 667)).toBeGreaterThan(0.1);
  });
});

describe("nextOffset", () => {
  it("maps simulated time a little ahead of the audio clock", () => {
    const offset = nextOffset(null, 10, 4);
    // Sim time 4 s plays 0.12 s from now.
    expect(4 + offset).toBeCloseTo(10.12);
  });

  it("follows small drift slowly and jumps on large ones", () => {
    const first = nextOffset(null, 10, 4);
    const drifted = nextOffset(first, 10.02, 4);
    expect(drifted).toBeGreaterThan(first);
    expect(drifted - first).toBeLessThan(0.001);
    const jumped = nextOffset(first, 11, 4);
    expect(jumped).toBeCloseTo(11.12 - 4);
  });
});

import { describe, expect, it } from "vitest";
import { CHANNEL_IDS, CHANNELS, DemoEngine, SCENARIOS, scenarioTarget } from "./synthetic.ts";

describe("DemoEngine", () => {
  it("is deterministic for a seed", () => {
    const a = new DemoEngine(7);
    const b = new DemoEngine(7);
    a.preroll(10);
    b.preroll(10);
    for (const id of CHANNEL_IDS) {
      expect(Array.from(a.channels[id].toArray())).toEqual(Array.from(b.channels[id].toArray()));
    }
  });

  it.each(SCENARIOS.map((s) => s.id))("keeps every channel inside its scale (%s)", (id) => {
    const e = new DemoEngine(3);
    e.setScenario(id);
    e.preroll(60);
    for (const ch of CHANNEL_IDS) {
      const { min, max } = CHANNELS[ch];
      for (const v of e.channels[ch].toArray()) {
        // NaN: a channel the demo does not model (suspension).
        if (Number.isNaN(v)) continue;
        expect(v).toBeGreaterThanOrEqual(min);
        expect(v).toBeLessThanOrEqual(max);
      }
    }
  });

  it("reaches cruise speed in the ride scenario", () => {
    const e = new DemoEngine();
    e.setScenario("ride-20");
    e.preroll(14);
    expect(e.latest.speed).toBeGreaterThan(19.5);
    expect(e.latest.speed).toBeLessThan(20.5);
  });

  it("emits boot events then scenario events in time order", () => {
    const e = new DemoEngine();
    e.preroll(30);
    const titles = e.events.map((x) => x.title);
    expect(titles.slice(0, 3)).toEqual([
      "main() reached",
      "IMU calibrated",
      "First telemetry frame",
    ]);
    expect(titles).toContain("Cruise at 20 km/h");
    const times = e.events.filter((x) => x.kind !== "firmware").map((x) => x.t);
    expect(times).toEqual([...times].sort((x, y) => x - y));
  });

  it("advances by wall time times the target real-time factor", () => {
    const e = new DemoEngine();
    e.setTargetRtf(2);
    let now = 0;
    for (let i = 0; i < 60; i++) e.advance(1 / 60, (now += 1 / 60));
    expect(e.latest.t).toBeCloseTo(2, 1);
    expect(e.measuredRtf()).toBeCloseTo(2, 1);
  });

  it("does not advance while paused and caps long gaps", () => {
    const e = new DemoEngine();
    e.setPlaying(false);
    expect(e.advance(1)).toBe(0);
    e.setPlaying(true);
    e.advance(10);
    expect(e.latest.t).toBeLessThanOrEqual(0.26);
  });

  it("resets history when the firmware changes", () => {
    const e = new DemoEngine();
    e.preroll(5);
    e.setFirmware("ours-clean");
    expect(e.time.length).toBe(0);
    expect(e.latest.t).toBe(0);
  });
});

describe("scenarioTarget", () => {
  it("eases between segment targets", () => {
    const s = SCENARIOS.find((x) => x.id === "ride-20");
    if (!s) throw new Error("missing scenario");
    expect(scenarioTarget(s, 0).speed).toBe(0);
    expect(scenarioTarget(s, 5.5).speed).toBeCloseTo(10, 5);
    expect(scenarioTarget(s, 12).speed).toBe(20);
    expect(scenarioTarget(s, 25).speed).toBe(0);
  });
});

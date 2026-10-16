import { readFileSync } from "node:fs";
import { beforeEach, describe, expect, it, vi } from "vitest";

// The Tauri IPC layer, faked: engine_connect hands back the two channels the client created.
const calls: { cmd: string; args?: Record<string, unknown> }[] = [];
let channels: {
  stream?: { onmessage: (b: ArrayBuffer) => void };
  messages?: { onmessage: (m: unknown) => void };
} = {};

vi.mock("@tauri-apps/api/core", () => ({
  Channel: class {
    onmessage: (m: unknown) => void;
    constructor(onmessage: (m: unknown) => void) {
      this.onmessage = onmessage;
    }
  },
  invoke: vi.fn(async (cmd: string, args?: Record<string, unknown>) => {
    calls.push({ cmd, ...(args ? { args } : {}) });
    if (cmd === "engine_connect") {
      channels = args as typeof channels;
      return {
        backend: "unicorn",
        scenarios: [{ id: "balance-60s", label: "Balance", detail: "" }],
        commands: [{ id: "horn", name: "horn", pattern: "b", safety: "comfort" }],
        perf_seconds: null,
      };
    }
    return undefined;
  }),
}));

const { LiveEngine } = await import("./live.ts");

function fixture(): ArrayBuffer {
  const bytes = readFileSync(new URL("./fixtures/batch-v1.bin", import.meta.url));
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

describe("LiveEngine", () => {
  beforeEach(() => {
    calls.length = 0;
    channels = {};
  });

  it("connects and takes the scenarios and phone commands from the backend", async () => {
    const e = new LiveEngine();
    await vi.waitFor(() => expect(e.phoneCommands).toHaveLength(1));
    expect(calls[0]?.cmd).toBe("engine_connect");
    expect(e.scenarios().map((s) => s.id)).toEqual(["balance-60s"]);
    expect(e.scenarioId).toBe("balance-60s");
    expect(e.status.backend).toBe("unicorn");
  });

  it("feeds batches into the rings and the latest view", async () => {
    const e = new LiveEngine();
    await vi.waitFor(() => expect(channels.stream).toBeDefined());
    channels.messages?.onmessage({
      type: "bodies",
      bodies: [
        { name: "wheel", role: "wheel", geoms: [] },
        { name: "shell", role: "shell", geoms: [] },
      ],
    });
    channels.stream?.onmessage(fixture());
    expect(e.time.length).toBe(3);
    expect(Array.from(e.channels.speed.toArray())).toEqual([0.5, 1.5, 2.5]);
    expect(e.latest.voltage).toBe(130);
    expect(Number.isNaN(e.latest.pwm)).toBe(true);
    expect(e.latest.poses?.[0]).toBe(2); // body 0 x of the last sample
    expect(e.bodies.map((b) => b.role)).toEqual(["wheel", "shell"]);
  });

  it("maps events, frames, status and resets", async () => {
    const e = new LiveEngine();
    await vi.waitFor(() => expect(channels.messages).toBeDefined());
    const m = channels.messages;
    if (!m) throw new Error("no message channel");
    m.onmessage({ type: "events", events: [{ t_s: 2, kind: "cutoff", text: "Roll cut-off" }] });
    m.onmessage({
      type: "frames",
      frames: [{ id: 1, t: 2, kind: "live", summary: "x", fields: [], hex: "" }],
    });
    m.onmessage({
      type: "status",
      state: "paused",
      message: "",
      build: "GW1825020",
      backend: "unicorn",
      scenario: "ride-20kmh-stop",
      t_s: 3,
      rtf: 1.02,
      target_rtf: 1,
      playing: false,
    });
    expect(e.events[0]).toMatchObject({ kind: "cutoff", title: "Roll cut-off", t: 2 });
    expect(e.frames).toHaveLength(1);
    expect([e.status.state, e.status.build, e.playing, e.scenarioId]).toEqual([
      "paused",
      "GW1825020",
      false,
      "ride-20kmh-stop",
    ]);
    m.onmessage({ type: "reset", scenario: "balance-60s" });
    expect([e.events.length, e.frames.length, e.time.length, e.scenarioId]).toEqual([
      0,
      0,
      0,
      "balance-60s",
    ]);
  });

  it("sends controls to the backend", async () => {
    const e = new LiveEngine();
    await vi.waitFor(() => expect(channels.stream).toBeDefined());
    e.setPlaying(false);
    e.setTargetRtf(0.5);
    e.setScenario("ride-20kmh-stop");
    await e.sendCommand("horn");
    expect(calls.slice(1).map((c) => c.cmd)).toEqual([
      "engine_play",
      "engine_set_rtf",
      "engine_restart",
      "engine_send",
    ]);
    expect(calls.at(-1)?.args).toEqual({ id: "horn", value: null });
  });
});

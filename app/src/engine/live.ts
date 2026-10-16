// The live engine: the real simulator in the Tauri backend (app/src-tauri/src/engine). Telemetry
// arrives as binary batches on one channel and status, bodies, events and phone frames as JSON
// on another. Samples go straight into the rings; React only hears about coarse changes.

import { Channel, invoke } from "@tauri-apps/api/core";
import { formatStats, measureFrames } from "../lib/perf.ts";
import { RingBuffer } from "../lib/ring.ts";
import { readBatch } from "./batch.ts";
import type { Tone } from "./buzzerAudio.ts";
import {
  type BodyInfo,
  CH,
  type Outputs,
  POSE_FLOATS,
  type TelemetryEvent,
} from "./telemetry.gen.ts";
import {
  CHANNEL_IDS,
  type ChannelId,
  type DisplayLayout,
  type LightsLayout,
  type EngineClient,
  type EngineStatus,
  type EventKind,
  type FirmwareId,
  type FirmwareInfo,
  HISTORY_SECONDS,
  type Latest,
  type PhoneCommand,
  type PhoneFrame,
  SAMPLE_RATE,
  type ScenarioInfo,
  type SimEvent,
} from "./types.ts";

interface BackendStatus {
  state: EngineStatus["state"];
  message: string;
  label: string | null;
  build: string | null;
  backend: string;
  scenario: string;
  t_s: number;
  rtf: number;
  target_rtf: number;
  playing: boolean;
}

type Message =
  | ({ type: "status" } & BackendStatus)
  | { type: "reset"; scenario: string }
  | { type: "bodies"; bodies: BodyInfo[] }
  | { type: "display"; layout: DisplayLayout | null }
  | { type: "lights"; layout: LightsLayout | null }
  | { type: "buzzer"; tones: Tone[] }
  | { type: "events"; events: TelemetryEvent[] }
  | { type: "frames"; frames: PhoneFrame[] }
  | { type: "outputs"; outputs: Outputs };

interface EngineInfo {
  backend: string;
  scenarios: ScenarioInfo[];
  commands: PhoneCommand[];
  perf_seconds: number | null;
}

/** Which telemetry channel feeds each UI lane. */
const LANE: Record<ChannelId, number> = {
  speed: CH.speed_kmh,
  pwm: CH.pwm_pct,
  current: CH.iq_a,
  voltage: CH.battery_v,
  pitch: CH.pitch_deg,
  temp: CH.motor_temp_c,
  battery: CH.soc_pct,
  isr: CH.isr_load_pct,
};

const MAX_EVENTS = 300;
const MAX_FRAMES = 400;

const FIRMWARES: FirmwareInfo[] = [
  { id: "stock", label: "Stock", detail: "Begode Extreme, official image" },
  {
    id: "ours-compat",
    label: "Ours-compat",
    detail: "Our C rewrite, stock behaviour",
    disabledReason: "Our firmware runs on the simulated controller from M5",
  },
  {
    id: "ours-clean",
    label: "Ours-clean",
    detail: "Our C rewrite, quirks fixed",
    disabledReason: "Our firmware runs on the simulated controller from M5",
  },
];

export class LiveEngine implements EngineClient {
  readonly kind = "live" as const;
  readonly time = new RingBuffer(SAMPLE_RATE * HISTORY_SECONDS);
  readonly channels = Object.fromEntries(
    CHANNEL_IDS.map((id) => [id, new RingBuffer(SAMPLE_RATE * HISTORY_SECONDS)]),
  ) as Record<ChannelId, RingBuffer>;
  readonly latest: Latest = {
    t: 0,
    speed: Number.NaN,
    pwm: Number.NaN,
    current: Number.NaN,
    voltage: Number.NaN,
    pitch: Number.NaN,
    roll: Number.NaN,
    temp: Number.NaN,
    battery: Number.NaN,
    isr: Number.NaN,
    grade: 0,
    distance: 0,
    wheelAngle: 0,
    poses: null,
  };
  bodies: BodyInfo[] = [];
  events: SimEvent[] = [];
  frames: PhoneFrame[] = [];
  phoneCommands: PhoneCommand[] = [];
  outputs: Outputs | null = null;
  display: DisplayLayout | null = null;
  lights: LightsLayout | null = null;
  beeps = 0;
  status: EngineStatus = { state: "starting", message: "Connecting to the engine", backend: "…" };
  playing = true;
  targetRtf = 1;
  readonly firmware: FirmwareId = "stock";
  scenarioId = "";

  private scenarioList: ScenarioInfo[] = [];
  private rtf = 0;
  private eventId = 0;
  private version = 0;
  private listeners = new Set<() => void>();
  private dataVersion = 0;
  private dataListeners = new Set<() => void>();
  private toneListeners = new Set<(tones: Tone[]) => void>();
  private buzzing = false;
  /** Batches received, samples dropped by the backend (for diagnostics). */
  readonly stats = { batches: 0, samples: 0, dropped: 0 };

  constructor() {
    void this.connect();
  }

  private async connect(): Promise<void> {
    const stream = new Channel<ArrayBuffer>((buffer) => this.onBatch(buffer));
    const messages = new Channel<Message>((message) => this.onMessage(message));
    try {
      const info = await invoke<EngineInfo>("engine_connect", { stream, messages });
      this.scenarioList = info.scenarios;
      this.phoneCommands = info.commands;
      this.status = { ...this.status, backend: info.backend };
      if (!this.scenarioId) this.scenarioId = info.scenarios[0]?.id ?? "";
      if (info.perf_seconds) void this.reportFrameTimes(info.perf_seconds);
    } catch (e) {
      this.status = { state: "error", message: String(e), backend: this.status.backend };
    }
    this.emit();
  }

  /**
   * Dev hook (EUC_STUDIO_PERF): after a warm-up, measure frame times and log them, again every
   * 10 s for a while. A hidden or covered window gets few frames; its lines show that in the
   * frame count.
   */
  private async reportFrameTimes(seconds: number): Promise<void> {
    const pause = (ms: number) => new Promise((r) => setTimeout(r, ms));
    await pause(8000);
    for (let run = 0; run < 30; run++) {
      const stats = await measureFrames(seconds);
      console.warn(
        `PERF ${formatStats(stats)}; engine ${this.status.state} at ${this.rtf.toFixed(2)}x, t ${this.latest.t.toFixed(1)} s, ${this.stats.batches} batches, ${this.stats.samples} samples, ${this.stats.dropped} dropped`,
      );
      await pause(10_000);
    }
  }

  /* ---- incoming ---- */

  private onBatch(buffer: ArrayBuffer): void {
    const b = readBatch(buffer);
    this.stats.batches++;
    this.stats.samples += b.n;
    this.stats.dropped += b.dropped;
    if (b.n === 0) return;
    const cols = CHANNEL_IDS.map((id) => [this.channels[id], b.column(LANE[id])] as const);
    for (let i = 0; i < b.n; i++) {
      this.time.push(b.t[i] as number);
      for (const [ring, col] of cols) ring.push(col[i] ?? Number.NaN);
    }
    const last = b.n - 1;
    const at = (id: number) => b.column(id)[last] ?? Number.NaN;
    const l = this.latest;
    l.t = b.t[last] as number;
    l.speed = at(CH.speed_kmh);
    l.pwm = at(CH.pwm_pct);
    l.current = at(CH.iq_a);
    l.voltage = at(CH.battery_v);
    l.pitch = at(CH.pitch_deg);
    l.roll = at(CH.roll_deg);
    l.temp = at(CH.motor_temp_c);
    l.battery = at(CH.soc_pct);
    l.isr = at(CH.isr_load_pct);
    l.distance = at(CH.distance_m);
    if (b.bodies > 0) {
      const size = b.bodies * POSE_FLOATS;
      const from = last * size;
      if (!l.poses || l.poses.length !== size) l.poses = new Float32Array(size);
      l.poses.set(b.poses.subarray(from, from + size));
    }
  }

  private onMessage(m: Message): void {
    switch (m.type) {
      case "status": {
        this.status = {
          state: m.state,
          message: m.message,
          backend: m.backend,
          ...(m.build ? { build: m.build } : {}),
          ...(m.label ? { label: m.label } : {}),
        };
        this.playing = m.playing;
        this.targetRtf = m.target_rtf;
        this.rtf = m.rtf;
        this.scenarioId = m.scenario;
        break;
      }
      case "reset":
        this.time.clear();
        for (const id of CHANNEL_IDS) this.channels[id].clear();
        this.events = [];
        this.frames = [];
        this.latest.poses = null;
        this.latest.t = 0;
        this.outputs = null;
        this.beeps = 0;
        this.buzzing = false;
        this.scenarioId = m.scenario;
        break;
      case "bodies":
        this.bodies = m.bodies;
        break;
      case "display":
        this.display = m.layout;
        break;
      case "lights":
        this.lights = m.layout;
        break;
      case "buzzer":
        for (const t of m.tones) {
          if (t.hz > 0 && !this.buzzing) this.beeps++;
          this.buzzing = t.hz > 0;
        }
        for (const fn of this.toneListeners) fn(m.tones);
        this.emitData();
        return;
      case "events":
        this.events = [
          ...this.events,
          ...m.events.map((e) => ({
            id: ++this.eventId,
            t: e.t_s,
            kind: e.kind as EventKind,
            title: e.text,
          })),
        ].slice(-MAX_EVENTS);
        break;
      case "frames":
        this.frames = [...this.frames, ...m.frames].slice(-MAX_FRAMES);
        this.emitData();
        return;
      case "outputs":
        this.outputs = m.outputs;
        this.emitData();
        return;
    }
    this.emit();
  }

  /* ---- EngineClient ---- */

  subscribe = (fn: () => void): (() => void) => {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  };

  getVersion = (): number => this.version;

  private emit(): void {
    this.version++;
    for (const fn of this.listeners) fn();
    this.emitData();
  }

  subscribeTones(fn: (tones: Tone[]) => void): () => void {
    this.toneListeners.add(fn);
    return () => this.toneListeners.delete(fn);
  }

  subscribeData = (fn: () => void): (() => void) => {
    this.dataListeners.add(fn);
    return () => this.dataListeners.delete(fn);
  };

  getDataVersion = (): number => this.dataVersion;

  private emitData(): void {
    this.dataVersion++;
    for (const fn of this.dataListeners) fn();
  }

  firmwares(): FirmwareInfo[] {
    // The stock entry names the build the engine loaded (the update server's label).
    const build = this.status.build;
    return build
      ? FIRMWARES.map((f) => (f.id === "stock" ? { ...f, detail: `${f.detail} ${build}` } : f))
      : FIRMWARES;
  }

  scenarios(): ScenarioInfo[] {
    return this.scenarioList;
  }

  private call(cmd: string, args?: Record<string, unknown>): void {
    invoke(cmd, args).catch((e: unknown) => {
      this.status = { ...this.status, state: "error", message: String(e) };
      this.emit();
    });
  }

  setPlaying(playing: boolean): void {
    this.playing = playing;
    this.call("engine_play", { playing });
    this.emit();
  }

  togglePlaying(): void {
    this.setPlaying(!this.playing);
  }

  setTargetRtf(rtf: number): void {
    this.targetRtf = rtf;
    this.call("engine_set_rtf", { rtf });
    this.emit();
  }

  setFirmware(): void {
    // Only the stock image runs on the simulated controller until M5.
  }

  setScenario(id: string): void {
    if (id === this.scenarioId) return;
    this.scenarioId = id;
    this.call("engine_restart", { scenario: id });
    this.emit();
  }

  restart(): void {
    this.call("engine_restart", { scenario: this.scenarioId || null });
  }

  step(): void {
    this.call("engine_step");
  }

  advance(): number {
    return 0;
  }

  measuredRtf(): number {
    return this.playing ? this.rtf : 0;
  }

  sendCommand(id: string, value?: number): Promise<string> {
    return invoke<string>("engine_send", { id, value: value ?? null });
  }
}

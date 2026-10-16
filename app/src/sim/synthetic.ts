// Demo engine: smooth, plausible EUC signals so the studio has something honest to show without
// the Rust engine (browser builds, tests, screenshots). It is deterministic for a given seed,
// advances in fixed 60 Hz sim steps, and writes into ring buffers that scopes read once per
// frame. React never sees individual samples: components read `latest` from a rAF loop or poll
// it at <= 8 Hz.

import type {
  ChannelId,
  EngineClient,
  EngineStatus,
  EventKind,
  FirmwareId,
  FirmwareInfo,
  Latest,
  PhoneCommand,
  PhoneFrame,
  ScenarioInfo,
  SimEvent,
} from "../engine/types.ts";
import { CHANNEL_IDS, HISTORY_SECONDS, SAMPLE_RATE } from "../engine/types.ts";
import type { BodyInfo } from "../engine/telemetry.gen.ts";
import { RingBuffer } from "../lib/ring.ts";

export {
  CHANNELS,
  CHANNEL_IDS,
  HISTORY_SECONDS,
  SAMPLE_RATE,
  type ChannelId,
  type EventKind,
  type FirmwareId,
  type Latest,
  type SimEvent,
} from "../engine/types.ts";

const DT = 1 / SAMPLE_RATE;

export const FIRMWARES: FirmwareInfo[] = [
  { id: "stock", label: "Stock", detail: "Begode Extreme, official image" },
  { id: "ours-compat", label: "Ours-compat", detail: "Our C rewrite, stock behaviour" },
  { id: "ours-clean", label: "Ours-clean", detail: "Our C rewrite, quirks fixed" },
];

export type ScenarioId = "idle" | "ride-20" | "hill" | "brake";

interface Segment {
  /** Segment end, seconds from the start of the loop. */
  until: number;
  /** Target speed at the end of the segment (km/h); eased from the previous target. */
  speed: number;
  /** Road grade at the end of the segment (rise/run). */
  grade?: number;
  event?: { kind: EventKind; title: string };
}

export interface Scenario {
  id: ScenarioId;
  label: string;
  detail: string;
  segments: Segment[];
}

export const SCENARIOS: Scenario[] = [
  {
    id: "ride-20",
    label: "Ride to 20 km/h and stop",
    detail: "Accelerate, cruise, brake, rest. Loops every 26 s.",
    segments: [
      { until: 2, speed: 0 },
      { until: 9, speed: 20, event: { kind: "scenario", title: "Rider leans forward" } },
      { until: 16, speed: 20, event: { kind: "scenario", title: "Cruise at 20 km/h" } },
      { until: 21, speed: 0, event: { kind: "scenario", title: "Rider brakes" } },
      { until: 26, speed: 0, event: { kind: "scenario", title: "Stopped" } },
    ],
  },
  {
    id: "idle",
    label: "Idle balance",
    detail: "Rider stands still. The firmware holds the pedals level.",
    segments: [{ until: 20, speed: 0 }],
  },
  {
    id: "hill",
    label: "Hill climb, 12 %",
    detail: "Steady climb at 16 km/h, high current, battery sag.",
    segments: [
      { until: 3, speed: 0 },
      { until: 9, speed: 16, event: { kind: "scenario", title: "Rider leans forward" } },
      {
        until: 11,
        speed: 16,
        grade: 0.12,
        event: { kind: "scenario", title: "Grade rises to 12 %" },
      },
      { until: 24, speed: 16, grade: 0.12 },
      { until: 27, speed: 0, grade: 0, event: { kind: "scenario", title: "Rider brakes" } },
      { until: 30, speed: 0, event: { kind: "scenario", title: "Stopped" } },
    ],
  },
  {
    id: "brake",
    label: "Hard brake from 30 km/h",
    detail: "Accelerate to 30 km/h, then stop as hard as the rider can.",
    segments: [
      { until: 2, speed: 0 },
      { until: 10, speed: 30, event: { kind: "scenario", title: "Rider leans forward" } },
      { until: 12, speed: 30 },
      { until: 14.5, speed: 0, event: { kind: "scenario", title: "Hard brake" } },
      { until: 22, speed: 0, event: { kind: "scenario", title: "Stopped" } },
    ],
  },
];

/** Deterministic PRNG (mulberry32). */
export function prng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const smoothstep = (x: number) => x * x * (3 - 2 * x);
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/** Target speed and grade at loop time `lt` for a scenario, eased between segments. */
export function scenarioTarget(s: Scenario, lt: number): { speed: number; grade: number } {
  let prevEnd = 0;
  let prevSpeed = 0;
  let prevGrade = 0;
  for (const seg of s.segments) {
    const grade = seg.grade ?? prevGrade;
    if (lt < seg.until) {
      const k = smoothstep(clamp((lt - prevEnd) / (seg.until - prevEnd), 0, 1));
      return {
        speed: prevSpeed + (seg.speed - prevSpeed) * k,
        grade: prevGrade + (grade - prevGrade) * k,
      };
    }
    prevEnd = seg.until;
    prevSpeed = seg.speed;
    prevGrade = grade;
  }
  return { speed: prevSpeed, grade: prevGrade };
}

const WHEEL_RADIUS = 0.28; // m, 22 inch tire
const CELLS = 32;
const PACK_AH = 17.8;
const PACK_R = 0.18; // ohm
const ABS_SPEED_MAX = 68; // km/h at 100 % PWM on a full pack, roughly

export class DemoEngine implements EngineClient {
  readonly kind = "demo" as const;
  readonly bodies: BodyInfo[] = [];
  readonly frames: PhoneFrame[] = [];
  readonly phoneCommands: PhoneCommand[] = [];
  readonly outputs = null;
  readonly display = null;
  readonly lights = null;
  readonly terrain = null;
  readonly beeps = 0;
  subscribeTones(): () => void {
    return () => {};
  }
  get status(): EngineStatus {
    return {
      state: this.playing ? "running" : "paused",
      message: "Synthetic signals",
      backend: "demo",
    };
  }
  readonly time = new RingBuffer(SAMPLE_RATE * HISTORY_SECONDS);
  readonly channels = Object.fromEntries(
    CHANNEL_IDS.map((id) => [id, new RingBuffer(SAMPLE_RATE * HISTORY_SECONDS)]),
  ) as Record<ChannelId, RingBuffer>;
  readonly latest: Latest = {
    t: 0,
    speed: 0,
    pwm: 0,
    current: 0,
    voltage: 0,
    pitch: 0,
    roll: 0,
    temp: 0,
    battery: 0,
    isr: 0,
    grade: 0,
    distance: 0,
    wheelAngle: 0,
    poses: null,
  };
  events: SimEvent[] = [];

  playing = true;
  /** Requested real-time factor. */
  targetRtf = 1;
  firmware: FirmwareId = "stock";
  scenario: Scenario = SCENARIOS[0] as Scenario;

  private seed: number;
  private rand: () => number;
  private debt = 0;
  private eventId = 0;
  private soc = 0.82;
  private accel = 0;
  private noise = { pitch: 0, current: 0, isr: 0, roll: 0 };
  private rtfWindow: { wall: number; sim: number }[] = [];
  private listeners = new Set<() => void>();
  private version = 0;
  private lastLoop = -1;

  constructor(seed = 1) {
    this.seed = seed;
    this.rand = prng(seed);
    this.reset();
  }

  /* ---- subscription (coarse state only: play/pause, firmware, scenario, events) ---- */

  subscribe = (fn: () => void): (() => void) => {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  };

  getVersion = (): number => this.version;

  /** The demo has no phone frames or board outputs. */
  subscribeData = (): (() => void) => () => {};

  getDataVersion = (): number => 0;

  private emit(): void {
    this.version++;
    for (const fn of this.listeners) fn();
  }

  /* ---- controls ---- */

  setPlaying(playing: boolean): void {
    if (this.playing === playing) return;
    this.playing = playing;
    this.rtfWindow = [];
    this.emit();
  }

  togglePlaying(): void {
    this.setPlaying(!this.playing);
  }

  setTargetRtf(rtf: number): void {
    this.targetRtf = clamp(rtf, 0.25, 8);
    this.rtfWindow = [];
    this.emit();
  }

  setFirmware(id: FirmwareId): void {
    if (this.firmware === id) return;
    this.firmware = id;
    this.reset();
    this.emit();
  }

  get scenarioId(): string {
    return this.scenario.id;
  }

  firmwares(): FirmwareInfo[] {
    return FIRMWARES;
  }

  scenarios(): ScenarioInfo[] {
    return SCENARIOS.map(({ id, label, detail }) => ({ id, label, detail }));
  }

  /** One fixed 1/60 s step (used while paused). */
  step(): void {
    this.stepOnce();
    this.emit();
  }

  sendCommand(): Promise<string> {
    return Promise.reject(new Error("The demo engine has no firmware to talk to"));
  }

  setScenario(id: string): void {
    const s = SCENARIOS.find((x) => x.id === id);
    if (!s || s === this.scenario) return;
    this.scenario = s;
    this.reset();
    this.emit();
  }

  restart(): void {
    this.reset();
    this.emit();
  }

  /* ---- time ---- */

  /**
   * Advances by `wallSeconds` of wall time scaled by the target real-time factor. Long gaps
   * (a hidden window) are capped so the sim never jumps more than a quarter second at once.
   */
  advance(wallSeconds: number, wallNow = performance.now() / 1000): number {
    if (!this.playing) return 0;
    this.debt += Math.min(wallSeconds, 0.25) * this.targetRtf;
    let steps = 0;
    while (this.debt >= DT) {
      this.stepOnce();
      this.debt -= DT;
      steps++;
    }
    this.rtfWindow.push({ wall: wallNow, sim: this.latest.t });
    while (
      this.rtfWindow.length > 2 &&
      wallNow - (this.rtfWindow[0] as { wall: number }).wall > 1
    ) {
      this.rtfWindow.shift();
    }
    return steps;
  }

  /** Measured sim seconds per wall second over the last second (0 when paused). */
  measuredRtf(): number {
    if (!this.playing || this.rtfWindow.length < 2) return this.playing ? this.targetRtf : 0;
    const a = this.rtfWindow[0] as { wall: number; sim: number };
    const b = this.rtfWindow[this.rtfWindow.length - 1] as { wall: number; sim: number };
    const wall = b.wall - a.wall;
    return wall > 0.2 ? (b.sim - a.sim) / wall : this.targetRtf;
  }

  /** Runs `seconds` of sim time immediately (fills scopes for screenshots and tests). */
  preroll(seconds: number): void {
    const n = Math.round(seconds * SAMPLE_RATE);
    for (let i = 0; i < n; i++) this.stepOnce();
    this.emit();
  }

  private reset(): void {
    this.rand = prng(this.seed);
    this.time.clear();
    for (const id of CHANNEL_IDS) this.channels[id].clear();
    this.events = [];
    this.debt = 0;
    this.soc = 0.82;
    this.accel = 0;
    this.noise = { pitch: 0, current: 0, isr: 0, roll: 0 };
    this.rtfWindow = [];
    this.lastLoop = -1;
    Object.assign(this.latest, {
      t: 0,
      speed: 0,
      pwm: 2,
      current: 0,
      voltage: this.ocv(),
      pitch: 0,
      roll: 0,
      temp: 34,
      battery: this.soc * 100,
      isr: 31,
      grade: 0,
      distance: 0,
      wheelAngle: 0,
    });
    this.pushEvent(0.000069, "firmware", "main() reached");
    this.pushEvent(0.5, "firmware", "IMU calibrated");
    this.pushEvent(2.0, "firmware", "First telemetry frame");
  }

  private ocv(): number {
    return CELLS * (3.3 + 0.9 * this.soc);
  }

  private pushEvent(t: number, kind: EventKind, title: string): void {
    this.events = [...this.events.slice(-199), { id: ++this.eventId, t, kind, title }];
    this.emit();
  }

  /** Low-pass filtered noise in roughly [-1, 1]. */
  private walk(key: keyof DemoEngine["noise"], k: number): number {
    const v = this.noise[key] + (this.rand() * 2 - 1 - this.noise[key]) * k;
    this.noise[key] = v;
    return v;
  }

  private stepOnce(): void {
    const L = this.latest;
    const t = L.t + DT;
    const s = this.scenario;
    const period = (s.segments[s.segments.length - 1] as Segment).until;
    const loop = Math.floor(t / period);
    const lt = t - loop * period;

    // Scenario events fire when the loop time crosses a segment start.
    let segStart = 0;
    for (const seg of s.segments) {
      if (seg.event && lt >= segStart && lt - DT < segStart && (loop > 0 || segStart > 0)) {
        this.pushEvent(t, seg.event.kind, seg.event.title);
      }
      segStart = seg.until;
    }
    if (loop !== this.lastLoop && loop > 0 && this.lastLoop >= 0 && s.id === "ride-20") {
      this.pushEvent(t, "command", "Command b (beep) from phone");
    }
    this.lastLoop = loop;

    const target = scenarioTarget(s, lt);
    const clean = this.firmware === "ours-clean";

    // Speed follows the rider's intent with a short lag; acceleration drives lean and current.
    const prevSpeed = L.speed;
    const speed = Math.max(0, prevSpeed + (target.speed - prevSpeed) * (1 - Math.exp(-DT / 0.35)));
    const accelRaw = (speed - prevSpeed) / 3.6 / DT; // m/s^2
    this.accel += (accelRaw - this.accel) * 0.2;

    const wobble = clean ? 0.12 : 0.22;
    const pitch =
      1.7 * this.accel +
      Math.atan(target.grade) * (180 / Math.PI) * 0.25 +
      wobble * Math.sin(2 * Math.PI * 1.3 * t) +
      0.07 * Math.sin(2 * Math.PI * 3.7 * t + 1.1) +
      0.12 * this.walk("pitch", 0.05);
    const roll = 0.35 * Math.sin(2 * Math.PI * 0.21 * t) + 0.2 * this.walk("roll", 0.02);

    // Braking current is regenerative and smaller than the drive current for the same rate.
    const drag = 0.012 * speed * speed;
    const inertial = this.accel >= 0 ? 9.5 * this.accel : 6.5 * this.accel;
    const current = inertial + drag + target.grade * 150 + 1.1 + 0.6 * this.walk("current", 0.15);

    this.soc = Math.max(0, this.soc - (Math.max(current, 0) * DT) / 3600 / PACK_AH);
    const voltage = this.ocv() - current * PACK_R;
    const pwm = clamp(
      (speed / ABS_SPEED_MAX) * 100 * (134.4 / voltage) + Math.abs(current) * 0.55 + 2,
      0,
      100,
    );
    const temp = L.temp + (current * current * 0.0009 - (L.temp - 34) * 0.01) * DT;
    const isr = 31 + pwm * 0.09 + 1.2 * this.walk("isr", 0.1);

    const distance = L.distance + (speed / 3.6) * DT;

    Object.assign(L, {
      t,
      speed,
      pwm,
      current,
      voltage,
      pitch,
      roll,
      temp,
      battery: this.soc * 100,
      isr,
      grade: target.grade,
      distance,
      wheelAngle: distance / WHEEL_RADIUS,
    });

    if (speed > 28 && prevSpeed <= 28) this.pushEvent(t, "alarm", "Speed alarm 1 at 28 km/h");
    if (pwm > 45 && this.channels.pwm.last() <= 45) this.pushEvent(t, "alarm", "PWM above 45 %");

    this.time.push(t);
    this.channels.speed.push(speed);
    this.channels.pwm.push(pwm);
    this.channels.current.push(current);
    this.channels.voltage.push(voltage);
    this.channels.pitch.push(pitch);
    this.channels.temp.push(temp);
    this.channels.battery.push(this.soc * 100);
    this.channels.isr.push(isr);
  }
}

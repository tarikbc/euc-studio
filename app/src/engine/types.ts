// What the studio needs from an engine, whichever one runs: the synthetic demo (browser builds,
// tests) or the real simulator behind Tauri. Components read rings and `latest` on the frame
// loop and re-render only on coarse changes (`subscribe`/`getVersion`).

import type { RingBuffer } from "../lib/ring.ts";
import type { BodyInfo, Outputs } from "./telemetry.gen.ts";

/** The lanes and readouts the UI shows. Each engine maps its own signals onto them. */
export type ChannelId =
  "speed" | "pwm" | "current" | "voltage" | "pitch" | "temp" | "battery" | "isr";

export interface ChannelInfo {
  label: string;
  unit: string;
  decimals: number;
  /** Fixed scale for scopes and gauges. */
  min: number;
  max: number;
}

export const CHANNELS: Record<ChannelId, ChannelInfo> = {
  speed: { label: "Speed", unit: "km/h", decimals: 1, min: 0, max: 40 },
  pwm: { label: "PWM", unit: "%", decimals: 0, min: 0, max: 100 },
  current: { label: "Phase current", unit: "A", decimals: 1, min: -40, max: 40 },
  voltage: { label: "Battery", unit: "V", decimals: 1, min: 100, max: 135 },
  pitch: { label: "Pitch", unit: "°", decimals: 1, min: -10, max: 10 },
  temp: { label: "Motor temp", unit: "°C", decimals: 1, min: 20, max: 90 },
  battery: { label: "Charge", unit: "%", decimals: 0, min: 0, max: 100 },
  isr: { label: "ISR load", unit: "%", decimals: 0, min: 0, max: 100 },
};

export const CHANNEL_IDS = Object.keys(CHANNELS) as ChannelId[];

/** Samples per second the rings are sized for. */
export const SAMPLE_RATE = 60;
export const HISTORY_SECONDS = 30;

export type FirmwareId = "stock" | "ours-compat" | "ours-clean";

export interface FirmwareInfo {
  id: FirmwareId;
  label: string;
  detail: string;
  /** Set when this engine cannot run it; the selector shows why. */
  disabledReason?: string;
}

export interface ScenarioInfo {
  id: string;
  label: string;
  detail: string;
}

export type EventKind =
  "firmware" | "scenario" | "command" | "alarm" | "tiltback" | "cutoff" | "fault";

export interface SimEvent {
  id: number;
  /** Sim time in seconds. */
  t: number;
  kind: EventKind;
  title: string;
}

export interface Latest {
  t: number;
  speed: number;
  pwm: number;
  current: number;
  voltage: number;
  pitch: number;
  roll: number;
  temp: number;
  battery: number;
  isr: number;
  grade: number;
  /** Distance travelled (m) and wheel angle (rad), for the demo's kinematic 3D view. */
  distance: number;
  wheelAngle: number;
  /**
   * Live engine only: the pose of every body in `EngineClient.bodies` order, 7 floats each
   * (x y z qw qx qy qz, physics world frame: z up, metres). `null` for the demo.
   */
  poses: Float32Array | null;
}

/** One decoded line in the virtual phone: a telemetry frame or a text reply. */
export interface PhoneFrame {
  id: number;
  t: number;
  /** "live", "bms", "cells", "status", "power", "other" or "text". */
  kind: string;
  /** Short summary for the list ("31.4 km/h, 128.2 V"). */
  summary: string;
  /** Decoded fields, in wire order. */
  fields: [string, string][];
  /** The bytes, hex. */
  hex: string;
}

/** A command the phone can send. Never-send commands are not listed at all. */
export interface PhoneCommand {
  id: string;
  name: string;
  pattern: string;
  safety: "read" | "comfort" | "ride" | "debug";
  /** Inclusive range for commands that take a number. */
  param?: { min: number; max: number; default: number };
  /** Fixed choices for commands with modes (light: off, on, strobe). */
  choices?: { value: number; label: string }[];
  /** The wheel ignores it above this speed (m/s). */
  maxSpeed?: number;
}

export type EngineState = "starting" | "running" | "paused" | "halted" | "error" | "unavailable";

export interface EngineStatus {
  state: EngineState;
  /** What the engine is doing or why it stopped, for the status bar. */
  message: string;
  /** Firmware build label (`GW1825020`) when known. */
  build?: string;
  /** CPU backend (`unicorn`, `demo`). */
  backend: string;
  /** What runs ("Stock firmware, no physics yet"). */
  label?: string;
}

/** `[grid, bit]`: a TM1640 grid byte and the bit in it that lights a piece. */
export type DisplaySource = [number, number];

/** One LED piece of the display replica, in board millimetres (y down). */
export type DisplayPiece =
  | { kind: "polygon"; points: [number, number][]; sources: DisplaySource[] }
  | { kind: "circle"; cx: number; cy: number; r: number; sources: DisplaySource[] }
  | {
      kind: "ring";
      cx: number;
      cy: number;
      rx: number;
      ry: number;
      width: number;
      sources: DisplaySource[];
    }
  | { kind: "path"; points: [number, number][]; width: number; sources: DisplaySource[] };

/** The wheel's display board as the profile describes it (`display.toml`). */
export interface DisplayLayout {
  board: string;
  /** The LED window, mm: `[x, y, w, h]`. */
  window: [number, number, number, number];
  lit: string;
  unlit: string;
  panel: string;
  board_up: "front" | "rear" | "left" | "right";
  board_right: "front" | "rear" | "left" | "right";
  pieces: DisplayPiece[];
}

export interface EngineClient {
  readonly kind: "demo" | "live";
  readonly time: RingBuffer;
  readonly channels: Record<ChannelId, RingBuffer>;
  readonly latest: Latest;
  /** Bodies whose poses `latest.poses` carries (empty for the demo). */
  readonly bodies: BodyInfo[];
  readonly events: SimEvent[];
  /** Virtual phone: decoded frames and replies, oldest first. Empty when unsupported. */
  readonly frames: PhoneFrame[];
  readonly phoneCommands: PhoneCommand[];
  /** What the wheel shows and sounds (display, LEDs, buzzer); null when not modelled. */
  readonly outputs: Outputs | null;
  /** The display replica's layout; null when the wheel has none (or for the demo). */
  readonly display: DisplayLayout | null;
  readonly status: EngineStatus;
  readonly playing: boolean;
  readonly targetRtf: number;
  readonly firmware: FirmwareId;
  readonly scenarioId: string;

  firmwares(): FirmwareInfo[];
  scenarios(): ScenarioInfo[];

  /** Coarse changes: status, play state, scenario, events, bodies. */
  subscribe(fn: () => void): () => void;
  getVersion(): number;
  /** Frequent changes: phone frames and board outputs (only their views listen). */
  subscribeData(fn: () => void): () => void;
  getDataVersion(): number;

  setPlaying(playing: boolean): void;
  togglePlaying(): void;
  setTargetRtf(rtf: number): void;
  setFirmware(id: FirmwareId): void;
  setScenario(id: string): void;
  restart(): void;
  /** Advances by one fixed step while paused (live engine; the demo steps 1/60 s). */
  step(): void;
  /** Called once per frame with wall time; the demo advances here, the live engine ignores it. */
  advance(wallSeconds: number, wallNow?: number): number;
  /** Measured sim seconds per wall second. */
  measuredRtf(): number;
  /** Sends a phone command (validated again by the engine). Resolves with what was sent. */
  sendCommand(id: string, value?: number): Promise<string>;
}

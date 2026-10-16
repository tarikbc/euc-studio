// The wheel's buzzer through the speakers. The engine streams every change of the PWM the
// firmware drives (frequency, duty, simulated time of the change); this schedules them with Web
// Audio against simulated time. A square wave, as the pin drives it, through a band-pass around
// a small piezo's resonance; starts and stops ramp over about 2.5 ms so they do not click.

/** A buzzer change: from `t_s` (simulated) on, `hz` at `duty` of the period high; 0 Hz is silent. */
export interface Tone {
  t_s: number;
  hz: number;
  duty: number;
}

/** Audio scheduled ahead of the newest simulated time, s: covers the stream's batching. */
const LEAD_S = 0.12;
/** A jump in the time mapping larger than this resynchronises it instead of following it. */
const RESYNC_S = 0.08;
/** Time constant of the gain ramps (about 2.5 ms to 95 %). */
const RAMP_TAU_S = 0.0008;
/** A typical 12 mm piezo disc resonates near 2.7 kHz. */
const PIEZO_HZ = 2700;
const PIEZO_Q = 2.2;
const MAX_GAIN = 0.22;

/** Loudness from the duty: the fundamental of a pulse wave scales with sin(pi * duty). */
export const toneGain = (duty: number) =>
  MAX_GAIN * Math.sin(Math.PI * Math.min(1, Math.max(0, duty)));

/** Audio-clock time of simulated time `t_s`, given the mapping `offset` (audio = sim + offset). */
export function nextOffset(offset: number | null, audioNow: number, simNow: number): number {
  const target = audioNow + LEAD_S - simNow;
  if (offset === null || Math.abs(target - offset) > RESYNC_S) return target;
  // Follow slow drift between the two clocks without jitter from batch arrival.
  return offset + (target - offset) * 0.02;
}

export class BuzzerAudio {
  private ctx: AudioContext | null = null;
  private osc: OscillatorNode | null = null;
  private gain: GainNode | null = null;
  private offset: number | null = null;

  get running(): boolean {
    return this.ctx !== null;
  }

  /** Opens the audio output. Call from a user gesture where possible (autoplay rules). */
  start(): void {
    if (this.ctx) {
      void this.ctx.resume();
      return;
    }
    const ctx = new AudioContext();
    const osc = ctx.createOscillator();
    osc.type = "square";
    osc.frequency.value = PIEZO_HZ;
    const gain = ctx.createGain();
    gain.gain.value = 0;
    const piezo = ctx.createBiquadFilter();
    piezo.type = "bandpass";
    piezo.frequency.value = PIEZO_HZ;
    piezo.Q.value = PIEZO_Q;
    osc.connect(gain).connect(piezo).connect(ctx.destination);
    osc.start();
    Object.assign(this, { ctx, osc, gain, offset: null });
  }

  /** Resumes a context the browser started suspended (no gesture yet). */
  resume(): void {
    void this.ctx?.resume();
  }

  stop(): void {
    void this.ctx?.close();
    Object.assign(this, { ctx: null, osc: null, gain: null, offset: null });
  }

  /** Schedules tone changes; `simNow` is the newest simulated time the client has. */
  play(tones: Tone[], simNow: number): void {
    const { ctx, osc, gain } = this;
    if (!ctx || !osc || !gain || tones.length === 0) return;
    this.offset = nextOffset(this.offset, ctx.currentTime, simNow);
    for (const t of tones) {
      const at = Math.max(ctx.currentTime, t.t_s + this.offset);
      if (t.hz > 0) osc.frequency.setValueAtTime(t.hz, at);
      gain.gain.setTargetAtTime(t.hz > 0 ? toneGain(t.duty) : 0, at, RAMP_TAU_S);
    }
  }

  /** Silences now (pause, a run far from real time) and forgets the time mapping. */
  silence(): void {
    const { ctx, gain } = this;
    if (ctx && gain) {
      gain.gain.cancelScheduledValues(ctx.currentTime);
      gain.gain.setTargetAtTime(0, ctx.currentTime, RAMP_TAU_S);
    }
    this.offset = null;
  }
}

import { useEffect, useState } from "react";
import { BuzzerAudio } from "../../engine/buzzerAudio.ts";
import { engine } from "../../engine/index.ts";
import { useUi } from "../../state/ui.ts";

/** Sound follows the simulation only near real time; outside this range it pauses. */
const MIN_RTF = 0.8;
const MAX_RTF = 1.25;

/** Why the sound is held: a short label for the header and the full reason for its tooltip. */
export interface SoundHold {
  label: string;
  detail: string;
}

/** Why the sound is held, or null while it plays. */
function pauseReason(): SoundHold | null {
  if (!engine.playing) return { label: "Sound paused", detail: "The simulation is paused." };
  const rtf = engine.measuredRtf();
  if (rtf > 0 && (rtf < MIN_RTF || rtf > MAX_RTF)) {
    return {
      label: "Sound paused",
      detail: `The simulation runs at ×${rtf.toFixed(rtf < 1 ? 2 : 1)}. Sound plays only between ×${MIN_RTF} and ×${MAX_RTF}, so every beep keeps its real timing.`,
    };
  }
  return null;
}

/**
 * Plays the wheel's buzzer while the user has turned the sound on. Returns why the sound is
 * paused (a run far from real time, or paused), or null.
 */
export function useBuzzerSound(): SoundHold | null {
  const on = useUi((s) => s.buzzerSound);
  const [reason, setReason] = useState<SoundHold | null>(null);

  useEffect(() => {
    if (!on) return;
    const audio = new BuzzerAudio();
    audio.start();
    // A context opened without a gesture (the setting was on at launch) starts suspended.
    const resume = () => audio.resume();
    window.addEventListener("pointerdown", resume);
    window.addEventListener("keydown", resume);
    let held: SoundHold | null = null;
    const check = () => {
      const next = pauseReason();
      if (next?.detail === held?.detail) return;
      held = next;
      if (next) audio.silence();
      setReason(next);
    };
    const offTones = engine.subscribeTones((tones) => {
      check();
      if (!held) audio.play(tones, engine.latest.t);
    });
    const offStatus = engine.subscribe(check);
    check();
    return () => {
      offTones();
      offStatus();
      window.removeEventListener("pointerdown", resume);
      window.removeEventListener("keydown", resume);
      audio.stop();
      setReason(null);
    };
  }, [on]);

  return on ? reason : null;
}

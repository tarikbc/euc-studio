import { engine, useEngineData } from "../../engine/index.ts";
import { cx } from "../../lib/cx.ts";
import { useUi } from "../../state/ui.ts";
import { Bell, BellOff } from "../../ui/icons.tsx";
import { Tooltip } from "../../ui/Tooltip.tsx";
import { DisplayReplica } from "./DisplayReplica.tsx";
import { SevenSegment } from "./SevenSegment.tsx";
import { ledCss, litCount } from "./ledColor.ts";
import { TailLight } from "./TailLight.tsx";
import { useBuzzerSound } from "./useBuzzerSound.ts";

// What the wheel shows and sounds, mirrored from the emulated hardware: the display (a replica
// of the wheel's own board when its profile describes it, else the driver's raw digits), the
// tail light (its rings when the profile places them, else the raw strip) and the buzzer.

export function WheelOutputs({ className }: { className?: string }) {
  useEngineData();
  const sound = useUi((s) => s.buzzerSound);
  const setSound = useUi((s) => s.setBuzzerSound);
  const soundHeld = useBuzzerSound();
  const o = engine.outputs;
  if (!o) return null;
  const lights = engine.lights;
  const leds = o.leds as [number, number, number][];
  const total = lights ? lights.leds.length : leds.length;
  const lit = lights ? litCount(lights, leds) : leds.filter(([r, g, b]) => r + g + b > 0).length;
  return (
    <div
      className={cx(
        "flex flex-col gap-8 rounded-12 border border-hairline bg-card/75 px-16 py-8 backdrop-blur-[12px]",
        className,
      )}
      data-testid="wheel-outputs"
    >
      <div className="flex items-center justify-between gap-16">
        <span className="label shrink-0">Display</span>
        <span className="flex min-w-0 flex-1 items-center gap-8">
          {/* The display sets the card's width; contain keeps this note from widening it. */}
          {soundHeld ? (
            <Tooltip content={soundHeld.detail} side="top">
              <span
                className="min-w-0 flex-1 truncate text-right text-11 text-fg-3 [contain:inline-size]"
                data-testid="sound-held"
              >
                {soundHeld.label}
              </span>
            </Tooltip>
          ) : (
            <span className="flex-1" />
          )}
          <Tooltip content="Play the wheel's buzzer through your speakers" side="top">
            <button
              type="button"
              aria-pressed={sound}
              aria-label="Buzzer sound"
              onClick={() => setSound(!sound)}
              className={cx(
                "relative grid size-[24px] shrink-0 place-items-center rounded-pill [&_svg]:size-[13px]",
                "transition-colors duration-(--dur-hover) ease-hover hover:bg-raised",
                o.buzzer_hz ? "bg-caution-tint text-caution" : sound ? "text-fg" : "text-fg-3",
              )}
            >
              {o.buzzer_hz > 0 && (
                <span className="absolute inset-0 animate-[live-pulse_0.5s_ease-in-out_infinite] rounded-pill border border-caution" />
              )}
              {/* A new key per beep replays the wiggle, sound on or off. */}
              <span key={engine.beeps} className={cx("grid", engine.beeps > 0 && "bell-ring")}>
                {sound ? <Bell /> : <BellOff />}
              </span>
            </button>
          </Tooltip>
        </span>
      </div>
      {engine.display ? (
        <DisplayReplica layout={engine.display} outputs={o} height={104} />
      ) : (
        <SevenSegment
          grid={o.display_grid}
          on={o.display_on}
          brightness={o.display_brightness}
          size={26}
        />
      )}
      <div className="flex items-center justify-between gap-8">
        <span className="label">Tail light</span>
        <span className="num text-11 text-fg-3">
          {lit}/{total}
        </span>
      </div>
      {lights ? (
        <TailLight layout={lights} outputs={o} width={156} />
      ) : (
        <div className="flex flex-wrap gap-2" aria-label={`${lit} of ${total} LEDs lit`}>
          {leds.map((c, i) => {
            const on = c[0] + c[1] + c[2] > 0;
            return (
              <span
                key={i}
                className="size-[6px] rounded-pill"
                style={{
                  background: on ? ledCss(c) : "var(--color-track)",
                  boxShadow: on ? `0 0 6px ${ledCss(c)}` : undefined,
                }}
              />
            );
          })}
          {leds.length === 0 && <span className="text-11 text-fg-3">Strip not driven yet</span>}
        </div>
      )}
    </div>
  );
}

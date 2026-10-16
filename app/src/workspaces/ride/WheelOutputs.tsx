import { engine, useEngineData } from "../../engine/index.ts";
import { cx } from "../../lib/cx.ts";
import { Bell } from "../../ui/icons.tsx";
import { Tooltip } from "../../ui/Tooltip.tsx";
import { DisplayReplica } from "./DisplayReplica.tsx";
import { SevenSegment } from "./SevenSegment.tsx";

// What the wheel shows and sounds, mirrored from the emulated hardware: the display (a replica
// of the wheel's own board when its profile describes it, else the driver's raw digits), the LED
// strip and the buzzer.

const css = ([r, g, b]: [number, number, number]) => `rgb(${r} ${g} ${b})`;

export function WheelOutputs({ className }: { className?: string }) {
  useEngineData();
  const o = engine.outputs;
  if (!o) return null;
  const lit = o.leds.filter(([r, g, b]) => r + g + b > 0).length;
  return (
    <div
      className={cx(
        "flex flex-col gap-8 rounded-12 border border-hairline bg-card/75 px-16 py-8 backdrop-blur-[12px]",
        className,
      )}
      data-testid="wheel-outputs"
    >
      <div className="flex items-center justify-between gap-16">
        <span className="label">Display</span>
        <Tooltip
          content={
            o.buzzer_hz ? `Buzzer on, ${(o.buzzer_hz / 1000).toFixed(2)} kHz` : "Buzzer silent"
          }
          side="top"
        >
          <span
            className={cx(
              "relative grid size-[20px] place-items-center rounded-pill [&_svg]:size-[12px]",
              o.buzzer_hz ? "bg-caution-tint text-caution" : "text-fg-3",
            )}
          >
            {o.buzzer_hz > 0 && (
              <span className="absolute inset-0 animate-[live-pulse_0.5s_ease-in-out_infinite] rounded-pill border border-caution" />
            )}
            <Bell />
          </span>
        </Tooltip>
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
        <span className="label">Lights</span>
        <span className="num text-11 text-fg-3">
          {lit}/{o.leds.length}
        </span>
      </div>
      <div className="flex flex-wrap gap-2" aria-label={`${lit} of ${o.leds.length} LEDs lit`}>
        {o.leds.map((c, i) => {
          const on = c[0] + c[1] + c[2] > 0;
          return (
            <span
              key={i}
              className="size-[6px] rounded-pill"
              style={{
                background: on ? css(c) : "var(--color-track)",
                boxShadow: on ? `0 0 6px ${css(c)}` : undefined,
              }}
            />
          );
        })}
        {o.leds.length === 0 && <span className="text-11 text-fg-3">Strip not driven yet</span>}
      </div>
    </div>
  );
}

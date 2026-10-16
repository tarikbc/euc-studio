import { AnimatePresence, motion } from "motion/react";
import { useState } from "react";
import { smooth } from "../../design/motion.ts";
import { series } from "../../design/tokens.ts";
import { cx } from "../../lib/cx.ts";
import { engine, useEngineVersion, useLive } from "../../engine/index.ts";
import { type CameraMode, useUi } from "../../state/ui.ts";
import { Gauge } from "../../ui/Gauge.tsx";
import { Crosshair, Orbit, RotateCcw } from "../../ui/icons.tsx";
import { Button } from "../../ui/Button.tsx";
import { Pill } from "../../ui/Pill.tsx";
import { RollingNumber, RollingText } from "../../ui/RollingNumber.tsx";
import { SegmentedControl } from "../../ui/SegmentedControl.tsx";
import { Tooltip } from "../../ui/Tooltip.tsx";
import { Wait } from "../../ui/Wait.tsx";
import { PitchProtractor } from "./PitchProtractor.tsx";
import { LOW_FPS, type Quality } from "./quality.ts";
import { Scene } from "./Scene.tsx";
import { WheelOutputs } from "./WheelOutputs.tsx";

/** While the live engine starts or after it failed: what it is doing, or why it stopped. */
function EngineOverlay() {
  useEngineVersion();
  const { state, message } = engine.status;
  const show =
    engine.kind === "live" && (state === "starting" || state === "error" || state === "halted");
  return (
    <AnimatePresence>
      {show && (
        <motion.div
          className="absolute inset-0 z-20 grid place-items-center bg-bg/70 backdrop-blur-[2px]"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={smooth}
        >
          {state === "starting" ? (
            <Wait
              step={message || "Starting the simulator"}
              detail="The stock firmware boots in the simulated controller"
              onRetry={() => engine.restart()}
              onCancel={() => engine.setPlaying(false)}
            />
          ) : (
            <div className="flex w-[360px] flex-col gap-8 rounded-12 border border-hairline-strong bg-card p-16">
              <span className="text-13 font-semibold text-fg">
                {state === "halted" ? "The firmware stopped" : "The engine could not start"}
              </span>
              <span className="text-12 break-words text-fg-2">{message}</span>
              <div className="flex gap-8 pt-8">
                <Button size="sm" icon={<RotateCcw />} onClick={() => engine.restart()}>
                  Restart the run
                </Button>
              </div>
            </div>
          )}
        </motion.div>
      )}
    </AnimatePresence>
  );
}

function HudCard({ children, className }: { children: React.ReactNode; className?: string }) {
  return (
    <div
      className={cx(
        "rounded-12 border border-hairline bg-card/75 backdrop-blur-[12px] backdrop-saturate-150",
        className,
      )}
    >
      {children}
    </div>
  );
}

/** Below this (km/h) the wheel counts as rolling backwards; smaller values are balance noise. */
const REVERSE_KMH = -0.5;

function Speed() {
  const speed = useLive((l) => l.speed, 8, 1);
  const reverse = speed < REVERSE_KMH;
  const digits = "text-44 font-medium tracking-[-0.03em]";
  const unit = <span className="text-13 font-semibold text-fg-2">km/h</span>;
  // Like the wheel's own display, the number is the magnitude; the direction is a tag, so a
  // sign never shifts the card. An invisible "88.8 km/h" holds the width up to 99.9 km/h.
  return (
    <div className="flex flex-col px-16 pt-8 pb-16">
      <span className="flex items-center gap-8">
        <span className="label">Speed</span>
        {reverse && (
          <span className="label text-caution" data-testid="speed-reverse">
            Reverse
          </span>
        )}
      </span>
      <span className="mt-4 grid">
        <span aria-hidden="true" className="invisible flex items-baseline gap-4 [grid-area:1/1]">
          <RollingText text="88.8" className={digits} />
          {unit}
        </span>
        <span className="flex items-baseline gap-4 [grid-area:1/1]">
          <RollingNumber value={Math.abs(speed)} decimals={1} className={cx(digits, "text-fg")} />
          {unit}
        </span>
      </span>
    </div>
  );
}

function Pwm() {
  const pwm = useLive((l) => l.pwm, 8, 0);
  return (
    <div className="grid place-items-center px-8">
      <Gauge
        value={pwm}
        label="PWM"
        unit="%"
        size={84}
        color={series.pwm}
        zones={[
          { from: 70, tone: "caution" },
          { from: 85, tone: "danger" },
        ]}
      />
    </div>
  );
}

function Battery() {
  const pct = useLive((l) => l.battery, 2, 0);
  const volts = useLive((l) => l.voltage, 4, 1);
  const tone = pct < 15 ? "bg-danger" : pct < 30 ? "bg-caution" : "bg-ok";
  return (
    <div className="flex min-w-[112px] flex-col px-16 pt-8 pb-16">
      <span className="label">Battery</span>
      <span className="mt-4 flex items-baseline gap-2 text-20 font-medium text-fg">
        <RollingNumber value={pct} />
        <span className="text-13 font-semibold text-fg-2">%</span>
      </span>
      <span className="relative mt-8 h-[4px] w-full overflow-hidden rounded-pill bg-track">
        <span
          className={cx(
            "absolute inset-y-0 left-0 rounded-pill transition-[width] duration-(--dur-data) ease-data",
            tone,
          )}
          style={{ width: `${pct}%` }}
        />
      </span>
      <span className="mt-8 text-12 text-fg-2">
        <RollingNumber value={volts} decimals={1} /> V, 32S
      </span>
    </div>
  );
}

function Pitch() {
  const pitch = useLive((l) => l.pitch, 8, 1);
  return (
    <HudCard className="px-16 pt-8 pb-8">
      <div className="flex items-center justify-between">
        <span className="label">Pitch</span>
        <span className="text-11 text-fg-2">nose up +</span>
      </div>
      <PitchProtractor pitch={pitch} className="mt-4" />
    </HudCard>
  );
}

/** `?scene=off` (tests that do not need the 3D view): no WebGL at all. */
const SCENE_OFF =
  typeof location !== "undefined" && new URLSearchParams(location.search).get("scene") === "off";

export function Viewport({ active }: { active: boolean }) {
  const camera = useUi((s) => s.camera);
  const setCamera = useUi((s) => s.setCamera);
  const [scene, setScene] = useState<{ quality: Quality; renderer: string } | null>(null);
  const [drawn, setDrawn] = useState(false);
  const ready = SCENE_OFF || scene !== null;

  return (
    <div
      className="relative h-full overflow-hidden rounded-12 border border-hairline bg-bg"
      data-testid="viewport"
      data-scene={SCENE_OFF ? "off" : drawn ? "ready" : "building"}
    >
      <div className="absolute inset-0">
        {SCENE_OFF ? (
          <p className="grid h-full place-items-center text-12 text-fg-3">3D view off</p>
        ) : (
          <Scene active={active} onReady={setScene} onFirstFrame={() => setDrawn(true)} />
        )}
      </div>

      <EngineOverlay />

      <AnimatePresence>
        {!ready && (
          <motion.div
            className="absolute inset-0 grid place-items-center bg-bg"
            exit={{ opacity: 0 }}
            transition={smooth}
          >
            <Wait step="Building the scene" detail="Lighting, floor and wheel model" />
          </motion.div>
        )}
      </AnimatePresence>

      <div className="pointer-events-none absolute inset-0 flex flex-col justify-between p-16">
        <div className="flex items-start justify-between">
          <div className="pointer-events-auto">
            <SegmentedControl<CameraMode>
              aria-label="Camera"
              size="sm"
              value={camera}
              onValueChange={setCamera}
              options={[
                {
                  value: "follow",
                  label: "Follow",
                  icon: <Crosshair />,
                  tooltip: "Camera follows the wheel",
                  shortcut: "c",
                },
                {
                  value: "orbit",
                  label: "Orbit",
                  icon: <Orbit />,
                  tooltip: "Drag to orbit, scroll to zoom",
                  shortcut: "c",
                },
              ]}
            />
          </div>
          <div className="pointer-events-auto">
            <Pitch />
          </div>
        </div>
        <div className="flex items-end justify-between">
          <HudCard className="pointer-events-auto flex items-stretch divide-x divide-hairline">
            <Speed />
            <Pwm />
            <Battery />
          </HudCard>
          <div className="flex flex-col items-end gap-8">
            {scene?.quality === "low" && (
              <Tooltip
                side="top"
                content={`No GPU acceleration (${scene.renderer.split(" (")[0] || "software WebGL"}). The 3D view draws at ${LOW_FPS} fps without shadows; telemetry is unaffected.`}
              >
                <span className="pointer-events-auto">
                  <Pill tone="neutral" size="md">
                    Reduced 3D quality
                  </Pill>
                </span>
              </Tooltip>
            )}
            <WheelOutputs className="pointer-events-auto max-w-[360px]" />
          </div>
        </div>
      </div>
    </div>
  );
}

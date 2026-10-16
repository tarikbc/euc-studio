import { AnimatePresence, motion } from "motion/react";
import { DropdownMenu } from "radix-ui";
import { snappy } from "../design/motion.ts";
import { cx } from "../lib/cx.ts";
import { type FirmwareId, engine, useEngineVersion, useLive } from "../engine/index.ts";
import { useUi } from "../state/ui.ts";
import { IconButton } from "../ui/Button.tsx";
import { Check, LogoMark, Pause, Play, RotateCcw, Search, StepForward } from "../ui/icons.tsx";
import { Shortcut } from "../ui/Kbd.tsx";
import { RollingNumber } from "../ui/RollingNumber.tsx";
import { formatClock } from "../ui/rolling.ts";
import { SegmentedControl } from "../ui/SegmentedControl.tsx";
import { Select } from "../ui/Select.tsx";
import { Tooltip } from "../ui/Tooltip.tsx";
import { RTF_STEPS } from "./commands.tsx";

function Readout({
  label,
  children,
  dim,
}: {
  label: string;
  children: React.ReactNode;
  dim?: boolean;
}) {
  return (
    <span className="flex flex-col items-start leading-none">
      <span
        className={cx(
          "text-15 font-medium transition-colors duration-(--dur-hover)",
          dim ? "text-fg-3" : "text-fg",
        )}
      >
        {children}
      </span>
      <span className="label mt-4 text-[10px] leading-[12px] tracking-[0.8px]">{label}</span>
    </span>
  );
}

function RtfReadout() {
  useEngineVersion();
  const rtf = useLive(() => engine.measuredRtf(), 4, 2);
  return (
    <DropdownMenu.Root>
      <Tooltip content="Real-time factor: sim seconds per wall second" shortcut="]">
        <DropdownMenu.Trigger
          className="group pressable -mx-8 flex h-[44px] items-center rounded-8 px-8 live:hover:bg-card data-[state=open]:bg-card"
          aria-label="Real-time factor"
        >
          <Readout label="Real time" dim={!engine.playing}>
            <RollingNumber value={rtf} decimals={2} />
            <span className="text-fg-2">×</span>
          </Readout>
        </DropdownMenu.Trigger>
      </Tooltip>
      <DropdownMenu.Portal>
        <DropdownMenu.Content
          sideOffset={8}
          align="start"
          className="pop z-50 min-w-[180px] rounded-12 border border-hairline-strong bg-raised p-4 shadow-overlay"
        >
          <DropdownMenu.Label className="label px-8 pt-4 pb-4">Target speed</DropdownMenu.Label>
          <DropdownMenu.RadioGroup
            value={String(engine.targetRtf)}
            onValueChange={(v) => engine.setTargetRtf(Number(v))}
          >
            {RTF_STEPS.map((r) => (
              <DropdownMenu.RadioItem
                key={r}
                value={String(r)}
                className="relative flex h-[32px] items-center rounded-8 pr-32 pl-8 text-13 font-medium text-fg outline-none data-[highlighted]:bg-track"
              >
                <span className="num">{r}×</span>
                <span className="ml-8 text-12 text-fg-2-safe">
                  {r === 1 ? "real time" : r < 1 ? "slow motion" : "fast forward"}
                </span>
                <DropdownMenu.ItemIndicator className="absolute right-8 text-accent-text [&_svg]:size-[16px]">
                  <Check />
                </DropdownMenu.ItemIndicator>
              </DropdownMenu.RadioItem>
            ))}
          </DropdownMenu.RadioGroup>
          <div className="mt-4 flex items-center justify-between border-t border-hairline px-8 pt-8 pb-4 text-12 text-fg-2-safe">
            Slower / faster
            <span className="flex gap-4">
              <Shortcut keys="[" />
              <Shortcut keys="]" />
            </span>
          </div>
        </DropdownMenu.Content>
      </DropdownMenu.Portal>
    </DropdownMenu.Root>
  );
}

function SimClock() {
  const t = useLive((l) => l.t, 8, 1);
  return (
    <Readout label="Sim time" dim={!engine.playing}>
      <RollingNumber value={t} format={formatClock} />
    </Readout>
  );
}

/** One frame forward while paused (1/60 s of simulated time). */
function StepButton() {
  useEngineVersion();
  return (
    <IconButton
      label="Step one frame"
      shortcut="."
      icon={<StepForward />}
      onClick={() => engine.step()}
      {...(engine.playing ? { disabledReason: "Pause the run to step it" } : {})}
    />
  );
}

function PlayButton() {
  useEngineVersion();
  const playing = engine.playing;
  return (
    <Tooltip content={playing ? "Pause" : "Run"} shortcut="space">
      <button
        type="button"
        aria-label={playing ? "Pause simulation" : "Run simulation"}
        onClick={() => engine.togglePlaying()}
        className="pressable relative grid size-(--control-md) place-items-center rounded-pill bg-accent text-on-accent live:hover:bg-accent-hover"
      >
        <AnimatePresence initial={false} mode="popLayout">
          <motion.span
            key={playing ? "pause" : "play"}
            initial={{ scale: 0.5, opacity: 0, rotate: -45 }}
            animate={{ scale: 1, opacity: 1, rotate: 0 }}
            exit={{ scale: 0.5, opacity: 0, rotate: 45 }}
            transition={snappy}
            className="grid place-items-center [&_svg]:size-[16px] [&_svg]:fill-current"
          >
            {playing ? (
              <Pause strokeWidth={0} />
            ) : (
              <Play className="translate-x-[1px]" strokeWidth={0} />
            )}
          </motion.span>
        </AnimatePresence>
      </button>
    </Tooltip>
  );
}

export function RunBar() {
  useEngineVersion();
  const setPaletteOpen = useUi((s) => s.setPaletteOpen);
  return (
    <header
      data-tauri-drag-region
      className="runbar flex h-(--runbar) shrink-0 items-center gap-16 border-b border-hairline bg-bg pr-16 pl-16"
    >
      <div data-tauri-drag-region className="flex items-center gap-8 pr-8">
        <LogoMark className="size-[20px] text-accent" />
        <span
          data-tauri-drag-region
          className="text-13 font-bold tracking-[-0.01em] max-[1279px]:hidden"
        >
          EUC Studio
        </span>
      </div>

      <span className="h-[24px] w-px bg-hairline" />

      <SegmentedControl<FirmwareId>
        aria-label="Firmware"
        value={engine.firmware}
        onValueChange={(v) => engine.setFirmware(v)}
        options={engine.firmwares().map((f) => ({
          value: f.id,
          label: f.label,
          tooltip: f.detail,
          ...(f.disabledReason ? { disabledReason: f.disabledReason } : {}),
        }))}
      />

      <Select<string>
        aria-label="Scenario"
        value={engine.scenarioId}
        onValueChange={(v) => engine.setScenario(v)}
        options={engine.scenarios().map((s) => ({ value: s.id, label: s.label, detail: s.detail }))}
        className="w-[236px] max-[1279px]:w-[196px]"
        contentClassName="w-[300px]"
      />

      <span className="h-[24px] w-px bg-hairline" />

      <div className="flex items-center gap-8">
        <IconButton
          label="Restart from t = 0"
          shortcut="r"
          icon={<RotateCcw />}
          onClick={() => engine.restart()}
        />
        <PlayButton />
        <StepButton />
      </div>

      <div className="flex items-center gap-24 pl-8">
        <RtfReadout />
        <SimClock />
      </div>

      <div data-tauri-drag-region className="h-full flex-1" />

      <button
        type="button"
        onClick={() => setPaletteOpen(true)}
        className="pressable flex h-(--control-md) items-center gap-8 rounded-8 border border-hairline bg-card pr-4 pl-8 text-13 text-fg-2 live:hover:bg-raised live:hover:text-fg-2-safe max-[1279px]:pr-4"
        aria-label="Search commands"
      >
        <Search className="size-[16px] text-fg-3" />
        <span className="w-[120px] text-left max-[1279px]:hidden">Search commands</span>
        <Shortcut keys="mod+k" />
      </button>
    </header>
  );
}

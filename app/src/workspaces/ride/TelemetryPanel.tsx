import { useState } from "react";
import { type RightTab, useUi } from "../../state/ui.ts";
import { Tabs } from "../../ui/Tabs.tsx";
import { PhoneBody } from "./PhonePanel.tsx";
import { series } from "../../design/tokens.ts";
import {
  CHANNELS,
  type ChannelId,
  SAMPLE_RATE,
  engine,
  useEngineVersion,
  useLive,
} from "../../engine/index.ts";
import { Activity, Smartphone } from "../../ui/icons.tsx";
import { Panel } from "../../ui/Panel.tsx";
import { RollingNumber } from "../../ui/RollingNumber.tsx";
import { SegmentedControl } from "../../ui/SegmentedControl.tsx";
import { Sparkline } from "../../ui/Sparkline.tsx";
import { Scope } from "./Scope.tsx";

const LANES: { id: ChannelId; color: string }[] = [
  { id: "pwm", color: series.pwm },
  { id: "modulation", color: series.modulation },
  { id: "speed", color: series.speed },
  { id: "current", color: series.current },
];

const CHANNEL_ROWS: { id: ChannelId; color: string }[] = [
  { id: "voltage", color: series.voltage },
  { id: "pitch", color: series.pitch },
  { id: "temp", color: series.temp },
  { id: "suspension", color: series.suspension },
];

function LiveValue({ id }: { id: ChannelId }) {
  const info = CHANNELS[id];
  const v = useLive((l) => l[id as keyof typeof l] as number, 8, info.decimals);
  return (
    <span className="flex items-baseline gap-4">
      <RollingNumber value={v} decimals={info.decimals} className="text-13 font-medium text-fg" />
      <span className="w-[28px] text-11 font-semibold text-fg-2">{info.unit}</span>
    </span>
  );
}

/** Shown over a lane whose signal the running engine does not model yet. */
function NotModelled({ id }: { id: ChannelId }) {
  const missing = useLive((l) => (Number.isNaN(l[id as keyof typeof l] as number) ? 1 : 0), 2, 0);
  if (!missing) return null;
  return (
    <div className="pointer-events-none absolute inset-0 z-10 grid place-items-center">
      <span className="rounded-pill border border-hairline bg-raised px-8 text-11 font-semibold text-fg-2-safe">
        Not modelled in this run
      </span>
    </div>
  );
}

function Lane({
  id,
  color,
  seconds,
  last,
}: {
  id: ChannelId;
  color: string;
  seconds: number;
  last: boolean;
}) {
  const info = CHANNELS[id];
  return (
    <div className="flex min-h-0 flex-1 flex-col">
      <div className="flex h-[24px] shrink-0 items-center gap-8 px-16">
        <span className="size-[8px] rounded-pill" style={{ background: color }} />
        <span className="text-12 font-semibold text-fg">{info.label}</span>
        <span className="ml-auto">
          <LiveValue id={id} />
        </span>
      </div>
      <div className="relative min-h-0 flex-1 pr-8 pl-4">
        <NotModelled id={id} />
        <Scope
          time={engine.time}
          values={engine.channels[id]}
          color={color}
          min={info.min}
          max={info.max}
          seconds={seconds}
          showTime={last}
          unit={info.unit}
        />
      </div>
    </div>
  );
}

type WindowSecs = "5" | "10" | "30";

function TelemetryBody({ seconds }: { seconds: number }) {
  return (
    <>
      <div className="flex min-h-[300px] flex-1 flex-col gap-8 py-8">
        {LANES.map((l, i) => (
          <Lane
            key={l.id}
            id={l.id}
            color={l.color}
            seconds={seconds}
            last={i === LANES.length - 1}
          />
        ))}
      </div>
      <div className="shrink-0 border-t border-hairline px-16 py-8">
        <div className="flex h-[24px] items-center justify-between">
          <span className="label">Channels</span>
          <span className="num text-11 text-fg-2">{SAMPLE_RATE} Hz</span>
        </div>
        <ul className="flex flex-col">
          {CHANNEL_ROWS.map((c) => (
            <li
              key={c.id}
              className="grid h-[40px] grid-cols-[96px_1fr_auto] items-center gap-16 border-b border-hairline last:border-b-0"
            >
              <span className="flex items-center gap-8 text-12 font-medium text-fg-2">
                <span className="size-[6px] rounded-pill" style={{ background: c.color }} />
                {CHANNELS[c.id].label}
              </span>
              <Sparkline
                ring={engine.channels[c.id]}
                window={seconds * SAMPLE_RATE}
                color={c.color}
                height={24}
              />
              <LiveValue id={c.id} />
            </li>
          ))}
        </ul>
      </div>
    </>
  );
}

/** The Ride workspace's right dock: telemetry scopes, or the virtual phone. */
export function RightDock() {
  useEngineVersion();
  const tab = useUi((s) => s.rightTab);
  const setTab = useUi((s) => s.setRightTab);
  const [win, setWin] = useState<WindowSecs>("10");
  return (
    <Panel
      title={
        <Tabs<RightTab>
          aria-label="Right dock"
          value={tab}
          onValueChange={setTab}
          tabs={[
            { value: "telemetry", label: "Telemetry" },
            {
              value: "phone",
              label: "Phone",
              ...(engine.frames.length > 0 ? { count: engine.frames.length } : {}),
            },
          ]}
          className="-mb-px h-(--dock-header) [&>div]:h-full"
        />
      }
      icon={tab === "telemetry" ? <Activity /> : <Smartphone />}
      className="min-w-[300px]"
      bodyClassName="flex flex-col overflow-hidden"
      actions={
        tab === "telemetry" ? (
          <SegmentedControl<WindowSecs>
            aria-label="Time window"
            size="sm"
            value={win}
            onValueChange={setWin}
            options={[
              { value: "5", label: "5 s" },
              { value: "10", label: "10 s" },
              { value: "30", label: "30 s" },
            ]}
          />
        ) : null
      }
    >
      {tab === "telemetry" ? <TelemetryBody seconds={Number(win)} /> : <PhoneBody />}
    </Panel>
  );
}

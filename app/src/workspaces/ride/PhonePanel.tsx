import { useState } from "react";
import { type PhoneCommand, type PhoneFrame, engine, useEngineData } from "../../engine/index.ts";
import { cx } from "../../lib/cx.ts";
import { Button } from "../../ui/Button.tsx";
import { ChevronDown, Send, Smartphone } from "../../ui/icons.tsx";
import { Kbd } from "../../ui/Kbd.tsx";
import { Pill, type PillTone } from "../../ui/Pill.tsx";
import { formatClock } from "../../ui/rolling.ts";
import { toast } from "../../ui/toast-store.ts";
import { Tooltip } from "../../ui/Tooltip.tsx";

// The virtual phone: what a phone app would see over BLE (the firmware's own frames and text
// replies, decoded) and the commands it may send. Commands that cut the motor, erase flash,
// enter the bootloader or power the wheel off are not offered: the engine has no way to build
// them.

const KIND_TONE: Record<string, PillTone> = {
  live: "accent",
  status: "info",
  power: "info",
  bms: "neutral",
  cells: "neutral",
  text: "ok",
  other: "neutral",
};

const SAFETY: Record<PhoneCommand["safety"], { label: string; note: string }> = {
  read: { label: "Read", note: "Asks the wheel; the answer arrives as text below." },
  comfort: { label: "Comfort", note: "Safe while riding." },
  ride: {
    label: "Ride settings",
    note: "Changes how the wheel rides. Real apps send these at standstill.",
  },
  debug: { label: "Debug", note: "The wheel prints a sensor dump instead of telemetry." },
};

function latestLive(frames: PhoneFrame[]): PhoneFrame | undefined {
  for (let i = frames.length - 1; i >= 0; i--) if (frames[i]?.kind === "live") return frames[i];
  return undefined;
}

function Dashboard() {
  const live = latestLive(engine.frames);
  const field = (name: string) => live?.fields.find(([k]) => k === name)?.[1] ?? "–";
  const tiles: [string, string][] = [
    ["Speed", field("Speed")],
    ["Voltage", field("Voltage")],
    ["Phase current", field("Phase current")],
    ["Board temp", field("Board temp")],
  ];
  return (
    <div className="grid grid-cols-2 gap-8 px-16 pt-16">
      {tiles.map(([label, value]) => (
        <div key={label} className="rounded-8 border border-hairline bg-bg px-8 py-8">
          <div className="text-11 font-semibold text-fg-2">{label}</div>
          <div className="num mt-2 truncate text-15 font-medium text-fg">{value}</div>
        </div>
      ))}
      <p className="col-span-2 text-11 text-fg-3">
        {live ? `From frame 00 at ${formatClock(live.t)}` : "Waiting for the first telemetry frame"}
      </p>
    </div>
  );
}

function CommandRow({ command }: { command: PhoneCommand }) {
  const initial = command.param?.default ?? command.choices?.[0]?.value;
  const [value, setValue] = useState<number | undefined>(initial);
  const [sending, setSending] = useState(false);
  const send = async () => {
    setSending(true);
    try {
      const bytes = await engine.sendCommand(command.id, value);
      toast({ tone: "success", title: `Sent ${command.name}`, description: `Bytes: ${bytes}` });
    } catch (e) {
      toast({ tone: "danger", title: `${command.name} was not sent`, description: String(e) });
    } finally {
      setSending(false);
    }
  };
  return (
    <li className="flex min-h-[40px] items-center gap-8 border-b border-hairline py-4 last:border-b-0">
      <span className="min-w-0 flex-1">
        <span className="block truncate text-13 font-medium text-fg first-letter:uppercase">
          {command.name}
        </span>
        <span className="num block truncate text-11 text-fg-3">{command.pattern}</span>
      </span>
      {command.choices && command.choices.length > 0 && (
        <select
          aria-label={`${command.name} value`}
          value={value}
          onChange={(e) => setValue(Number(e.target.value))}
          className="h-(--control-sm) rounded-8 border border-hairline bg-raised px-4 text-12 text-fg outline-none focus-visible:outline-2 focus-visible:outline-accent"
        >
          {command.choices.map((c) => (
            <option key={c.value} value={c.value}>
              {c.label}
            </option>
          ))}
        </select>
      )}
      {command.param && (
        <input
          aria-label={`${command.name} value`}
          type="number"
          min={command.param.min}
          max={command.param.max}
          value={value}
          onChange={(e) => setValue(Number(e.target.value))}
          className="num h-(--control-sm) w-[56px] rounded-8 border border-hairline bg-raised px-4 text-right text-12 text-fg outline-none focus-visible:outline-2 focus-visible:outline-accent"
        />
      )}
      <Button
        size="sm"
        variant={command.safety === "ride" ? "secondary" : "ghost"}
        icon={<Send />}
        onClick={() => void send()}
        {...(engine.status.state === "running" || engine.status.state === "paused"
          ? {}
          : { disabledReason: "The engine is not running" })}
        aria-busy={sending}
      >
        Send
      </Button>
    </li>
  );
}

function Commands() {
  const groups = (["read", "comfort", "ride", "debug"] as const)
    .map((safety) => ({ safety, items: engine.phoneCommands.filter((c) => c.safety === safety) }))
    .filter((g) => g.items.length > 0);
  return (
    <div className="flex flex-col gap-16 px-16 pt-16">
      {groups.map((g) => (
        <section key={g.safety}>
          <div className="flex items-baseline justify-between gap-8">
            <h3 className="label">{SAFETY[g.safety].label}</h3>
            <span className="truncate text-11 text-fg-3">{SAFETY[g.safety].note}</span>
          </div>
          <ul className="mt-4">
            {g.items.map((c) => (
              <CommandRow key={c.id} command={c} />
            ))}
          </ul>
        </section>
      ))}
    </div>
  );
}

function Log() {
  const [open, setOpen] = useState<number | null>(null);
  const frames = engine.frames.slice(-80).reverse();
  return (
    <div className="px-16 pt-16 pb-16">
      <div className="flex items-baseline justify-between">
        <h3 className="label">Received</h3>
        <span className="num text-11 text-fg-3">{engine.frames.length} lines</span>
      </div>
      <ul className="mt-4">
        {frames.map((f) => (
          <li key={f.id} className="border-b border-hairline last:border-b-0">
            <button
              type="button"
              onClick={() => setOpen(open === f.id ? null : f.id)}
              aria-expanded={open === f.id}
              className="grid h-[32px] w-full grid-cols-[56px_52px_1fr_16px] items-center gap-8 rounded-4 text-left hover:bg-raised"
            >
              <span className="num text-11 text-fg-2">{formatClock(f.t)}</span>
              <Pill tone={KIND_TONE[f.kind] ?? "neutral"}>{f.kind}</Pill>
              <span className="num truncate text-12 text-fg">{f.summary}</span>
              <ChevronDown
                className={cx(
                  "size-[14px] text-fg-3 transition-transform duration-(--dur-snappy) ease-snappy",
                  open === f.id && "rotate-180",
                )}
              />
            </button>
            {open === f.id && (
              <div className="pb-8 pl-8">
                {f.fields.map(([k, v]) => (
                  <div key={k} className="flex justify-between gap-16 text-12">
                    <span className="text-fg-2">{k}</span>
                    <span className="num text-fg">{v}</span>
                  </div>
                ))}
                <p className="num mt-4 text-11 break-all text-fg-3">{f.hex}</p>
              </div>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
}

export function PhoneBody() {
  useEngineData();
  if (engine.phoneCommands.length === 0) {
    return (
      <div className="grid h-full place-items-center p-24 text-center">
        <div className="flex max-w-[280px] flex-col items-center gap-8">
          <Smartphone className="size-[24px] text-fg-3" />
          <p className="text-13 font-semibold text-fg">The phone needs the live engine</p>
          <p className="text-12 text-fg-2">
            In the desktop app the phone shows the firmware&apos;s own BLE frames and can send the
            commands a phone app may send. The browser demo has no firmware to talk to.
          </p>
        </div>
      </div>
    );
  }
  return (
    <div className="min-h-0 flex-1 overflow-y-auto">
      <Dashboard />
      <Commands />
      <Log />
      <div className="flex items-center gap-8 px-16 pb-16 text-11 text-fg-3">
        <Tooltip content="Motor-off sequences, flash erase, bootloader entry and power-off">
          <span className="underline decoration-dotted underline-offset-2">
            Never-send commands
          </span>
        </Tooltip>
        are not offered.
        <Kbd>c y</Kbd>
        <Kbd>! @</Kbd>
        <Kbd>+ -</Kbd>
      </div>
    </div>
  );
}

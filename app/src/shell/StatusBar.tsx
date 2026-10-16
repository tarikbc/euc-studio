import { cx } from "../lib/cx.ts";
import { SAMPLE_RATE, engine, useEngineVersion, useLive } from "../engine/index.ts";
import { BluetoothOff, Cpu } from "../ui/icons.tsx";
import { RollingNumber } from "../ui/RollingNumber.tsx";
import { Tooltip } from "../ui/Tooltip.tsx";

function Item({ children, className }: { children: React.ReactNode; className?: string }) {
  return (
    <span className={cx("flex h-full items-center gap-4 [&_svg]:size-[12px]", className)}>
      {children}
    </span>
  );
}

function IsrLoad() {
  const isr = useLive((l) => l.isr, 4, 0);
  const tone = isr >= 85 ? "bg-danger" : isr >= 70 ? "bg-caution" : "bg-ok";
  return (
    <Tooltip content="Share of CPU time spent in interrupt handlers" side="top">
      <Item className="cursor-default">
        <Cpu className="text-fg-3" />
        ISR
        <span className="relative ml-2 h-[4px] w-[32px] overflow-hidden rounded-pill bg-track">
          <span
            className={cx(
              "absolute inset-y-0 left-0 rounded-pill transition-[width] duration-(--dur-data) ease-data",
              tone,
            )}
            style={{ width: `${Math.min(100, isr)}%` }}
          />
        </span>
        <span className="num w-[32px] text-right text-fg-2">
          <RollingNumber value={isr} />%
        </span>
      </Item>
    </Tooltip>
  );
}

const STATE_DOT: Record<string, string> = {
  running: "animate-[live-pulse_1.6s_ease-in-out_infinite] bg-ok",
  paused: "bg-caution",
  starting: "animate-[live-pulse_1.6s_ease-in-out_infinite] bg-caution",
  halted: "bg-danger",
  error: "bg-danger",
  unavailable: "bg-fg-3",
};

function EngineItem() {
  const { state, message, backend } = engine.status;
  const live = engine.kind === "live";
  const tip = live
    ? `${message || "The stock firmware runs in the simulator"}${backend === "unicorn" ? ". CPU: Unicorn, development builds only" : ""}`
    : "Synthetic signals; the desktop app runs the real firmware";
  return (
    <Tooltip content={tip} side="top">
      <Item className="cursor-default">
        <span className={cx("size-[6px] rounded-pill", STATE_DOT[state] ?? "bg-fg-3")} />
        <span className="text-fg">
          {live ? `Engine: ${engine.status.label ?? "connecting"}` : "Demo engine"}
        </span>
        <span>{state}</span>
        {live && state === "starting" && message && <span className="text-fg-3">{message}</span>}
      </Item>
    </Tooltip>
  );
}

export function StatusBar() {
  useEngineVersion();
  const fw = engine.firmwares().find((f) => f.id === engine.firmware);
  return (
    <footer className="flex h-(--statusbar) shrink-0 items-center gap-16 border-t border-hairline bg-bg px-16 text-12 text-fg-2">
      <EngineItem />
      <Item>
        {fw?.label} firmware
        {engine.status.build && <span className="num text-fg-3">{engine.status.build}</span>}
      </Item>
      <IsrLoad />
      <Tooltip content="Connect a wheel in the Flash workspace (M6)" side="top">
        <Item className="cursor-default">
          <BluetoothOff className="text-fg-3" />
          No wheel connected
        </Item>
      </Tooltip>
      <span className="flex-1" />
      <Item>
        <span className="num">{SAMPLE_RATE} Hz</span> telemetry
      </Item>
      <Item className="text-fg-3">
        EUC Studio <span className="num">{__APP_VERSION__}</span>
      </Item>
    </footer>
  );
}

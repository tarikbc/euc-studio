import { useEffect, useRef, useState } from "react";
import { cx } from "../../lib/cx.ts";
import { onFrame } from "../../lib/frameloop.ts";
import { type EventKind, type SimEvent, engine, useEngineVersion } from "../../engine/index.ts";
import { Flag } from "../../ui/icons.tsx";
import { Panel } from "../../ui/Panel.tsx";
import { Pill, type PillTone } from "../../ui/Pill.tsx";
import { formatClock } from "../../ui/rolling.ts";
import { Tabs } from "../../ui/Tabs.tsx";
import { Tooltip } from "../../ui/Tooltip.tsx";

const WINDOW = 30; // seconds on the strip

const KIND: Record<EventKind, { label: string; tone: PillTone; dot: string }> = {
  firmware: { label: "Firmware", tone: "info", dot: "bg-info" },
  scenario: { label: "Ride", tone: "neutral", dot: "bg-fg-2-safe" },
  command: { label: "Command", tone: "accent", dot: "bg-accent" },
  alarm: { label: "Alarm", tone: "caution", dot: "bg-caution" },
  tiltback: { label: "Tiltback", tone: "caution", dot: "bg-caution" },
  cutoff: { label: "Cutoff", tone: "danger", dot: "bg-danger" },
  fault: { label: "Fault", tone: "danger", dot: "bg-danger" },
};

/** Tabs group kinds: everything the firmware does to protect the rider sits under Safety. */
const GROUPS = {
  all: null,
  safety: ["alarm", "tiltback", "cutoff", "fault"],
  firmware: ["firmware"],
  scenario: ["scenario"],
  command: ["command"],
} as const satisfies Record<string, readonly EventKind[] | null>;

type Filter = keyof typeof GROUPS;

const inGroup = (filter: Filter, e: SimEvent) => {
  const kinds = GROUPS[filter] as readonly EventKind[] | null;
  return kinds === null || kinds.includes(e.kind);
};

/** Markers ride a 30 s strip; one CSS variable per frame moves them, React never re-renders. */
function Strip({ events }: { events: SimEvent[] }) {
  const ref = useRef<HTMLDivElement>(null);
  useEffect(
    () =>
      onFrame(() => {
        ref.current?.style.setProperty("--now", String(engine.latest.t));
      }),
    [],
  );
  return (
    <div ref={ref} className="relative mx-16 mt-8 h-[44px] shrink-0">
      <div className="absolute inset-x-0 top-[14px] h-px bg-hairline-strong" />
      {Array.from({ length: 7 }, (_, i) => (
        <div
          key={i}
          className={cx(
            "absolute top-[24px] text-[10px] text-fg-3",
            i === 0 ? "" : i === 6 ? "-translate-x-full" : "-translate-x-1/2",
          )}
          style={{ left: `${(i / 6) * 100}%` }}
        >
          <span className="num">{i === 6 ? "now" : `${-WINDOW + i * 5} s`}</span>
        </div>
      ))}
      <div className="absolute inset-0 overflow-hidden">
        {events.map((e) => (
          <Tooltip key={e.id} content={`${formatClock(e.t)}  ${e.title}`} side="top">
            <span
              className={cx(
                "absolute top-[9px] size-[10px] -translate-x-1/2 rounded-pill border-2 border-card",
                KIND[e.kind].dot,
              )}
              style={{ left: `calc((${e.t} - var(--now, 0) + ${WINDOW}) / ${WINDOW} * 100%)` }}
            />
          </Tooltip>
        ))}
      </div>
      <span className="absolute top-[8px] right-0 size-[12px] translate-x-1/2 rounded-pill border-2 border-card bg-accent" />
    </div>
  );
}

/** One timeline row; a row that arrives while the window is in view slides in. */
function EventRow({ event: e, fresh }: { event: SimEvent; fresh: boolean }) {
  const [slide] = useState(() => fresh && !document.hidden);
  return (
    <li className={cx(slide && "row-in")}>
      <div className="grid h-[32px] grid-cols-[64px_80px_1fr] items-center gap-8 rounded-8 px-8 hover:bg-raised">
        <span className="num text-12 text-fg-2">{formatClock(e.t)}</span>
        <span>
          <Pill tone={KIND[e.kind].tone}>{KIND[e.kind].label}</Pill>
        </span>
        <span className="truncate text-13 text-fg">{e.title}</span>
      </div>
    </li>
  );
}

export function EventsPanel() {
  useEngineVersion();
  const [filter, setFilter] = useState<Filter>("all");
  const all = engine.events;
  const visible = all.filter((e) => inGroup(filter, e));
  const shown = visible.slice(-60).reverse();
  // Rows that arrive after the panel mounted slide in; the ones already there do not.
  const [firstNew] = useState(() => (all.at(-1)?.id ?? 0) + 1);
  const count = (f: Filter) => all.filter((e) => inGroup(f, e)).length;

  return (
    <Panel
      title="Events"
      icon={<Flag />}
      className="min-h-[140px]"
      bodyClassName="flex flex-col overflow-hidden"
      actions={
        <Tabs<Filter>
          aria-label="Filter events"
          value={filter}
          onValueChange={setFilter}
          tabs={[
            { value: "all", label: "All", count: all.length },
            { value: "safety", label: "Safety", count: count("safety") },
            { value: "firmware", label: "Firmware", count: count("firmware") },
            { value: "scenario", label: "Ride", count: count("scenario") },
            { value: "command", label: "Commands", count: count("command") },
          ]}
          className="-mb-px h-(--dock-header) [&>div]:h-full"
        />
      }
    >
      <Strip events={visible} />
      <ul className="min-h-0 flex-1 overflow-y-auto px-8 pb-8">
        {shown.map((e) => (
          <EventRow key={e.id} event={e} fresh={e.id >= firstNew} />
        ))}
        {shown.length === 0 && (
          <li className="px-8 py-16 text-13 text-fg-2">
            No {filter === "all" ? "" : `${filter === "scenario" ? "ride" : filter} `}events yet in
            this run.
          </li>
        )}
      </ul>
    </Panel>
  );
}

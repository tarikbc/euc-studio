import { useEffect, useMemo } from "react";
import { comboFromEvent } from "../lib/keys.ts";
import { BUDGET_MS, formatStats, measureFrames } from "../lib/perf.ts";
import { accents, type AccentName } from "../design/tokens.ts";
import { engine, useEngineVersion } from "../engine/index.ts";
import { WORKSPACE_IDS, useUi } from "../state/ui.ts";
import type { PaletteCommand } from "../ui/CommandPalette.tsx";
import {
  Camera,
  Cpu,
  Keyboard,
  LayoutGrid,
  Palette,
  PanelBottom,
  PanelRight,
  Pause,
  Play,
  RotateCcw,
  StepForward,
  Timer,
  Waves,
} from "../ui/icons.tsx";
import { IS_MAC } from "../lib/platform.ts";
import { toast } from "../ui/toast-store.ts";
import { WORKSPACES } from "./workspaces.tsx";

export const RTF_STEPS = [0.25, 0.5, 1, 2, 4];

function stepRtf(dir: 1 | -1) {
  const i = RTF_STEPS.findIndex((r) => r >= engine.targetRtf - 1e-9);
  const next = RTF_STEPS[Math.min(RTF_STEPS.length - 1, Math.max(0, (i < 0 ? 2 : i) + dir))] ?? 1;
  engine.setTargetRtf(next);
}

/** Every action in the app. The palette, the shortcut sheet and the hotkeys all read this list. */
export function useCommands(): PaletteCommand[] {
  useEngineVersion();
  const ui = useUi();

  return useMemo<PaletteCommand[]>(() => {
    const docks = ui.docks[ui.workspace];
    const list: PaletteCommand[] = [
      ...WORKSPACE_IDS.map((id, i) => ({
        id: `workspace.${id}`,
        title: `Go to ${WORKSPACES[id].label}`,
        group: "Workspaces",
        icon: WORKSPACES[id].icon,
        shortcut: `mod+${i + 1}`,
        keywords: [WORKSPACES[id].label, "workspace", "switch"],
        hint: ui.route === id ? "Current" : undefined,
        run: () => ui.navigate(id),
      })),
      {
        id: "gallery.open",
        title: "Open component gallery",
        group: "Workspaces",
        icon: <LayoutGrid />,
        keywords: ["design", "system", "tokens", "components"],
        run: () => ui.navigate("gallery"),
      },
      {
        id: "sim.toggle",
        title: engine.playing ? "Pause simulation" : "Run simulation",
        group: "Simulation",
        icon: engine.playing ? <Pause /> : <Play />,
        shortcut: "space",
        keywords: ["play", "pause", "run", "stop"],
        run: () => engine.togglePlaying(),
      },
      {
        id: "sim.restart",
        title: "Restart from t = 0",
        group: "Simulation",
        icon: <RotateCcw />,
        shortcut: "r",
        keywords: ["reset", "reboot"],
        run: () => engine.restart(),
      },
      {
        id: "sim.step",
        title: "Step one frame (while paused)",
        group: "Simulation",
        icon: <StepForward />,
        shortcut: ".",
        keywords: ["frame", "advance", "next"],
        run: () => {
          if (engine.playing) engine.setPlaying(false);
          else engine.step();
        },
      },
      {
        id: "sim.slower",
        title: "Slower (halve the real-time factor)",
        group: "Simulation",
        icon: <Timer />,
        shortcut: "[",
        keywords: ["speed", "rtf", "slow motion"],
        run: () => stepRtf(-1),
      },
      {
        id: "sim.faster",
        title: "Faster (double the real-time factor)",
        group: "Simulation",
        icon: <Timer />,
        shortcut: "]",
        keywords: ["speed", "rtf", "fast forward"],
        run: () => stepRtf(1),
      },
      ...engine
        .firmwares()
        .filter((f) => !f.disabledReason)
        .map((f) => ({
          id: `firmware.${f.id}`,
          title: `Run ${f.label} firmware`,
          group: "Simulation",
          icon: <Cpu />,
          keywords: ["firmware", f.detail],
          hint: engine.firmware === f.id ? "Current" : undefined,
          run: () => engine.setFirmware(f.id),
        })),
      ...engine.scenarios().map((s) => ({
        id: `scenario.${s.id}`,
        title: `Scenario: ${s.label}`,
        group: "Simulation",
        icon: <Waves />,
        keywords: ["scenario", s.detail],
        hint: engine.scenarioId === s.id ? "Current" : undefined,
        run: () => engine.setScenario(s.id),
      })),
      {
        id: "view.right",
        title: docks.right ? "Hide right dock" : "Show right dock",
        group: "View",
        icon: <PanelRight />,
        shortcut: "mod+b",
        keywords: ["telemetry", "panel", "sidebar"],
        run: () => ui.toggleDock("right"),
      },
      {
        id: "view.bottom",
        title: docks.bottom ? "Hide bottom dock" : "Show bottom dock",
        group: "View",
        icon: <PanelBottom />,
        shortcut: "mod+j",
        keywords: ["events", "timeline", "panel"],
        run: () => ui.toggleDock("bottom"),
      },
      {
        id: "view.reset",
        title: "Reset dock layout",
        group: "View",
        icon: <LayoutGrid />,
        keywords: ["layout", "panels", "default"],
        run: () => {
          ui.resetLayout();
          toast({
            tone: "success",
            title: "Layout reset",
            description: "Docks are back to their default sizes.",
          });
        },
      },
      {
        id: "view.camera",
        title: ui.camera === "follow" ? "Orbit camera" : "Follow camera",
        group: "View",
        icon: <Camera />,
        shortcut: "c",
        keywords: ["camera", "3d", "viewport", "orbit", "follow"],
        run: () => ui.setCamera(ui.camera === "follow" ? "orbit" : "follow"),
      },
      ...(Object.keys(accents) as AccentName[]).map((a) => ({
        id: `accent.${a}`,
        title: `Accent color: ${accents[a].label}`,
        group: "Appearance",
        icon: <Palette />,
        keywords: ["accent", "color", "theme"],
        hint: ui.accent === a ? "Current" : undefined,
        run: () => {
          ui.setAccent(a);
          toast({ tone: "success", title: `Accent changed to ${accents[a].label}` });
        },
      })),
      {
        id: "dev.frames",
        title: "Measure frame time (10 s)",
        group: "Help",
        icon: <Timer />,
        keywords: ["performance", "fps", "frame", "p95"],
        run: () => {
          toast({
            tone: "info",
            title: "Measuring frame time",
            description: "Keep the window in front for 10 s.",
          });
          void measureFrames(10).then((s) => {
            const ok = s.work.p95 <= BUDGET_MS;
            toast({
              tone: ok ? "success" : "caution",
              title: `Frame time p95 ${s.work.p95.toFixed(1)} ms`,
              description: formatStats(s),
            });
            console.warn(`PERF ${formatStats(s)}`);
          });
        },
      },
      {
        id: "help.shortcuts",
        title: "Keyboard shortcuts",
        group: "Help",
        icon: <Keyboard />,
        shortcut: "?",
        keywords: ["keys", "help", "hotkeys"],
        run: () => ui.setShortcutsOpen(true),
      },
    ];
    return list;
  }, [ui]);
}

const INTERACTIVE = "input, textarea, select, [contenteditable=''], [contenteditable='true']";
const ACTIVATES_ON_SPACE =
  "button, a, [role='button'], [role='radio'], [role='tab'], [role='switch'], [role='option'], [role='slider']";

export function useHotkeys(commands: PaletteCommand[]) {
  useEffect(() => {
    const byCombo = new Map(
      commands.filter((c) => c.shortcut).map((c) => [c.shortcut as string, c]),
    );
    const onKey = (e: KeyboardEvent) => {
      if (e.defaultPrevented || e.isComposing) return;
      const combo = comboFromEvent(e, IS_MAC);
      const ui = useUi.getState();
      if (combo === "mod+k") {
        e.preventDefault();
        ui.setShortcutsOpen(false);
        ui.setPaletteOpen(!ui.paletteOpen);
        return;
      }
      if (ui.paletteOpen || ui.shortcutsOpen) return;
      const cmd = byCombo.get(combo);
      if (!cmd) return;
      const target = e.target instanceof Element ? e.target : null;
      const hasMod = combo.startsWith("mod+");
      if (!hasMod && target?.closest(INTERACTIVE)) return;
      if (combo === "space" && target?.closest(ACTIVATES_ON_SPACE)) return;
      e.preventDefault();
      cmd.run();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [commands]);
}

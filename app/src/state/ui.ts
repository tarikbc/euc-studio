import { create } from "zustand";
import { DEFAULT_ACCENT, accents, type AccentName } from "../design/tokens.ts";
import { readStored, writeStored } from "../lib/storage.ts";

export type WorkspaceId = "ride" | "tune" | "compare" | "flash";
export const WORKSPACE_IDS: WorkspaceId[] = ["ride", "tune", "compare", "flash"];
export type Route = WorkspaceId | "gallery";
export type CameraMode = "follow" | "orbit";
export type DockId = "right" | "bottom";
export type RightTab = "telemetry" | "phone";

type DockState = Record<DockId, boolean>;

interface UiState {
  route: Route;
  /** Last workspace, so leaving the gallery returns there. */
  workspace: WorkspaceId;
  accent: AccentName;
  camera: CameraMode;
  docks: Record<WorkspaceId, DockState>;
  paletteOpen: boolean;
  /** What the right dock shows in Ride. */
  rightTab: RightTab;
  shortcutsOpen: boolean;
  /** Bumped to ask the dock layout to reset to defaults. */
  layoutEpoch: number;

  navigate: (route: Route) => void;
  setAccent: (accent: AccentName) => void;
  setCamera: (camera: CameraMode) => void;
  toggleDock: (dock: DockId, open?: boolean) => void;
  resetLayout: () => void;
  setPaletteOpen: (open: boolean) => void;
  setRightTab: (tab: RightTab) => void;
  setShortcutsOpen: (open: boolean) => void;
}

const DEFAULT_DOCKS: DockState = { right: true, bottom: true };

function routeFromHash(): Route {
  const h = typeof location !== "undefined" ? location.hash.replace(/^#\/?/, "") : "";
  if (h === "gallery") return "gallery";
  return (WORKSPACE_IDS as string[]).includes(h) ? (h as WorkspaceId) : "ride";
}

function applyAccent(accent: AccentName) {
  if (typeof document === "undefined") return;
  if (accent === DEFAULT_ACCENT) delete document.documentElement.dataset.accent;
  else document.documentElement.dataset.accent = accent;
}

const initialRoute = routeFromHash();
const storedAccent = readStored<string>("accent", DEFAULT_ACCENT);
const initialAccent: AccentName =
  storedAccent in accents ? (storedAccent as AccentName) : DEFAULT_ACCENT;
applyAccent(initialAccent);

export const useUi = create<UiState>((set, get) => ({
  route: initialRoute,
  workspace: initialRoute === "gallery" ? "ride" : initialRoute,
  accent: initialAccent,
  camera: readStored<CameraMode>("camera", "follow") === "orbit" ? "orbit" : "follow",
  docks: readStored("docks", {
    ride: DEFAULT_DOCKS,
    tune: DEFAULT_DOCKS,
    compare: DEFAULT_DOCKS,
    flash: DEFAULT_DOCKS,
  }),
  paletteOpen: false,
  rightTab: readStored<RightTab>("rightTab", "telemetry") === "phone" ? "phone" : "telemetry",
  shortcutsOpen: false,
  layoutEpoch: 0,

  navigate: (route) => {
    if (get().route === route) return;
    set(route === "gallery" ? { route } : { route, workspace: route });
    const hash = `#/${route}`;
    if (location.hash !== hash) history.replaceState(null, "", hash);
  },
  setAccent: (accent) => {
    applyAccent(accent);
    writeStored("accent", accent);
    set({ accent });
  },
  setCamera: (camera) => {
    writeStored("camera", camera);
    set({ camera });
  },
  toggleDock: (dock, open) => {
    const { workspace, docks } = get();
    const current = docks[workspace];
    const next = { ...docks, [workspace]: { ...current, [dock]: open ?? !current[dock] } };
    writeStored("docks", next);
    set({ docks: next });
  },
  resetLayout: () => {
    const { workspace, docks, layoutEpoch } = get();
    const next = { ...docks, [workspace]: DEFAULT_DOCKS };
    writeStored("docks", next);
    set({ docks: next, layoutEpoch: layoutEpoch + 1 });
  },
  setPaletteOpen: (paletteOpen) => set({ paletteOpen }),
  setRightTab: (rightTab) => {
    writeStored("rightTab", rightTab);
    set({ rightTab });
  },
  setShortcutsOpen: (shortcutsOpen) => set({ shortcutsOpen }),
}));

if (typeof window !== "undefined") {
  window.addEventListener("hashchange", () => useUi.getState().navigate(routeFromHash()));
}

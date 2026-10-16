// The one hand-written design token file for EUC Studio.
//
// `pnpm tokens` turns this file into src/design/tokens.css (a Tailwind 4 @theme block).
// `pnpm tokens:check` fails when the CSS is out of date, and `pnpm contrast` checks every
// text/surface pair declared in `contrastPairs` against WCAG AA. Edit values here, never in CSS.

import { mix } from "./color.ts";

/* ----------------------------------------------------------------------------------------- */
/* Color                                                                                     */
/* ----------------------------------------------------------------------------------------- */

/** Surfaces, darkest first. Hover raises an element exactly one step. */
export const surface = {
  bg: "#000000",
  card: "#1C1C1E",
  raised: "#2C2C2E",
  track: "#3A3A3C",
  lift: "#48484A",
} as const;
export type SurfaceName = keyof typeof surface;

/** The surface one step above `name` (the hover state). */
export const surfaceStep: Record<SurfaceName, SurfaceName> = {
  bg: "card",
  card: "raised",
  raised: "track",
  track: "lift",
  lift: "lift",
};

export const text = {
  /** Primary text. */
  fg: "#F2F2F7",
  /** Secondary text on bg and card. */
  "fg-2": "#8E8E93",
  /** Text-safe secondary for raised and track, where #8E8E93 fails AA. */
  "fg-2-safe": "#AEAEB2",
  /** Tertiary: icons, ticks, disabled. Meets 3:1 for UI graphics; never body text. */
  "fg-3": "#6E6E73",
} as const;

/** Mix amount used for tinted fills (pills, selected rows): the hue painted at 16% on card. */
export const TINT = 0.16;

const tint = (hue: string) => mix(hue, surface.card, TINT);

export const semantic = {
  caution: "#FFD60A",
  danger: "#FF3B30",
  ok: "#30D158",
  info: "#64D2FF",
  /** Text-safe danger: #FF3B30 fails AA on raised and on its own tint. */
  "danger-text": "#FF6961",
  "caution-tint": tint("#FFD60A"),
  "danger-tint": tint("#FF3B30"),
  "ok-tint": tint("#30D158"),
  "info-tint": tint("#64D2FF"),
  /** Hover fill for danger buttons (one step stronger than the tint). */
  "danger-tint-hover": mix("#FF3B30", surface.card, 0.24),
} as const;

/** Text on any accent fill is black: white on orange fails contrast. */
export const ON_ACCENT = "#000000";

export interface AccentTheme {
  label: string;
  /** Fill: buttons, knobs, focus ring, selected indicators. */
  accent: string;
  /** Text-safe variant for accent-colored text on bg, card, raised and the tint. */
  text: string;
  /** Tinted fill for pills and selected rows. */
  tint: string;
  /** Hover fill for accent buttons: the accent lifted toward white. */
  hover: string;
}

const accentTheme = (label: string, accent: string, text = accent): AccentTheme => ({
  label,
  accent,
  text,
  tint: tint(accent),
  hover: mix("#FFFFFF", accent, 0.16),
});

export const accents = {
  orange: accentTheme("Orange", "#FF7A1A"),
  green: accentTheme("Green", "#30D158"),
  cyan: accentTheme("Cyan", "#64D2FF"),
  pink: accentTheme("Pink", "#FF375F", "#FF6482"),
  white: accentTheme("White", "#F2F2F7"),
} as const satisfies Record<string, AccentTheme>;
export type AccentName = keyof typeof accents;
export const DEFAULT_ACCENT: AccentName = "orange";

/** Fixed per-signal colors for scopes and gauges. They do not follow the UI accent. */
export const series = {
  speed: "#64D2FF",
  pwm: "#FF7A1A",
  /** PWM's pair: the voltage really applied, a lighter orange. */
  modulation: "#FFB066",
  current: "#FF375F",
  voltage: "#30D158",
  pitch: "#F2F2F7",
  temp: "#FFD60A",
  suspension: "#BF5AF2",
} as const;

/** 1px hairlines are the only borders. They are translucent so they work on any surface. */
export const hairline = {
  hairline: "rgba(255, 255, 255, 0.08)",
  "hairline-strong": "rgba(255, 255, 255, 0.14)",
} as const;

/* ----------------------------------------------------------------------------------------- */
/* Contrast pairs (checked by scripts/check-contrast.ts, shown in the gallery)               */
/* ----------------------------------------------------------------------------------------- */

export interface ContrastPair {
  fg: string;
  bg: string;
  fgName: string;
  bgName: string;
  /** "text" needs 4.5:1 (WCAG AA body text); "ui" needs 3:1 (graphics, focus ring). */
  kind: "text" | "ui";
  use: string;
}

const pair = (
  fgName: string,
  fg: string,
  bgName: string,
  bg: string,
  use: string,
  kind: ContrastPair["kind"] = "text",
): ContrastPair => ({ fg, bg, fgName, bgName, kind, use });

export const contrastPairs: ContrastPair[] = [
  ...(["bg", "card", "raised", "track", "lift"] as const).map((s) =>
    pair("fg", text.fg, s, surface[s], "Primary text"),
  ),
  pair("fg-2", text["fg-2"], "bg", surface.bg, "Secondary text"),
  pair("fg-2", text["fg-2"], "card", surface.card, "Secondary text"),
  pair("fg-2-safe", text["fg-2-safe"], "raised", surface.raised, "Secondary text, raised"),
  pair("fg-2-safe", text["fg-2-safe"], "track", surface.track, "Secondary text, track"),
  pair("fg-3", text["fg-3"], "bg", surface.bg, "Icons, ticks", "ui"),
  pair("fg-3", text["fg-3"], "card", surface.card, "Icons, ticks", "ui"),
  ...Object.entries(accents).flatMap(([name, t]) => [
    pair(`on-accent`, ON_ACCENT, `accent (${name})`, t.accent, "Label on accent fill"),
    pair(`on-accent`, ON_ACCENT, `accent-hover (${name})`, t.hover, "Label on hovered fill"),
    pair(`accent-text (${name})`, t.text, "bg", surface.bg, "Accent text"),
    pair(`accent-text (${name})`, t.text, "card", surface.card, "Accent text"),
    pair(`accent-text (${name})`, t.text, "raised", surface.raised, "Accent text, raised"),
    pair(`accent-text (${name})`, t.text, `accent-tint (${name})`, t.tint, "Accent pill"),
    pair(`accent (${name})`, t.accent, "raised", surface.raised, "Focus ring", "ui"),
  ]),
  pair("caution", semantic.caution, "card", surface.card, "Caution text"),
  pair("caution", semantic.caution, "caution-tint", semantic["caution-tint"], "Caution pill"),
  pair("danger-text", semantic["danger-text"], "card", surface.card, "Danger text"),
  pair("danger-text", semantic["danger-text"], "raised", surface.raised, "Danger text, raised"),
  pair(
    "danger-text",
    semantic["danger-text"],
    "danger-tint",
    semantic["danger-tint"],
    "Danger pill",
  ),
  pair(
    "danger-text",
    semantic["danger-text"],
    "danger-tint-hover",
    semantic["danger-tint-hover"],
    "Danger button, hover",
  ),
  pair("on-accent", ON_ACCENT, "danger", semantic.danger, "Label on danger fill"),
  pair("ok", semantic.ok, "card", surface.card, "OK text"),
  pair("ok", semantic.ok, "ok-tint", semantic["ok-tint"], "OK pill"),
  pair("info", semantic.info, "card", surface.card, "Info text"),
  pair("info", semantic.info, "info-tint", semantic["info-tint"], "Info pill"),
  ...Object.entries(series).map(([name, c]) =>
    pair(`series ${name}`, c, "card", surface.card, "Scope trace", "ui"),
  ),
];

export const MIN_RATIO: Record<ContrastPair["kind"], number> = { text: 4.5, ui: 3 };

/* ----------------------------------------------------------------------------------------- */
/* Type                                                                                      */
/* ----------------------------------------------------------------------------------------- */

export const font = {
  ui: 'ui-rounded, "SF Pro Rounded", system-ui, "Segoe UI Variable", sans-serif',
  mono: '"JetBrains Mono", ui-monospace, "SF Mono", Menlo, Consolas, monospace',
} as const;

/** Size → [line height, letter spacing]. Labels (11 px, 700, +1 px) come from the `label` class. */
export const typeScale = {
  11: [14, "0.01em"],
  12: [16, "0"],
  13: [18, "0"],
  15: [20, "-0.005em"],
  20: [26, "-0.01em"],
  28: [34, "-0.015em"],
  44: [48, "-0.02em"],
} as const satisfies Record<number, readonly [number, string]>;

/* ----------------------------------------------------------------------------------------- */
/* Space, radius, sizes                                                                      */
/* ----------------------------------------------------------------------------------------- */

/** The only spacing values. Class names are the pixel value: `p-16` is 16 px. */
export const spacing = [0, 2, 4, 8, 16, 24, 32, 48] as const;

export const radius = { 4: "4px", 8: "8px", 12: "12px", 16: "16px", pill: "9999px" } as const;

/** Fixed component and chrome dimensions (not spacing). Used as `h-(--control-md)`. */
export const size = {
  "control-sm": 24,
  "control-md": 32,
  "control-lg": 40,
  runbar: 52,
  rail: 56,
  statusbar: 28,
  "dock-header": 40,
} as const;

/* ----------------------------------------------------------------------------------------- */
/* Motion: only four tokens                                                                  */
/* ----------------------------------------------------------------------------------------- */

export const motion = {
  /** Buttons, toggles, pills, knobs. */
  snappy: { type: "spring", duration: 0.3, bounce: 0.25 },
  /** Panels, sheets, workspace changes. */
  smooth: { type: "spring", duration: 0.55, bounce: 0 },
  /** Success moments only. */
  bouncy: { type: "spring", duration: 0.5, bounce: 0.45 },
  /** Gauges and live values. */
  data: { type: "tween", duration: 0.25, ease: "linear" },
} as const;
export type MotionName = keyof typeof motion;

/** CSS hover curve (overshoots slightly) and its duration. */
export const hover = { ease: "cubic-bezier(.3, 1.25, .4, 1)", duration: 0.2 } as const;

/** Rolling digits: per-digit slots, direction follows the value, at most 8 updates per second. */
export const digits = {
  enter: { duration: 0.34, ease: [0.2, 0.9, 0.3, 1.15], y: "0.32em", scale: 0.96, blur: 5 },
  exit: { duration: 0.26, ease: "easeIn", y: "0.32em", scale: 0.96, blur: 5 },
  maxUpdatesPerSecond: 8,
} as const;

/** A wait that runs longer than this offers Show log / Retry / Cancel. */
export const STUCK_AFTER_MS = 10_000;

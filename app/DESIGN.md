# EUC Studio design system

EUC Studio is a developer tool: people watch firmware ride a wheel, read numbers that change
sixty times a second, and later write images to real hardware. The design follows from that.
Dark surfaces keep the 3D view and the scopes in front. One accent marks what is active. Numbers
never jitter. Motion explains what changed and never decorates. Anything that waits says what it is
waiting for.

The language is the one approved for EUC Remote (the watch app): black surfaces, an orange accent,
semantic yellow, red and green, four motion tokens and rolling digits.

Everything visual comes from one hand-written file, [`src/design/tokens.ts`](src/design/tokens.ts).
The live reference is the gallery: run the app and press the grid icon in the rail, or open
`#/gallery`.

## Tokens

`pnpm tokens` renders `tokens.ts` into [`src/design/tokens.css`](src/design/tokens.css), a Tailwind 4
`@theme` block. CI runs `pnpm tokens:check` (fails when the CSS is stale) and `pnpm contrast`
(fails when any declared text/surface pair misses WCAG AA). Never edit `tokens.css` by hand.

### Color

| Token                                | Value                                         | Use                                                               |
| ------------------------------------ | --------------------------------------------- | ----------------------------------------------------------------- |
| `bg`                                 | `#000000`                                     | App background, 3D viewport                                       |
| `card`                               | `#1C1C1E`                                     | Docks, panels, HUD                                                |
| `raised`                             | `#2C2C2E`                                     | Hover on card, selected rows, tooltips, menus                     |
| `track`                              | `#3A3A3C`                                     | Slider and gauge tracks, hover on raised                          |
| `lift`                               | `#48484A`                                     | Top step (hover on track)                                         |
| `fg`                                 | `#F2F2F7`                                     | Primary text                                                      |
| `fg-2`                               | `#8E8E93`                                     | Secondary text on `bg` and `card`                                 |
| `fg-2-safe`                          | `#AEAEB2`                                     | Secondary text on `raised` and `track` (where `#8E8E93` fails AA) |
| `fg-3`                               | `#6E6E73`                                     | Icons, ticks, disabled; 3:1 graphics only, never body text        |
| `accent`                             | `#FF7A1A`                                     | Fills, focus ring, selection; text on it is always black          |
| `caution` / `danger` / `ok` / `info` | `#FFD60A` / `#FF3B30` / `#30D158` / `#64D2FF` | Semantic states                                                   |
| `danger-text`                        | `#FF6961`                                     | Text-safe red (`#FF3B30` fails AA on `raised` and on its tint)    |
| `*-tint`                             | hue at 16 % over `card`                       | Pill and selected-row fills                                       |
| `hairline`, `hairline-strong`        | white at 8 % / 14 %                           | The only borders                                                  |

Alternate accents (green, cyan, pink, white) switch at runtime through `data-accent` on `<html>`
(command palette: "Accent color"). Each theme defines `accent`, `accent-text` (pink uses `#FF6482`
for text), `accent-tint` and `accent-hover`. Signal colors (`series-*`) are fixed per channel and
never follow the accent: PWM is always orange, speed always cyan.

Contrast is declared, not assumed: `contrastPairs` in `tokens.ts` lists every pair the UI uses
(63 today), and the gallery shows each ratio.

### Type

- Interface: `ui-rounded, "SF Pro Rounded", system-ui, "Segoe UI Variable", sans-serif`. Inside
  Tauri on macOS (WebKit) this is SF Pro Rounded; Chromium and other platforms fall back to the
  system face.
- Numbers: JetBrains Mono (bundled, OFL) with tabular figures, via the `num` utility.
- Scale (px, line height): 11/14, 12/16, 13/18, 15/20, 20/26, 28/34, 44/48. Classes are
  `text-11` ... `text-44`.
- Labels (`label` utility): 11 px, weight 700, 1 px tracking, uppercase. Only for panel titles and
  HUD captions.

### Space, radius, size

- Spacing: `2 4 8 16 24 32 48` only. Tailwind's scale is removed, and a class name is its size in
  pixels: `p-16` is 16 px, `gap-8` is 8 px. `p-3` does not exist.
- Radius: `rounded-4/8/12/16/pill`. Radii nest: 16 px sheets hold 12 px panels hold 8 px controls;
  segmented controls and pills are fully round so their knobs stay concentric.
- Fixed sizes (CSS variables, not spacing): `--control-sm/md/lg` 24/32/40, `--runbar` 52,
  `--rail` 56, `--statusbar` 28, `--dock-header` 40.
- Elevation is a surface step plus a hairline. One soft shadow (`shadow-overlay`) exists for
  floating overlays only.

## Motion

Four tokens, nothing else.

| Token    | Definition                | Use                                           |
| -------- | ------------------------- | --------------------------------------------- |
| `snappy` | spring 0.3 s, bounce 0.25 | Buttons, toggles, pills, knobs, tab underline |
| `smooth` | spring 0.55 s, bounce 0   | Panels, sheets, docks, workspace changes      |
| `bouncy` | spring 0.5 s, bounce 0.45 | Success moments only (the toast check)        |
| `data`   | linear 0.25 s             | Gauges, bars, needles, live values            |

- The token generator samples Motion's own spring into CSS `linear()` easings (`ease-snappy`,
  `ease-smooth`, `ease-bouncy`), so a CSS transition and a Motion animation with the same token
  move identically.
- Hover uses `cubic-bezier(.3, 1.25, .4, 1)` over 200 ms (CSS only).
- Rolling digits: each digit has a slot; the new digit enters from `translateY(.32em) scale(.96)`,
  opacity 0, `blur(5px)` over 0.34 s `cubic-bezier(.2, .9, .3, 1.15)`; the old one leaves over
  0.26 s ease-in; direction follows the value; at most 8 updates per second. A digit that changes
  again before its roll ends swaps in place (see decisions).
- Reduced motion: `MotionConfig reducedMotion="user"` drops transforms; digits cross-fade; the
  follow camera cuts instead of easing; CSS transitions and animations shrink to 1 ms.
- Low power: when WebGL runs on a CPU rasterizer (SwiftShader, llvmpipe, Microsoft Basic Render
  Driver) the 3D view drops to reduced quality (1x pixel ratio, no shadow maps, a static contact
  shadow, no camera sway, 20 fps and only while the sim moves) and says so with a "Reduced 3D
  quality" pill. The shared frame loop and the scopes cap at 30 fps and digits cross-fade. Scopes
  and sparklines never redraw without new samples. `?quality=high|low` overrides the detection.

## States

- Hover raises a surface one step (`card` to `raised`, `raised` to `track`); accent fills lighten
  (`accent-hover`).
- Press scales to 0.97 with the snappy spring (`pressable` class).
- Focus is a 2 px accent outline with a 2 px offset (`:focus-visible`).
- Disabled dims to 40 % and keeps pointer events: its tooltip says why (`disabledReason`).
- Waits are never a bare spinner. `Wait` names the step ("Building the scene"); after 10 s it adds
  Show log / Retry / Cancel.
- The gallery renders hover, press and focus statically through `data-force` and the
  `force-hover:` / `force-press:` / `force-focus:` variants.

## Components (`src/ui/`)

| Component              | Notes                                                                                                                      |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `Button`, `IconButton` | primary / secondary / ghost / danger; sm / md / lg; `disabledReason`; icon buttons always have a tooltip with the shortcut |
| `SegmentedControl`     | Radix RadioGroup; pill knob slides with `snappy` (`layoutId`)                                                              |
| `Toggle`               | Radix Switch; thumb travels with `snappy` and stretches while pressed                                                      |
| `Slider`               | Radix Slider; value readout lights up while dragging                                                                       |
| `Pill`                 | six tones, optional status dot (`live` pulses)                                                                             |
| `Tabs`                 | Radix Tabs; accent underline slides with `snappy`; optional counts                                                         |
| `Select`               | Radix Select; pops with `snappy`; detail line per option                                                                   |
| `Panel`                | dock card: 40 px header (label + actions) and scrollable body                                                              |
| `Tooltip`              | Radix Tooltip; optional shortcut keys                                                                                      |
| `Toast`                | Radix Toast; slides in with `smooth`; success icon pops with `bouncy`                                                      |
| `Kbd`, `Shortcut`      | key caps; `mod` renders as ⌘ on macOS and Ctrl elsewhere                                                                   |
| `RollingNumber`        | per-digit slots, 8 Hz limit, direction-aware                                                                               |
| `Gauge`                | 270° arc, caution and danger zones with ticks, `data` token                                                                |
| `Sparkline`            | canvas line with a fading fill; reads a ring buffer on the frame loop                                                      |
| `CommandPalette`       | cmdk inside a `Sheet`; word-prefix matching; sliding highlight                                                             |
| `Sheet`                | Radix Dialog; dims the app; rises in with `smooth`                                                                         |
| `Wait`                 | named wait with the 10 s escape hatch                                                                                      |
| `icons.tsx`            | Lucide set plus `Unicycle`, `Diverge`, `ChipWrite`, `LogoMark` drawn on the same 24 px, 2 px-stroke grid                   |

## Shell

```
┌ run bar: logo │ firmware │ scenario │ restart  play │ real time  sim time │ … │ ⌘K ┐
│ rail │ center (3D viewport)                         │ right dock: Telemetry/Phone│
│      ├──────────────────────────────────────────────┤                            │
│      │ bottom dock (Events)                         │                            │
└ status bar: engine │ firmware │ ISR load │ BLE                    │ rate │ version ┘
```

- Docks are cards in 8 px gutters (`react-resizable-panels`). A gutter shows a grabber on hover and
  an accent line while dragging. Sizes persist per workspace in localStorage; opening and closing
  animates flex-grow with `smooth`, never during a drag.
- Workspaces (Ride, Tune, Compare, Flash) stay mounted after the first visit and swap with
  `smooth`; the 3D loop stops while Ride is hidden.
- On macOS inside Tauri the window uses an overlay title bar; the run bar leaves room for the
  traffic lights and is the drag region.

Keyboard: ⌘1-⌘4 workspaces, ⌘K palette, Space run/pause, R restart, [ and ] slower/faster,
⌘B right dock, ⌘J bottom dock, C camera, ? shortcut sheet. The palette, the sheet and the hotkeys
all read one command list (`src/shell/commands.tsx`).

## Data path

Telemetry never goes through React state. The engine writes into per-channel `RingBuffer`s
(`src/lib/ring.ts`); one `requestAnimationFrame` loop (`src/lib/frameloop.ts`) advances the engine,
then scopes copy their window into preallocated typed arrays and call uPlot `setData`. The 3D view
reads the same `latest` object in `useFrame`. Readouts poll at most 8 times a second (`useLive`)
and re-render only when the rounded value changes.

Two engines sit behind one `EngineClient` interface (`src/engine/types.ts`). In the desktop window
`LiveEngine` (`src/engine/live.ts`) talks to the Rust backend: a simulation thread steps the
emulated board in 1/120 s steps, packs snapshots into the binary batch format of `euc-telemetry`
and sends them on a `tauri::ipc::Channel` about every 16 ms; JSON messages carry status, events,
decoded BLE frames and board outputs (display, LEDs, buzzer). In a browser and in tests the
deterministic demo (`src/sim/synthetic.ts`) runs instead. The status bar names the engine and what
it models; channels the run does not model show "Not modelled in this run" instead of a flat line.

## Decisions log

| Date       | Decision                                                                                                                                           | Why                                                                                                                                                        |
| ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2026-10-16 | Spacing classes are named by pixel value (`p-16`), Tailwind's scale removed                                                                        | Only seven values are allowed; the name is the value, so a review can spot a stray one                                                                     |
| 2026-10-16 | Text-safe variants `fg-2-safe`, `danger-text`, pink `accent-text`                                                                                  | The brief's colors fail AA on `raised`, `track` or their tints; the originals stay for fills                                                               |
| 2026-10-16 | Danger buttons are tinted (`danger-tint` + `danger-text`); solid red carries black text                                                            | White on `#FF3B30` is 3.3:1                                                                                                                                |
| 2026-10-16 | A digit that changes again within 0.34 s swaps in place instead of rolling                                                                         | At 8 Hz a tenths digit would stay mid-blur forever and be unreadable                                                                                       |
| 2026-10-16 | Segmented controls and the scope window picker are pill-shaped                                                                                     | Concentric radii with a 2 px inset need a 6 px knob radius, which is not a token; pills nest cleanly                                                       |
| 2026-10-16 | Docks are separate cards in 8 px gutters; separators show only on hover and drag                                                                   | Reads as distinct instruments; no permanent divider lines                                                                                                  |
| 2026-10-16 | The slider shows its value in the header (accent while dragging), no bubble                                                                        | A bubble covered the label row in a dense panel                                                                                                            |
| 2026-10-16 | Command palette matches word prefixes, title above keywords                                                                                        | cmdk's fuzzy scoring ranked "Open component gallery" first for "accent green"                                                                              |
| 2026-10-16 | Workspaces stay mounted after the first visit                                                                                                      | The 3D scene and scope history survive switching; rendering pauses while hidden                                                                            |
| 2026-10-16 | Signal colors are fixed and independent of the UI accent                                                                                           | A trace must keep its meaning when someone changes the accent                                                                                              |
| 2026-10-16 | Lucide (ISC) for general icons, four custom glyphs in the same grid                                                                                | Consistent stroke quality; no unicycle or flashing glyph exists upstream                                                                                   |
| 2026-10-16 | The demo engine is labeled in the status bar and its tooltip                                                                                       | No screen may suggest real firmware is running before M1                                                                                                   |
| 2026-10-16 | Empty workspaces name their milestone and show an illustrative preview                                                                             | They explain what will live there; disabled actions say what they wait for                                                                                 |
| 2026-10-16 | The 3D scene uses in-scene Lightformers for reflections, no HDR file                                                                               | Works offline and ships no third-party assets                                                                                                              |
| 2026-10-16 | TypeScript 6.0 and Vitest 4                                                                                                                        | typescript-eslint supports TS < 6.1; Vitest 5 needs Node 22, the dev machines run Node 20.19+                                                              |
| 2026-10-16 | Low-power mode when WebGL has no GPU                                                                                                               | Full quality on SwiftShader used about 9 CPU cores and kept the main thread near 100 % busy; the first Linux CI run timed out on keyboard and DOM actions  |
| 2026-10-16 | Under browser automation, toasts stay until dismissed and the reduced 3D view draws at 5 fps                                                       | A slow runner must not miss a confirmation that came and went; tests need the picture, not smooth motion                                                   |
| 2026-10-16 | Engine selection: real engine inside Tauri, demo everywhere else, `?engine=demo` to force it                                                       | Tests and the web build must not need a Rust toolchain or a firmware image; the desktop window must never show synthetic data as real                      |
| 2026-10-16 | Telemetry batches are binary (f32 columns); status, events and frames are JSON                                                                     | 60 Hz of 19 channels plus poses cost more to parse as JSON than to draw; rare messages stay readable in the devtools                                       |
| 2026-10-16 | Before physics, the engine shows the stock firmware's real outputs on a still wheel                                                                | First light: boot markers, BLE frames, display, LEDs and buzzer are real now; motion waits for the physics model, and the status bar says "no physics yet" |
| 2026-10-16 | The phone panel sends only commands the catalogue marks safe; the others are listed as never sent                                                  | Some commands change calibration or stored settings; the studio must not do that by accident                                                               |
| 2026-10-16 | Channels a run does not model show an overlay, not zero                                                                                            | A zero current trace reads as a measurement                                                                                                                |
| 2026-10-16 | The display replica comes from the profile's `display.toml`; one canvas painter draws the HUD card and the decal on the shell                      | Both show the same LEDs; the firmware blinks by rewriting bits, so mirroring the grid bytes shows every blink without a timer of our own                   |
| 2026-10-16 | Scenarios are the repository's `scenarios/*.toml`, built into the app; their checks post to the timeline when the run ends                         | The studio and `euc sim run` ride and judge the same file                                                                                                  |
| 2026-10-16 | Frame time is the main thread's work per frame (rAF to the end of the rendering update), reported next to the frame interval and the dropped share | WebKit gives whole-millisecond rAF times locked to vsync, so an interval p95 cannot go below 16.7 ms at 60 Hz even when every frame is on time             |
| 2026-10-16 | Orbit keeps the chosen direction but steps back to the follow distance when a rider appears                                                        | The wheel-only distance put the camera inside the rider's legs                                                                                             |

# EUC Studio

[![ci-engine](https://github.com/tarikbc/euc-studio/actions/workflows/ci-engine.yml/badge.svg?branch=main)](https://github.com/tarikbc/euc-studio/actions/workflows/ci-engine.yml)
[![ci-firmware](https://github.com/tarikbc/euc-studio/actions/workflows/ci-firmware.yml/badge.svg?branch=main)](https://github.com/tarikbc/euc-studio/actions/workflows/ci-firmware.yml)
[![app](https://github.com/tarikbc/euc-studio/actions/workflows/ci-app.yml/badge.svg?branch=main)](https://github.com/tarikbc/euc-studio/actions/workflows/ci-app.yml)
[![vendor-guard](https://github.com/tarikbc/euc-studio/actions/workflows/vendor-guard.yml/badge.svg?branch=main)](https://github.com/tarikbc/euc-studio/actions/workflows/vendor-guard.yml)
[![License: PolyForm Noncommercial 1.0.0](https://img.shields.io/badge/license-PolyForm_Noncommercial_1.0.0-blue)](LICENSE)

**Run real electric-unicycle firmware in a 3D simulation with a physical rider. Inspect it, compare it, improve it.**

[![EUC Studio replaying real runs: a Free Ride through the city, a pothole, a cut-out at top speed, and stock against our firmware](docs/assets/video/showcase-loop.webp)](docs/assets/video/showcase.mp4)

<sub>The studio itself, filmed headless, replaying real runs of the stock firmware (26 s). With sound:
[showcase.mp4](docs/assets/video/showcase.mp4). How it is made: [video/](video/README.md).</sub>

EUC Studio is a workbench for people who want better and safer EUC controller firmware. It runs the controller's
actual firmware image on an emulated microcontroller. Motor, battery, wheel, rider and city models supply its inputs.
The simulator helps you inspect behavior under stated conditions. It does not establish real riding safety.

![Free Ride in the generated city: the rider on the simulated Begode Extreme, seen from the Chase camera](app/docs/screenshots/free-ride-city.png)

<sub>Free Ride in the generated city (seed 42): the stock firmware (GW1825007) on the real engine carries the rider
through a left turn at 20 km/h in drive mode, on the neutral pedal preset. Chase camera, with the HUD, the pitch
gauge and the wheel's display replica; replayed from a recording. Some physical parameters are estimates.</sub>

![The firmware inspector: named variables, register bit fields, and a pin to scope](app/docs/screenshots/firmware-inspector.png)

<sub>The firmware inspector: named variables live, register bit fields decoded, and any value pinned to a scope.
Filmed headless the same way as the rest ([video/](video/README.md)).</sub>

> [!WARNING]
> Custom firmware can make a wheel cut out, injure its rider or destroy its power stage. Read [SAFETY.md](SAFETY.md)
> before you build, flash or ride anything from this repository. Today no build of this repository flashes a real
> wheel ([flasher rules](SAFETY.md#flasher-rules)).

## What it is for

- **See the firmware work.** Watch balance, motor control, tiltback and cutoffs react to a rider in 3D, with scopes,
  an event timeline and the firmware's own variables live.
- **Compare stock and custom firmware.** Run both on the same ride, side by side, and find the first moment they differ.
- **Change one thing at a time.** Our own firmware for the Begode Extreme reproduces the stock behavior, quirks
  included (the compat profile), and then fixes them one by one (the clean profile), each fix shown in a scenario.
- **Ride it yourself.** Free Ride puts you on the simulated wheel in an endless generated city, with the keyboard or a
  gamepad, while the real firmware balances it.
- **Grow to other wheels.** Wheels, boards and processors are described as data, so new models and brands can follow.

| Ride | Compare |
| --- | --- |
| ![Ride workspace: scopes show the spike of a pothole](app/docs/screenshots/ride-pothole.png) | ![Compare workspace: stock and our compat build ride the same pothole](app/docs/screenshots/compare-pothole.png) |
| The stock firmware about 1 s after a 5 cm pothole at 20 km/h: the PWM, modulation and phase-current scopes show the spike. Recording playback. | Stock against our compat build on the 20 km/h pothole scenario: run B rides as a ghost, the charts are synced, and the comparison is within tolerance so far. |

## How it works

```
 firmware image ─▶ emulated MCU (ARMv7-M core + peripheral models)
                        │ PWM ▲ currents, halls, voltages, IMU
                        ▼     │
                   inverter ─▶ motor ─▶ battery          (electrical plant, 5 µs steps)
                        │ torque ▲ wheel speed
                        ▼        │
              wheel, tyre, shell, human rider, ground    (MuJoCo, 0.5 ms steps; flat floor,
                        │                                 terrain strip or streamed city)
                        ▼
          EUC Studio app · CLI `euc` · Python bindings
```

Everything runs on one deterministic clock counted in microcontroller cycles, so every run can be replayed exactly.
Our firmware can also run as a host library (SIL) in place of the emulated image, with its quirks switched at run
time. See [docs/architecture.md](docs/architecture.md).

The first wheel is the **Begode Extreme**. Its stock firmware boots unchanged in the Rust engine.
The simulator reproduces the observed interfaces. Unknown hardware values remain explicit estimates.

## Status

Updated 2026-10-23. Open work lives on the public [EUC Studio Roadmap](https://github.com/users/tarikbc/projects/3)
board, grouped by [milestone](https://github.com/tarikbc/euc-studio/milestones).
[docs/project-status.md](docs/project-status.md) has the evidence and the checks for each area.

| Milestone | State | Done | Open |
| --- | --- | --- | --- |
| M0 Foundation | Done | Repository, CI, vendor guard, pinned stock fetch, image gate | Keep the gates for new features |
| M1 Stock boots | Done | Stock firmware boots, sends telemetry and answers app commands faster than real time | Broader hardware calibration |
| M2 Stock rides in 3D | Done | Electrical plant, MuJoCo body, rear suspension linkage, terrain, display, lights, sound, Ride workspace | Measured motor, battery and suspension values ([calibration](https://github.com/tarikbc/euc-studio/milestone/5)) |
| M3 Own CPU core | Done | Own ARMv7-M core, lock-step checks against Unicorn, a selectable per-instruction flash timing model | Exact peripheral timestamps inside a quantum; Tune memory writes and gain edits |
| M4 Human rider | Most done | Articulated rider (the default), opt-in contact feet and lateral rider (L1-L3), 26 scenarios, `.eucrec` recordings, Python bindings, Free Ride | Rider balance without external aids ([lateral rider L4-L6](https://github.com/tarikbc/euc-studio/milestone/3)), measured human response |
| M5 Our firmware | Most done | Compat and clean images boot and ride; the SIL library switches every quirk at run time ([SIL](docs/sil.md)) | The 50 % charge full-effort difference ([#42](https://github.com/tarikbc/euc-studio/issues/42)), [firmware findings](https://github.com/tarikbc/euc-studio/milestone/6), hardware validation |
| M6 Flasher | Simulator only | Gated flasher against a simulated wheel with an emulated bootloader; 320-run fault campaign with no unrecoverable run ([spec](docs/spec/flasher.md)) | Image signing, the emulator ride of gate 3, bootloader facts from the SWD dump, the app wizard; no real flash ([M6](https://github.com/tarikbc/euc-studio/milestone/7)) |
| M7 Release | Started | Desktop source builds, a local macOS package ([guide](docs/local-macos-install.md)), the wheel onboarding guide | Signed bundles, docs site, browser demo, a second wheel ([M7](https://github.com/tarikbc/euc-studio/milestone/8)) |
| M8 Bench | Planned | The safety procedure only | Controlled bench work and trace comparison ([M8](https://github.com/tarikbc/euc-studio/milestone/9)) |

Current work:

| Track | State | Open |
| --- | --- | --- |
| W2 Scenarios in the city | Done (2026-10-23) | The shipped scenarios ride the generated city ([world model](docs/models/world.md), [#11](https://github.com/tarikbc/euc-studio/issues/11)) |
| P1 Physics realism bundle | Draft [PR #62](https://github.com/tarikbc/euc-studio/pull/62) | A rubber tyre at 25 psi with rolling resistance, a finer city grid, the motor fit, the roll fix, and kerb absorb and brace are in the draft ([epic #57](https://github.com/tarikbc/euc-studio/issues/57)). The open step is the re-baseline of every scenario, [#18](https://github.com/tarikbc/euc-studio/issues/18): a good task for a contributor |
| Lateral rider L4-L6 | L1-L3 done | The rider balances and turns with its own body instead of external aids ([epic #58](https://github.com/tarikbc/euc-studio/issues/58)) |
| Free Ride polish | Play mode, drive mode, Chase camera, daylight, sound and HUD done | Impact sounds, city look up close, buildings, trees, a minimap ([milestone](https://github.com/tarikbc/euc-studio/milestone/4)) |
| Calibration | Open | Tools and measurements that tie the simulator to real wheels ([epic #59](https://github.com/tarikbc/euc-studio/issues/59)) |

Plans and limits per milestone: [docs/roadmap.md](docs/roadmap.md). All documents: [docs/README.md](docs/README.md).

## Quick start

You need:

- Rust through [rustup](https://rustup.rs). The toolchain version is pinned in `rust-toolchain.toml`; rustup installs it
  on the first `cargo` command.
- Python 3.11 or newer.
- For the desktop app: Node 20.19 or newer, pnpm 10, and the
  [Tauri prerequisites](https://v2.tauri.app/start/prerequisites/) for your OS.
- For our firmware: CMake 3.25 or newer, Ninja, and the Arm GNU Toolchain (see [firmware/README.md](firmware/README.md)).

The engine and the CLI build on macOS, Linux and Windows. The BLE radio features run on macOS only.

```sh
git clone https://github.com/tarikbc/euc-studio && cd euc-studio
git config core.hooksPath tools/hooks          # once per clone: the vendor guard runs before each commit
python3 tools/fetch-mujoco/fetch.py            # once per clone: the pinned MuJoCo library the physics links

cargo run --release -p euc-cli -- fetch begode-extreme   # download and verify the official Begode Extreme build
```

Run a scenario on the stock firmware. Motor, battery, body, rider and city are simulated, and the scenario's checks
are judged at the end:

```sh
cargo run --release -p euc-cli -- sim run --scenario scenarios/ride-20kmh-stop.toml
```

Build our firmware, then ride it and compare it with stock:

```sh
(cd firmware && cmake --workflow --preset target-compat && cmake --workflow --preset target-clean)
cargo run --release -p euc-cli -- sim run --scenario scenarios/ride-20kmh-stop.toml --firmware clean
cargo run --release -p euc-cli -- sim compare --scenario scenarios/ride-20kmh-stop.toml --b compat \
    --tolerance scenarios/compare/compat-vs-stock.toml
```

Start the desktop studio (the first build takes several minutes):

```sh
cd app && pnpm install && pnpm tauri dev
```

More: `cargo run -p euc-cli -- --help` lists every command, [docs/scenarios/writing-scenarios.md](docs/scenarios/writing-scenarios.md)
explains the scenario files, [docs/compare.md](docs/compare.md) the comparison, [docs/python.md](docs/python.md) the
Python API, and [docs/spec/flasher.md](docs/spec/flasher.md) the simulated flasher (`euc flash --sim --synthetic`).
The checks for each area are in [CONTRIBUTING.md](CONTRIBUTING.md#checks-per-area).

## Free Ride

In the desktop app, open the Ride workspace and press **Free ride** above the 3D view. The ride starts at once in the
generated city: the firmware boots (about 1.5 s of simulated time) and the keys work. A crash never pauses; press R
to respawn.

| Key | Gamepad | Action |
| --- | --- | --- |
| W / ↑, S / ↓ | Left stick, RT / LT | Ride forward, brake |
| A / ←, D / → | Left stick | Turn |
| Space (hold) | LB (hold) | Crouch; at speed also a tuck (less drag) |
| X | B | Step off |
| R, Shift+R | X, Back | Respawn here, at the start |
| P (hold), H (hold) | | The wheel's power and handle buttons |
| L, B, M | | Light, horn, pedal mode |
| Esc | Start | Pause, go on |
| C | Y | Camera |

Keys can be rebound in the panel. The full reference, including drive mode and lean mode, is in
[app/src/workspaces/freeRide/README.md](app/src/workspaces/freeRide/README.md).
Free Ride is a simulation for firmware work, not a riding trainer.

## How work is organised

Open work lives as issues on the public [roadmap board](https://github.com/users/tarikbc/projects/3), grouped into
[milestones](https://github.com/tarikbc/euc-studio/milestones); a large topic has an epic issue that lists its parts. Comment on an issue before you
start larger work, so the plan is agreed first. Every change goes in as a pull request: CI runs the checks of the
areas it touches (engine, firmware, app, Python bindings, the stock-against-compat comparison and the vendor guard),
every commit carries a DCO sign-off (`git commit -s`), and a safety-relevant change brings its scenario evidence.
[CONTRIBUTING.md](CONTRIBUTING.md) has the details.

## Where to start

- Look at the [roadmap board](https://github.com/users/tarikbc/projects/3) and the
  [good first issues](https://github.com/tarikbc/euc-studio/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22):
  trees and vegetation in the city ([#29](https://github.com/tarikbc/euc-studio/issues/29)), smooth route steering
  ([#31](https://github.com/tarikbc/euc-studio/issues/31)), the IMU's low-pass filter
  ([#39](https://github.com/tarikbc/euc-studio/issues/39)), README visuals
  ([#51](https://github.com/tarikbc/euc-studio/issues/51)) and 3D model follow-ups
  ([#56](https://github.com/tarikbc/euc-studio/issues/56)).
  Issues labelled [help wanted](https://github.com/tarikbc/euc-studio/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22)
  and [needs: data](https://github.com/tarikbc/euc-studio/issues?q=is%3Aissue+is%3Aopen+label%3A%22needs%3A+data%22)
  are open to anyone with the skills or a wheel to measure.

Skills that help, and where they fit:

| You know | Where it helps | Labels |
| --- | --- | --- |
| Rust, CPU emulation, microcontroller peripherals | The ARMv7-M core, STM32F1 peripherals, engine speed (`crates/euc-armv7m`, `crates/euc-stm32f1`, `crates/euc-sim`) | [`area: engine`](https://github.com/tarikbc/euc-studio/labels/area%3A%20engine) |
| MuJoCo, rigid-body physics, biomechanics | Tyre, suspension, the rider models and their controllers (`crates/euc-physics`) | [`area: physics`](https://github.com/tarikbc/euc-studio/labels/area%3A%20physics), [`area: tyre`](https://github.com/tarikbc/euc-studio/labels/area%3A%20tyre), [`area: rider`](https://github.com/tarikbc/euc-studio/labels/area%3A%20rider) |
| Motors, inverters, batteries | The electrical plant and its calibration (`crates/euc-plant`) | [`area: plant`](https://github.com/tarikbc/euc-studio/labels/area%3A%20plant), [`area: calibration`](https://github.com/tarikbc/euc-studio/labels/area%3A%20calibration) |
| Embedded C, motor control | Our firmware's compat and clean profiles (`firmware/`) | [`area: firmware`](https://github.com/tarikbc/euc-studio/labels/area%3A%20firmware) |
| three.js, React, TypeScript | The studio, Free Ride and the city's look (`app/`) | [`area: studio`](https://github.com/tarikbc/euc-studio/labels/area%3A%20studio), [`area: world`](https://github.com/tarikbc/euc-studio/labels/area%3A%20world), [`area: model`](https://github.com/tarikbc/euc-studio/labels/area%3A%20model) |
| Audio synthesis | Free Ride's sound, made from the simulator's signals (`app/src-tauri/src/engine/ride_sound.rs`) | [`area: sound`](https://github.com/tarikbc/euc-studio/labels/area%3A%20sound) |
| Python, data analysis | Comparing simulated runs with real ride logs (`python/`; the tool is [#35](https://github.com/tarikbc/euc-studio/issues/35)) | [`area: calibration`](https://github.com/tarikbc/euc-studio/labels/area%3A%20calibration) |
| A Begode wheel (or another EUC) | Measurements and ride numbers that check the simulator | [`needs: data`](https://github.com/tarikbc/euc-studio/labels/needs%3A%20data) |
- Read [CONTRIBUTING.md](CONTRIBUTING.md) for the ground rules, the area map and the checks per area.
- Read [docs/architecture.md](docs/architecture.md), then the guide of the area you work on
  ([docs/README.md](docs/README.md) maps every document).
- Own a wheel? Measurements and ride data help most. Use the
  [real-wheel data](https://github.com/tarikbc/euc-studio/issues/new?template=real-wheel-data.yml) issue form, and
  never post GPS tracks or raw logs that show where you ride.

## Stock firmware is never in this repository

`euc fetch <wheel>` downloads the official image from the manufacturer's own update server to your machine and checks
it against the sha256 pins built into the tool. Nothing from the vendor image is stored here, and a
[guard](tools/vendor-guard/README.md) blocks it on every commit and in CI.

## Repository

| Path | What |
| --- | --- |
| [`crates/`](crates) | Rust engine: CPU core, peripherals, plant, physics, world, recordings, flasher, CLI `euc`, Python bindings |
| [`firmware/`](firmware) | Our controller firmware in C11 (compat and clean profiles, SIL library) |
| [`app/`](app) | The EUC Studio desktop app (Tauri, React, three.js) and its design system |
| [`scenarios/`](scenarios) | Scenario files with their checks, and the compare tolerances |
| [`profiles/`](profiles) | Wheel definitions: board, plant, geometry, probes, stock firmware pins |
| [`python/`](python) | The `euc_studio` Python package and its tests |
| [`docs/`](docs) | Architecture, roadmap, specifications, model notes, findings ([index](docs/README.md)) |
| [`tests/data/`](tests/data) | Real-wheel captures used as test data |
| [`tools/`](tools) | Vendor guard, git hooks, MuJoCo fetch |

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). In short: sign off your commits (`git commit -s`), never add vendor material,
and back every safety-relevant change with a scenario. Report a behavior that could hurt a rider privately, as
[SAFETY.md](SAFETY.md#reporting-a-safety-problem) explains.

## License

EUC Studio is **source-available** under the [PolyForm Noncommercial License 1.0.0](LICENSE). You may use, study,
change and share it for any noncommercial purpose. Commercial use is not allowed. See [NOTICE](NOTICE) for
third-party components and trademarks.

EUC Studio is an independent project and is not affiliated with, endorsed by or sponsored by Begode or any other EUC
manufacturer.

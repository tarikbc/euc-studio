# Documentation

Every document in the repository, by topic. New here? Read the [README](../README.md),
[CONTRIBUTING.md](../CONTRIBUTING.md) and [SAFETY.md](../SAFETY.md), then [Architecture](architecture.md).

Many model notes are dated evidence reports: they keep the numbers of the day they were measured. The date or the
base commit at the top of a report says which build it describes. [Project status](project-status.md) says what is
true now.

## Start here

| Document | What it covers |
| --- | --- |
| [README](../README.md) | What EUC Studio is, status, quick start, Free Ride keys |
| [CONTRIBUTING.md](../CONTRIBUTING.md) | Ground rules, how to pick an issue, area map, checks per area, commits |
| [SAFETY.md](../SAFETY.md) | Safety rules, flasher rules, how to report a safety problem |
| [Architecture](architecture.md) | Engine, plant, body, world, firmware, and how they connect |
| [Project status](project-status.md) | Current behavior and open work per area, with the evidence |
| [Roadmap](roadmap.md) | Milestones M0 to M8, the current tracks and the next steps |
| [NOTICE](../NOTICE) | Third-party components, attribution and trademarks |

## Use the tools

| Document | What it covers |
| --- | --- |
| [Writing a scenario](scenarios/writing-scenarios.md) | The scenario TOML format: rider, settings, terrain, city world, faults, checks |
| [Comparing firmware](compare.md) | `euc sim compare`: two firmwares on the same inputs, tolerances, the Compare workspace |
| [Software in the loop](sil.md) | Our firmware as a host library, run-time quirks, `ours-sil` |
| [Recordings](recordings.md) | The `.eucrec` format, `euc rec`, playback in the studio |
| [Python API](python.md) | The `euc_studio` package: build, sessions, scenarios, recordings |
| [BLE bridge](bridge.md) | `euc sim serve`: the simulated wheel's BLE link over TCP, settings presets |
| [BLE radio](ble-radio.md) | The simulated or replayed wheel on this Mac's Bluetooth radio (macOS) |
| [Local macOS app](local-macos-install.md) | Build, check and install an ad-hoc signed `.app` |
| [Adding a wheel](adding-a-wheel.md) | The path to a second wheel, brand or processor, with evidence checklists |

## Desktop app

| Document | What it covers |
| --- | --- |
| [app/README.md](../app/README.md) | Run and check the studio, its workspaces and layout |
| [app/DESIGN.md](../app/DESIGN.md) | Design system, motion rules and the decisions log |
| [Free Ride README](../app/src/workspaces/freeRide/README.md) | Play mode, keys, gamepad, drive and lean modes, sound, native API |
| [Free Ride design](design/free-ride.md) | The Free Ride redesign phases (FR1 to FR9) and their decisions |
| [Tune scope](../app/src/workspaces/tune/SCOPE.md), [Tune extensions](../app/src/workspaces/tune/EXTENSIONS.md) | The Tune workspace: variables, scopes, triggers, ISR estimates |
| [3D model](../app/public/models/begode-extreme/README.md) | The Begode Extreme model files and their nodes |
| [e2e fixtures](../app/e2e/fixtures/README.md) | Test fixtures of the browser suites |
| [app/THIRD_PARTY.md](../app/THIRD_PARTY.md) | Licenses of the app's packages and fonts |

## Engine and models

| Document | What it covers |
| --- | --- |
| [CPU core](../crates/euc-armv7m/README.md) | Our ARMv7-M interpreter, its tests and lock-step checks |
| [CPU timing](models/cpu-timing.md) | Fixed and per-instruction flash timing, validation against a real wheel |
| [Electrical plant](models/electrical.md) | Inverter, motor, halls, current sensing, battery, thermal |
| [Battery validity](models/battery-validity.md) | Where the battery model holds and where a run stops |
| [Controller supply](models/controller-supply.md) | Controller power rail switching while the pack and the world go on |
| [Body physics](models/body.md) | Wheel, tyre, suspension, the rider models and their controllers |
| [Procedural world](models/world.md) | The seeded city: generator, streaming, physics ground, scenario spots |
| [Coverage matrix](models/coverage-matrix.md) | Which features exist and which physical claims are validated |
| [crates/THIRD_PARTY.md](../crates/THIRD_PARTY.md) | Licenses of the engine's Rust crates |

## Evidence reports (dated)

| Document | What it covers |
| --- | --- |
| [Scenario results](models/scenarios.md) | What the stock firmware does in each shipped scenario |
| [Full-effort acceptance](models/full-effort-acceptance.md) | 70 km/h acceleration and braking at three charge levels |
| [SOC50 investigation](models/soc50-investigation.md), [high-speed follow-up](models/soc50-high-speed-investigation.md), [full-run equivalence](models/soc50-full-run-equivalence.md) | Why compat and stock part ways at 50 % charge |
| [Clean cut-off edge cases](models/clean-cutoff-next.md) | Decisions and evidence for the clean profile's cut-offs |
| [Articulated rider acceptance](models/articulated-acceptance.md) | The articulated rider and contact feet |
| [Free Ride acceptance](models/free-ride-acceptance.md) | Native Free Ride checks: play mode, drive mode, respawn |
| [Recording acceptance](models/recording-acceptance.md) | Capture, replay and import limits |
| [Tune acceptance](models/tune-acceptance.md) | Native Tune checks |
| [Ride-log analysis](models/ride-log-analysis.md) | What one real ride says about the plant: speed scale, extra draw, current reading, battery, headroom |
| [Generated map](models/generated-map-implementation.md) | The finite 32 m test pad that preceded the city (old recordings) |
| [Human and Free Ride plan](models/human-and-free-ride.md) | The 2026-10-16 plan for the human model and Free Ride (historical) |

## Hardware, protocol and specifications

| Document | What it covers |
| --- | --- |
| [Body geometry](hardware/geometry.md) | Sizes and layout of the Begode Extreme's body, pads and collision shapes |
| [Suspension linkage](hardware/suspension-linkage.md) | The rear linkage readings and their numbers |
| [Front display](hardware/display.md) | The display board and its replica |
| [Tail light](hardware/lights.md) | The tail light and LED strip |
| [Begode protocol](protocol/begode.md) | The BLE serial protocol between wheel and phone |
| [Stock firmware findings](findings/stock-firmware.md) | Behaviors of the stock firmware found in the simulator |
| [Image info block](spec/euc-info.md) | The `.euc_info` block in our images, sealing |
| [Flasher](spec/flasher.md) | The gated OTA flasher, its gates, the emulated bootloader, the fault campaign |
| [Flash simulation](spec/flash-simulation.md) | The application-slot flash model under the flasher |

## Firmware

| Document | What it covers |
| --- | --- |
| [firmware/README.md](../firmware/README.md) | Layout, build presets, image layout, profiles and quirks, fail-closed rules |
| [Cut-offs and protections](../firmware/docs/cutoffs.md) | Every stop, limit and warning, stock and ours |
| [Quirks](../firmware/docs/quirks.md) | The stock behaviors compat keeps and clean fixes (generated) |
| [Riderless detection](../firmware/docs/riderless-detection.md) | The clean profile's empty-wheel detection |
| [Watchdog and over-current](../firmware/docs/watchdog-and-overcurrent.md) | The clean profile's watchdog and over-current trip |

## Python and tools

| Document | What it covers |
| --- | --- |
| [python/README.md](../python/README.md), [python/DESIGN.md](../python/DESIGN.md) | The Python package and its design |
| [Vendor guard](../tools/vendor-guard/README.md) | What the guard blocks and allows, how to run it |
| [MuJoCo fetch](../tools/fetch-mujoco/README.md) | The pinned MuJoCo download |
| [Ride-log compare](../tools/ride-log/README.md) | A real ride log against a simulated run, per speed band |
| [Test data](../tests/data/begode-extreme/README.md) | Real-wheel captures used as test data |

## Historical plans and reviews

Kept for their decisions and evidence. They describe the state of their date, not the current one.

- [Articulated rider plan](history/2026-10-16-articulated-rider.md) (2026-10-16)
- [Project completion plan](history/2026-10-16-project-completion.md) (2026-10-16)
- [Desktop change review](history/code-review-2026-10-16-desktop-ux.md) (2026-10-16)

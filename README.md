# EUC Studio

**Run real electric-unicycle firmware in a 3D simulation with a physical rider. Inspect it, compare it, improve it.**

EUC Studio is a workbench for people who want better and safer EUC controller firmware. It runs the controller's
actual firmware image on an emulated microcontroller, wired to models of the motor, the battery, the wheel and a full
human rider, so you can watch what the firmware does in every situation before anyone rides it.

![EUC Studio, Ride workspace](app/docs/screenshots/ride.png)

<sub>Milestone M0: the app shell and design system run on a demo engine with synthetic telemetry. The real engine
lands in the next milestones.</sub>

> [!WARNING]
> Custom firmware can make a wheel cut out, injure its rider or destroy its power stage. Read [SAFETY.md](SAFETY.md)
> before you build, flash or ride anything from this repository.

## What it is for

- **See the firmware work.** Watch balance, motor control, tiltback and cutoffs react to a rider in 3D, with scopes,
  an event timeline and the firmware's own variables live.
- **Compare stock and custom firmware.** Run both on the same ride, side by side, and find the first moment they differ.
- **Change one thing at a time.** Our own firmware for the Begode Extreme reproduces the stock behavior, quirks
  included, and then fixes them one by one, each fix shown in a scenario.
- **Grow to other wheels.** Wheels, boards and processors are described as data, so new models and brands can follow.

## How it works

```
 firmware image ─▶ emulated MCU (ARMv7-M core + peripheral models)
                        │ PWM ▲ currents, halls, voltages, IMU
                        ▼     │
                   inverter ─▶ motor ─▶ battery          (electrical plant, 5 µs steps)
                        │ torque ▲ wheel speed
                        ▼        │
                   wheel, tire, shell, human rider       (MuJoCo, 0.5 ms steps)
                        │
                        ▼
          EUC Studio app · CLI `euc` · Python bindings
```

Everything runs on one deterministic clock counted in microcontroller cycles, so every run can be replayed exactly.
See [docs/architecture.md](docs/architecture.md).

The first wheel is the **Begode Extreme**. Its stock firmware already boots unchanged in our prototype emulator, and we
have mapped how every part of it behaves; the engine in this repository is the production version of that work.

## Status

| Milestone | Goal | State |
|---|---|---|
| M0 Foundation | Repo, CI, vendor guard, engine core traits, firmware fetch, firmware skeleton, app shell | ✅ done |
| M1 Stock boots | Stock firmware boots in the Rust engine | ✅ done: 2.3× real time, byte-identical to the reference emulator |
| M2 Stock rides in 3D | Motor, battery and body physics, 3D viewport | next |
| M3 Own CPU core | ARMv7-M interpreter verified against a reference emulator | |
| M4 Human rider | Full rider, recordings, Python, scenarios | |
| M5 Our firmware | Complete C firmware, compat and clean profiles | |
| M6 Flasher | BLE flasher with safety gates | |
| M7 Release | Desktop builds, docs site, a second wheel | |

Details: [docs/roadmap.md](docs/roadmap.md).

## Stock firmware is never in this repository

`euc fetch <wheel>` downloads the official image from the manufacturer's own update server to your machine and checks
it against the sha256 pins built into the tool. Nothing from the vendor image is stored here, and a
[guard](tools/vendor-guard/README.md) blocks it on every commit.

## Try it

You need Rust (the version is pinned in `rust-toolchain.toml`), and for the other parts the tools listed in each
area's README.

```sh
git config core.hooksPath tools/hooks             # once per clone: the vendor guard runs before each commit

cargo run -p euc-cli -- fetch begode-extreme       # download and verify the official Begode Extreme build
cargo run -p euc-cli -- firmware list
cargo run -p euc-cli -- image validate <file>      # the image gate the flasher will use

# run the stock firmware in the simulator (development backend, needs CMake to build Unicorn)
cargo run --release -p euc-cli --features unicorn -- sim run begode-extreme --seconds 10 --inject V@2.6

(cd firmware && cmake --workflow --preset target-clean)  # our firmware image (Arm GNU Toolchain)
(cd app && pnpm install && pnpm tauri dev)                # the EUC Studio app (Node, pnpm, Tauri)
```

## Repository

| Path | What |
|---|---|
| [`crates/`](crates) | Rust engine, firmware sources, image tools, CLI `euc` |
| [`firmware/`](firmware) | our controller firmware in C11 |
| [`app/`](app) | the EUC Studio desktop app (Tauri, React) and its design system |
| [`profiles/`](profiles) | wheel definitions |
| [`docs/`](docs) | architecture, roadmap, specifications |
| [`tests/data/`](tests/data) | real-wheel captures used as test data |
| [`tools/`](tools) | vendor guard and git hooks |

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). In short: sign off your commits, never add vendor material, and back every
safety-relevant change with a scenario.

## License

EUC Studio is **source-available** under the [PolyForm Noncommercial License 1.0.0](LICENSE). You may use, study,
change and share it for any noncommercial purpose. Commercial use is not allowed. See [NOTICE](NOTICE) for
third-party components and trademarks.

EUC Studio is an independent project and is not affiliated with, endorsed by or sponsored by Begode or any other EUC
manufacturer.

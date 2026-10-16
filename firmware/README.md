# Firmware

A clean C11 rewrite of the Begode Extreme controller firmware. One CMake tree builds it two ways:

- **target**: an image for the controller MCU (STM32F103RC class, Cortex-M3, no FPU), linked behind the
  vendor bootloader at `0x08003000`, so the stock OTA update path keeps working;
- **host**: unit tests, and a software-in-the-loop (SIL) shared library that the simulator loads in place
  of the emulated image.

Status: milestone M1, steps F1 to F5. Core has the BLE protocol (`core/protocol/`, described in
[docs/protocol/begode.md](../docs/protocol/begode.md), tested against a real capture), the balance task:
attitude estimation (`core/attitude/`), balance law, tiltbacks, cut-offs and recovery (`core/balance/`), with
our own fdlibm-derived math (`core/math/`) so host and target compute the same bits, the motor drive
(`core/motor/`): hall decoding, phase-current sensing and integer field-oriented control for both drive modes,
and the services around them: settings and odometer with the flash journal and its backup-register mirror
(`core/settings/`), beeper and alarms (`core/alarms/`), power latch and buttons (`core/power/`), the PB12 light
(`core/lights/`), the BMS link (`core/bms/`), the slow analog sensors (`core/sensors/`), the display content
(`core/display/`) and the BLE service sequences (`core/service/`). The FOC lookup tables are computed from
their formulas at build time (`tools/gen_motor_tables.py`). `ports/stm32f1/` drives the MCU at register level
and `apps/begode-extreme/` wires everything into the stock execution contexts, so the image boots in the
simulator: it sets the power latch, answers `V` and `N`, sends telemetry and handles `!@`, `cy` and `+-`
(`euc sim run begode-extreme --firmware build/target-compat/euc-fw-begode-extreme.bin`). The LED strip driver
runs, but the stock strip patterns are not reproduced yet: the strip stays dark. It is not meant for a wheel:
nothing in this tree is flashed before the gates in the project plan (sim dry-run, stock-first flash, SWD
backup) are in place.

## Layout and layer rules

| Directory | Holds | Rule |
|---|---|---|
| `core/` | control logic: math, protocol, attitude, balance, FOC, settings | Pure C. No HAL calls, no mutable globals: each module takes an `in` struct and a `state` struct and fills an `out` struct. Builds unchanged for target and host. |
| `ports/stm32f1/` | reset path, vector table, clock, register-level drivers (`include/stm32f1/port.h`) | Own register definitions (`regs.h`) on CMSIS-Core, written from RM0008; no ST SPL or ST motor-control library code. Core never includes it; the app passes driver results into core. |
| `ports/sim/` | SIL library: C ABI for the simulator engine | Owns the run-time quirk mask; the simulator drives the core modules through it. |
| `boards/<board>/` | `board.h` constants, `link_app.ld` memory map, stock tuning | Facts about one PCB. No code. |
| `apps/<board>/` | `main`, init order, task table (TIM8 / ADC / RTC / main-loop contexts) | Composes core modules with a port and a board, and schedules them. |
| `config/quirks.def` | compat / clean quirk list | See [Profiles and quirks](#profiles-and-quirks). |
| `tests/unit/` | Unity tests, host only | One executable per area, run by CTest. Shared captures live in `../tests/data/`. |
| `tools/` | size report, quirk doc generator, FOC table generator, simulator probe generator, cppcheck deviation record | Python standard library only. |

Code style: fixed-width types, units in names (`_bytes`, `_ms`, `_q15`), no heap, no recursion, no dead code,
comments that say why.

## Build

Tools: CMake 3.25 or newer (3.27 for clang-tidy), Ninja, Python 3.11+, and for target builds the Arm GNU
Toolchain (tested: 15.3.rel2). The toolchain file finds it through `ARM_GNU_TOOLCHAIN_DIR`, then
`~/Applications/arm-gnu-toolchain/<release>/`, then `PATH`. CMSIS 6 Core (Apache-2.0) and Unity (MIT) are
fetched at configure time from pinned release archives with sha256 checks.

```sh
cd firmware
cmake --workflow --preset target-compat   # build/target-compat/euc-fw-begode-extreme.{elf,bin,map}
cmake --workflow --preset target-clean    # build/target-clean/euc-fw-begode-extreme.{elf,bin,map}
cmake --workflow --preset host-debug      # unit tests with AddressSanitizer + UBSan
cmake --workflow --preset host-sil        # build/host-sil/libeuc_sil.{dylib,so} + tests
```

| Preset | What | Notes |
|---|---|---|
| `target-compat` | image, compat profile | `-Os`, `-ffunction-sections -fdata-sections`, `--gc-sections`, `-nostdlib` + libgcc |
| `target-clean` | image, clean profile | same flags |
| `host-debug` | unit tests | Debug, `-fsanitize=address,undefined` |
| `host-sil` | SIL shared library + tests | RelWithDebInfo, runtime quirk profile |

Every build uses `-ffp-contract=off` (no fused multiply-add), so host and target round floats the same way,
and `-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror` plus a few more.

## Image layout

`boards/begode-extreme/link_app.ld` is the single source of the memory map and checks it at link time.

| Image offset | Address | Content |
|---|---|---|
| `0x000` | `0x08003000` | vector table, 76 entries (16 Cortex-M3 + 60 IRQs) |
| `0x130` | `0x08003130` | 16 bytes of zero padding |
| `0x140` | `0x08003140` | `.euc_info`, 128 bytes ([spec](../docs/spec/euc-info.md)) |
| `0x1C0` | `0x080031C0` | `.text`, `.rodata`, initial values of `.data` |
| ≤ 49,152 B | up to `0x0800EFFF` | the settings page at `0x0800F000` must never be reached |

RAM: 20 KB assumed, `.data` + `.bss` from `0x20000000`, 2 KB main stack below `0x20005000`, no heap.

The build writes every `.euc_info` field except `image_len`, `crc32` and `signature` (`euc image seal` fills
those later). Version comes from `project(VERSION)` in `CMakeLists.txt`, the git sha from the source tree at
configure time, the profile from the preset; the dev flag is set while images are unsigned.

## Profiles and quirks

`config/quirks.def` lists every stock behaviour we reproduce only so our image can run in lock-step with the
stock binary (the table is in [docs/quirks.md](docs/quirks.md); every cut-off and protection, stock and
clean, is in [docs/cutoffs.md](docs/cutoffs.md)). Code tests one with:

```c
if (EUC_QUIRK(ASIN_ACCEL)) { /* stock behaviour */ } else { /* clean behaviour */ }
```

- `target-compat` and `target-clean`: `EUC_QUIRK(ID)` is a compile-time constant, so the unused branch is
  not in the image.
- `host-sil`: `EUC_QUIRK(ID)` reads a mask the simulator sets through `euc_sil_quirk_set_mask()`, so the
  studio can switch one quirk at a time.

Neither profile changes telemetry frame bytes, command letters, the `!@` update entry, the flash map or
USART1. After editing the list, run `cmake --build --preset <any> --target quirks-doc`.

## Fail-closed rules

Lock-step fidelity never outranks a safe actuator. The balance task and the motor drive follow these rules
(`tests/unit/test_failsafe.c` injects NaN and infinity at every balance stage; `tests/unit/test_motor.c`
covers the motor gates):

- **Actuator boundary, both profiles.** The motor command the current loop reads (`motor_command`) is always
  finite and within the soft-start limit; a NaN or infinity is published as 0. This is a deliberate difference
  from the stock firmware, also in compat. Finite commands pass unchanged, so normal-path lock-step stays
  bit-exact; the lock-step harness reports NaN events separately instead of comparing them.
- **Clean profile (quirk `NAN_FAIL_OPEN` off).** A NaN or infinity in the attitude filter, the law, the
  ramp-down or the output stage sets the command to 0 at once, resets the attitude filter, keeps the motor off
  until power-off (no automatic recovery) and latches `numeric_fault` for the alarm task and telemetry. The
  pitch sine is clamped to ±1 before `asin`.
- **Clamps.** NaN compares false, so `if (limit < x) x = limit` lets a NaN through and
  `if (!(x <= limit)) x = limit` turns it into full power. Check finiteness first, then clamp.
- **Ride settings over BLE.** Clean applies settings that change riding behaviour only at standstill
  (quirk `SETTINGS_WHILE_MOVING`); the stock BLE link has no authentication.
- **Motor boundary, both profiles.** The drive checks the command again where it enters
  (`euc_motor_set_command()`: finite, within ±1050). A hall fault, the start-up current-sensor check or a
  system hard cut turns the bridge off until reset, as in the stock firmware.
- **Settings storage.** Clean records carry a CRC-16 in bits the stock firmware never uses, so a torn write
  is skipped, and either firmware can still read the other's journal. A field that no command can produce
  (a record from stock, the backup registers or a damaged slot) loads as its default
  (`RECORD_FIELDS_UNCHECKED`). Clean never erases the journal page
  while the drive is enabled: a page erase stalls the CPU for tens of milliseconds.
- **Faults and hangs.** Any exception without its own handler turns the bridge off (TIM8 main output enable)
  before it parks the core, in both profiles; the stock handler leaves the last PWM running. Clean starts
  the independent watchdog (250 ms) with the first RTC tick and feeds it only from a tick in which the FOC
  ran and the balance published a valid command; the boot after a watchdog reset keeps the bridge off. A
  stalled main loop does not reset the wheel: it latches limp mode and beeps (quirk `NO_WATCHDOG`,
  [docs/watchdog-and-overcurrent.md](docs/watchdog-and-overcurrent.md)).
- **Clean motor drive.** Out-of-range FOC values saturate instead of wrapping (`INT16_WRAP`), and the angle
  holds while the hall state is invalid (`HALL_INVALID_ANGLE_RUN`). The phase over-current trip exists but is
  off in both profiles until its level is measured on the bench (`NO_OVERCURRENT_TRIP` is on in clean too;
  a false trip mid-ride is a fall).
- **One lost hall line.** The stock firmware never notices one stuck hall line: it rides on with a wrong
  angle in two sectors of six and a speed reading a third of the truth. Clean finds the silent line within
  three electrical turns, rebuilds the state from the other two lines, raises error bit 0x40 (fault beeps),
  lowers the speed tiltback to 10 km/h and allows no automatic restart after the next stop
  (`HALL_SINGLE_LINE_UNDETECTED`). The 62.5 ms invalid-state fault stays on the lines as read.
- **Gyro range.** Stock sets the MPU-6500 gyro to +-500 deg/s; a pothole edge turns the shell faster, the gyro
  clips and the attitude can read level while the shell is 24 deg nose down. Clean uses +-2000 deg/s (0.061
  instead of 0.015 deg/s per count; calibration pages stay in +-500 deg/s counts) and, if an axis still reaches
  its rail, re-seeds the low-passed accelerometer and raises the correction gain for 1 s
  (`GYRO_500DPS_CLIP`).
- **Service sequences.** `!@`, `cy` and `+-` come from an unpaired BLE link. Clean runs one only when nobody
  rides (the balance is not running) and the power button gets a short press within 10 s; it replies
  `SERVICE:CONFIRM`, `SERVICE:OK` or `SERVICE:DENIED` (`SERVICE_WITHOUT_CONFIRM`, docs/protocol/begode.md).

## Size budget

After each target link, `tools/size_report.py` reads the map file, charges every input section to its
module, writes `size-report.json` next to the image and checks `size-budget.toml`: a warning above 44 KiB of
flash, a failed build above 48 KiB of flash or 16 KiB of static RAM (`.data` + `.bss`).

## Simulator probes

`profiles/begode-extreme/probes.toml` tells the simulator where our firmware keeps the quantities that the stock
map names (`speed_cms`, `alerts`, the cut-off flags, ...), so scenario checks and `euc sim compare` read both
firmwares the same way. `tools/gen_probes.py` writes it from `offsetof()` compiled by the target compiler; the
image build fails when the file no longer matches `struct app`. After a change, run
`cmake --build --preset target-clean --target probes`.

## Style and static analysis

| Command (from `firmware/`) | Does |
|---|---|
| `cmake --build --preset host-debug --target format-check` | clang-format check (`.clang-format`: LLVM based, 110 columns) |
| `cmake --build --preset host-debug --target format` | reformat |
| `cmake --build --preset <preset> --target cppcheck` | cppcheck; MISRA C:2012 addon on firmware sources |
| `cmake --preset host-debug -B build/tidy -DEUC_CLANG_TIDY=ON && cmake --build build/tidy` | clang-tidy on every compile (`.clang-tidy`, findings are errors) |

`tools/cppcheck-suppressions.txt` is the MISRA deviation record: each entry names the rule, the file and the
reason. Add a deviation only with a reason, and as narrow as the reason.

## Licences

The firmware is part of EUC Studio (PolyForm Noncommercial 1.0.0, see the repository root). It is written
from documented behaviour and hardware facts; it contains no vendor code. Fetched at build time and not
stored here: CMSIS 6 Core headers (Apache-2.0), Unity (MIT).

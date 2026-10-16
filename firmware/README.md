# Firmware

A clean C11 rewrite of the Begode Extreme controller firmware. One CMake tree builds it two ways:

- **target**: an image for the controller MCU (STM32F103RC class, Cortex-M3, no FPU), linked behind the
  vendor bootloader at `0x08003000`, so the stock OTA update path keeps working;
- **host**: unit tests, and a software-in-the-loop (SIL) shared library that the simulator loads in place
  of the emulated image.

Status: milestone M1, steps F1 and F2. Core has the BLE protocol (`core/protocol/`, described in
[docs/protocol/begode.md](../docs/protocol/begode.md), tested against a real capture) and the balance task:
attitude estimation (`core/attitude/`), balance law, tiltbacks, cut-offs and recovery (`core/balance/`), with
our own fdlibm-derived math (`core/math/`) so host and target compute the same bits. The image links core but
does not call it yet, configures no peripheral and drives no pin. It is not meant for a wheel: nothing in this
tree is flashed before the gates in the project plan (sim dry-run, stock-first flash, SWD backup) are in place.

## Layout and layer rules

| Directory | Holds | Rule |
|---|---|---|
| `core/` | control logic: math, protocol, attitude, balance, FOC, settings | Pure C. No HAL calls, no mutable globals: each module takes an `in` struct and a `state` struct and fills an `out` struct. Builds unchanged for target and host. |
| `hal/` | hardware interface headers (from F1) | Declarations only. Core never includes them; apps pass HAL results into core. |
| `ports/stm32f1/` | reset path, vector table, system set-up, register-level drivers | Implements `hal/` for the real MCU. Own register definitions on CMSIS-Core; no ST SPL or ST motor-control library code. |
| `ports/sim/` | SIL library: C ABI for the simulator engine | Implements `hal/` on top of simulated peripherals. Owns the run-time quirk mask. |
| `boards/<board>/` | `board.h` constants, `link_app.ld` memory map, stock tuning | Facts about one PCB. No code. |
| `apps/<board>/` | `main`, init order, task table (TIM8 / ADC / RTC / main-loop contexts) | Composes core modules with a port and a board, and schedules them. |
| `config/quirks.def` | compat / clean quirk list | See [Profiles and quirks](#profiles-and-quirks). |
| `tests/unit/` | Unity tests, host only | One executable per area, run by CTest. Shared captures live in `../tests/data/`. |
| `tools/` | size report, quirk doc generator, cppcheck deviation record | Python standard library only. |

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
stock binary (the table is in [docs/quirks.md](docs/quirks.md)). Code tests one with:

```c
if (EUC_QUIRK(ASIN_ACCEL)) { /* stock behaviour */ } else { /* clean behaviour */ }
```

- `target-compat` and `target-clean`: `EUC_QUIRK(ID)` is a compile-time constant, so the unused branch is
  not in the image.
- `host-sil`: `EUC_QUIRK(ID)` reads a mask the simulator sets through `euc_sil_quirk_set_mask()`, so the
  studio can switch one quirk at a time.

Neither profile changes telemetry frame bytes, command letters, the `!@` update entry, the flash map or
USART1. After editing the list, run `cmake --build --preset <any> --target quirks-doc`.

## Size budget

After each target link, `tools/size_report.py` reads the map file, charges every input section to its
module, writes `size-report.json` next to the image and checks `size-budget.toml`: a warning above 44 KiB of
flash, a failed build above 48 KiB of flash or 16 KiB of static RAM (`.data` + `.bss`).

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

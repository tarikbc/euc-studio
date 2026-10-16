# euc-armv7m

Our own Cortex-M3 interpreter: the CPU backend EUC Studio ships. It implements the full
ARMv7-M Thumb/Thumb-2 integer instruction set (no DSP extension, no FPU) behind
`euc_core::Cpu`, so the engine can run real controller firmware without Unicorn, which stays a
GPL-2.0 development oracle that never ships.

```rust,ignore
// The same contract as UnicornCpu::new: the engine lends its memory by pointer.
let cpu = unsafe { euc_armv7m::Armv7m::new(regions) }?;
```

## Structure

| File | Role |
|---|---|
| `src/decode16.rs`, `src/decode32.rs` | ARM ARM A5.2 / A5.3 encoding tables to an `Op` with operands extracted and PC-relative addresses resolved |
| `src/op.rs` | the `Op` enum and its operand types |
| `src/exec.rs` | one `Op`, following the A7 / B5.2 pseudocode; loads, stores and the bus |
| `src/alu.rs` | `AddWithCarry`, `Shift_C`, `ThumbExpandImm_C`, `SignedSatQ`, `UnsignedSatQ` |
| `src/it.rs` | condition codes and `ITSTATE` |
| `src/timing.rs` | Cortex-M3 TRM r2p1 cycle counts (`Armv7m::cycles`) |
| `src/mem.rs` | regions, the decode cache (one slot per halfword, invalidated by our own stores and by `Cpu::invalidate`), and the crate's only `unsafe` |
| `src/lib.rs` | `Armv7m`, the run loop, register sync, exception returns, watches |

Exception entry, return, priorities and tail-chaining belong to the engine's `ExceptionUnit`.
The core reports `SVC`, `BKPT`, `WFI`, `WFE` and faults through `Exit`, and an `EXC_RETURN`
load into PC (any value `0xFxxxxxxx`, ARM ARM B1.5.8) through `Bus::exception_return`, inline.
UNDEFINED and UNPREDICTABLE encodings both end the run with `Fault::Undefined`.

Choices where the ARM ARM leaves the behaviour to the implementation follow the Cortex-M3:

- The local exclusive monitor holds no address: any `STREX` after an `LDREX` stores. Exception
  entry and return clear it.
- Unaligned `LDR`/`STR`/`LDRH`/`STRH` work unless `CCR.UNALIGN_TRP` (`Armv7m::set_ccr`); `LDM`,
  `STM`, `LDRD`, `STRD`, `LDREX`, `STREX` always need alignment and check it before any access.
- An unaligned MMIO access reaches the bus as the largest naturally aligned pieces, lowest
  address first (a word at offset 1 is a byte, a halfword and a byte).
- Writes to SP clear bits 1:0. `CONTROL[31:2]` is RAZ/WI (no FP extension).
- Division by zero returns 0, or faults with `CCR.DIV_0_TRP`.

## Verification

| Layer | What | Result |
|---|---|---|
| Unit tests (`src/`) | `AddWithCarry`, `Shift_C` at 0/32/33, `ThumbExpandImm_C`, saturation, ITSTATE, decoder tables (encodings checked against `llvm-mc`) | 17 tests |
| `tests/encodings.rs` | per-instruction behaviour from the ARM ARM: flags, shift edges, IT blocks across runs, `Align(PC, 4)`, alignment rules, LDM write-back, interworking, exception returns, multiply/divide, saturation, bit fields, special registers and stack banking, the monitor, the `Exit` contract, MMIO splitting, self-modifying code, watches, cycle counts | 28 tests |
| `tests/fuzz_unicorn.rs` | differential fuzzing against Unicorn, one case = random state and RAM plus one instruction (half fully random, half from per-group templates), sometimes inside an IT block or as an `LDREX`/`STREX` pair; compares exit, all registers, flags, IT state, RAM and MMIO effects | final core: 10 M cases over ten seeds, 9.2 M compared (the rest UNPREDICTABLE encodings, skipped), **0 unexplained divergences**; about 0.96 M explained ones, all in the classes below |
| `tests/lockstep.rs` | the stock Begode Extreme image boots on Unicorn while our core shadows every quantum on a copy of memory, with Unicorn's bus traffic replayed; registers, memory and bus traffic compared after each quantum | 30 s simulated: 701,930 quanta, 1.65 G instructions, 11.3 M bus calls, 1.19 M exception returns, **0 divergences** |
| `tests/stock_boot.rs` | the stock image on our core alone: the Unicorn backend's boot assertions (`main` at 0.069 ms, first frame at 2.00 s, `V` and `N` replies, 40 frames in 6 s), and 30 s with no blocker | pass |

```text
cargo test  -p euc-armv7m                                            # unit, encodings, stock boot
cargo test  -p euc-armv7m --features unicorn --release --test fuzz_unicorn -- --nocapture
EUC_FUZZ_CASES=10000000 EUC_FUZZ_SEED=0x5EED2026 cargo test ...      # more cases, a fixed seed
cargo test  -p euc-armv7m --features unicorn --release --test lockstep -- --nocapture
cargo bench -p euc-armv7m --bench mips
```

The stock-image tests read the image from the firmware cache (`euc fetch begode-extreme`) and
pass with a printed reason when it is missing; the image and traces of it never enter the repo.

### Where Unicorn and the ARM ARM disagree

The fuzzer fails on any divergence that is not one of these classes. In each, the ARM ARM (or
the Cortex-M3 TRM where it leaves a choice) decides, and our core follows it.

| Class | ARM ARM / TRM | Unicorn (QEMU 5) |
|---|---|---|
| DSP-extension encodings (`UADD8`, `SXTAH`, `SMLALxy`, `PKHBT`, `SSAT16`, ...) | ARMv7E-M only: UNDEFINED on the M3 | runs them |
| Coprocessor / FP space (`VLDR`, `VSTR`, ...) | no coprocessor: NOCP UsageFault | runs VFP loads and stores |
| Advanced SIMD element loads, `SETEND`, `BXJ`, `SUBS PC, LR`, `LDREXD`/`STREXD`, `BLX (immediate)` | ARMv7-A only: UNDEFINED | runs them |
| `LDA`/`STL` and their exclusive forms | ARMv8 only: UNDEFINED | runs them |
| Stores with `Rn = PC` | UNDEFINED (A7.7.158-163) | stores |
| Unaligned `LDM`/`STM`/`LDRD`/`STRD` | MemA: UsageFault before any access (A3.2.1) | performs the access |
| Unaligned `LDREX`/`STREX` to unmapped memory | alignment checked first (A3.4.3) | bus fault first |
| `STREX` after an `LDREX` to another address | M3 monitor holds no address: stores | tags the address: fails |
| SP written with bits 1:0 set (`ADD SP, Rm`, `MOV SP, Rm`, `LDR SP`, `MSR MSP/PSP`, write-back) | bits 1:0 forced to zero (B1.4.1, B5.2.3) | kept, so later SP-relative results differ |
| `CONTROL` bits 3:2 | RAZ/WI without the FP extension | kept |
| PC loaded with `0xF0000000`-`0xFFFFFEFF` | EXC_RETURN in handler mode (B1.5.8) | branches (backend reports only `0xFFFFFF00` up) |
| PC loaded with an even, unmapped address | INVSTATE on the next instruction | fetch fault first |
| An instruction faulting inside an IT block | PC stays on it | reports PC past the block |
| `ISB` inside an IT block | IT state continues | ends the block and loses it, so later slots run unconditionally |
| `YIELD`, `WFE` | hints; `WFE` continues when the event register is set | stops with an error |
| `WFI` | `Exit::WaitForInterrupt` (the trait contract) | the backend cannot report it |
| UNPREDICTABLE encodings (reached through a branch) | our core faults by design | runs them |

Harness artifacts, also classified: Unicorn's instruction counter skips IT slots that fail
their condition, never stops inside an IT block, and translates the next block before stopping
(so a fault or undefined encoding there surfaces early); its register API cannot write `CONTROL`
once `nPRIV` is set, cannot write IT state, and reads the masks as 0 when unprivileged. The
fuzzer compensates, rebuilds Unicorn when needed, and confirms every unexplained difference on a
fresh Unicorn before counting it.

## Performance

Apple M3 Max, release build:

| Measure | Result |
|---|---|
| `benches/mips.rs` (criterion, medians) | 181 MIPS load/ALU/store loop, 165 MIPS CRC-32 with an IT block, 162 MIPS calls with `PUSH`/`POP` |
| `examples/spin` (quanta of 4000, as the machine runs) | 200, 170 and 160 MIPS on the same kernels |
| Stock image, whole machine on our core (`tests/stock_boot.rs`) | 30 s simulated in 11.0 s: **2.7x real time**, 150 MIPS of firmware with the engine |
| Modeled CPI of the stock image (TRM cycle model) | 1.57 (the machine charges a fixed 1.25 today) |

The cycle model is the Cortex-M3 TRM's at zero flash wait states, with three choices where the
TRM gives a range: pipeline refill `P = 2`, early-terminating multiplies and divides spread over
their operand sizes, and load/store single pipelining with a neighbour.

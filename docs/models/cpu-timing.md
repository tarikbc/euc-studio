# CPU timing (open investigation)

**Status: open.** Nothing in the engine has changed yet. This page records what was measured,
which causes were found, what was ruled out and what to do next.

Evidence tags as elsewhere: **OBSERVED** = measured in the simulator or in a real recording;
**INFERRED** = follows from the code or the measurements; **ESTIMATE** = a model or a figure from
outside the project.

## The question

The stock firmware's standing tail-light pattern breathes with a period of about **2.1 s** on the
owner's wheel (video, one cycle, ±0.2 s) and **0.49 s** in the simulator (display's run: blank
settings, Unicorn block quanta, fixed CPI 1.25). Is the simulated main loop too fast, and why?

How the firmware makes the period (OBSERVED, see [lights.md](../hardware/lights.md)):
`led_strip_effects` (0x08005A5C) runs once per main-loop pass. While the wheel stands, every pass
that runs during a tick whose LED tick counter is a multiple of 10 steps the brightness by ±1
(0 to 131 and back). So the period is inversely proportional to the main-loop passes per RTC tick
(496 Hz, 2.016 ms). In our runs, period × passes per tick × 49.6 Hz ≈ 289 steps per cycle.

## Measurements at today's timing

Own core, stock GW1825007, idle (`euc sim run begode-extreme`), 20 simulated seconds. A study-only
wrapper single-stepped the core and counted instructions per exception context, main-loop passes
and the brightness byte. The profiled runs were bit-identical to plain runs (same instruction
count, same BLE frame times). OBSERVED.

| | blank settings | owner preset `tarik-2026-10-15` |
|---|---|---|
| LED mode, drive mode | 0, voltage 10 kHz | 1, current 12 kHz |
| TIM8_UP / ADC1_2 interrupts per s | 19.6 k / 19.2 k | 23.6 k / 23.1 k |
| CPU time: thread / RTC / ADC1_2 / TIM8_UP | 63 / 12 / 12 / 7 % | 61 / 12 / 13 / 9 % |
| Thread time inside `led_strip_effects` | 97 % | 97 % |
| Instructions per standing main-loop pass | 6.7 k | 11.9 k |
| Main-loop passes per RTC tick | 11.75 | 6.32 |
| Breathing period | **0.495 s** | **0.922 s** |
| BLE frame-group period | 304.42 ms | 304.42 ms |

- The 0.495 s matches display's 0.49 s, so block versus exact quanta does not matter here.
- `ws2812_set_pixel` alone is 62 % of all instructions in LED mode 0.
- The BLE scheduler queues a frame group every 151 counted ticks. The simulator gives exactly
  151 ticks (304.4 ms). The real capture (`tests/data/begode-extreme/idle-2026-10-15.hex`, 76
  groups in 25 s) gives **309.6 ms**: the real wheel loses about 1.7 % of its ticks. This is a
  second, independent measure of the CPU speed, and more precise than one video cycle.

## Causes found

1. **LED mode (INFERRED from the capture).** The owner's capture reports `W M` 1, so his wheel is
   in LED mode 1 (the preset boots in mode 1, OBSERVED). The 0.49 s was measured in mode 0. A
   mode-1 pass costs 1.8 times a mode-0 pass, so mode 1 alone gives 0.92 s. This explains a
   factor of 1.9 of the 4.3.
2. **CPI (ESTIMATE).** The machine charges a fixed 1.25 cycles per instruction. Our core's TRM
   model (`euc-armv7m` `timing`) gives 1.52 to 1.88 per context. The STM32F1 runs from flash with
   2 wait states at 72 MHz (the firmware sets latency 2 and the prefetch buffer on, OBSERVED). A
   flash model on top of the TRM counts:
   - fetch bandwidth: 8 bytes per 3 HCLK cycles in each straight-line run;
   - W extra cycles per taken branch and per data read from flash (literal pools, constant tables,
     TBB/TBH);
   - 12 + W cycles for each exception entry and each return (today these cost nothing).

   With W = 2 the effective CPI is 2.07 (thread 2.02, RTC 1.88, ADC1_2 2.01, TIM8_UP 2.40).
   Cross-check: ST's family figure of 177 CoreMark at 72 MHz (2.46 per MHz) against about 3.3 per
   MHz for a Cortex-M3 at zero wait states is a 1.35x slow-down over the TRM ideal. The model gives
   1.3x.
3. **Interrupt share (INFERRED).** The interrupt work per tick is fixed, so a slower CPU takes it
   from the main loop. At CPI 1.25 the handlers use about 36 % of the time. At CPI 2.07 they use
   about 60 %, and the main loop keeps 40 % instead of 64 %. Passes per tick fall by the CPI ratio
   (1.65) times 1.6, about 2.6.
4. **Lost RTC ticks (INFERRED).** With the slower CPU, about 1 % of the RTC ticks are lost.
   Every 100th tick the RTC handler runs `display_refresh`. Its TM1640 bit-banging uses software
   delay loops of about 77 k instructions. With FOC pre-emption, that handler then lasts more than
   2 ticks, and a second-flag event is lost (the flag does not count). The BLE scheduler counts
   ticks, so its group period grows. This is the mechanism behind the real 309.6 ms.

## Prototype results (study snapshot only, not in the tree)

Owner preset, idle, 30 simulated seconds. Fixed CPI = the machine's existing `cpi`. Model = the
per-instruction flash model above, with the quantum's time taken from the model. OBSERVED in the
snapshot.

| Timing | Breathing | Passes per tick | RTC ticks lost | BLE group period |
|---|---|---|---|---|
| CPI 1.25 (today) | 0.92 s | 6.32 | 0 | 304.4 ms |
| CPI 2.0 | **2.17 s** | 2.70 | 0.98 % | 307.4 ms |
| Model, W = 1 | 1.84 s | 3.16 | 0.98 % | 307.4 ms |
| Model, W = 2 | **2.31 s** | 2.53 | 0.98 % | 307.5 ms |
| Model, W = 3 | 2.94 s | 2.03 | 1.95 % | 310.5 ms |
| Real wheel | 2.1 ± 0.2 s | about 2.5 | about 1.7 % | 309.6 ms |

- CPI 2.0 and the W = 2 model are both within 10 % of the video. The BLE period prefers a CPU
  slightly slower than both. The lost-tick count is a threshold effect (1 or 2 ticks per display
  refresh), so it moves in steps.
- LED mode 0 with the W = 2 model gives 1.10 s and 5.35 passes per tick.
- Scenarios (stock, own core, snapshot) pass in all three timings:

| Scenario | CPI 1.25 | CPI 2.0 | Model W = 2 |
|---|---|---|---|
| balance-60s | 4/4, max pitch 1.028° | 4/4, 1.026° | 4/4, 1.025° |
| ride-20kmh-stop | 5/5, top 20.3975 km/h, 81.764 m | 5/5, 20.4003 km/h, 81.766 m | 5/5, 20.4010 km/h, 81.767 m |

## Ruled out

- **Poll fast-forward.** It acts in thread mode only and skips to the next event, so it keeps
  time. In steady state it is small: `usart3_rx_parse` reading `DMA1.CNDTR3` (0.52 s in 30 s);
  the rest is boot `delay_ms`.
- **Blocking waits in the main loop.** 97 % of the thread time is LED code. The UART frame pacing
  is modeled (2.24 ms between frames, one frame per pass).
- **SPI3 transfer time (not modeled).** After init the firmware sets the MPU6500 SPI to PCLK1/2 =
  18 MHz; one IMU read is about 0.5 k cycles per tick (0.3 %).
- **ADC conversion time (not modeled).** The FOC is interrupt-driven (`TIM8_UP` starts the
  injected group, `ADC1_2` on JEOC runs the loop); nothing busy-waits, so the CPU load is the same.
- **The app's "ISR 31 %".** It comes from the synthetic source (`app/src/sim/synthetic.ts`); the
  live engine never fills `isr_load_pct`.

Not modeled, small (ESTIMATE, not checked): the RTC `RTOFF` wait after clearing `SECF` (up to
3 RTC clocks, 48 µs per tick, at most 2.4 %), and bus contention from the WS2812 DMA at 800 kHz
(about 2 %).

## Next steps

1. Choose the fix:
   - **Fixed CPI 2.0** in every caller (CLI `--cpi` default, `serve.rs`, `compare.rs`, the app's
     `firmware.rs`). This is simple, and the own and Unicorn backends keep identical timing. Keep
     1.25 where a test compares with the Python reference harness.
   - **Per-instruction model** in `euc-armv7m` (the flash model above), with the machine taking
     quantum time from the core. It is more faithful when the code mix changes (delay loops,
     soft-float, our own firmware). Unicorn has no model, so its timing would then differ from our
     core's. It needs a cycle budget in `Cpu::run`, or a CPI estimate for the budget.
2. Charge exception entry and return (12 + W cycles) in the machine for every backend.
3. Re-run every scenario and the compat-versus-stock compare tolerances after the change.
4. Correct [lights.md](../hardware/lights.md): 0.49 s is LED mode 0; the owner's wheel and preset
   are LED mode 1 (0.92 s at today's timing).
5. Better real data: a 20 s video on a tripod with the LED mode noted (period to ±2 %), and a
   second BLE capture. Both pin W between 2 and 3.
6. If the project wants a profiler, add an `euc sim profile` command, built from the prototype.

Prototype tools: private research repo, `research/timing-proto/` (the single-step profiler, the
flash-model prototype and the analysis scripts, with a README on how to run them).

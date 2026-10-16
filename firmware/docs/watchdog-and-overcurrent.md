# Watchdog and over-current trip (clean profile)

Status: for review. Both are protections the stock firmware does not have (quirks `NO_WATCHDOG` and
`NO_OVERCURRENT_TRIP`, [quirks.md](quirks.md)). On an electric unicycle a protection that stops the motor is
itself a fall risk, so each one may act only where the drive is already lost. This note gives the design, the
reasons and what is still open on the bench.

## Where things run

| Context | Rate | Work |
|---|---|---|
| TIM8 update interrupt | 2 x fPWM (20 or 24 kHz) | hall sensors, light PWM |
| ADC interrupt | fPWM (10 or 12 kHz) | FOC: currents, angle, PI, SVPWM |
| RTC tick interrupt | 496 Hz | IMU, balance, cut-offs, commands, alarms, lights, storage decisions |
| Main loop | as fast as it runs | telemetry, BMS link, deferred flash writes, power-off, bootloader hand-off |

The balance and the FOC run in interrupts, so they do not need the main loop. A main loop that hangs while
the interrupts run still balances the wheel; it only loses telemetry, the BMS link and saving.

## Watchdog

The independent watchdog (IWDG, LSI clock) starts at the first tick with a 250 ms timeout. The supervisor
(`core/safety/supervisor.c`) runs at the end of every tick and decides:

- **Feed** only when the FOC ran since the last tick (the ADC interrupt counts its steps) and the balance
  published a finite command within +-1050. A dead tick, a dead FOC or a broken command leaves the drive state
  unknown; then nothing feeds the watchdog and it resets the MCU after 250 ms.
- **Main loop stalled**: no main-loop pass for 248 ticks (0.5 s) while the tick runs. This does not reset the
  wheel: the balance still works, and a reset (more than 1 s of boot before the tick runs again) is a fall at
  speed. Instead it latches until power-off:
  - limp mode: speed tiltback at 10 km/h at most, no automatic restart after the next stop (the same limp as a
    lost hall line, balance input `limp`);
  - the stall alarm: 3 x 20-tick beeps every 500 ticks.
  The rider slows down and steps off. Telemetry stops with the main loop, so the app sees the link go quiet.
- **The boot after a watchdog reset** reads the reset flag (RCC CSR IWDGRSTF) and keeps the bridge off until
  the next power-on (hard cut request, drive disabled, the hard-cut reminder beeps). The drive state before the
  reset was unknown; the wheel must not drive again on its own.

Why the main loop no longer feeds: the earlier design fed the watchdog from the main loop when the tick had
advanced, so a hung main loop reset a wheel that was still balancing. That turned a lost telemetry link into
a fall.

What the reset path does to the outputs: an MCU reset puts TIM8 back to its reset state (main output enable
off) and every pin back to a floating input, so the bridge is not driven from the MCU. Whether the gate driver
then holds every switch off depends on pull-downs on its inputs (bench item 1). Our init leaves the bridge off
until the hall sensors report a standstill and the balance start-up check passes, and the power-on hold check
keeps the wheel off while it turns (warm start) or until the button is held (cold start).

Timing limits:

- LSI is 30..60 kHz on the STM32F1, so the 250 ms timeout is 167..333 ms. The tick period is 2 ms; the longest
  normal gap between feeds is one tick.
- A flash page erase stalls every interrupt that runs from flash for 20..40 ms. Clean erases the settings page
  only at power-off or with the drive disabled, and that stall is far below the timeout.
- The power-off and bootloader paths wait 200 ms in the main loop with the motor already off; the supervisor
  latches no stall for them (248 ticks is 0.5 s).

Tests: `tests/unit/test_supervisor.c` (feeding rules, stall latch, compat does neither),
`tests/unit/test_alarms.c` (stall beeps), `tests/unit/test_balance.c` (limp tiltback and no restart).

## Over-current trip

The current module (`core/motor/current.c`) counts phase readings above a level; the trip latches after a
number of consecutive readings and cuts the bridge. It is **off in both profiles** (`NO_OVERCURRENT_TRIP` is on
in clean too) until the level is measured, because a false trip mid-ride is a fall. The code and its tests stay
(the tests run it with the quirk bit cleared).

The level in `boards/begode-extreme/board.h` is a placeholder: 31500 counts (about 420 A on the stock scale,
the level the stock code only counts) for 2 readings (167 us at 12 kHz).

Before it is switched on, the trip needs a design decision as well as a level: a cut at speed is a fall, so
the first response to high current should be to limit it (lower the current or voltage command), and a cut
only for a current that limiting cannot explain (a shorted phase, a failed switch).

## Open bench items

1. Gate driver inputs: confirm pull-downs (or a defined off state) on all six inputs while the MCU pins float
   during and after a reset.
2. Power latch (PB14): after a watchdog reset the pin floats; measure whether the controller keeps power and
   how long the boot takes on the real board.
3. LSI frequency of the board's MCU, for the real watchdog timeout.
4. Power stage limits for the over-current level: MOSFET pulse current and safe operating area, current sensor
   range and saturation, wiring and connector limits.
5. The TIM8 break input: PA6 (TIM8_BKIN) is used for the LED strip (TIM3), so no hardware break is available;
   decide whether a hardware over-current path exists on the board (comparator, gate driver desaturation).
6. A main-loop stall on the real wheel: confirm that the tick and the FOC keep running and the limp tiltback
   and beeps work (inject a stall in a bench build).

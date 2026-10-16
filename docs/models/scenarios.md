# Scenario library: what the stock firmware does

The situations EUC riders fear most, ridden in the simulator on the stock Begode Extreme firmware
(GW1825007) with the electrical plant ([electrical.md](electrical.md)) and the body physics
([body.md](body.md)). Each scenario in `scenarios/` records what happened with `record` checks, which
always pass, and gives its safety verdict with ordinary checks (`upright`, `no_cutoff`, ...), which
may fail. A failing verdict is a finding about the stock firmware, not a broken scenario.

**The numbers depend on defaults.** Motor inductance and flux, battery resistance, the rider model and
the aids are estimates until measured; the top-speed numbers in particular move with the motor's
inductance (it eats voltage headroom at speed). Read them as how the stock firmware behaves on this
model, and compare runs with each other rather than with a real wheel.

Run one with `euc sim run --scenario scenarios/<name>.toml` (Unicorn backend). Results of
2026-10-16:

| Scenario | Settings | Stock outcome | Key numbers |
|---|---|---|---|
| `overlean-top-speed` | maintainer's (current mode, hard pedals, PWM alarm 80 % with tiltback, speed tiltback 69 km/h) | **Cutout: falls.** The PWM alarm comes, the PWM tiltback starts, and 0.4 s later the wheel nose-dives. | Accelerating at 5 km/h/s: PWM alarm (80 %) at 22.86 s, about 62 km/h; pitch past −20° at 23.28 s; fell at 23.76 s at 63.6 km/h with 101.6 A of Iq, PWM tiltback only 1.5°; bus 121.2 V. Speed tiltback (69 km/h) never reached. |
| `overlean-top-speed-30pct` | same, battery at 30 % (115 V at rest) | **Cutout: falls,** about 1 s earlier and at a lower speed. | PWM alarm at 21.71 s, about 58 km/h; pitch past −20° at 22.24 s (0.5 s after the alarm); the wheel then pitches between −12° and −31° for 2 s before it falls at 24.49 s; bus sags to 108 V. |
| `hard-braking-30kmh` | maintainer's | **Holds.** No cutoff, no fall, no alert. | Stops from 32.6 km/h in 2.75 s (3.3 m/s² mean); braking torque peaks at −207 N·m with −259 A of Iq, close to the firmware's 280 A current limit; regeneration only −13.9 A at the battery (+1.3 V on the bus); pedals tip 10.8° nose up. |
| `hall-single-line-loss` | neutral | **No cutoff,** degraded drive. | Firmware speed reads a third of the truth (6.7 against 20.3 km/h): speed tiltback and alarms act at three times their setting. Torque ripple 16.5 N·m (3.5 healthy). |
| `hall-loss-while-riding` | neutral | **Cut, then falls.** | Hall fault 70 ms after the connector comes off; alert 0x40; nose dive to −33° within 0.1 s. |
| `low-battery-tiltback` | neutral | **Warns and tilts back,** no cutoff. | 98.5 V: alert 0x08 at 1.7 s; pedals past 8° nose up 1.6 s after reaching 15 km/h, 26° by 20 s. |
| hill climb at low battery | | pending: needs a slope in `euc-physics` | |
| pothole or bump at 20 km/h | | pending: needs ground features in `euc-physics` | |

## What the overlean runs show

In current mode the PWM does not climb gradually to the 80 % alarm. It sits near 65-70 % while the
q-current loop keeps up, then jumps to 100 % within a fraction of a second when the rider asks for
more torque than the voltage headroom allows: the loop saturates, the current stops following its
reference, and the balance command runs to its ±1050 limit. The PWM alarm and the PWM tiltback start
at that jump, so the rider gets about 0.4-0.5 s of warning before the nose goes down, and the
tiltback has reached only 1.5-5° by then. On a 30 % battery the alarm comes about 4 km/h earlier.

## Open question

The hard brake on factory settings (voltage mode, medium pedals, speed tiltback at 20 km/h) is not in
the library yet: with the same rider the wheel overshoots to 40 km/h before the brake, then hits the
current limit (−282 A) and falls backwards at 16.5 s. How the 20 km/h tiltback interacts with the
rider model is not understood yet, so the result may say more about the model than about the firmware.

## Not modeled yet

Rider reactions: the rider model follows a target speed by leaning and does not react to beeps or
tiltback the way a person would, and the rigid rider with its aids can hold on longer than a person
during the nose-down struggle of the 30 % run. Terrain: slopes, bumps and potholes wait for
`euc-physics`.

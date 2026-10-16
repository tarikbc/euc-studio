# Scenario library: what the stock firmware does

The situations EUC riders fear most, ridden in the simulator on the stock Begode Extreme firmware
(GW1825007) with the electrical plant ([electrical.md](electrical.md)) and the body physics
([body.md](body.md)). Each scenario in `scenarios/` records what happened with `record` checks, which
always pass, and gives its safety verdict with ordinary checks (`upright`, `no_cutoff`, ...), which
may fail. A failing verdict is a finding about the stock firmware, not a broken scenario.

**The numbers depend on defaults.** Motor inductance and flux, battery resistance, the rider model and
the aids are estimates until measured; the top-speed numbers move with the motor's inductance, and
anything with an impact moves with the body model (see below). Read them as how the stock firmware
behaves on this model, and compare runs with each other rather than with a real wheel.

Run one with `euc sim run --scenario scenarios/<name>.toml` (Unicorn backend). Results of 2026-10-16
with the measured 3.00-12 tire (rolling radius 0.2251 m, 28 pole pairs); "maintainer's" settings are
the `tarik-2026-10-15` preset (current-mode drive, hard pedals, PWM alarm 80 % with PWM tiltback, speed
tiltback 69 km/h):

| Scenario | Settings | Stock outcome | Key numbers |
|---|---|---|---|
| `overlean-top-speed` | maintainer's; the rider ignores tiltback | **Cutout: falls.** | Accelerating at 5 km/h per second: PWM alarm (80 %) at 24.2 s, about 62 km/h; the PWM first saturates at 26.0 s; the firmware disables the drive at 29.2 s at 70.8 km/h and the wheel falls at 30.0 s, 5.9 s after the first alarm; PWM tiltback 7.9° by then. |
| `overlean-top-speed-30pct` | same, battery at 30 % (115 V at rest) | **Cutout: falls,** earlier and slower. | PWM alarm at 22.3 s, about 58 km/h; saturated at 24.1 s; drive disabled at 25.3 s at 64.8 km/h and the wheel falls at once, 3.0 s after the alarm; bus 108 V. |
| `hard-braking-30kmh` | maintainer's | **Holds.** No cutoff, no fall, no alert. | Stops from 32.5 km/h in 2.83 s (3.2 m/s² mean); braking torque peaks at −206 N·m with −240 A of Iq (the firmware's limit is 280 A); regeneration only −12.0 A at the battery (130.8 V on the bus); pedals tip 10.8° nose up. |
| `hill-climb-low-battery` | maintainer's, battery at 3 % (101.8 V at rest); the rider ignores tiltback | **Holds, tilts back** on the climb; no cutoff. | 20 % grade from 40 m (reached at 17.0 s, 25 km/h): low-voltage alert already on from boot (1.7 s); the climb sags the bus below 99.2 V and the battery tiltback passes 2° at 20.8 s and keeps growing, to 26° at 36 s (pedals 30° nose up); bus down to 97.0 V, Iq up to 77 A. |
| `low-battery-tiltback` | neutral | **Warns and tilts back,** no cutoff. | 98.5 V: alert 0x08 at 1.7 s; the pedals pass 8° nose up at 12.5 s and the rider, easing off, gets no faster than 6.9 km/h. |
| `hall-single-line-loss` | neutral | **No cutoff,** degraded drive. | Firmware speed reads a third of the truth (6.7 against 20.1 km/h): speed tiltback and alarms act at three times their setting. Torque ripple 19.9 N·m (3.9 healthy). |
| `hall-loss-while-riding` | neutral | **Cut, then falls.** | Hall fault 70 ms after the connector comes off; alert 0x40; nose dive past −30° within 0.13 s. |
| `pothole-20kmh` | maintainer's | **PRELIMINARY** (rigid body, no suspension yet): `upright` fails, as on the compat firmware; the pedals dip past 45° at 20.4 s, the firmware does not cut, and the wheel rides on nose down. | A 2 cm deep, 50 cm long pothole with steep edges: the far edge takes the wheel from 20.0 to 16.1 km/h in 20 ms; full forward torque (285 A, the current limit); shell to −57°; the gyro clips (below) and for the next seconds the firmware balances 26° nose down (RMS error of its attitude 27° over 20.8-22 s). |
| `ble-service-cut-standing` | maintainer's | **Cut, then falls.** | A phone sends "c", then "y" while the rider stands still: the motor is cut 10 ms after the "y" (drive fault and balance disabled), and wheel and rider fall 1.3 s later. No pairing, no confirmation. |
| `balance-60s`, `ride-20kmh-stop` | neutral | Hold (CI regression runs). | |

## What the overlean runs show

The overlean runs use a rider who never eases off for tiltback (`tiltback_feel_deg = 90`): the default
rider eases off and stays up, which is what the PWM tiltback is for; the danger is the rider who leans
through it.

The PWM alarm comes before the cutout, but not by a fixed margin. On a full battery the PWM sits near
55-78 % above 60 km/h, touches the 80 % alarm at about 62 km/h, and saturates for good about 6 s later
at 70 km/h, when the rider asks for more than the voltage headroom allows: the q-current loop
saturates, the current stops following its reference, and the balance command runs to its ±1050
limit. On a 30 % battery the whole sequence comes about 4 km/h earlier and the margin from alarm to
fall halves to 3 s. The PWM tiltback has reached only 4-8° by the fall: it warns, but at this
acceleration it cannot stop a rider who keeps leaning.

Pedal height (an ESTIMATE from the scan, ±15 mm) moves the cutout along this ramp but hardly its speed:
with the pedals 5 mm below the axle the drive is disabled at 28.5 s at 70.4 km/h, with them 25 mm
above at 35.4 s at 71.2 km/h (now 10 mm above: 29.2 s, 70.8 km/h). It mostly changes how long the
rider can hang at the limit before the nose goes down.

Before the measured tire and pedals (the 16 × 3 tire guess, 26 pole pairs, 7.7 % less torque per amp
at the axle, pedals level with the axle), the full-battery run gave: PWM alarm at 22.86 s at about
62 km/h, nose past −20° at 23.28 s, fall at 23.76 s at 63.6 km/h, only 0.4 s after the alarm; the
30 % run: alarm at 21.71 s at about 58 km/h, fall at 24.49 s. So the warning time, from the first PWM
alarm to the loss of balance, depends on the motor values: 3-6 s with the measured tire and pedals,
0.4 s before them. It needs measured motor constants before it means anything for a real wheel.

## The gyro clips, and the attitude estimate stays wrong

The stock firmware sets the MPU-6500's gyro to ±500 °/s (65.5 LSB per °/s). In the pothole run the
shell's pitch rate reaches 820 °/s and then 1360 °/s for a few tens of milliseconds. The IMU model,
like the real part, saturates its 16-bit output at the range limit, so the firmware integrates
500 °/s where the shell turns faster, and its Mahony filter loses the excess. After the swing the
firmware believes the shell is level (−1.4°) while it is 26° nose down, and it balances that error:
the wheel rides on with the pedals tilted. The accelerometer correction (hard pedals: Mahony gain 0.2)
removes it slowly, by about 5° per second.

The trigger is probably exaggerated (see the pothole caveat), but the behavior is the firmware's: any
event that turns the shell faster than 500 °/s leaves the stock firmware riding with a wrong idea of
level for seconds. A wider gyro range (±1000 or ±2000 °/s) or a check of the accelerometer after a
saturated sample would avoid it.

## Preliminary: impacts on a rigid body

The Extreme has a rear shock between the shell and the wheel; the simulated body is rigid, the rider
has no knees, and the tire is a soft contact rather than a carcass. Impacts therefore reach the shell
harder than on the real wheel. Pothole and bump results stay PRELIMINARY until the body model has
suspension. Depths tried at 20 km/h with steep edges (measured tire): 1 cm rides through (pedals dip
7°, attitude error 0.4°); 2 cm dips past 45° and recovers nose down (above); 3 and 5 cm fall, and the
firmware disables the drive 1.0 and 0.3 s later. A smooth 6 cm, 50 cm long bump also made the wheel
fall (drive disabled 0.3 s later).

## Open questions

- The hard brake on factory settings (voltage mode, medium pedals, speed tiltback at 20 km/h) is not
  in the library: with the same rider the wheel overshoots to 40 km/h before the brake, then hits the
  current limit and falls backwards. How the 20 km/h tiltback interacts with the rider model is not
  understood yet.
- The hill climb stops at 36 s on purpose. Run on, the battery tiltback reaches 40° by 39 s, the
  rider who never gives up on 25 km/h stalls the wheel on the slope, the bus dips to 89 V and the
  firmware disables the drive at 40.7 s; a person would have stepped off long before. With the
  default rider, who eases off for tiltback, the wheel slows on the 20 % grade, stalls, rolls back
  and falls at 23.5 s (drive disabled at 23.9 s): the easing rule is not yet right for steep slopes.

## Terrain

Scenarios can ride on terrain (`crates/euc-physics/src/terrain.rs`): a `[terrain]` table with a
constant grade (`slope_pct`, from `slope_start_m`, built up over `slope_ramp_m`, 10 m by default, so
the wheel meets no kink) and `[[terrain.feature]]` bumps (a smooth hump) and potholes (steep walls),
each with `at_m`, `length_m` and `height_m` or `depth_m`. Flat ground is the infinite floor plane;
anything else is a heightfield strip sampled every 2 cm along the path, from 5 m behind the start to
`length_m`.

## Not modeled yet

Rider reactions: the rider model follows a target speed by leaning and does not react to beeps or
tiltback the way a person would, and the rigid rider with its aids can hold on longer than a person
during a nose-down struggle. Suspension: see above.

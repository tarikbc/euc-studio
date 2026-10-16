# Riderless detection (clean profile)

Status: option 2 (beeps and the limp speed limit) is in the clean profile behind quirk `RIDERLESS_UNDETECTED`
(core/safety/riderless.c). Option 1 (probe, then stop) is parked until bench values exist (M8). All numbers
are from the simulator with the default motor values; the torque constant (0.86 N m/A) is not measured.

## What happens without it

When the rider steps off at speed, every profile keeps balancing the light wheel until it falls on its side;
the roll cut-off then disables the drive.

| Scenario | Step-off | Wheel on its side | Behaviour until then |
|---|---|---|---|
| rider-steps-off-15kmh | 15.0 s at 14.6 km/h | 17.7 s (2.7 s later), 11-12 km/h, 31.2 m | rolls on, weaving |
| rider-steps-off-5kmh | 12.0 s at 5.0 km/h | 12.6 s (0.6 s later) | falls at once |

At 5 km/h the wheel is down before any detector could act, so only the 15 km/h case can gain.

## Signal

The motor torque. With a rider at a steady speed it carries the rider's weight against the drag and the
rolling resistance of the loaded tyre (about 9-13 N m at 15-20 km/h); without a rider the light wheel needs
about 1 N m. At the step-off the torque falls from 11 N m to under 3 N m within 0.1 s, and the speed does not
change.

A low torque alone is not enough. Three riders also pass zero torque:

- braking (ride-20kmh-stop): the torque crosses zero in under 0.5 s and goes on to a braking torque;
- coasting to a stop (coasting-stop, 0.4 m/s^2): the rider leans back slowly, the torque passes zero over
  about 2 s while the speed falls;
- cruising down a gentle slope (downhill-cruise-2pct): the torque stays near zero for as long as the slope
  lasts.

A first rule (low torque for 0.6 s at an even speed) suspected the coasting rider (trigger 2 s into the coast,
7 deg of speed tiltback until the current reached 7 A three seconds later). The rule as built therefore looks
for the step-off itself: a sudden drop of the load, then a quiet wheel.

The firmware's own low-passed current (`iq_lpf_ma`, the balance's feedback) is no use here: its integer filter
has a time constant of about 0.8 s and truncates towards zero, so it lags the step-off by almost a second. The
detector filters the q current of the last FOC step itself, once per tick. The speed comes from the balance's
hall estimate; its derivative is too noisy for an acceleration term (0.4 m/s^2 swings after a 0.3 s filter), so
the rule checks the speed drop over the hold instead.

## Rule (as built)

    load drop  the q current low-passed over 0.05 s under 2.5 A while the one low-passed over 0.3 s is still
               4 A above it, faster than 1 m/s; it opens a 1.5 s window
    suspect    within that window: faster than 1 m/s (speed low-passed over 0.2 s)
               and |q current low-passed over 0.3 s| < 3.5 A        (3 N m at 0.86 N m/A)
               for 0.6 s, while the speed drops by less than 0.15 m/s (a drop starts the count again)
               never in the first 3 s of balancing, nor while a cut-off, limp mode, the battery limit or any
               tiltback (above 0.5 deg) is active
    respond    beeps (a 50-tick beep every 250 ticks) and the limp speed limit: speed tiltback at 10 km/h,
               growing at most 2.5 deg/s, at most 12 deg
    release    |q current low-passed over 0.3 s| > 7 A (a rider leans against the tiltback; a weaving empty
               wheel swings the current both ways), slower than 1 m/s, or the balance stops

## Results (clean 4054b4d0, every shipped scenario, quantum sizes 4000, 3000 and 2000)

| Scenario | Triggers | Speed tiltback max | Notes |
|---|---|---|---|
| rider-steps-off-15kmh | 1 in each run, at 16.00 s (1.0 s after the step-off) | 2.9 deg | falls at 17.72 s at 10.7-11.6 km/h |
| rider-steps-off-5kmh | 0 | 0 | falls first |
| downhill-cruise-2pct, coasting-stop (guards) | 0 | 0 | |
| balance-60s, ble-service-cut-standing, hall-loss-while-riding, hard-braking-30kmh, low-battery-tiltback, pothole-20kmh, ride-20kmh-stop | 0 | 0-0.4 deg (not from the detector) | |
| hall-single-line-loss, hill-climb-low-battery, overlean-top-speed, overlean-top-speed-30pct, overlean-no-pwm-tiltback | 0 | from limp mode, the battery limit or the speed setting | |

What option 2 changes in the step-off runs: nothing that matters. Without the detector the 15 km/h wheel falls
at 17.68 s at 11.0-12.2 km/h after 31.2 m; with it, at 17.72 s at 10.7-11.6 km/h after 31.3 m. The speed
tiltback has a 4 s time constant, so 1.7 s of suspicion gives only 2.9 deg. The gain is the beeps, which tell
people near the wheel that it is running away.

## Blind spots

- A rider who steps off while cruising down a slope (near-zero torque already) makes no load drop: no
  suspicion, as before.
- A rider whose load falls suddenly for another reason (a jump, both feet off a bump) and who then rides on
  quietly for 0.6 s at an even speed looks the same as a step-off. Hence the gentle response: never a cut or a
  hard stop on this signal alone, no abrupt pedal movement, and a release as soon as the rider leans against
  the tiltback.

## What it must never do

- Cut the motor or stop the wheel with a rider on: a false trigger at speed is a fall.
- Act at walking speed or below (under 1 m/s): the roll cut-off and the stop brake cover it.
- Act in the first 3 s after the balance starts, or while any other cut-off, limp or tiltback is active.

## Parked: option 1 (probe, then stop)

On a suspicion, ramp a nose-up probe tiltback; an empty wheel slows with little current, a rider leans against
it and the current rises. Confirmed, a stop tiltback brings the wheel under 1 km/h and the drive is disabled.
Time budget at 15 km/h: about 1 s to suspect, 0.8 s to probe, 1.2 s to stop, which is not sooner than the
tip-over in the simulator. The current an empty wheel draws while the probe slows it is not measured yet. It
needs the bench values first: the q current of a real rider at constant speed on flat ground (it also sets the
3.5 A), and the torque constant.

## Open items

- Bench: the q current of a real rider at constant speed (sets 3.5 A and 7 A) and of a real step-off (sets the
  load-drop thresholds).
- Rerun the results table when the suspension model is final.

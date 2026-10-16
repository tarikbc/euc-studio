# Riderless detection (clean profile): design note

Status: DRAFT for review, no code yet. Data: clean image on every shipped scenario (recordings of the
simulator's true torque, speed, pitch and roll at 100 Hz; analysis scripts kept outside the repo).

## What happens now

When the rider steps off at speed, every profile keeps balancing the light wheel until it falls on its side;
the roll cut-off then disables the drive.

| Scenario | Step-off | Wheel on its side (roll 40 deg) | Behaviour until then |
|---|---|---|---|
| rider-steps-off-15kmh | 15.0 s at 14.6 km/h | 17.3 s (2.3 s later), 11-14 km/h | rolls on, weaving |
| rider-steps-off-5kmh | 12.0 s at 5.0 km/h | 12.6 s (0.6 s later) | falls at once |

At 5 km/h the wheel is down before any detector could act, so only the 15 km/h case can gain.

## Signal

The steady motor torque. With a rider it carries the rider's weight against the drag and the rolling
resistance of the loaded tyre (about 9-13 N m at 15-20 km/h in the simulator); without a rider the light wheel
needs about 1 N m. At the step-off the torque falls from 11 N m to under 3 N m within 0.1 s, and the speed does
not change.

A rider who stops pushing (to slow down) also brings the torque near 0, but then the wheel decelerates: the
rider's centre of mass moves back and the balance follows it. So the candidate rule is:

    moving faster than 1 m/s (3.6 km/h)
    and |torque low-passed over 0.3 s| < 3 N m
    and acceleration low-passed over 0.3 s > -0.25 m/s^2   (not slowing down)
    for 0.3 s

The firmware has the torque as Iq (FOC, 12 kHz) and the acceleration from the hall period speed.

Check on every shipped scenario (clean, simulator truth):

| Scenario | First trigger |
|---|---|
| rider-steps-off-15kmh | 15.72 s, 0.72 s after the step-off |
| rider-steps-off-5kmh | never (the wheel falls first) |
| ride-20kmh-stop, hard-braking-30kmh, pothole-20kmh, overlean-top-speed, hill-climb-low-battery, balance-60s | never |

The torque term alone does not separate them. The lowest low-passed torque while moving is 0.05 N m in
ride-20kmh-stop (the rider starts to brake), 0.17 N m in overlean-top-speed, 0.25 N m in pothole-20kmh and
1.43 N m in hard-braking-30kmh. In each case only the deceleration term keeps the rule out. balance-60s never
moves faster than 1 m/s, so it does not test the rule.

## Blind spot

A rider cruising at constant speed down a gentle slope (1-2 %) needs almost no torque either, so the rule above
cannot tell that rider from an empty wheel. Any response must therefore be safe if a rider is on:

- never a cut or a hard stop at speed on this signal alone;
- no abrupt pedal movement.

## Response options (decision needed)

1. **Two stages: probe, then stop.** On the rule above, ramp a nose-up tiltback (8 deg/s to 4 deg). An empty
   wheel then decelerates at about g x tan(4 deg) = 0.7 m/s^2 with the torque still under 3 N m; a rider feels
   a tiltback start, leans against it, and the torque rises above 3 N m, which clears the suspicion and
   releases the tiltback. Confirmed after 0.3 s: a stop tiltback (up to 20 deg, about 3.5 m/s^2 for the empty
   wheel) to under 1 km/h, then the drive is disabled. Time budget at 15 km/h: 0.7 s detect + 0.8 s probe +
   1.2 s stop = about 2.7 s, which is not sooner than the 2.3 s tip-over in the simulator.
2. **One stage, gentle.** On the rule above: beeps and the limp speed limit (10 km/h, 2.5 deg/s, at most 12
   deg), released when the torque returns. Safe with a rider on, but it slows an empty wheel only a little
   before it falls.
3. **No change.** Keep the roll cut-off as the end of a riderless run; document the 2.3 s.

The goal "a controlled stop within about 1 s" is not reachable with this signal: detection alone takes 0.7 s,
and a faster rule (shorter filters, a higher torque threshold) starts to fire on riders who brake gently.

## What it must never do

- Cut the motor or stop the wheel with a rider on: a false trigger at speed is a fall.
- Act at walking speed or below (under 1 m/s): the roll cut-off and the stop brake cover it.
- Act in the first 3 s after the balance starts, or while any other cut-off, limp or tiltback is active.

## Proof plan (once a response is chosen)

- The two step-off scenarios: time from the step-off to under 1 km/h, against the tip-over now.
- No trigger on every other shipped scenario, plus two new guards from periph: cruising down a 2 % slope at
  20 km/h, and coasting to a stop with a rider (zero torque, gentle deceleration).
- Bench item: the torque level of a real rider at constant speed on flat ground (it sets the 3 N m).

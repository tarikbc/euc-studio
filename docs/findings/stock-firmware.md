# Stock firmware findings: Begode Extreme

What the simulator shows about the stock Begode Extreme controller firmware (build GW1825007), for
people who write firmware for this wheel. Each finding gives the behaviour, the scenario that shows it
and the command that reruns it, the numbers, how sure we are, and what our clean profile does about it.

This page describes behaviour only. The scenarios run the unmodified stock image in the emulator,
against the electrical plant ([models/electrical.md](../models/electrical.md)) and the body physics
([models/body.md](../models/body.md)). The scenario library and its caveats are in
[models/scenarios.md](../models/scenarios.md), the protections of both firmware profiles in
[firmware/docs/cutoffs.md](../../firmware/docs/cutoffs.md), and the clean profile's deliberate
differences in [firmware/docs/quirks.md](../../firmware/docs/quirks.md).

Rerun a scenario on the stock image (on our own Cortex-M3 core, the default):

```sh
cargo run --release -p euc-cli -- sim run --scenario scenarios/<name>.toml
```

Add `--firmware firmware/build/target-clean/euc-fw-begode-extreme.bin` (or `target-compat`) to ride
our builds, or compare two firmwares on one scenario with
`euc sim compare --scenario scenarios/<name>.toml --a stock --b clean`. Development builds with the
`unicorn` feature can add `--cpu unicorn` to check a run against the Unicorn reference core.

Status:
- **CONFIRMED in sim**: the stock image does it in the simulator, and the mechanism agrees with the
  static analysis of the firmware. The trigger and the numbers still rest on model values that are
  estimates until measured on a real wheel (motor constants, tire, pedals, rider).
- **PRELIMINARY**: the stock behaviour is real, but the run depends on a part of the model that we
  know is too simple. The reason is given.

"Maintainer's settings" is the `tarik-2026-10-15` preset (current-mode drive, hard pedals, PWM alarm
80 % with PWM tiltback, speed tiltback 69 km/h). "Neutral" is factory defaults with level pedals. The
numbers are from 2026-10-16, with the measured tire and the rider welded to the pedals.

## 1. One lost hall line goes undetected

**Behaviour.** When one of the three hall lines is lost (it reads high, like an open-collector input
pulled up), the hall code shows the invalid state 7 for one sector in six. The firmware's invalid-state
check counts consecutive invalid readings and resets on every valid one, so it never trips while the
wheel turns. The motor runs on with a wrong rotor angle in two sectors of six. The speed estimate counts
hall edges and sees two per electrical turn instead of six, so everything that uses speed works from a
third of the truth: the speed tiltback set to 20 km/h acts at 60 km/h, and the speed alarms and the
speed-dependent balance gains are off by the same factor.

**Scenario.** `hall-single-line-loss` (neutral settings): line a is lost at 20 km/h.

**Numbers.** No cutoff, no alert. The firmware speed reads 6.7 km/h at a true 20.1 km/h (RMS error
13.4 km/h, 0.10 km/h before the fault). Torque ripple, as a standard deviation, is 19.6 N·m (3.8 N·m
before), with 22.8 A of Iq. The pitch disturbance is small: a standard deviation of 0.57° (0.27°
before), at most 3.5°.

**Status.** CONFIRMED in sim.

**Clean profile.** Finds the stuck line after three invalid-state episodes in a row, about three
electrical turns (27 ms at 20 km/h; `HALL_SINGLE_LINE_UNDETECTED` off). It rebuilds the hall state from
the two working lines, so the angle, the step count and the speed estimate are right again, raises alert
0x40 with the fault beeps, and goes into limp mode: a speed tiltback at 10 km/h at most, with no
automatic restart after the next stop. No cut. In fw's comparison of this scenario the torque ripple
after the fault is 0.60 N·m on clean against 16.5 N·m on stock (Iq 0.75 against 20.6 A); the pitch
rises on clean only because the 10 km/h tiltback meets a simulated rider who holds 20 km/h.

## 2. Hall connector off: cut at +70 ms

**Behaviour.** With all three lines lost the code reads 7 continuously. After its invalid-state count
runs out (62.5 ms) the firmware latches a hall fault, sets alert 0x40, turns the inverter off and leaves
the motor free-wheeling until a power cycle. At speed the wheel then nose-dives.

**Scenario.** `hall-loss-while-riding` (neutral): the connector comes off at 20 km/h.

**Numbers.** Hall fault and alert 70 ms after the connector comes off; the shell passes −30° nose down
140 ms after it.

**Status.** CONFIRMED in sim.

**Clean profile.** The same cut. Its 62.5 ms timer reads the raw lines also while it rebuilds a lost line,
so a second lost line still ends in this fault. Compat and clean cut at the same 15.07 s, with the hall
fault and alert 0x40.

## 3. Overlean: the PWM saturates, and the warning time depends on the motor

**Behaviour.** A rider who keeps accelerating past what the motor can give, and ignores the tiltback,
meets the voltage limit. The back-EMF and the winding's reactance use up the headroom, the q-current
loop saturates (PWM 100 %), the current stops following its reference, the balance command runs to its
limit, and the nose goes down. The PWM alarm (80 %) and, with `e`, the PWM tiltback start before that,
but the PWM does not climb gradually: it sits near 55-78 % above 60 km/h, touches 80 % in short peaks,
and then saturates (finding 10 explains why). So the time from the first alarm to the loss of balance depends on the motor's
constants. The overload detector (|Vq| above 31000 for 1.52 s) or the error detector then disables the
drive.

**Scenarios.** `overlean-top-speed` (maintainer's settings, full battery) and
`overlean-top-speed-30pct` (the same with a 30 % battery): 5 km/h per second towards 100 km/h, with a
rider who never eases off.

**Numbers.** Full battery: first PWM alarm at 64.7 km/h; the drive is disabled 5.8 s later at
71.0 km/h, and the wheel falls 0.15 s after that; the PWM tiltback is at 8.4°. On a 30 % battery: the
alarm comes at 57.5 km/h, and the drive is disabled 4.0 s later at 66.4 km/h. With the earlier motor and
tire guesses the nose went down 0.4 s after the first alarm; the measured tire and pedals give 4-6 s.
The pedal height (an estimate, ±15 mm) moves the cutout along the ramp by several seconds, but its
speed by less than 1 km/h.

**Status.** CONFIRMED in sim for the mechanism: saturation, then the cutout, with an alarm that does
not measure the margin that is left. The speeds and the warning time depend on the model until the
motor's inductance and flux are measured.

**Clean profile.** Adds a headroom tiltback (`PWM_TILTBACK_VQ_ONLY` off): the PWM tiltback (2.7° per
second, at most 12°) also starts whenever the voltage vector |Vd, Vq| passes 75 %, with or without `e`.
The stock check reads only Vq before the circle limit, which sits near 66 % at speed while the vector is
already full. The PWM alarm (`W` `P`, 80 %) and the overload and error detectors are the same. The
telemetry PWM % still reports Vq, as stock does; whether it should report the true headroom is an open
question ([cutoffs.md](../../firmware/docs/cutoffs.md), "Telemetry PWM % and the true headroom").

## 4. Hard braking holds

**Behaviour.** A rider who brakes as hard as they dare from 30 km/h (asking for up to 0.6 g) gets a
clean stop. The braking current stays under the firmware's 280 A limit, and regeneration into the
battery stays small.

**Scenario.** `hard-braking-30kmh` (maintainer's settings).

**Numbers.** From 29.2 km/h at the brake: below 15 km/h after 1.5 s, below 5 km/h after 2.9 s
(2.3 m/s² mean), stopped after 4.7 s. The braking torque peaks at −161 N·m with −187 A of Iq; −14 A at
the battery, bus at 131.1 V; pedals 6.8° nose up. No cutoff, no alert. (With a more aggressive
simulated rider, who set off a 4 Hz rider-wheel oscillation before braking, the peak was −234 A and the
stop took 2.8 s: still no cutoff.)

**Status.** CONFIRMED in sim.

**Clean profile.** Not changed yet: the same current limits (current mode: command ±1050 for ±280 A of
Iq; voltage mode: back-off above 280 A). The phase over-current trip is off in both profiles until a
bench measures a level (`NO_OVERCURRENT_TRIP`).

## 5. Low battery: an alarm early, then a tiltback that keeps growing

**Behaviour.** Below 105.6 V the firmware sets the low-voltage alert (0x08) and beeps. Below 99.2 V
(filtered bus voltage) it also adds a nose-up offset to the balance target whenever the wheel rolls
forward. The offset ramps at about 2.4° per second at first and keeps growing towards 48°. It holds until the
voltage is back above about 100.2 V, and then releases over about 20 s. A rider who keeps going is
pushed further and further back.

**Scenario.** `low-battery-tiltback` (neutral): 98.5 V at rest, a ride towards 15 km/h with a rider
who eases off when the pedals tip back.

**Numbers.** Alert 0x08 at the first control tick (1.7 s). The rider sets off at 8.5 s, once the
pedals are level. The pedals pass 8° nose up at 12.6 s; the rider, easing off, gets no faster than
11.4 km/h and stops at 16.0 s with the pedals 22° nose up (a battery tiltback of 15°). The wheel then
rolls back at up to 1.5 km/h; the tiltback does not let go while the voltage stays low.

**Status.** CONFIRMED in sim.

**Clean profile.** The same ramp (2.38° per second nose up below 99.20 V), but at most 8°; from there
the 10 km/h speed limit of limp mode takes over until the voltage is back in the inner window
(`BATTERY_TILTBACK_UNBOUNDED` off).

## 6. Hill climb on a low battery

**Behaviour.** A nearly empty battery that rests above the tiltback threshold sags below it on a climb.
The tiltback then starts in the middle of the hill and keeps growing for as long as the rider climbs.

**Scenario.** `hill-climb-low-battery` (maintainer's settings, 3 % battery, 101.8 V at rest): 25 km/h,
then a 20 % grade from 40 m, with a rider who keeps climbing whatever the tiltback.

**Numbers.** The low-voltage alert is on from boot (101.8 V is below 105.6 V). The climb starts at
17.0 s; the battery tiltback passes 2° at 20.9 s and reaches 26° at 36 s (pedals 30° nose up), with the
bus at 97 V and Iq up to 79 A. No cutoff. If the run goes on, the tiltback reaches 40°, the wheel
stalls on the slope and the firmware disables the drive at about 41 s.

**Status.** CONFIRMED in sim for the tiltback. What a rider does on a steep slope while the pedals tip
back is not modelled well yet, so the outcome after 36 s is not a finding. (The default rider, who eases
off for tiltback, falls at 23.7 s: at 10° nose up and 15 km/h the firmware swings Iq from +25 to −227 A
in 0.3 s while the rider leans back only 5°. Whether a real rider holds on there is open.)

**Clean profile.** The battery tiltback stops at 8°, and the 10 km/h limit of limp mode takes over
(`BATTERY_TILTBACK_UNBOUNDED` off; see finding 5).

## 7. The gyro clips at ±500 °/s, and the attitude stays wrong (PRELIMINARY)

**Behaviour.** The firmware sets the IMU's gyro range to ±500 °/s. An impact that turns the shell
faster saturates the gyro: the attitude filter integrates 500 °/s where the shell turns faster, and its
accelerometer correction takes seconds to bring the estimate back. Until then the firmware balances its
wrong idea of level: the wheel rides on with the pedals tipped, and the pitch and error cut-offs, which
work on the same estimate, do not see the real angle.

**Scenario.** `pothole-20kmh` (maintainer's settings): a 2 cm deep, 50 cm long pothole with steep
edges at 20 km/h.

**Numbers.** The far edge takes the wheel from 20 to 16.2 km/h in 20 ms, and the firmware answers with
253 A forward. The shell pitch rate passes the gyro range (820 °/s and then 1360 °/s in an earlier run
of the same scenario), the shell dips to −54°, and the `upright` check fails at 20.2 s. In fw's analysis
the firmware's estimate never passes 35°, so its 40° cut-offs never trip. From 20.8 s to 22 s it believes the shell is
at +1.2° while the shell is at −19.8° (RMS error 21°), and the wheel rides on nose down. fw's
stock-against-clean comparison of the same scenario, on the model before the rider weld: stock RMS error
27.5° (+1.4° believed against −26.0°), compat 18.6° (same failure, chaotic impact), clean 4.3°.

**Status.** PRELIMINARY. The body is rigid and has no suspension (the Extreme has a rear shock), the
rider has no knees, and the tire is a soft contact, so impacts reach the shell harder than on the real
wheel. The gyro range and the slow recovery are the firmware's own; whether a real pothole turns the
shell faster than 500 °/s is not known yet.

**Clean profile.** Runs the gyro at ±2000 °/s and re-anchors faster after a saturated sample
(`GYRO_500DPS_CLIP` off), so it never clips here and tracks within about 2°. Because it sees the true
+44° swing that follows its own full reverse command, its error cut-off then trips and the wheel falls:
the stock firmware stays up here only because its attitude is wrong. Limiting that overshoot is an open
question in [cutoffs.md](../../firmware/docs/cutoffs.md), to settle once the body has suspension.

## 8. BLE has no authentication, and service sequences cut the motor under a rider

**Behaviour.** The BLE link has no pairing and no authentication: any phone in range can send
commands. Three two-letter service sequences (IMU calibration request, firmware update, drive-mode
toggle) are accepted whenever the wheel stands still, whether or not someone stands on it, and each one
cuts the motor at once. Settings that change how the wheel behaves (roll modes, tiltback, trims) are
accepted at any speed.

**Scenario.** `ble-service-cut-standing` (maintainer's settings): a rider balances at a standstill; at
10 s a phone sends "c", and 0.3 s later "y".

**Numbers.** Stock and compat cut the motor 10 ms after the "y" (drive fault and balance disabled), and
wheel and rider fall 1.2 s later. Clean answers `SERVICE:DENIED` and keeps balancing (pitch 1.3°).

**Status.** CONFIRMED in sim. This run is the evidence for rule 7 of [SAFETY.md](../../SAFETY.md).

**Clean profile.** Runs the service sequences only when nobody rides (the wheel is off, on its side, or
cut with the handle button) and the power button is pressed within 10 s, and answers
`SERVICE:CONFIRM`, `SERVICE:OK` or `SERVICE:DENIED` (`SERVICE_WITHOUT_CONFIRM` off). It applies every
ride setting only below 0.5 m/s and answers "stop first" otherwise (`SETTINGS_WHILE_MOVING` off): the
roll cut-off modes, the PWM tiltback switch and the `W` trims (stock already checks the speed for the
pedal modes, the speed tiltback and the PWM alarm). It also range-checks every field of a loaded
settings record (`RECORD_FIELDS_UNCHECKED` off), so a stock record or the backup registers cannot bring
in, for example, a slow roll cut-off at 115°.

## 9. A riderless wheel keeps driving

**Behaviour.** When the rider steps off, the firmware does not notice: it has no rider sensor and no
load or lift detection, and a light wheel without a rider is easy to balance. It keeps driving the wheel
on its own until the wheel tips onto its side; then the roll cut-off disables the drive, about 70 ms
after the roll passes 40°. Until then a riderless wheel can roll on at speed.

**Scenarios.** `rider-steps-off-15kmh` and `rider-steps-off-5kmh` (maintainer's settings): the rider
steps off at 15 or 5 km/h; the stepping foot gives the shell a short sideways push. These are also the
regression guards for the cut-offs ([cutoffs.md](../../firmware/docs/cutoffs.md), "Acceptance: what
must still cut").

**Numbers.** At 15 km/h: the riderless wheel rolls on for 2.3 s until the roll passes 40°, and the
drive is disabled 70 ms later, 2.4 s after the step-off, at 10.8 km/h. At 5 km/h: the roll passes 40°
after 0.56 s and the drive is disabled after 0.63 s, at 4.0 km/h. Stock, compat and clean behave the
same, and the wheel ends standing still in all of them.

**Status.** CONFIRMED in sim that nothing but the tip-over ends the drive. How long a real riderless
wheel stays up depends on how the rider steps off; in the model it is set by the sideways push.

**Clean profile.** Not changed yet: the same cut-offs. Riderless detection (no load on the pedals) is
planned.

## 10. The PWM value reads Vq only, so it under-reads the voltage headroom (PRELIMINARY)

**Behaviour.** The stock firmware's PWM value, which the app shows as PWM % and which drives the PWM
alarm and the PWM tiltback, is |Vq| before the FOC's circle limit. The inverter must apply the whole
vector |Vd, Vq|: at speed the winding's reactance needs a d voltage of about ω·L·Iq, so the vector is
larger than Vq, and the value shows less headroom used than there is. When the vector reaches the
circle limit, the current loop's outputs rail and Vq jumps: that is the jump from about 70 % to
saturation in finding 3.

**Scenario.** `overlean-top-speed`, run on our compat build with probes on the FOC output after the
circle limit (the stock map does not name Vd yet; compat follows stock here, with the first PWM alarm
at 26.3 s against 25.8 s). Percentages are on the app's PWM scale (|Vq| / 315), where the circle limit
is at 103 %.

**Numbers.** On the default motor values, from 22 to 27 s (57 to 66 km/h) the app shows 62-70 % while
the vector is at 74-84 %. At 27.6 s the vector reaches the limit, and the firmware disables the drive at
30.9 s at 71.4 km/h. The gap grows with the motor's inductance, an estimate (200 µH, not measured), so
the same run with the inductance halved and doubled gives:

| Inductance | First PWM alarm | Shown / true at the alarm | Saturation | Cut |
|---|---|---|---|---|
| 100 µH (0.5×) | 26.1 s, 65 km/h | 73 % / 78 % (gap 5) | none: the speed tiltback holds the wheel near 70 km/h | none in 40 s |
| 200 µH (estimate) | 26.3 s, 66 km/h | 70 % / 84 % (gap 14) | 27.6 s | 30.9 s at 71.4 km/h |
| 400 µH (2×) | 18.3 s, 43 km/h | 53 % / 80 % (gap 27) | 18.4 s | 19.5 s at 48.3 km/h |

The "shown" and "true" values are 0.1 s means; the alarm itself fires on short Vq peaks above 80 %. At
twice the estimate the alarm comes 0.1 s before saturation, so it gives no useful warning.

**Status.** PRELIMINARY for the size of the gap. That the value leaves out Vd is in the firmware and
holds for any motor; how much it leaves out scales with the inductance: 5 points at half the estimate,
14 at the estimate, 27 at twice it. It needs the measured inductance of the Extreme's motor.

**Clean profile.** The PWM tiltback also starts whenever |Vd, Vq| passes 75 %, with or without `e`
(`PWM_TILTBACK_VQ_ONLY` off). On the default motor values it starts 4.4 s earlier than the stock
firmware's. With the PWM tiltback switched off (`x`) the stock firmware falls at 78.3 km/h, while clean
slows to 70.3 km/h and stays up. Two limits: a rider who ignores the tiltback (this scenario) still
falls on clean (drive disabled at 31.8 s at 69.9 km/h); and at twice the inductance clean's tiltback
starts only 0.12 s before saturation, and the wheel falls at 18.8 s, so the 75 % margin is too small for
such a motor. The telemetry PWM % still reports Vq, as stock does; whether it should report the true
headroom is an open question ([cutoffs.md](../../firmware/docs/cutoffs.md), "Telemetry PWM % and the
true headroom").

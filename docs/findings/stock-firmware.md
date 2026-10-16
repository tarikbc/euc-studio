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
numbers are from 2026-10-16, with the measured tire, the rear suspension (measured spring, estimated
stroke and linkage), and a rider who is welded to the pedals, steers to keep the line, feels the
firmware's own tiltback and lets go of the pedals after a cut that tips the wheel. Our builds in these
runs: clean `4054b4d0`, compat `00bb2466`. Each fall-or-survive result was checked at three emulator
quantum sizes (`--max-quantum` 4000, 3000 and 2000); a result that changes between them is marked
**borderline**.

## 1. One lost hall line goes undetected

**Behaviour.** When one of the three hall lines is lost (it reads high, like an open-collector input
pulled up), the hall code shows the invalid state 7 for one sector in six. The firmware's invalid-state
check counts consecutive invalid readings and resets on every valid one, so it never trips while the
wheel turns. The motor runs on with a wrong rotor angle in two sectors of six. The speed estimate counts
hall edges and sees two per electrical turn instead of six, so everything that uses speed works from a
third of the truth: the speed tiltback set to 20 km/h acts at 60 km/h, and the speed alarms and the
speed-dependent balance gains are off by the same factor.

**Scenario.** `hall-single-line-loss` (neutral settings): line a is lost at 20 km/h.

**Numbers.** No cutoff, no alert. The firmware speed reads 6.7 km/h at a true 20.2 km/h (RMS error
13.5 km/h, 0.14 km/h before the fault). Torque ripple, as a standard deviation, is 19.0 N·m (3.8 N·m
before), with 22.1 A of Iq. The pitch disturbance is small: a standard deviation of 0.59° (0.26°
before), at most 3.5°. The same at all three quantum sizes.

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
120 ms after it, and the rider lets go of the pedals at 15.19 s. The same at all three quantum sizes.

**Status.** CONFIRMED in sim.

**Clean profile.** The same cut. Its 62.5 ms timer reads the raw lines also while it rebuilds a lost line,
so a second lost line still ends in this fault. Compat and clean cut at the same 15.07 s, with the hall
fault and alert 0x40.

## 3. Overlean: the PWM saturates, and the warning time depends on the motor

**Behaviour.** A rider who keeps accelerating past what the motor can give, and ignores the tiltback,
meets the voltage limit. The back-EMF and the winding's reactance use up the headroom, the q-current
loop saturates (PWM 100 %), the current stops following its reference, the balance command runs to its
limit, and the nose goes down. The PWM alarm (80 %) and, with `e`, the PWM tiltback start before that,
but the PWM does not climb gradually: it sits below 80 % and then saturates (finding 10 explains why).
So the time from the first alarm to the loss of balance depends on the motor's constants. The overload
detector (|Vq| above 31000 for 1.52 s) or the error detector then disables the drive.

**Scenarios.** `overlean-top-speed-30pct` (maintainer's settings, 30 % battery, a rider who ignores
tiltback and leans up to 15°), `overlean-top-speed` (full battery, the same rider leaning up to 18°),
and `overlean-no-pwm-tiltback` (30 % battery, the default rider who eases off for tiltback, the PWM
tiltback turned off by `x`): 5 km/h per second towards 100 km/h.

**Numbers.** All three fall at all three quantum sizes. On a 30 % battery: first PWM alarm at 23.2 s,
saturation at 25.8 s, and the drive is disabled at 27.5 s at 67.5 km/h, 4.3 s after the alarm; the
wheel falls at once (27.5-28.2 s over the three sizes), with the PWM tiltback at 5°. On a full battery
with the harder-leaning rider: alarm at 26.0 s, saturation 0.1 s later, drive disabled at 27.7 s at
73.3 km/h, only 1.7 s after the alarm. With the PWM tiltback off, the easing rider gets no warning on
the pedals: alarm at 23.4 s, saturation at 29.3 s, drive disabled at 30.8 s at 70.4 km/h
(29.5-30.8 s). A rider leaning at most 15° on a full battery is held near 71 km/h by the speed and PWM
tiltbacks; from 16° the stock wheel falls (lean sweep in [scenarios.md](../models/scenarios.md)). With
the earlier motor and tire guesses the nose went down 0.4 s after the first alarm.

**Status.** CONFIRMED in sim for the mechanism: saturation, then the cutout, with an alarm that does
not measure the margin that is left; robust at all three quantum sizes. The speeds and the warning time
depend on the model until the motor's inductance and flux are measured.

**Clean profile.** Adds a headroom tiltback (`PWM_TILTBACK_VQ_ONLY` off): the PWM tiltback (2.7° per
second, at most 12°) also starts whenever the voltage vector |Vd, Vq| passes 75 %, with or without `e`.
The stock check reads only Vq before the circle limit, which sits near 66 % at speed while the vector is
already near full. The PWM alarm (`W` `P`, 80 %) and the overload and error detectors are the same, and
the telemetry PWM % still reports Vq, as stock does (an open question in
[cutoffs.md](../../firmware/docs/cutoffs.md), "Telemetry PWM % and the true headroom").

In all three scenarios clean holds at all three quantum sizes where stock and compat fall; with the
PWM tiltback off it tilts back from 20.0 s (at most 3.1°), and the easing rider holds about 57 km/h.
The margin is narrow, though: in the lean sweep clean holds where stock falls only for a lean of
16-18° on a full battery and at 15° on a 30 % battery; beyond that both fall.

Cause, from one-quirk-off clean builds: only the headroom tiltback. With `PWM_TILTBACK_VQ_ONLY` on,
clean falls like stock (3 of 3); with the battery cap or riderless detection put back to stock, it
still holds. Mechanism: clean's tiltback starts when the vector passes 75 %, at about 20 s and 51 km/h
on a 30 % battery, while the displayed Vq is only 64 %, and it reaches its 12° limit about 5 s later.
The nose-up target makes the balance law ask for less forward torque even though the rider keeps the
full lean, so the acceleration falls from about 3.9 to 0.8 km/h per second and the vector never
saturates. With the stock rule the tiltback starts about 5 s later and has 1-2° when the vector
saturates. Two caveats: in the runs with a rider who ignores the tiltback, the save comes from the
balance law asking for less torque, not from the rider; and the motor values are ESTIMATES: at twice
the estimated inductance clean's tiltback starts only after the vector saturates, so the save is not
guaranteed for every motor (fw is making the trigger robust).

## 4. Hard braking holds

**Behaviour.** A rider who brakes as hard as they dare from 30 km/h (asking for up to 0.6 g) gets a
clean stop. The braking current stays under the firmware's 280 A limit, and regeneration into the
battery stays small.

**Scenario.** `hard-braking-30kmh` (maintainer's settings).

**Numbers.** Braking from about 29 km/h: below 1 km/h after 4.8 s. The braking torque peaks at
−154 N·m with −179 A of Iq; −14 A at the battery, bus at 131.2 V; pedals 6.9° nose up. No cutoff, no
alert, at all three quantum sizes. (With a more aggressive simulated rider, who set off a 4 Hz
rider-wheel oscillation before braking, the peak was −234 A and the stop took 2.8 s: still no cutoff.)

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

**Numbers.** Alert 0x08 at the first control tick (1.7 s). The tiltback grows as soon as the wheel
rolls; the rider eases off from 2° of it, gets no faster than 6.6 km/h, and the pedals pass 8° at
10.1 s and reach 13.5°. The rider stops, rolls back a little and sets off again once the tiltback has
gone: stop and go, for as long as the voltage stays low. The same at all three quantum sizes.

**Status.** CONFIRMED in sim.

**Clean profile.** The same ramp (2.38° per second nose up below 99.20 V), but at most 8°; from there
the 10 km/h speed limit of limp mode takes over until the voltage is back in the inner window
(`BATTERY_TILTBACK_UNBOUNDED` off). In this scenario the pedals pass 8° at 12.3 s against 10.1 s on
stock, and the pitch stays below 10.3° against 13.5°. Both stay up, with no cut, at all three quantum
sizes.

## 6. Hill climb on a low battery

**Behaviour.** A nearly empty battery that rests above the tiltback threshold sags below it on a climb.
The tiltback then starts in the middle of the hill and keeps growing for as long as the rider climbs.

**Scenario.** `hill-climb-low-battery` (maintainer's settings, 3 % battery, 101.8 V at rest): 25 km/h,
then a 20 % grade from 40 m, with a rider who keeps climbing whatever the tiltback.

**Numbers.** The low-voltage alert is on from boot (101.8 V is below 105.6 V). The climb starts at
17.0 s; the battery tiltback passes 2° at 20.9 s and reaches 26° at 36 s (pedals 31° nose up), with the
bus at 97.3 V and Iq up to 73 A. No cutoff, at all three quantum sizes. Before the suspension, a run
that went on reached 40° of tiltback, stalled on the slope, and the firmware disabled the drive at about
41 s.

**Status.** CONFIRMED in sim for the tiltback. What a rider does on a steep slope while the pedals tip
back is not modelled well yet, so the outcome after 36 s is not a finding. A fall of the default
rider, who eases off for tiltback, was a model effect and is fixed: before the rider could steer, a
wheel that slowed on the grade turned across it (heading 6° at 23.2 s, 27° at 23.4 s) and fell at
23.7 s, after which the firmware's current swung from +25 to −227 A. With rider steering the default
rider stays up on the stock firmware: it eases off from 20 s, stops at 26.2 s with the pedals 12.7°
nose up and then rolls back at about 3 km/h, with Iq positive throughout (25 to 57 A)
([scenarios.md](../models/scenarios.md), "Open questions").

**Clean profile.** The battery tiltback stops at 8°, and the 10 km/h limit of limp mode takes over
(`BATTERY_TILTBACK_UNBOUNDED` off; see finding 5). In this scenario: battery tiltback at most 8.0°
against 26.2° on stock, plus the speed tiltback of the 10 km/h limit (12° in fw's run); pitch at most
21.8° against 30.6°; lowest speed after 15 s 19.8 against 19.2 km/h. Both stay up, with no cut, at all
three quantum sizes.

## 7. The gyro clips at ±500 °/s, and the attitude stays wrong (PRELIMINARY)

**Behaviour.** The firmware sets the IMU's gyro range to ±500 °/s. An impact that turns the shell
faster saturates the gyro: the attitude filter integrates 500 °/s where the shell turns faster, and its
accelerometer correction takes seconds to bring the estimate back. Until then the firmware balances its
wrong idea of level: the wheel rides on with the pedals tipped, and the pitch and error cut-offs, which
work on the same estimate, do not see the real angle.

**Scenario.** `pothole-20kmh` (maintainer's settings): a 2 cm deep, 50 cm long pothole with steep
edges at 20 km/h, and the same pothole 3 and 5 cm deep.

**Numbers.** With the rear suspension the 2 cm pothole turns the shell at most about 250 °/s (from the
true pitch at 100 Hz, so short peaks are higher), inside the range: the shell dips to −8.4°, and the
firmware's attitude stays within 0.4° (RMS) of the truth. The 3 cm pothole reaches about 550 °/s and
leaves the stock attitude 1.6° off (RMS over 20.8-22 s). The 5 cm pothole reaches about 700 °/s, the
shell dips to −42°, and afterwards the stock firmware believes +1.0° while the shell is at −4.0° (RMS
error 5.0°). Stock and compat ride through all three, at all three quantum sizes. On the rigid body
before the suspension, the 2 cm pothole turned the shell at up to 1360 °/s and left an error of 21°.

**Status.** PRELIMINARY. The gyro range and the slow recovery are the firmware's own. How fast a real
pothole turns the shell depends on the suspension's stroke and linkage (estimates) and the rider's
knees (not modelled).

**Clean profile.** Runs the gyro at ±2000 °/s and re-anchors faster after a saturated sample
(`GYRO_500DPS_CLIP` off): its attitude error is 0.7° (3 cm) and 0.5° (5 cm). In the 5 cm pothole it
therefore sees the real dip, which reaches about −40°, and the result is **borderline**: at quantum
3000 and 2000 its 40° pitch cut-off trips at 19.89 s and the wheel falls; at 4000 the dip stops at
−40.4° and it rides through. The stock firmware rides through there because its attitude is wrong. How
the cut-offs should treat a deep, recovering dip is an open item in
[cutoffs.md](../../firmware/docs/cutoffs.md).

## 8. BLE has no authentication, and service sequences cut the motor under a rider

**Behaviour.** The BLE link has no pairing and no authentication: any phone in range can send
commands. Three two-letter service sequences (IMU calibration request, firmware update, drive-mode
toggle) are accepted whenever the wheel stands still, whether or not someone stands on it, and each one
cuts the motor at once. Settings that change how the wheel behaves (roll modes, tiltback, trims) are
accepted at any speed.

**Scenario.** `ble-service-cut-standing` (maintainer's settings): a rider balances at a standstill; at
10 s a phone sends "c", and 0.3 s later "y".

**Numbers.** Stock and compat cut the motor 10 ms after the "y" (drive fault and balance disabled);
the rider lets go 0.9 s later, and wheel and rider fall 1.0 s after the cut. Clean answers
`SERVICE:DENIED` and keeps balancing (pitch 1.3°). The same at all three quantum sizes.

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
steps off at 15 or 5 km/h; the stepping foot gives the shell a short sideways push, and the rider's
steering stops with the step-off. These are also the
regression guards for the cut-offs ([cutoffs.md](../../firmware/docs/cutoffs.md), "Acceptance: what
must still cut").

**Numbers.** At 15 km/h: the riderless wheel rolls on for 2.7 s until the roll passes 40°, and the
drive is disabled 70 ms later, 2.75 s after the step-off, at about 7.8 km/h. At 5 km/h: the roll passes
40° after 0.56 s and the drive is disabled after 0.64 s, at 4.0 km/h. The same at all three quantum
sizes.

**Status.** CONFIRMED in sim that nothing but the tip-over ends the drive. How long a real riderless
wheel stays up depends on how the rider steps off; in the model it is set by the sideways push.

**Clean profile.** The same cut-offs, plus riderless detection (`RIDERLESS_UNDETECTED` off, a
load-drop rule): 1.0 s after the 15 km/h step-off clean suspects a riderless wheel, beeps and applies
its limp speed limit (2.9° of speed tiltback); the wheel still rolls on until the tip-over cut. With a
rider on it never triggers in the library, including the two guards `downhill-cruise-2pct` and
`coasting-stop` (fw's runs, all three quantum sizes; an earlier rule triggered in `coasting-stop` and
was replaced).

## 10. The PWM value reads Vq only, so it under-reads the voltage headroom (PRELIMINARY)

**Behaviour.** The stock firmware's PWM value, which the app shows as PWM % and which drives the PWM
alarm and the PWM tiltback, is |Vq| before the FOC's circle limit. The inverter must apply the whole
vector |Vd, Vq|: at speed the winding's reactance needs a d voltage of about ω·L·Iq, so the vector is
larger than Vq, and the value shows less headroom used than there is. When the vector reaches the
circle limit, the current loop's outputs rail and Vq jumps: that is the jump from about 70 % to
saturation in finding 3.

**Scenario.** `overlean-top-speed` as it was before its rider leaned 18° (the default 15° rider) and
before the measured spring, run on our compat build with probes on the FOC output after the circle
limit (the stock map does not name Vd yet; compat follows stock here, with the first PWM alarm
at 29.1 s against 30.1 s). Percentages are on the app's PWM scale (|Vq| / 315), where the circle limit
is at 103 %.

**Numbers.** On the default motor values, from 22 to 29 s (57 to 69 km/h) the app shows 62-72 % while
the vector is at 74-86 %. At 30.3 s the vector reaches the limit, and the firmware disables the drive at
34.4 s at 72.5 km/h. The gap grows with the motor's inductance and shrinks with its flux, both
estimates (200 µH and 20.5 mWb, not measured), so the same run with each changed gives:

| Motor | First PWM alarm | Shown / true at the alarm | Saturation | Cut |
|---|---|---|---|---|
| inductance 100 µH (0.5×) | 29.2 s, 69 km/h | 75 % / 81 % (gap 5) | none: the speed tiltback holds the wheel near 70 km/h | none in 40 s |
| estimate (200 µH, 20.5 mWb) | 29.1 s, 69 km/h | 71 % / 85 % (gap 14) | 30.3 s | 34.4 s at 72.5 km/h |
| inductance 400 µH (2×) | 18.6 s, 45 km/h | 55 % / 80 % (gap 26) | 18.7 s | 19.7 s at 49.2 km/h |
| flux 16.4 mWb (0.8×) | 30.1 s, 70 km/h | 60 % / 81 % (gap 21) | 30.3 s | 33.8 s at 72.6 km/h |
| flux 24.6 mWb (1.2×) | 22.9 s, 59 km/h | 75 % / 84 % (gap 9) | none: the speed tiltback holds the wheel near 69 km/h | none in 40 s |

The "shown" and "true" values are 0.1 s means; the alarm itself fires on short Vq peaks above 80 %. At
twice the inductance the alarm comes 0.1 s before saturation, and at 0.8× the flux 0.2 s before: then
it gives no useful warning.

**Status.** PRELIMINARY for the size of the gap. That the value leaves out Vd is in the firmware and
holds for any motor; how much it leaves out scales with the inductance (5 points at half the
estimate, 14 at the estimate, 26 at twice it) and falls with the flux (21 points at 0.8×, 9 at 1.2×).
The inductance and the flux are ESTIMATES. The real Extreme is ridden well above 49 km/h, so the
doubled inductance (cut at 49 km/h) is unlikely for the real motor.
Ride logs from a real wheel at high speed (speed, PWM, voltage and current) would bound both values.

**Clean profile.** The PWM tiltback also starts whenever |Vd, Vq| passes 75 %, with or without `e`
(`PWM_TILTBACK_VQ_ONLY` off). On the default motor values it starts at 22.3 s, 7.8 s before the stock
firmware's first PWM alarm, and the vector never reaches 80 %. In fw's runs with the PWM tiltback
switched off (`x`), the stock firmware gives no PWM tiltback at all, while clean's headroom tiltback
starts at about 20.5 s (52 km/h) and reaches 11-12°; whether the wheel then falls near the limit
depends on the emulator's timing, so that is not a fall-or-survive result. Two limits: the save covers
only a narrow lean range (finding 3); and at twice the inductance the vector
saturates (18.10 s) before clean's tiltback starts (18.11 s), and the wheel falls at 19.5 s at
48.5 km/h, so the 75 % threshold is too late for such a motor. The telemetry PWM % still reports Vq,
as stock does; whether it should report the true headroom is an open question ([cutoffs.md](../../firmware/docs/cutoffs.md), "Telemetry PWM % and the
true headroom").

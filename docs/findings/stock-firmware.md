# Stock firmware findings: Begode Extreme

What the simulator shows about the stock Begode Extreme controller firmware (build GW1825007), for
people who write firmware for this wheel. Each finding names the scenario that shows it, the command
that reruns it, the numbers, how sure we are, and what our clean profile does about it.

This page describes behaviour only. The scenarios run the unmodified stock image in the emulator,
against the electrical plant ([models/electrical.md](../models/electrical.md)) and the body physics
([models/body.md](../models/body.md)); the scenario library and its caveats are in
[models/scenarios.md](../models/scenarios.md), and the clean profile's deliberate differences in
[firmware/docs/quirks.md](../../firmware/docs/quirks.md).

Rerun any scenario with the development backend:

```sh
cargo run --release -p euc-cli --features unicorn -- sim run --scenario scenarios/<name>.toml
```

Status:
- **CONFIRMED in sim**: the stock image does it in the simulator, and the mechanism agrees with the
  static analysis of the firmware. The trigger and the numbers still rest on model values that are
  estimates until measured on a real wheel (motor constants, tire, pedals, rider).
- **PRELIMINARY**: the stock behaviour is real, but the run depends on a part of the model we know
  is too simple; the reason is given.

"Maintainer's settings" is the `tarik-2026-10-15` preset (current-mode drive, hard pedals, PWM alarm
80 % with PWM tiltback, speed tiltback 69 km/h); "neutral" is factory defaults with level pedals.

## 1. One lost hall line goes undetected

**Behaviour.** When one of the three hall lines is lost (it reads high, as an open-collector input
pulled up), the hall code shows the invalid state 7 for one sector in six. The firmware's invalid-state
check counts consecutive invalid readings and resets on every valid one, so it never trips while the
wheel turns. The motor runs on with a wrong rotor angle in two sectors of six, and the speed estimate,
which counts hall edges, sees two edges per electrical turn instead of six. Everything that uses speed
(speed tiltback, speed alarms, the speed-dependent balance gains) works from a speed a third of the
truth: the speed tiltback set to 20 km/h acts at 60 km/h.

**Scenario.** `hall-single-line-loss` (neutral settings): line a lost at 20 km/h.

**Numbers.** No cutoff, no alert. Firmware speed 6.7 km/h at a true 20.1 km/h (RMS error 13.4 km/h,
0.11 km/h before the fault). Torque ripple, standard deviation 19.9 N·m (3.9 N·m before), Iq 23.1 A.
Pitch disturbance small: standard deviation 0.54° (0.34° before), at most 3.4°.

**Status.** CONFIRMED in sim.

**Clean profile.** CLEAN_SINGLE_HALL

## 2. Hall connector off: cut at +70 ms

**Behaviour.** With all three lines lost the code reads 7 continuously. After its invalid-state
count runs out (about 62 ms) the firmware latches a hall fault, sets alert bit 0x40, turns the
inverter off and leaves the motor free-wheeling until a power cycle. At speed the wheel then
nose-dives.

**Scenario.** `hall-loss-while-riding` (neutral): the connector comes off at 20 km/h.

**Numbers.** Hall fault and alert 70 ms after the connector comes off; the shell passes −30° nose
down 130 ms after it.

**Status.** CONFIRMED in sim.

**Clean profile.** CLEAN_ALL_HALL

## 3. Overlean: the PWM saturates, and the warning time depends on the motor

**Behaviour.** A rider who keeps accelerating past what the motor can give (and ignores the
tiltback) meets the voltage limit: the back-EMF and the winding's reactance eat the headroom, the
q-current loop saturates (PWM 100 %), the current stops following its reference, the balance
command runs to its limit and the nose goes down. The PWM alarm (80 %) and, with `e`, the PWM
tiltback start before that, but the PWM does not climb gradually: it sits near 55-78 % above
60 km/h, touches 80 % in short peaks, then saturates. The time from the first alarm to the loss of
balance therefore depends on the motor's constants.

**Scenarios.** `overlean-top-speed` (maintainer's settings, full battery) and
`overlean-top-speed-30pct` (same, 30 % battery): 5 km/h per second towards 100 km/h.

**Numbers.** Full battery: alarm at about 62 km/h; drive disabled at 70.8 km/h, 5.1 s later; fall
0.8 s after that; PWM tiltback 7.9° by then. 30 % battery: alarm at about 58 km/h; drive disabled
and fall at 64.8 km/h, 3.0 s after the alarm. With the earlier motor and tire guesses the nose went
down 0.4 s after the first alarm; the measured tire and pedals give 3-6 s. The pedal height (an
estimate, ±15 mm) moves the cutout between 28.5 s and 35.4 s along this ramp, but its speed by less
than 1 km/h (70.4-71.2 km/h).

**Status.** CONFIRMED in sim for the mechanism (saturation, then cutout, with an alarm that is not
proportional to the remaining margin). The speeds and the warning time are model-dependent until the
motor's inductance and flux are measured.

**Clean profile.** CLEAN_OVERLEAN

## 4. Hard braking holds

**Behaviour.** A rider braking as hard as they dare from 30 km/h (asking for up to 0.6 g) gets a
clean stop. The braking current approaches, but stays under, the firmware's 280 A limit.
Regeneration into the battery stays small.

**Scenario.** `hard-braking-30kmh` (maintainer's settings).

**Numbers.** Stops from 32.5 km/h in 2.83 s (3.2 m/s² mean); peak braking torque −206 N·m at
−240 A of Iq; −12 A at the battery, bus 130.8 V; pedals 10.8° nose up; no cutoff, no alert.

**Status.** CONFIRMED in sim.

**Clean profile.** CLEAN_BRAKING

## 5. Low battery: a tiltback that keeps growing

**Behaviour.** Below 99.2 V (filtered bus voltage) the firmware raises the low-voltage alert and adds
a nose-up offset to the balance target whenever the wheel rolls forward. The offset ramps at about
2.4° per second at first and settles towards 48°; it releases only above about 100.2 V. A rider who
keeps going is pushed further and further back.

**Scenario.** `low-battery-tiltback` (neutral): 98.5 V at rest, a ride at 15 km/h.

**Numbers.** Alert 0x08 at the first control tick (1.7 s). The pedals pass 8° nose up at 12.5 s,
6.5 s after the rider sets off; the rider, easing off for the tiltback, gets no faster than 6.9 km/h.

**Status.** CONFIRMED in sim.

**Clean profile.** CLEAN_LOW_BATTERY

## 6. Hill climb on a low battery

**Behaviour.** A nearly empty battery that rests above the tiltback threshold sags below it on a
climb. The tiltback then starts in the middle of the hill and keeps growing for as long as the
rider climbs.

**Scenario.** `hill-climb-low-battery` (maintainer's settings, 3 % battery, 101.8 V at rest): 25 km/h,
then a 20 % grade from 40 m; the rider keeps climbing whatever the tiltback.

**Numbers.** The low-voltage alert is on from boot. The climb starts at 17.0 s; the battery tiltback
passes 2° at 20.8 s and reaches 26° at 36 s (pedals 30° nose up), with the bus at 97 V and Iq up to
77 A. No cutoff. Run on, it reaches 40°, the wheel stalls on the slope and the firmware disables the
drive at 40.7 s.

**Status.** CONFIRMED in sim for the tiltback. What a rider does on a steep slope while the pedals
tip back is not modelled well yet (the easing rider stalls and rolls back), so the outcome after
36 s is not a finding.

**Clean profile.** CLEAN_LOW_BATTERY

## 7. The gyro clips at ±500 °/s, and the attitude stays wrong (PRELIMINARY)

**Behaviour.** The firmware sets the IMU's gyro range to ±500 °/s. An impact that turns the shell
faster saturates the gyro: the attitude filter integrates 500 °/s where the shell turns faster, and
its accelerometer correction needs seconds to bring the estimate back. In between the firmware
balances its wrong idea of level: the wheel rides on with the pedals tipped, and the pitch cut-off,
which works on the same estimate, does not see the real angle.

**Scenario.** `pothole-20kmh` (maintainer's settings): a 2 cm deep, 50 cm long pothole with steep
edges at 20 km/h.

**Numbers.** The far edge takes the wheel from 20.0 to 16.1 km/h in 20 ms; full forward current
(285 A); the shell pitch rate reaches 820 °/s and then 1360 °/s; the shell swings from −57° nose down
to +44° nose up. The stock estimate never passes 35°, so its 40° cut-off never trips; from 20.8 s to
22 s it believes the shell is at +1.4° while it is at −26° (RMS error 27°), and the wheel rides on
nose down.

**Status.** PRELIMINARY: the body is rigid and has no suspension (the Extreme has a rear shock), the
rider has no knees and the tire is a soft contact, so impacts reach the shell harder than on the real
wheel. The gyro range and the slow recovery are the firmware's; whether a real pothole turns the
shell faster than 500 °/s is not yet known.

**Clean profile.** CLEAN_GYRO

## 8. BLE has no authentication, and service sequences cut the motor under a rider

**Behaviour.** The BLE link has no pairing and no authentication: any phone in range can send
commands. Three two-letter service sequences (calibration request, firmware update, drive-mode
toggle) are accepted whenever the wheel stands still, whether or not someone stands on it, and each
cuts the motor at once. Settings that change how the wheel behaves (roll modes, tiltback, trims) are
accepted at any speed.

**Scenario.** `ble-service-cut-standing` (maintainer's settings): a rider balances at a standstill;
at 10 s a phone sends "c", and 0.3 s later "y".

**Numbers.** The motor is cut 10 ms after the second letter (drive fault and balance disabled); the
wheel and rider fall 1.3 s later.

**Status.** CONFIRMED in sim.

**Clean profile.** CLEAN_BLE

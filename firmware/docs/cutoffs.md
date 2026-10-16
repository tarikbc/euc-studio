# Cut-offs and protections

Every place where the Begode Extreme firmware stops, limits or warns, as the stock firmware does it and as our
two profiles do it. Times are in RTC ticks of 2.016 ms (496 Hz) unless a unit is given. "Soft disable" means
the balance stops its law and ramps the motor command to zero by 2 per tick; "hard cut" means the bridge
outputs go off (TIM8 main output) until the next power-on. Sources: `core/balance/cutoff.c`,
`core/balance/tiltback.c`, `core/motor/`, `core/alarms/alarms.c`, `core/power/power.c`; the stock references
are in [quirks.md](quirks.md).

Variable names are the ones of `profiles/begode-extreme/stockmap.toml` (stock) and `probes.toml` (our
images), so a scenario check or `euc sim compare` reads the same quantity on every firmware:
`balance_disabled` (soft disable), `drive_fault` (hard cut), `hall_fault`, `alerts` (frame 04 byte 14),
`attitude_pitch_deg`, `drive_command`, `vq_active`, `pwm_alarm`, `tiltback_speed_deg`, `tiltback_battery_deg`,
`tiltback_pwm_deg`.

## Fall and attitude cut-offs (balance, every running tick)

| Detector | Input | Threshold | Time | Action | Restart | Depends on speed |
|---|---|---|---|---|---|---|
| Pitch | `attitude_pitch_deg`, or `tipped` | \|pitch\| > 40 deg | 52 ticks (105 ms) | `balance_disabled`, `fallen` | automatic: 101 ticks level (accelerometer tilt) and \|v\| <= 0.2 m/s, one 20-tick beep | no |
| Error | error angle (`attitude_pitch_deg` + tiltbacks + pedal trim - restart and handle offsets), or `tipped` | \|error\| > 40 deg | 11 ticks (22 ms) | `balance_disabled`, one 60-tick beep | none (only with the pitch detector) | no |
| Overload | `vq_active` | \|Vq\| > 31000 (command above 1033 of 1050) | 752 ticks (1.52 s) | `balance_disabled` | none | no; in practice only at high speed or a stalled wheel |
| Slow roll | roll | \|roll\| > 40 + 5 x `W X` deg | 21 ticks (42 ms) | `tipped` (feeds pitch and error) | with the pitch restart | no |
| Fast roll | roll | \|roll\| > 50 / 58 / 66 deg (`<` `=` `>`) and Vq > 30000 | 1 tick | `tipped` | with the pitch restart | through Vq only |
| Handle | handle button | two presses at a standstill | at once | soft disable; the third press restores | third press | yes: standstill only |

None of the angle thresholds or times depends on speed in either profile. Speed only enters the balance
through the gain schedule (Kp + 1 per m/s, Kd + 0.32 per cm/s), the tiltbacks and the restart condition.

The error detector is the fastest one: 22 ms past 40 deg ends the ride, and it has no automatic restart. In
the pothole scenario (PRELIMINARY, rigid body) the clean firmware, which reads the true attitude, trips this
detector on a +44 deg nose-up swing that its own full reverse command causes; the stock firmware never trips it
because its clipped gyro makes it believe the wheel is level. See "Open questions".

## Speed and power limits (balance, no cut)

| Protection | Trigger | Response | Depends on speed |
|---|---|---|---|
| Speed tiltback | speed above `W Y` (km/h, 200 = off) | nose-up offset, 10 deg per m/s over, time constant 4 s; releases with 2 s | yes |
| PWM tiltback | PWM alarm active and `e` set | nose-up offset at 2.7 deg/s, at most 12 deg | through PWM |
| Battery / temperature tiltback | voltage outside 99.20..137.60 V or board above about 80 degC | nose-up offset at 2.38 deg/s (1.98 backward); holds in the gap to the inner window, releases in 20 s | no |
| Clean: lost hall line | one stuck hall line found (`HALL_SINGLE_LINE_UNDETECTED`) | speed tiltback at 10 km/h at most, no automatic restart after the next stop | yes |
| Output slew | every tick | command changes at most 120 per tick, soft start +1 per tick up to 1050 | no |
| Voltage-mode current back-off | \|Iq\| > 280 A (drive mode 0) | command reduced each step, slew 60 per step | no |
| q-integral limit | every tick | the current loop's integral supplies at most 160 x \|v\| (0.1 m/s, up to 20 m/s) of voltage | yes |
| Stop brake | soft-disabled and \|v\| <= 0.4 m/s | Vq 0, all low sides on | yes |

## Motor and sensor faults (hard cut unless stated)

| Fault | Trigger | Time | Action | Clean change |
|---|---|---|---|---|
| Hall fault | hall state 0 or 7 | more than fPWM / 8 calls (62.5 ms) | `hall_fault`, hard cut, `alerts` bit 0x40, 2 beeps every 250 ticks | the timer reads the raw lines also while a lost line is rebuilt |
| Lost hall line | one line stuck | never detected (stock) | none: wrong angle in 2 sectors of 6, speed reads 1/3 | found within about 3 electrical turns; state rebuilt from 2 lines, alert 0x40, 10 km/h tiltback |
| Current sensor start check | any phase reading above 80 A in readings 1..999 after power-on (3 of them before the tick starts) | 1 reading | hard cut, stop the drive, alert 0x02 | same |
| Over-current | phase current above the board level | never (stock counts only) | none | trips the bridge (`NO_OVERCURRENT_TRIP`; level is a board placeholder) |
| IMU at power-on | WHO_AM_I wrong | once | hard cut, alert 0x04 | same |
| Gyro clip | pitch rate above the gyro range | while it lasts | none: the attitude loses the excess (+-500 deg/s) | +-2000 deg/s and a faster re-anchor after a rail (`GYRO_500DPS_CLIP`) |
| Numeric | NaN or infinity in attitude, law, ramp-down or output | at once | stock passes it on | command 0, motor off until power-off (`NAN_FAIL_OPEN`) |
| Exception | any fault handler | at once | both profiles turn the bridge off first | - |
| Hang | main loop or tick stops | 250 ms | stock: nothing | independent watchdog resets (`NO_WATCHDOG`) |
| Free fall | all axes below 0.3 g for 10 ticks | while it lasts | gravity reference held at 1 g; never cuts | same |

## Battery, temperature and BMS (alarms only, no cut)

| Alarm | Trigger | Response |
|---|---|---|
| Low voltage | below 105.60 V | alert 0x08; 1 / 2 / 3 beeps every 1000 ticks below 2 / 4 / above 4 m/s |
| High voltage | 137.61 V or more | alert 0x10, 3 beeps every 1000 ticks |
| Board temperature | NTC above about 80 degC | alert 0x20, 2 beeps every 1000 ticks (plus the temperature tiltback above) |
| PWM | \|Vq / 30\| > 10 x `W P` | alert 0x01, 35-tick beep every 100 ticks at full volume |
| Speed | mode 0 above 8.3 m/s, modes 0 and 1 above 12.5 m/s | 2 or 3 beeps every 500 ticks |
| BMS | 3 failed health checks in a row on a block | 500-tick beep every 5000 ticks; nothing in the drive reads the BMS |

## System cuts (hard cut)

| Cause | Condition | Clean change |
|---|---|---|
| Power-off button | held 401 ticks (0.81 s) at \|v\| <= 0.50 m/s | same |
| Auto power-off | no hall step for 7200 s | same |
| Transport lock | gesture at a standstill with the handle held, or the lock state at power-on | the light also goes off (`LOCK_LIGHT_STAYS_ON`) |
| Service sequences `!@` `cy` `+-` | BLE bytes, no authentication | only with no rider and a power-button press within 10 s (`SERVICE_WITHOUT_CONFIRM`) |
| Ride settings over BLE | any speed (stock) | below 0.5 m/s only (`SETTINGS_WHILE_MOVING`) |

## Open questions

### The error detector against a recovery swing

The error detector's 22 ms is shorter than one recovery swing. With a true attitude, a hard pothole edge
(PRELIMINARY, rigid body) trips it on the overshoot after a -40 deg nose dive: the law's rate terms
(0.1 x Kd(v) x pitch rate and 160 x the change of the error) are not clamped, only the angle term is, so at
about 540 deg/s they command full reverse (+73 to -1050 in 20 ms, the 120-per-tick slew) while the shell is
still nose down, and it swings on to +44 deg.

Decision (2026-10-16): no change before the suspension model lands. Then rerun pothole-20kmh on stock, compat
and clean, and if clean still trips, change in this order, each proven by one scenario:

1. Clamp the rate terms of the law (for example to +-500), so the rate alone cannot command full reverse while
   the angle is still on the other side.
2. A lower command slew (for example 40 per tick) when the command crosses zero while \|error\| > 15 deg.
3. The pitch detector's 105 ms for the error detector too: this changes when a fall cuts, not whether.
4. Only if a realistic body still trips: a speed-aware cut-off, with evidence.

### Acceptance: what must still cut

Any change to a cut-off or to the law's limits must keep every case below cutting (`balance_disabled` or
`drive_fault` set), in both profiles, checked by a scenario:

- the shell stays past the limit and does not come back (rider off, wheel down): rider-steps-off-15kmh;
- the angle keeps growing while the command is saturated (overlean past the motor's limit);
- any over-angle at walking speed;
- roll past the slow or fast roll limit (wheel on its side);
- the hall fault, the start-up current-sensor check and the system cuts above, unchanged.

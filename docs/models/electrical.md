# Electrical plant

The electrical plant is everything between the controller's TIM8 outputs and its ADC inputs: the
inverter, the hub motor, the hall sensors, the current and voltage sensing, the battery and the heat
these produce. It lives in `crates/euc-plant`; the wheel's values live in
`profiles/<wheel>/plant.toml`. This page describes the models, the sign conventions that make the
stock Begode Extreme firmware drive the motor correctly, and where each default value comes from.

**Defaults are not measurements.** Pole pairs, resistance, inductance, flux, dead time, battery and
thermal values are reasoned estimates (marked DEFAULT in `plant.toml`). They are consistent with the
firmware's own constants and with published specifications, and they make the stock firmware's
current loop and balance behave sensibly, but the real wheel may differ. Replace each one when it is
measured, and record the measurement here. Pole pairs and flux are fitted to the owner's ride log
(INFERRED; "Calibration against the owner's ride log", below).

## How it runs

```
TIM8 oc1 / oc3 / oc2 ──▶ inverter ──▶ PMSM (dq) ──▶ torque ──▶ body physics (MuJoCo)
  (Signal::Pwm, or Released = MOE off)    ▲         ▲ angle, speed ◀──┘
                                bus V │   │ bus current
                          battery (2 × 32S) ──▶ smart BMS readings
   hall code ──▶ PB7 / PB6 / PB8        phase a, phase b, bus current, bus voltage, NTC ──▶ ADC
```

- One fixed grid, 5 µs by default (360 cycles at 72 MHz). `ElectricalPlant::advance(now)` integrates
  every whole step up to the engine's cycle. The engine calls it before the firmware can observe the
  plant (an ADC conversion, a GPIO read of the halls, a BMS poll) and before changing an input, so
  every reading is at most one grid step old. The PWM itself is averaged: a 12 kHz period is 83 µs,
  much longer than the grid, so the grid resolves the current between PWM updates, not the ripple
  within one period. The firmware samples the phase currents at the centre of the PWM period, where
  the ripple crosses its mean, so the averaged current is what it would read.
- Inputs: the three inverter legs (`set_leg`, from the timer's PWM wave or its release) and the
  wheel's angle and speed (`set_wheel`, from body physics every 0.5 ms; between calls the plant
  extrapolates the angle at constant speed).
- Outputs: ADC counts (`adc_raw`), the hall code (`halls`), the wheel torque (`torque_nm`, and
  `mean_torque_nm` over the last physics step, so the energy handed to physics matches), the battery as
  its BMS boards see it (`battery_readings`), and a `snapshot` for telemetry.
- Noise is deterministic: hall noise uses a seeded generator; ADC noise is derived from the seed, the
  time and the channel, so the same run reads the same counts.
- Hall edges need an event of their own. The firmware polls the hall lines at twice the PWM rate,
  more often than any sensor samples the plant, so an engine that advances the plant only when a
  sensor samples it puts each edge on the wires up to one ADC period late (about 20° electrical at
  130 rad/s). `next_hall_edge()` returns the grid point where the code next changes at the current
  wheel speed; schedule an event there and ask again after each `set_wheel`.

## Inverter

Each leg is either switching or off.

- **Switching** (`Signal::Pwm` from the timer, duty `d = high / period`). In centre-aligned PWM mode 1
  the duty of TIM8 channel *x* is `CCRx / ARR`. Average pole voltage to the negative rail:
  `v = d·Vbus − sign(i)·(td/T)·Vbus`. The second term is the dead-time error: during the dead time
  the current flows through a body diode, which loses `td` of each period when the current flows into
  the motor and gains it when it flows out. The sign fades linearly to zero below 0.5 A, so a current
  near zero does not chatter. A leg at duty 0 or 1 does not switch and has no dead-time error. Duty 0
  on all three legs is the stock firmware's short-circuit brake (CCR = 0 with MOE on).
- **Off** (`Signal::Released`: MOE cleared, or the channel disabled). Only the body diodes conduct:
  a leg carrying current into the motor clamps at −Vf (low diode), current out of the motor clamps at
  Vbus + Vf (high diode), and with no current the leg is open. A current that would reverse stops at
  zero. When every leg is open, current flows only if the line back-EMF exceeds Vbus + 2 Vf: then the
  bridge rectifies into the battery and brakes the wheel. With the default motor this needs about
  140 rad/s (105 km/h), so a cut-out at normal speed simply lets the motor freewheel.
- Battery (DC bus) current: `Σ s·i`, where `s` is the leg's duty, or 1 while its high diode conducts.
- Losses into the power-stage thermal node: conduction `Rsw·Σi²`, switching `Vbus·|i|·tsw/T` per
  switching leg, and diode conduction `Vf·|i|` while off.

Board wiring (`board.toml`, not the plant): phase **a** on TIM8 CH1 (CCR1), phase **b** on CH3 (CCR3),
phase **c** on CH2 (CCR2). The firmware writes its phase A to CCR1, B to CCR3 and C to CCR2 (motor.md
§2), so with this wiring the firmware's phase names are the motor's.

Not modeled: ringing, gate driver faults, partial bridge (one leg off while the others switch) beyond
the open-leg rule above, DC link capacitor (the battery is the bus).

PWM and modulation in telemetry: `pwm_pct` is the firmware's own figure, as its app frame reports it
(frame 07: the commanded q voltage `vq_active`, Q15 of Vbus/√3, divided by 315 and truncated, so
31 500 is 100 %). Values can exceed 100 % near saturation; telemetry does not clamp them.
`modulation_pct` is the voltage the inverter really applies in the plant, |Vd, Vq|
of the pole voltages, on the same scale. It includes Vd, but it does not have a general lower
bound of `pwm_pct`: circle limitation reduces the commanded vector, dead time changes the
applied voltage, and sampled values can represent different update instants. The firmware's
`vq_active` precedes circle limitation, except during stop braking.
At 20 km/h with the default 0.5 µs dead time, the applied voltage was about 2 % below the
command on average (up to 6 %) in the diagnostic run. This is a scenario result, not an invariant.
Modulation is 0 while any leg is off.

## Motor

A PMSM in the rotor frame (dq), amplitude-invariant transforms:

```
Ld·did/dt = vd − R·id + ωe·Lq·iq
Lq·diq/dt = vq − R·iq − ωe·Ld·id − ωe·ψ
T_e = 1.5·p·(ψ·iq + (Ld − Lq)·id·iq)          (positive towards increasing electrical angle)
```

`R` here is the phase resistance plus one switch's on-resistance. Each step solves the whole linear
system by backward Euler with the voltages and the speed held for the step (unconditionally stable).
The electrical angle and speed come from the wheel: `θe = direction·p·φ + offset`, `ωe = direction·p·ω`.
The wheel torque is `direction·T_e`. Inertia, friction and everything mechanical belong to the body
physics. While any leg is off, the step runs in the stationary frame with the mean inductance
(see the inverter).

Not modeled: iron loss, saturation, cogging, temperature rise of `R` and fall of `ψ`, current
ripple within a PWM period.

## Halls

Three digital sensors report the rotor's 60° electrical sector as a 3-bit code (bit 0 = sensor a,
bit 1 = b, bit 2 = c). The model reads the sector of `θe + offset` from the stock firmware's own
table (motor.md §4.2): sector centres 6 at +30°, 4 at +90°, 5 at +150°, 1 at −150°, 3 at −90°,
2 at −30°. Options: `noise_deg` adds Gaussian noise to the sensed angle (edge jitter and chatter near
an edge), `stuck` holds a line high or low (a failed sensor or wire). Board wiring: sensor a → PB7,
b → PB6, c → PB8, which gives the firmware's `state = PB7 | PB6<<1 | PB8<<2`.

## Sensors

| Signal | Pin, ADC | Model |
|---|---|---|
| Phase a current | PA4, ADC1 ch4 | `2048 − i_a / 0.2136 A` (inverted) |
| Phase b current | PA5, ADC2 ch5 | `2048 + i_b / 0.2136 A` |
| Battery current | PC0, ADC2 ch10 | `2048 + I_bus / 0.2136 A` (positive = discharge) |
| Bus voltage | PA0, ADC1 ch0 | `V / 0.04202 V` |
| Board NTC | PC1, ADC1 ch11 | 47 kΩ, B 4150, 10 kΩ pull-up to 3.3 V, at the power-stage temperature |

Counts are rounded half to even (as the Python reference harness) and clamped to 0..4095; the ADC
model applies its own offsets and left alignment. Each sensor has an optional zero error
(`offset_counts`) and all share `noise_counts`.

## Battery

Two packs in parallel on the bus, each 32 cell groups in series. A cell group is an open-circuit
voltage OCV(SOC), an ohmic resistance R0 and one RC pair (R1, τ). The packs share the load by their
impedance (different charge states share unevenly); a negative load (regeneration) charges them. Pack
temperature is a lumped node heated by `I²·R0` and the polarisation loss. The BMS readings per pack:
32 cell voltages (each cell group's OCV less its share of the pack's ohmic and polarisation drops,
so a healthy pack shows the bus voltage / 32 on every cell and a `cell_low` fault shows on its
cell), the temperature at each of its 6 sensors, and the pack current (discharge positive; the sign
the real boards use is UNKNOWN). The simulator passes them through `PlantView::pack`, and the smart
BMS model reports pack A as blocks 0 and 1 and pack B as blocks 2 and 3: cells 0-15 and 16-31,
sensors 0-2 and 3-5, the pack current on both. A plant without packs (M1's static one) gives every
cell the battery voltage / 32, as before. Checked on the stock image: a `cell_low` of 0.5 V on pack B
cell 30 reaches the app's frame 03 as 3561 mV next to 4061 mV, and pack B, now the weaker one, shows
−1.0 A as pack A charges it.

## Thermal

Two lumped nodes, each a heat capacity with a thermal resistance to ambient, solved exactly for the
power of each step: the power stage (inverter losses; the board NTC reads it) and the windings
(copper loss; reported in the snapshot, not wired to an ADC: this board has no motor temperature
sensor).

## Faults

A scenario injects faults on its timeline. Each `[[timeline]]` table gives the start, an optional
length and the fault's kind; its other fields belong to the fault:

```toml
[[timeline]]
at_s = 12.0
for_s = 3.0          # optional: without it the fault lasts to the end of the run
fault = "hall_stuck"
line = "a"
level = "low"
```

| `fault` | Fields (default) | Effect |
|---|---|---|
| `hall_stuck` | `line` a/b/c, `level` high/low | The line reads that level. |
| `hall_lost` | `line` | The line reads high: hall sensors have open-collector outputs, so the board pulls the inputs up (INFERRED). |
| `hall_noise` | `noise_deg` | Extra jitter on the sensed angle (standard deviation, electrical degrees), added to `[halls] noise_deg`. |
| `current_sensor` | `sensor` a/b/c/bus, `offset_a` (0), `gain` (1), `noise_a` (0) | The sensor reads `gain · i + offset_a` plus noise. The Extreme has sensors on a, b and the bus only. |
| `open_phase` | `phase` a/b/c | The phase wire is open: the other two phases carry all the current. |
| `stage_overtemp` | `rate_k_per_s`, `max_c` | The power stage climbs at that rate above what its losses give, up to `max_c`, and holds; the board NTC reads it. After the window it cools with the stage time constant (200 s). |
| `cell_low` | `pack`, `cell`, `drop_v` | That cell group's open-circuit voltage sits `drop_v` low: its BMS reading and the pack voltage drop. |
| `pack_imbalance` | `pack`, `soc_delta` | The pack's state of charge jumps once at the start and stays; the packs then even out through the bus. |
| `battery_soc` | `soc` | Every pack's state of charge is set once at the start (a battery ridden nearly empty) and stays. |
| `high_resistance` | `pack`, `factor` | The pack's R0 and R1 are `factor` times normal: it sags more and carries less of the load. |
| `imu_noise` | `accel_g` (0), `gyro_dps` (0) | Gaussian noise on each axis of every sample. |
| `imu_bias` | `accel_g` [x, y, z], `gyro_dps` [x, y, z] | Constant offsets. |
| `imu_stuck` | none | The sample registers keep the last sample; the bus and `WHO_AM_I` still work. |
| `bms_silent` | none | The BMS does not answer the poll. |
| `bms_garbled` | `byte_error_rate` (0), `missing_bytes` (0) | Each reply byte is random with that probability, and the reply loses its last bytes. The firmware checks no header, length or checksum, so the damage becomes cell, temperature and current values. |

The first ten are `euc_plant::fault::PlantFault`, the last five `euc_devices::fault::DeviceFault`
(an optional `device = "<instance>"` picks the device when a board has more than one of its kind).
The simulator converts the window to cycles once and hands each fault to its target:
`ElectricalPlant::add_fault`, `Mpu6500::add_fault` or `SmartBms::add_fault`. The plant applies a
fault at its grid points, from the first one at or after the start to the first one at or after the
end, and `next_hall_edge()` also returns those points, so a stuck or lost line reaches the GPIO on
time. A device checks the window when it samples (IMU chip select) or answers (the poll's last byte).
Fault noise is seeded by the time, so a run repeats exactly. Faults live in the world, not in the
controller, so they continue through resets and power cycles.

Not modeled: a shorted MOSFET or a failed gate driver, an intermittent contact (use short windows),
and BMS boards that fail one at a time (the four boards answer as one reply).

Scenario checks for a fault's outcome: `cutoff` (the firmware cuts within a time, optionally by a
named stock-map flag), `firmware_bits` (a stock-map variable such as `alerts` gets bits set),
`pitch_reaches` (a tiltback or a nose dive), `ripple` (the standard deviation of a telemetry signal
in a window) and `firmware_tracks` (the RMS error of a firmware estimate, such as `speed_cms`,
against the true signal). What the stock firmware does, on the Begode Extreme
(2026-10-16, Unicorn):

- `scenarios/hall-loss-while-riding.toml`: the hall connector comes off at 20 km/h (all three lines
  read high, code 7). The firmware latches `hall_fault` 70 ms later, sets alert bit 0x40, turns the
  inverter off, and the wheel nose-dives (−33° within 0.1 s).
- `scenarios/hall-single-line-loss.toml`: one line lost at 20 km/h. The code reads 7 for only one
  sector in six, the invalid count keeps resetting, and the firmware never cuts. It rides on with a
  wrong angle in two sectors of six (torque standard deviation 16.5 N·m against 3.5 before) and
  counts two hall edges per electrical turn instead of six: its speed reads 6.7 km/h at 20.3 km/h,
  so its speed tiltback and alarms would act at three times their setting. The checks pass on the
  stock build and record this; a fixed firmware should tighten them.
- `scenarios/low-battery-tiltback.toml`: at 98.5 V (1.3 % charge) the firmware sets alert bit 0x08
  at its first control tick (1.7 s) and, once the wheel rolls, tilts the pedals back: the shell
  passes 8° nose up 1.6 s after the rider reaches speed, and keeps rising (26° by 20 s) without a cut.

## Sign conventions that make the stock firmware's FOC work

These follow from motor.md and balance.md (both OBSERVED in the firmware) and the requirement that the
stock firmware rides: positive balance command = forward torque, forward motion = negative hall count.

1. **The firmware's current frame is the motor's frame mirrored in β.** With phase a measured on PA4
   (inverted, so the firmware's `Ia` is the true current into phase a) and phase b on PA5, the ST
   Clarke `Iβ = −(Ia + 2·Ib)/√3` is the negative of the standard β. ST's SVPWM with A → CCR1,
   B → CCR3, C → CCR2 mirrors its voltage β the same way (checked by evaluating motor.md's equations:
   `Vβ > 0` lowers phase b and raises phase c), so voltages and currents agree.
2. **In the standard frame the firmware's q axis points at its angle θfw, its d axis at θfw − 90°**
   (from ST's Park `Iq = Iα·cosθ − Iβ·sinθ`, `Id = Iα·sinθ + Iβ·cosθ` with the mirrored β).
3. **The firmware's positive hall direction (5 → 1 → 3 → 2 → 6 → 4, θfw increasing) gives a negative
   speed**, and forward motion is a negative hall count (balance.md). So rolling forward must lower the
   electrical angle: `direction = −1`.
4. **A positive firmware Iq must push forward**, that is towards a falling angle. With the rotor flux
   at θe, that holds when the firmware's angle is `θe − 90°`: its q axis is then the motor's −q, and
   `+Iq_fw` is `iq < 0`, torque towards −θe = forward. Hence the hall table offset
   `offset_deg = −90`: the hall pattern reads sector `θe − 90°`. (With +90 the controller would still
   regulate current, but every command would push the wrong way.)
5. **Current scale.** The firmware converts phase current as `Q15 = 16·(raw − 2048)` with 32768 Q15
   counts = 437.5 A (motor.md §3.2). The sensor gain 437.5 A / 2048 = 0.2136 A per count makes the
   firmware's amperes true amperes, so `Iq* = 20 × command` (mode 1) is a real current and ±1050 is
   ±280 A.

`crates/euc-plant/tests/firmware_foc.rs` checks 1-5 end to end: a current loop written with the
firmware's equations (ST Clarke/Park/SVPWM, the hall sector table, the CCR mapping, the ADC scaling)
drives the plant from `plant.toml`. A held wheel gets `+0.85…1.0 × 1.5·p·ψ·Iq` of forward torque
for a positive command (the sector-centre angle is within ±30° of the rotor), a negative command
pushes backward, and a free wheel spins up smoothly in either direction with the halls running in the
firmware's negative order when rolling forward.

## Validation against the stock firmware

On 2026-10-16 the stock image (GW1825007, Unicorn backend) ran with this plant wired as above and a
one-axis test stand (`rig::WheelRig`, stepped every 1 ms) in place of the body physics: the wheel
held, or free with 0.5 kg·m² and light friction. The frame stood level (IMU pitch 0) and the
pedal trim `W U d` over BLE set the balance error: `WU8` gives +4.8° (forward), `WU0` gives −4.8°
(backward). Drive mode 1 (current control) was set the firmware's own way: `+` then `-` over BLE at
standstill (the firmware saves the mode to its flash settings log and powers off), then a power
cycle. A trace read the firmware's command and measured Iq from RAM every 2 ms; `Kt·Iq_fw` below is
`1.5·p·ψ` times that Iq.

| Case | Mode | Result |
|---|---|---|
| Held, +4.8° | 0 (voltage) | command +255, firmware Iq +197 A, plant torque +157.1 N·m forward; `Kt·Iq_fw` 157.2 N·m |
| Held, about −2° | 0 | backward, −52 N·m; same match |
| Held, +4.8° | 1 (current) | Iq* = 20 × 718 = 192 A; firmware Iq 163 A (85 %: the q loop is PD only at standstill, as predicted above); plant torque +130.1 N·m = `Kt·Iq_fw` |
| Free, ±4.8° | 0 | 0 → ±131 rad/s in 2.1 s; firmware speed 27.7 m/s against 27.9 m/s at the tire |
| Free, ±4.8° | 1 | 0 → ±133 rad/s in 1.8 s |

In every case the sign is right, the torque matches `Kt·Iq_fw` (mean error 0.05 N·m held, 0.2-0.4 N·m
spinning), the wheel speed rises at every sample while the drive is on, no hall step goes against the
motion, and rolling forward lowers the firmware's hall count. With the plant the run is 2.2-2.7×
real time (3.2× without).

What else the runs showed:

- On a stand the frame cannot tilt back, so after the spin-up the speed tiltback keeps raising the
  balance error until the firmware's error detector disables the drive (|error| > 40°). The command
  then ramps to zero; in mode 0 the drive brakes the wheel by regeneration (up to 69 A into the
  battery, bus 137 V) to a stop, in mode 1 it holds Iq at zero and the wheel coasts. Firmware
  behavior on a stand, not plant faults.
- **Firmware quirk, observed in these runs and in the code:** the Iq low-pass filter (0x200001AC)
  multiplies the Q15 Iq by 437 500 in 32 bits, which wraps when |Iq| ≥ 4909 counts (65.5 A). Above
  that the filtered current takes the wrong sign. In mode 0 the balance law adds this filter as its
  current term, so with a held wheel the command settles where |Iq| sits on a wrap boundary
  (65.5 A, or 196.6 A for a larger error) instead of following the tilt. Mode 1 does not use the
  term in its balance law. A real controller does the same; the plant does not cause it.

## Calibration against the owner's ride log (2026-10-23)

**Data.** The owner's Extreme on its stock firmware, one 30-minute ride (2026-10-17, EUC Remote CSV,
4 Hz rows of the wheel's own values: speed, pack voltage, PWM, battery current, plus GPS speed),
battery 94 to 64 %. Only steady stretches count: 2 s windows whose speed stays within 1 km/h (57 rows;
the 4 Hz grid smears transients). The log itself stays private; only derived numbers are here.

**Speed scale.** On 54 steady rows GPS reads 0.905 of the displayed speed (quartiles 0.868-0.916). The
firmware shows speed as electrical turns times 0.0505 m, so the display reads 1.105 times the true
speed. Two explanations are electrically the same for the firmware: more pole pairs than the 28 the
tire geometry suggested (31 gives 1.107), or a smaller effective rolling radius (0.204 m) or a display
scale set above true. 31 pole pairs keeps the measured tire geometry, so the plant uses it. A count of
hall edges over one wheel turn would settle it. The consequence: **every firmware speed threshold
triggers at about 0.905 of its displayed value in true speed**, as on the owner's wheel (a 20 km/h
tiltback at about 18 km/h true, a 69 km/h one at about 62 km/h true).

**Back-EMF.** The firmware's PWM figure is |Vq| / (0.961·Vbus/√3) (the sim reproduces this within 0.3
points). At steady speed Vq ≈ ψ·ωe + R·Iq, so the log gives ψ directly from PWM, voltage and speed:
a fit over the steady bands gives 17.9 mWb through zero, or 16.1 mWb with a speed-independent offset
of 3.0 V. The plant's 20.5 mWb read about 10 % high at 40-74 km/h; 18.0 mWb matches there.

| Displayed speed | Log: PWM, V, Ibat (steady) | Sim before: PWM, Ibat | Sim now: PWM, Ibat |
|---|---|---|---|
| 20 km/h | 19 % at 18.7 km/h, 132 V; 24 % at 23.5 km/h, 128 V; 2-3 A | 21.7 %, 1.2 A | 19.0 %, 0.9 A |
| 30 km/h | 31 % at 29.7 km/h, 128 V, 4.9 A | 32.0 %, 3.1 A | 28.1 %, 2.2 A |
| 40 km/h | 42 % at 43.9 km/h, 128 V, 10.2 A | 42.9 %, 6.2 A | 37.9 %, 5.4 A |
| 50 km/h | 49 % at 52.3 km/h, 127 V, 11.3 A | 53.2 %, 10.4 A | 46.7 %, 8.1 A |
| 60 km/h | 59 % at 61.0 km/h, 126 V, 15.9 A | 63.8 %, 17.6 A | 55.6 %, 15.0 A |
| 70-74 km/h | 65 % median (62-71 %) at 70-75 km/h, 123.4 V, 23.8 A | cannot reach it | 67.1 % at 74.1 km/h, 123.0 V, 24.1 A |

The sim rows: stock firmware, the mesh tire, the city world's ideal road, 80 % charge (123-127 V), a
rider of the owner's build (private inputs) held at each speed by a direct lean. At 20-30 km/h the sim
reads about 10 % low (the log has 10 steady rows there): the real firmware's Vq carries an offset of
about 3 V. A 2 µs dead time reproduces it, but in the sim it makes the idle wheel hunt (peak pitch
1.3° instead of 0.5°) and stops the handle button's standstill cut, so the dead time stays at its
DEFAULT. Battery current is 2-4 A below the log at 40-50 km/h on both plants: the real rolling
resistance is likely higher (the tire's `crr` is calibrated separately).

**Top speed.** The owner reports 78 km/h displayed on an 80 % pack with headroom left (light rider,
another ride). With a tucked rider (drag area 0.42 m²) the sim holds 78.2 km/h displayed (70.7 km/h
true) on an 80 % pack with 70.1 % firmware PWM and 78.4 % true modulation; the plant before could not
pass 74 km/h. Upright (0.6 m²) the wheel overleans near 77 km/h displayed (69 km/h true, 86 % true
modulation while it still speeds up). So the 78 km/h report is met with a tuck.

**Still DEFAULT.** Phase resistance, inductance (it sets the gap between the firmware's PWM and the
true modulation, findings 10), dead time, battery resistance and the OCV curve: the log has only the
firmware's Vq. A spin test (back-EMF and L) or raw frames with phase current at top speed would
measure them.

## Default values and why

| Value | Default | Why |
|---|---|---|
| `pole_pairs` | 31 | INFERRED from the owner's ride log (2026-10-17): GPS reads 0.905 of the displayed speed over 54 steady rows. The firmware counts 118.7 hall edges per metre (0.0505 m per electrical turn, motor.md §4.3); on the 3.00-12 tire's 0.2251 m loaded radius 31 pole pairs make it show 1.107 × the true speed. Before: 28, the count whose radius p × 0.0505 m / 2π fits the tire if the display were true. |
| `flux_wb` | 0.0180 Wb | INFERRED from the owner's ride log (2026-10-17): the firmware's PWM at steady speed is \|Vq\| / (0.961·Vbus/√3) with Vq ≈ ψ·ωe + R·Iq; at 18.0 mWb the sim matches the log within 5 % from 40 to 74 km/h displayed. The published 109 km/h no-load speed (one retailer), read as a true speed with 31 pole pairs, gives 18.6 mWb. Torque constant 1.5·p·ψ = 0.84 N·m/A: 280 A (the firmware's limit) gives 234 N·m, 1040 N at the tire. Before: 0.0205 Wb from that no-load speed read as a displayed one. |
| `resistance_ohm` | 0.080 Ω | Typical for a 3.5 kW nominal, 134 V hub motor. Locked-rotor current at full PWM would be far above the firmware's 280 A limit, as on a real wheel. |
| `ld_h`, `lq_h` | 0.20 mH | Surface magnets (Ld = Lq). L/R = 2.5 ms. With the firmware's q-axis PI (Kp 2.93 Q15 V per Q15 A = 0.52 V/A at 134 V) the current loop has about 400 Hz bandwidth, well below the 12 kHz PWM. At standstill the q loop is PD only (its integral limit is ±160·|v|), so a held wheel settles about 14 % below its command. |
| `direction` | −1 | Convention 3 above. |
| `halls.offset_deg` | −90 | Convention 4 above. Shift it to model a misplaced hall board. |
| `dead_time_s` | 0.5 µs | The firmware sets 1 timer tick (14 ns) at 10 kHz and 29 ticks (403 ns) at 12 kHz (platform.md §8); a 14 ns dead time cannot be safe, so the gate driver must add its own. 0.5 µs is a common driver default. |
| `switch_resistance_ohm` | 2 mΩ | Several 150 V MOSFETs in parallel per switch position. |
| `diode_drop_v` | 0.7 V | Silicon body diode. |
| `switching_time_s` | 100 ns | Rise plus fall at the bus voltage. |
| Phase current sensors | ±0.2136 A per count | Convention 5 above; the real sensor gain is UNKNOWN. |
| Bus current sensor | 0.2136 A per count, + = discharge | Same scale in the firmware (`43750/32767 × 16 counts = 0.2136 A per count`, motor.md §3.2); its sign is UNKNOWN. |
| Bus voltage | 1377/32768 V per count | The firmware's own conversion (motor.md §3.2). |
| Board NTC | 47 kΩ, B 4150, 10 kΩ | The firmware's own conversion (motor.md §3.2). |
| `battery.series`, `packs` | 32, 2 | The BMS reports 2 × 32 cells (comms.md §5). |
| `capacity_ah` | 10 Ah per pack | Published 2400 Wh at 134.4 V: 32S4P of 5 Ah cells, split into two 32S2P packs. |
| `r0_ohm`, `r1_ohm`, `tau_s` | 7 mΩ, 3 mΩ, 30 s per 2P group | About 14 mΩ DC resistance per high-power 21700 cell. 100 A sags the bus by 11 V. |
| `ocv` | NMC curve, 3.00-4.20 V | Typical for high-power 21700 NMC cells. |
| `soc` | 0.883 | 129.7 V at rest, the bus voltage in the maintainer's 2026-10-15 capture. |
| Battery thermal | 6000 J/K, 3 W/K per pack | 64 cells of 70 g plus housing; natural convection inside the shell. |
| Power stage thermal | 400 J/K, 0.5 K/W | An aluminium heat sink of about 0.4 kg on the shell. |
| Winding thermal | 3000 J/K, 0.3 K/W | Stator copper and iron of a hub motor. |
| `ambient_c` | 25 °C | Matches M1's static plant. |

## To measure on a real wheel

1. Pole pairs: count hall edges over one wheel turn (6 per electrical turn). The rolling circumference
   divided by 0.0505 m gives them only if the display shows the true speed; the owner's wheel does not.
2. Flux linkage: spin the wheel by hand (or tow it) and measure the line back-EMF peak and frequency:
   `ψ = V_line,peak / (√3 · ωe)`.
3. Phase resistance and inductance: line-to-line with an LCR meter (half of the line value each).
4. Current sensor gain and sign: a known DC current through phase a and the frame 00 phase current.
5. Hall offset: back-EMF zero crossings against hall edges while spinning.
6. Battery: rest voltage against SOC, and the voltage step under a known load.
7. Battery current while riding: the owner's ride log 2026-10-17 reads about -0.8 A at a standstill
   (motor not driven), and above 15 km/h the wheel draws about 2 A (270-400 W) more than the model
   at every speed, a gap that rolling resistance cannot make (models/body.md, "Tire"). Log a
   standstill with the motor on and the rider aboard, and a coast-down or a known climb; a raw
   capture with phase current (EUC Remote) separates a sensor offset or gain from an unmodelled
   loss such as PWM current ripple.

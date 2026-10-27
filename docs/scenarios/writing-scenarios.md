# Writing a scenario

A scenario is a TOML file in `scenarios/` that rides one firmware in the simulator: the wheel boots on
its stand, the stand lets go, a simulated rider follows a target speed, and the electrical plant, the
body physics and the ground do the rest. Checks at the end of the file say what must hold. The
library and its results are in [models/scenarios.md](../models/scenarios.md); the format is defined
in `crates/euc-sim/src/scenario.rs` (with `crates/euc-physics/src/terrain.rs` for the ground).

The quickest start is to copy a scenario that is close to what you want and change it.

## Run it

```sh
# On the stock firmware (fetched and cached on first use), on our own Cortex-M3 core:
cargo run --release -p euc-cli -- sim run --scenario scenarios/<name>.toml

# On one of our builds:
cargo run --release -p euc-cli -- sim run --scenario scenarios/<name>.toml \
    --firmware firmware/build/target-clean/euc-fw-begode-extreme.bin

# Two firmwares in lock step, with the first divergence and per-signal statistics:
cargo run --release -p euc-cli -- sim compare --scenario scenarios/<name>.toml --a stock --b clean
```

Useful options of `sim run`: `--settings <preset>` (or `blank`) overrides the scenario's settings,
`--max-quantum <n>` sets the emulator's quantum (see "Robustness" below), `--record <file.eucrec>`
writes a recording (see [recordings.md](../recordings.md): `euc rec info`, CSV export, replay in the
studio, Python), and `--json` prints the result as JSON. `sim compare` takes
`--tolerance <file>` (see [compare.md](../compare.md)). Each check prints one line, `PASS` or `FAIL`,
then `RESULT`.

The studio lists scenarios in its run bar: a development build reads every `scenarios/*.toml` when the
web view connects (a new file shows without a rebuild); a release build offers the two built-in ones,
`ride-20kmh-stop` and `balance-60s`.

The CLI and studio use CPI 2.0 with exception overhead.
For historical reference timing, `sim run` accepts `--cpi 1.25 --no-exception-timing`.
Changing `--cpi` alone retains exception overhead.
Record the timing settings with each result. See [CPU timing](../models/cpu-timing.md).

## The file

### Top level

| Field | Required | Meaning |
|---|---|---|
| `schema` | yes | Format version: `1`. |
| `name` | yes | Short name, the same as the file name. |
| `description` | yes | One sentence for people. |
| `wheel` | yes | Wheel profile id: `begode-extreme`. |
| `seconds` | yes | Simulated seconds from power-on. |
| `release_s` | yes | When the stand lets the wheel go, inside the run. The stock firmware boots in about 1.7 s; `2.5` is usual. |
| `speed_kmh` | yes | The rider's target speed as `[seconds, km/h]` points, in time order, linear in between and held after the last. Negative is backwards. |
| `phase_start_s` | no | Start time for measured-speed lean phases, at or after release. Defaults to `release_s`. |
| `[[phase]]` | no | Ordered lean commands with measured-speed gates and finite timeouts. See below. |
| `settings` | no | Settings preset the wheel boots with, from `profiles/<wheel>/settings.toml` (`neutral`, `tiltback-69`, `tarik-2026-10-15`). Without it the settings journal is blank, which tilts the stock pedals 4.8° nose down. |
| `step_off_s` | no | When the rider steps off the pedals (between `release_s` and the end); the wheel goes on alone. |
| `push` | no | Pushes on the rider, `[[start s, newtons forward, duration s], ...]`; overlapping pushes add up. |
| `[rider]`, `[rider.control]` | no | The rider, below. |
| `[assist]` | no | The roll aid, below. |
| `[world]` | no | The ground: a generated city, below. Every shipped scenario has one; start new scenarios with it. |
| `[terrain]` | no | A synthetic 1-D strip instead (below), for a test whose exact shape is the point. Without `[world]` or `[terrain]` the ground is an infinite plane. |
| `[[timeline]]` | no | Faults, below. |
| `[[inject]]` | no | BLE commands, below. |
| `[[expect]]` | no | Checks, below. |

Unknown fields are errors, so a typo does not pass silently.

Start the file with a comment that says what the run shows and why, and a line `# Rider: ...` that
says which rider it uses and how it differs from the default.

### The rider: `[rider]`, `[rider.articulation]` and `[rider.control]`

Every field has a default; give only what you change. The model is described in
[models/body.md](../models/body.md).

The default rider is articulated (`articulated_sagittal_v1`): an upper body on two legs with hip,
knee and ankle hinges, the feet welded to the pedals. `model = "articulated_contact_v1"` is the same
rider with each sole on its own pedal through friction contact (opt-in), and
`model = "articulated_lateral_v1"` the rider with legs that also move sideways and twist, on soft
knees (opt-in, built in stages). The rigid rider
(`model = "rigid_ankle_v1"`) stays for controlled comparisons with the results before 2026-10-22: a
scenario that measures the firmware against that baseline on purpose pins it and says why in its
header. `euc sim run --rider-model ...` overrides the scenario's model for one run.

| Field | Default | Meaning |
|---|---|---|
| `model` | `"articulated_sagittal_v1"` | Or `"rigid_ankle_v1"`, or `"articulated_contact_v1"` (contact feet, opt-in; see [body.md](../models/body.md#the-articulated-rider-default)), or `"articulated_lateral_v1"` (lateral legs, opt-in; see [body.md](../models/body.md#the-lateral-rider-opt-in)). |
| `mass_kg` | 80 | Rider mass; every segment's mass scales with it. |
| `height_m` | 1.78 | Standing height; every segment's length scales with it. |
| `ankle_above_pedals_m` | 0.08 | Ankle joints above the pedals, shoe included. |
| `com_above_pedals_m` | 1.0 | Height of the rider's centre of mass above the pedals. |
| `gyration_m` | 0.35 | Radius of gyration of the rigid body (the articulated upper body's inertia). |
| `cda_m2` | 0.6 | Drag area of rider and wheel. |
| `tuck_cda_m2` | 0.42 | Drag area at a full crouch, riding by hand only (Free Ride's crouch). A scenario's own riding keeps `cda_m2`. |
| `articulation.joint_kp`, `articulation.joint_kd`, `articulation.joint_max_nm` | 1000, 60, 180 | Hip and knee posture: stiffness (N·m/rad), damping (N·m·s/rad), strongest torque per joint (N·m). |
| `articulation.sole_friction`, `articulation.sole_contact_time_s` | 1.0, 0.005 | Sole-on-pedal friction and contact softness (s), for `articulated_contact_v1`. |
| `articulation.soft_knee_rad` | 0.52 | Knee bend of the riding stance (rad), for `articulated_lateral_v1`. |
| `articulation.foot_offset_m` | 0.015 | How far each foot stands outside its pedal's centre (m), for `articulated_lateral_v1`. |
| `articulation.hip_adduction_max_nm`, `articulation.hip_rotation_max_nm`, `articulation.knee_rotation_max_nm`, `articulation.ankle_roll_max_nm` | 150, 50, 25, 35 | Strongest hip ab/adduction, hip rotation, tibial rotation and ankle roll per joint (N·m), for `articulated_lateral_v1`. |
| `articulation.pad_friction`, `articulation.pad_contact_time_s`, `articulation.pad_preload_n` | 0.3, 0.03, 20 | A leg on a side pad: friction, contact softness (s), and how hard it rests there (N), for `articulated_lateral_v1`. |
| `articulation.turn_crouch`, `articulation.turn_spread`, `articulation.turn_press_n_per_rad` | 0.5, 1.3, 100 | Per radian of a turn's lean: how much deeper the knees bend (rad), how far the inside knee swings out from its pad (rad) and how much harder the outside leg rests on its pad (N), for `articulated_lateral_v1`. |
| `control.lean_kp`, `control.lean_kd` | 6000, 900 | Ankle stiffness (N·m/rad) and damping (N·m·s/rad) about the lean. |
| `control.ankle_max_nm` | 400 | Strongest ankle torque, both legs. Contact feet carry less: about 70 N·m backward and 120 N·m forward for the default rider, while the centre of pressure stays under the soles. |
| `control.speed_kp`, `control.speed_ki` | 0.6, 0.03 | Acceleration asked per m/s of speed error, and its integral. |
| `control.accel_max`, `control.brake_max` | 1.5, 2.5 | Strongest acceleration and braking asked, m/s². |
| `control.lean_max_deg`, `control.lean_rate_deg_s` | 15, 12 | Largest lean, and how fast it changes. |
| `control.tiltback_feel_deg` | 2 | Tiltback the rider tolerates before easing off; `90` means the rider ignores tiltback. |
| `control.tiltback_ease_kmh_per_deg` | 1.5 | Speed given up per degree beyond that (`0`: ignores tiltback). |
| `control.tiltback_ease_brake` | 1.0 | Strongest braking while easing off, m/s². |
| `control.steer_kp`, `control.steer_kd`, `control.steer_max_nm` | 2000, 200, 60 | How the rider keeps the line (world +x). |
| `control.manual_turn_rate_max_rad_s`, `control.manual_turn_lean_max_deg`, `control.manual_turn_lean_rate_deg_s`, `control.manual_lean_rate_deg_s` | 2.5, 35, 60, 30 | Riding by hand only (Free Ride, Ride's manual mode): the turn cap at a stop, the lean into turns and how fast the leans change. A scenario's own riding never uses them. |
| `control.manual_drive_lean_max_deg`, `control.manual_drive_kp` | 22, 1.2 | Free Ride's drive mode only: the largest lean when asked for an acceleration, and how hard the rider holds the speed the asks add up to. A scenario's own riding never uses them. |

For example, a rider who ignores tiltback and leans harder:

```toml
[rider.control]
tiltback_feel_deg = 90.0
lean_max_deg = 18.0
```

### Measured-speed lean phases: `[[phase]]`

Use phases when the next rider command depends on measured ground speed.
The `speed_kmh` schedule still controls the rider before `phase_start_s`.
Each phase then sends `Intent::Lean` through the existing rider controller.
The scenario never sets body speed, body pose, or motor torque.

```toml
# Top-level fields, before any tables:
seconds = 68.0
release_s = 4.0
phase_start_s = 6.0
speed_kmh = [[0, 0]]

[rider.control]
lean_max_deg = 30.0
lean_rate_deg_s = 60.0
ankle_max_nm = 400.0

[[phase]]
name = "accelerate"
lean_deg = 30.0
until_speed_kmh = 70.0
crossing = "above"
timeout_s = 40.0

[[phase]]
name = "brake"
lean_deg = -30.0
until_speed_kmh = 1.0
crossing = "below"
timeout_s = 20.0
```

| Phase field | Meaning |
|---|---|
| `name` | Unique, nonempty name, at most 64 bytes. |
| `lean_deg` | Desired lean from vertical. Forward is positive. Its magnitude must stay within `rider.control.lean_max_deg`. |
| `until_speed_kmh` | Finite signed forward ground speed. This is a gate, not a speed command. |
| `crossing` | `above`: previous speed is below the threshold, then reaches or exceeds it. `below`: the reverse. |
| `timeout_s` | Positive finite time from this phase's first command. |

The parser accepts at most 16 phases.
The run duration must include the start time, all timeouts, and two seconds of final observation.
A phase completes only at a healthy measured crossing before its timeout.
A crossing at the exact timeout can complete the phase.
Cutoff, fall, halt, and nonfinite speed take priority over a crossing.
A cutoff or fall after release also aborts a pending launch.
Normal boot cutoff flags before release do not abort it.

The next command starts from the crossing sample.
At the default 100 Hz, detection can take up to one sample period.
No interpolation moves the command back in time.
The existing command delay, lean slew limit, ankle torque cap, and rigid rider aids still apply.
Manual lean does not use the speed controller's acceleration limits or tiltback easing.
Firmware protections remain active.
The target speed telemetry channel is unavailable during manual lean.

`Outcome.phases` adds structured measurements to JSON results.
Each phase also adds a verdict check, so an incomplete gate cannot produce a passing run.
The phase status is `reached`, `target_unreachable`, or `not_started`.
Failure reasons include `timeout`, `cutoff`, `fall`, `halted`, `nonfinite_speed`, and `duration_ended`.
Later phases remain `not_started` after failure.
Their time, distance, speed, current, and voltage measurements stay null.

Phase results include command and end times, elapsed time, start and end speeds, and maximum forward speed.
`distance_m` is the signed forward distance change from `Signals.distance_m`, not total path length.
For a straight forward brake, it measures braking distance from the measured 70 km/h crossing.
The stopping gate uses 1 km/h, not exact zero speed.
Current results include the largest absolute motor phase sample and signed q-current and battery-current ranges.
Voltage results give the minimum and maximum bus samples.
These are sample-rate measurements, not electrical-step peak bounds.

After the last healthy crossing, the rider requests zero speed for two seconds of physical observation.
After failure, the rider requests neutral lean for two seconds, unless the machine already halted.
No pose or velocity reset occurs.
Upright, cutoff, and final-speed checks still cover this observation period.
A reached stopping gate does not prove stable standing afterward.

#### Full effort to 70 km/h, then stop

The public cases are `full-effort-70kmh-stop-soc100`, `full-effort-70kmh-stop-soc50`, and `full-effort-70kmh-stop-soc20`.
They use the documented generic 80 kg rider and initial SOC values of 100%, 50%, and 20%.
A `battery_soc` event at time zero sets the initial charge once.
Normal discharge and regeneration then change the charge.
These cases use the same settings, ground, rider limits, and rigid rider aids.

The current cases use five phases, a 30° lean cap, a 24°/s slew limit, and a 400 N·m ankle cap.
The stand releases at 4 s; the first effort command starts at 6 s.

| Phase | Desired lean | Measured forward-speed crossing | Timeout |
|---|---:|---|---:|
| `launch` | +30° | Above 35 km/h | 15 s |
| `build` | +20° | Above 50 km/h | 10 s |
| `accelerate` | +15° | Above 70 km/h | 40 s |
| `brake` | -30° | Below 15 km/h | 15 s |
| `settle` | -5° | Below 1 km/h | 15 s |

Every transition requires an actual measured crossing in the specified direction.
The thresholds do not command speed or motor torque.
A cutoff, fall, halt, or timeout prevents later phases from starting.
The generated phase checks prevent an incomplete target from passing.

Full braking includes **both `brake` and `settle`**.
Its time starts at the measured 70 km/h crossing and ends at the measured crossing below 1 km/h.
Calculate that time as `settle.end_s - brake.start_s`.
Calculate braking distance as `brake.distance_m + settle.distance_m`.
These distances are signed forward changes, not total path length.
Both stopping phases must reach their gates before full stopping time or distance becomes available.
If the 70 km/h target fails, stopping metrics remain **unavailable**, and later phase measurements remain null.
A failed stopping phase can retain partial measurements; those values do not describe a completed stop.

Two seconds of zero-speed rider control follow the final gate.
The upright, cutoff, and final-speed checks still apply during this observation.
The 103 s duration covers all phase timeouts and that observation.
Run-wide top speed and distance include the observation after success or failure.
They do not establish a healthy 70 km/h crossing or a completed stopping distance.

Full effort describes commands inside this bounded envelope.
It does not define a measured human maximum or prove a global acceleration or braking optimum.
Keep failed gates as results when comparing firmware images.
Motor, tire, suspension, battery, and rigid rider results remain PRELIMINARY.
See [full-effort acceptance](../models/full-effort-acceptance.md) for the corrected-RTC evidence and rejected alternatives.

An initial 100% SOC does not guarantee a full battery at the braking gate.
Acceleration consumes charge first.
A battery capacity-limit halt means the model cannot accept further charge.
It does not establish real BMS cutoff voltage, pack isolation, or wheel recovery behavior.
Run all three firmware images at quanta 4000, 3000, and 2000 before a robustness claim.
Keep separate rider-mass studies outside public scenarios and identify their mass in private results.

### The roll aid: `[assist]`

The rider cannot balance sideways yet (the joints bend only forward and back), so a roll spring on
the shell holds it upright:
`roll_stiffness` (5000 N·m/rad) and `roll_damping` (500 N·m·s/rad). Set `roll_stiffness = 0.0` to
see the wheel fall over sideways.

### A synthetic ground: `[terrain]` and `[[terrain.feature]]`

The shipped scenarios ride the city (`[world]`, below). A strip suits a test that needs an exact,
synthetic shape: a rectangular pothole of a given length, a bump of a given profile.

| Field | Default | Meaning |
|---|---|---|
| `slope_pct` | 0 | Grade from `slope_start_m` on, percent (uphill positive, within ±50). |
| `slope_start_m` | 0 | Where the grade starts, metres along the path. |
| `slope_ramp_m` | 10 | Length over which the grade builds up (a parabola, no kink). |
| `length_m` | 300 | Length of the ground (at least 1 m); features must end before it. |

Each `[[terrain.feature]]` has a `kind`:

| `kind` | Fields | Shape |
|---|---|---|
| `pothole` | `at_m`, `depth_m`, `length_m` | A hole with steep walls. |
| `bump` | `at_m`, `height_m`, `length_m` | A smooth hump, `height · (1 − cos)/2` over its length. |
| `curb` | `at_m`, `height_m`, `yaw_deg` (default 0) | A step up onto a sidewalk with a sharp edge: a box whose face crosses the path at `at_m`, turned `yaw_deg` from square on (within ±80, its face 1 m or more ahead of the start across the strip). The path rides it for 4 m / cos(yaw). |

A feature must start at 1 m or later, end before `length_m`, be at least 8 cm long, and have a depth
or height between 0 and 1 m. Features add to the slope. The path runs along world +x; the ground is a
3 m wide strip sampled every 2 cm (flat ground is an infinite plane). The wheel starts at x = 0, so a
feature at `at_m` is reached after that distance: at 20 km/h about 5.6 m per second, after the
speed-up.

### The ground: `[world]`

A scenario can ride the generated city of [models/world.md](../models/world.md) instead of a
terrain strip: streets, curbs, crosswalks, potholes, stairs, ramps and their surfaces. A scenario
has either a non-flat `[terrain]` or `[world]`, never both.

| Field | Default | Meaning |
|---|---|---|
| `seed` | required | The world's seed. The same seed always builds the same world. |
| `revision` | required | The generator revision the scenario was written for: `2`. Another revision is refused, because the world would not be the one the scenario describes. |
| `biome` | `"city"` | The only biome. |
| `start` | the spawn | `{ x_m, y_m, heading_deg }`: the ground point under the wheel and its heading, world coordinates (heading 0 is world +x, left positive). |
| `route` | none | Waypoints `[[x, y], ...]` in world metres, at most 256. After release the rider steers for the next one and moves on within 1 m of it; after the last it holds its heading. |
| `slope_pct` | 0 | A grade over the whole city along the start heading, percent (within ±40; negative is downhill), for a slope test. Streets, buildings and features follow it; the ground behind its start is unchanged. |
| `slope_start_m` | 0 | Where the grade starts, metres ahead of the start. |
| `slope_ramp_m` | 10 | Over which it builds up (a parabola, no kink), as `[terrain]`'s. |
| `[world.densities]` | the defaults | How often things occur (`potholes`, `manholes`, `cracks`, `rough_patches`, `wet_patches`, `speed_bumps`, `crosswalks`, `driveways`, `plazas`, `props`), and `hills`, the scale of the terrain's relief (1 by default, 0 a level city, up to 2); see models/world.md. |

Where the road itself is not the test, use an ideal road: `hills = 0` and no `potholes`,
`manholes`, `cracks`, `rough_patches` or `wet_patches`. The speed, braking and electrical runs in
`scenarios/` do that on the spawn arterial, and say why in their notes. A level platform of a plaza
suits a standstill; a real pothole, curb or stairs suits a terrain test.

To ride the plain flat floor on purpose, write `[world]` with only `flat = "<reason>"`: the reason is
required and no other `[world]` key goes with it. Use it only when the city exposes a model limit
that is not the test; `full-effort-70kmh-stop-soc100` is the one shipped example.

`euc-world-preview --seed <n> --print` (models/world.md, "Preview") draws a world and lists its
plazas, which helps to pick a start and a route. The shipped scenarios are examples (`scenarios/`:
the spawn arterial, a plaza, two real potholes, a 2 % and a 20 % grade), and so are the acceptance
runs in `scenarios/world/`: a curb up straight on and at 30°, a curb down, a pothole, stairs down and
a ramp up.
`sim compare` and the studio run them like any scenario; the studio draws the world around the
wheel.

### Faults: `[[timeline]]`

Each entry has `at_s` (start), optional `for_s` (length; without it the fault lasts to the end),
`fault = "<kind>"`, and the kind's fields. Device faults can name the board instance with `device`
(`imu`, `bms`).

| `fault` | Fields | What breaks |
|---|---|---|
| `hall_stuck` | `line` (`a`, `b`, `c`), `level` (`high`, `low`) | A hall line held at a level. |
| `hall_lost` | `line` | A hall line disconnected (reads high). |
| `hall_noise` | `noise_deg` | Extra jitter on the sensed angle, electrical degrees. |
| `current_sensor` | `sensor` (`a`, `b`, `c`, `bus`), `offset_a`, `gain` (1), `noise_a` | A current sensor reads `gain · i + offset_a`, plus noise. |
| `open_phase` | `phase` (`a`, `b`, `c`) | A motor phase wire open. |
| `stage_overtemp` | `rate_k_per_s`, `max_c` | The power stage heats up as if its cooling failed. |
| `cell_low` | `pack`, `cell`, `drop_v` | One cell group sits below its curve. |
| `pack_imbalance` | `pack`, `soc_delta` | One pack's charge jumps at the start (stays). |
| `battery_soc` | `soc` (0..1) | Every pack's charge is set at the start (stays). |
| `high_resistance` | `pack`, `factor` | One pack's internal resistance times `factor`. |
| `imu_noise` | `accel_g`, `gyro_dps` | Gaussian noise on every IMU sample. |
| `imu_bias` | `accel_g` `[x, y, z]`, `gyro_dps` `[x, y, z]` | Constant offsets on the IMU. |
| `imu_stuck` | none | The IMU's sample registers stop updating. |
| `bms_silent` | none | The smart BMS never answers. |
| `bms_garbled` | `byte_error_rate`, `missing_bytes` | The smart BMS answers with damaged bytes. |

```toml
[[timeline]]
at_s = 15.0
fault = "hall_lost"
line = "a"
```

### BLE commands: `[[inject]]`

What a phone sends: `at_s` and `bytes`, the command as text (the wheel's protocol is in
[protocol/](../protocol/)).

```toml
[[inject]]
at_s = 10.0
bytes = "c"
```

### Checks: `[[expect]]`

Two kinds. Verdict checks can fail: a failing verdict is a finding about the firmware, not a broken
scenario. `record` checks never fail: they print a measurement. Times default to the release
(verdicts) or to power-on (`record`, `firmware_bits`), and to the end of the run.

| `check` | Fields | Passes when |
|---|---|---|
| `upright` | `from_s` | The wheel never falls over (pitch or roll beyond 45°). |
| `pitch` | `max_abs_deg`, `from_s` | \|pitch\| stays within the bound. |
| `no_cutoff` | `from_s` | The firmware never sets a cutoff flag. |
| `cutoff` | `from_s`, `within_s`, `flag` | The firmware cuts within `within_s` of `from_s`, with that flag set if given (`hall_fault`, `balance_disabled`, `drive_fault`). |
| `speed_reached` | `kmh` | The ground speed reaches at least this. |
| `stopped_at_end` | `max_abs_kmh` | \|speed\| at the end is at most this. |
| `rider_on_pedals` | `from_s` | The rider never comes off the pedals while the wheel stays up. Every scenario without `step_off_s` gets it by default; list it to set `from_s`. Welded feet and the rigid rider cannot come off that way, so only `articulated_contact_v1` can fail it. |
| `pitch_reaches` | `deg`, `from_s`, `by_s` | The pitch reaches `deg` (nose up positive; a negative value means a nose dive at least that deep). |
| `ripple` | `signal`, `max`, `from_s`, `by_s` | The signal's standard deviation stays at most `max`. |
| `firmware_tracks` | `var`, `scale`, `signal`, `max_rms`, `from_s`, `by_s` | A firmware variable times `scale` follows a signal within an RMS error. |
| `firmware_bits` | `var`, `bits`, `from_s`, `by_s` | A firmware variable has every bit of `bits` set. |
| `record` | `signal` or `var`, `scale`, `stat`, `threshold`, `from_s`, `by_s` | Always: prints the statistic. |

`stat` is one of `max`, `min`, `mean`, `first_above` and `first_below` (both need `threshold`),
`at_fall` (the value when the wheel first falls) and `at_cutoff` (the value when the firmware first
cuts).

`signal` is a telemetry key: `speed_kmh`, `wheel_rad_s`, `pwm_pct`, `modulation_pct`, `phase_a_a`,
`phase_b_a`, `phase_c_a`, `iq_a`, `torque_nm`, `battery_v`, `battery_a`, `soc_pct`, `pitch_deg`
(nose up positive), `roll_deg`, `board_temp_c`, `motor_temp_c`, `distance_m`, `isr_load_pct`,
`rider_lean_deg`, `target_speed_kmh`, `suspension_mm`, `shock_mm`. An unknown key is an error that
lists them.

`var` is a firmware variable: by stock-map name for the stock image (`profiles/<wheel>/stockmap.toml`:
`alerts`, `speed_cms`, `vq_active`, `pwm_alarm`, `tiltback_speed_deg`, `tiltback_battery_deg`,
`tiltback_pwm_deg`, `attitude_pitch_deg` (nose down positive), `drive_command`, `balance_disabled`,
`drive_fault`, `hall_fault`), and by the same names for our builds (`profiles/<wheel>/probes.toml`, which
adds a few of its own, such as `riderless_triggers`). A name the firmware's map does not have stops
the run with an error before it starts.

## Two worked examples

### A city pothole

A real 2 cm pothole of the city (seed 42), ridden at 20 km/h after a level, straight approach of
50 m (`euc-world-preview` draws the area, so the approach can be checked by eye):

```toml
# A 2 cm city pothole at 20 km/h, on the maintainer's settings.
#
# Rider: the default rider.
schema = 1
name = "city-pothole-20kmh"
description = "Ride into a 2 cm deep city pothole at 20 km/h: does the wheel stay up?"
wheel = "begode-extreme"
seconds = 25.0
release_s = 2.5
speed_kmh = [[0, 0], [6, 0], [11, 20]]
settings = "tarik-2026-10-15"

# The pothole at (-31.13, -70.91), 50 m south of the start; a level city (hills = 0) with no other
# defects on the approach. Potholes keep their density, so this one is there.
[world]
seed = 42
revision = 2
start = { x_m = -31.13, y_m = -20.91, heading_deg = -90.0 }

[world.densities]
hills = 0.0
manholes = 0.0
cracks = 0.0
rough_patches = 0.0
wet_patches = 0.0

# The safety verdict.
[[expect]]
check = "upright"

[[expect]]
check = "no_cutoff"

# Measurements: they always pass.
[[expect]]
check = "record"
signal = "distance_m"
stat = "first_above"
threshold = 50.0

[[expect]]
check = "record"
signal = "pitch_deg"
stat = "min"
from_s = 15.0

[[expect]]
check = "record"
signal = "suspension_mm"
stat = "max"
from_s = 15.0

[[expect]]
check = "record"
signal = "iq_a"
stat = "max"
from_s = 15.0
```

The `distance_m` record tells when the pothole comes (at about 17.5 s), so a later check can start
just before it.

### A synthetic bump, then a slope

When the shape is the test, a `[terrain]` strip gives it exactly: a 6 cm bump at 15 km/h, then an
8 % climb that builds up from 60 m:

```toml
# A 6 cm speed bump at 15 km/h, then an 8 % climb, on the maintainer's settings.
#
# Rider: the default rider.
schema = 1
name = "bump-then-hill-15kmh"
description = "A 6 cm bump at 15 km/h, then an 8 % climb: the impact, then the climbing current."
wheel = "begode-extreme"
seconds = 40.0
release_s = 2.5
speed_kmh = [[0, 0], [6, 0], [10, 15]]
settings = "tarik-2026-10-15"

[terrain]
slope_pct = 8.0
slope_start_m = 60.0      # the grade builds up over slope_ramp_m (10 m by default)
length_m = 200.0

[[terrain.feature]]
kind = "bump"
at_m = 30.0
height_m = 0.06
length_m = 0.5

[[expect]]
check = "upright"

[[expect]]
check = "no_cutoff"

[[expect]]
check = "record"
signal = "pitch_deg"
stat = "max"
from_s = 10.0
by_s = 18.0

[[expect]]
check = "record"
signal = "iq_a"
stat = "mean"
from_s = 30.0

[[expect]]
check = "record"
signal = "speed_kmh"
stat = "min"
from_s = 30.0
```

## Robustness: check a fall or a survival at three quantum sizes

The emulator runs the firmware in quanta of up to 4000 instructions. Near a limit, whether the wheel
falls can depend on that timing. Before you write down a fall or a survival as a result, run the
scenario at `--max-quantum 4000`, `3000` and `2000` (on every firmware you report). If the verdict is
the same at all three, it is robust; if it changes, call it **borderline** in the scenario's header and
in [models/scenarios.md](../models/scenarios.md), and do not use it as evidence on its own.

## PRELIMINARY results

Mark a result PRELIMINARY, in the scenario's header and wherever it is reported, when it depends on a
part of the model that is known to be too simple for that case, and say which part (for example: the
rider has no knees; the suspension values are estimates; the tire is a soft contact). Remove the mark
when that part is modelled or measured. Never tune a scenario's values until a check passes: change
the scenario only when the situation it describes needs it, and say why in the header.

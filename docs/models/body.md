# Body physics

The body model is everything mechanical: the tire on the ground, the wheel spinning in the shell,
the shell with its pedals and IMU, and the rider standing on the pedals. It runs in MuJoCo 3.9
(through `mujoco-rs` 5.0) in `crates/euc-physics`, and couples to the electrical plant
([electrical.md](electrical.md)) and the emulated controller in `crates/euc-sim`. The wheel's values
live under `[body]` in `profiles/<wheel>/plant.toml`; the rider and the aids come from the scenario,
or their defaults.

**Defaults are not measurements.** Most values below are estimates (marked DEFAULT in `plant.toml`).
They make a plausible Begode Extreme with an 80 kg rider, and they let the stock firmware balance and
ride, but the real wheel differs. Replace each one when it is measured, and record the measurement here.

## How it runs

```
             motor torque (mean over the step)                  wheel angle, speed
 electrical plant ─────────────────────────▶ axle hinge ──────────────────────────▶ electrical plant
   (5 µs grid)                                 │ reaction on the shell                (halls, back-EMF)
                                               ▼
                      MuJoCo, 0.5 ms: tire, wheel, shell, rider ──▶ IMU site ──▶ MPU-6500 model
                              ▲ ankle torque          ▲ roll spring, air drag
                         rider controller        aids (M2)
```

- **One clock.** Everything counts the controller's HCLK cycles (72 MHz). A physics step is 36 000
  cycles (0.5 ms); the electrical plant steps every 360 cycles (5 µs), lazily. The `plant` board
  component schedules each physics step as an event on the cycle grid, so a run is deterministic and
  repeats exactly.
- **Each step:** advance the electrical plant to the step's cycle, take its mean wheel torque since
  the last step, apply it to the axle hinge (MuJoCo applies the reaction to the shell), step MuJoCo,
  hand the wheel's new angle and speed back to the plant (which extrapolates the angle at that speed
  until the next step), and sample the IMU.
- **Hall edges** get their own events at the cycle the plant predicts (see electrical.md), so the
  firmware sees each edge on time, not on the 0.5 ms grid.
- The step uses MuJoCo's two-phase step: positions and velocities first, then the rider controller
  and the aids act on that state, then accelerations and integration (`implicitfast`). The IMU and
  the body poses describe the state at the start of the last step: 0.5 ms old, against a firmware
  that reads the IMU every 2 ms.

## Frames and signs

| Quantity | Convention |
|---|---|
| World, shell | x forward, y left, z up, metres |
| Wheel angle and speed | rotation of the wheel about the shell's +y; positive rolls forward |
| Motor torque | on the wheel about +y, positive forward; the shell takes the opposite |
| Rider lean | angle of the rider from vertical, positive forward |
| Telemetry pitch | shell pitch, **nose up positive** (the firmware's own pitch is nose down positive) |
| Telemetry roll | right side down positive |
| IMU | the shell frame at the IMU site: x forward, y left, z up; specific force reads +1 g on z at rest |

The IMU mapping follows the balance findings: the firmware's pitch is the rotation about the IMU's
+y, positive nose down, with x forward. A nose-down shell therefore reads `ax = −g·sin(pitch)` and a
positive y rate. The roll sign of the real mounting is not known (UNKNOWN in the findings); it does not
matter while the roll spring keeps the wheel upright.

## The model

```
world ── floor (plane)
  └─ shell (free joint)     battery packs, handle, pedals, IMU site; origin at the axle
       ├─ wheel (hinge "axle", y)   tire: an ellipsoid round in the wheel plane
       └─ rider (hinge "lean", y)   rigid body hinged at the ankles, ankle motor "ankle"
```

`Physics::mjcf()` returns the generated MJCF for inspection.

- **Tire.** An ellipsoid with semi-axes (r, half width, r): a circle of radius r in the wheel plane,
  so it rolls exactly, and rounded across the tread. Contact `condim = 6` gives sliding, torsional and
  rolling friction. Rolling resistance is MuJoCo's rolling friction, a length: `crr · r`, so the
  resisting torque is `crr · N · r`, the classic `crr · N` force at the contact. The contact time
  constant (`solref`) is the tire's softness: 20 ms is about 200 kN/m under the loaded wheel. The
  ellipsoid's crown is sharper than a real tire's; this only matters for leaning in turns, which M2 does
  not simulate.
- **Wheel.** The spinning part of the hub motor (rotor, magnets, rim, tire). The stator belongs to the
  shell. The spin inertia is set directly; the other two axes are half of it (a ring).
- **Shell.** Mass, centre of mass and inertia from the parameters; the shapes (two battery packs, a
  handle, two pedals) are for drawing and for the contacts of a fall only.
- **Stand.** A weld from the shell to the world holds the wheel upright and still while the firmware
  boots; the scenario releases it. Before time starts, the model settles on the stand for 0.2 s, so
  the first IMU sample already reads a wheel at rest (both the weld and the tire contact are soft).

## The rider (M2: rigid)

The rider is one rigid body: mass, centre of mass height above the pedals and a radius of gyration,
hinged at the ankles. The ankles are a motor driven by the rider controller:

1. **Target speed to lean.** `a = kp·(v_target − v) + ki·∫(v_target − v)`, limited to the rider's
   strongest acceleration and braking; `lean = atan((a + drag(v_target)/m) / g)`, limited to ±15°
   and ramped at 12°/s. The integral only runs within 1 m/s of the target and while the request is
   not limited, so it trims the steady speed without winding up.
2. **Lean to ankle torque.** `τ = kp_lean·(lean_cmd − lean) − kd_lean·lean_rate`, limited to the
   strongest ankle torque. The pedals take the opposite torque; that is what the firmware's balance
   loop feels when the rider leans.

Leaning works on a real wheel the same way: the firmware keeps the pedals near its set angle, so a
rider who shifts the centre of mass forward makes it accelerate to stay under them. In a steady
acceleration `a = g·tan(lean)` the ankles need no torque at all, which is why the controller holds a
lean and does not push.

What the rigid rider cannot do, and the M4 human rider will: bend knees and hips, absorb bumps,
step on and off, steer, and balance sideways.

## Aids (M2)

- **Roll spring.** A rigid rider cannot balance sideways (a real rider steers and twists), so a
  torque about the heading holds the shell upright: `τ = −k·roll − c·roll_rate`, k = 5000 N·m/rad,
  c = 500 N·m·s/rad, far stiffer than the toppling moment of wheel and rider (about 1200 N·m/rad).
  It shows in recordings as a roll that stays at zero. Set `[assist] roll_stiffness = 0` in a scenario
  to see the wheel fall over sideways.
- **Air drag** acts on the rider's centre of mass: `F = ½·ρ·CdA·v·|v|`, ρ = 1.2 kg/m³,
  CdA = 0.6 m² (sim.py's value, an upright adult).

## Parameters

Wheel (`[body]` in `plant.toml`, Begode Extreme values):

| Parameter | Value | Source |
|---|---|---|
| `tire.radius_m` | 0.209 m | rolling circumference 1.313 m, the same the motor model assumes with 26 pole pairs, so the firmware's speed equals the true speed (firmware: 0.0505 m per electrical turn) |
| `tire.half_width_m` | 0.038 m | 3 in tread |
| `tire.friction` | 0.9 | DEFAULT: rubber on dry asphalt |
| `tire.crr` | 0.012 | DEFAULT: research sim.py |
| `tire.contact_time_s` | 0.02 s | DEFAULT: about 200 kN/m |
| `wheel.mass_kg` | 16 kg | DEFAULT: rotor, rim and tire |
| `wheel.spin_inertia_kgm2` | 0.42 kg·m² | DEFAULT: 0.6·m·r² |
| `shell.mass_kg` | 26 kg | DEFAULT: 42 kg with the wheel |
| `shell.com_above_axle_m` | 0.10 m | DEFAULT |
| `shell.inertia_kgm2` | 0.85, 0.92, 0.46 kg·m² | DEFAULT: a 0.35 × 0.30 × 0.55 m box |
| `shell.pedal_below_axle_m` | 0.03 m | DEFAULT |
| `shell.imu_above_axle_m` | 0.15 m | DEFAULT: research sim.py |

Rider and aids (scenario `[rider]`, `[rider.control]`, `[assist]`; these are the defaults):

| Parameter | Value | Note |
|---|---|---|
| `mass_kg` | 80 kg | |
| `com_above_pedals_m` | 1.0 m | about 55 % of a 1.78 m adult's height |
| `ankle_above_pedals_m` | 0.08 m | the hinge |
| `gyration_m` | 0.35 m | pitch and roll; 9.8 kg·m² about the centre of mass |
| `height_m` | 1.78 m | drawing only |
| `cda_m2` | 0.6 m² | air drag |
| `control.lean_kp`, `lean_kd` | 6000 N·m/rad, 900 N·m·s/rad | about 1.3 Hz, damping 0.6 about the ankles |
| `control.ankle_max_nm` | 400 N·m | both legs |
| `control.speed_kp`, `speed_ki` | 0.6 /s, 0.03 /s² | |
| `control.accel_max`, `brake_max` | 1.5, 2.5 m/s² | |
| `control.lean_max_deg`, `lean_rate_deg_s` | 15°, 12°/s | |
| `assist.roll_stiffness`, `roll_damping` | 5000 N·m/rad, 500 N·m·s/rad | |

## Scenarios

A scenario (`scenarios/*.toml`) is a scripted ride with expectations; `euc sim run --scenario FILE`
runs it on the wheel it names and prints each check. The target speed is a list of `[seconds, km/h]`
points, linear in between. `release_s` is when the stand lets go; BLE commands go in `[[inject]]`;
`push = [[start s, N, duration s]]` pushes the rider forward (or back, negative) at the centre of mass;
`settings = "<preset>"` boots the wheel with a settings preset (see [../bridge.md](../bridge.md#settings-presets)).
Both shipped scenarios use `neutral`: factory settings with level pedals.
The stand should let go soon after the firmware starts balancing: held against a rigid stand, the
stock firmware's balance output climbs to its limit if its set angle is not the stand's.
Checks (`[[expect]]`, judged after the run, from the release unless `from_s` says otherwise):

| `check` | Passes when |
|---|---|
| `upright` | pitch and roll stay within 45° |
| `no_cutoff` | none of the firmware's cutoff flags (stock map RAM variables with `role = "cutoff"`) is set |
| `pitch` | \|pitch\| stays within `max_abs_deg` |
| `speed_reached` | the ground speed reaches `kmh` |
| `stopped_at_end` | \|speed\| at the end is within `max_abs_kmh` |

`--record FILE` writes a recording the studio replays: a JSON header (wheel, firmware, scenario,
bodies with their shapes, channel schema), the telemetry batches of `euc-telemetry`, and the events
(see `crates/euc-sim/src/recording.rs` for the layout).

## To measure on a real wheel

- Rolling radius: mark the tire, roll one turn with a rider on, measure.
- Masses: weigh the wheel; weigh the shell side with the motor out if it is ever apart.
- IMU position and mounting signs: open the shell; check the signs with the wheel tilted.
- Shell inertia: swing the wheel as a pendulum from the handle.
- Rolling resistance: coast-down from 10 km/h with the motor off (free-wheel), log the speed.

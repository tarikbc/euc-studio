# Body physics

The body model is everything mechanical: the tire on the ground, the wheel spinning in the shell,
the rear suspension between them, the shell with its pedals and IMU, and the rider standing on the
pedals. It runs in MuJoCo 3.9
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
world ── floor (plane, or the scenario's terrain)
  └─ shell (free joint)     scanned solids (packs, seat, handle, roller, shock, pedals), IMU site;
       │                    origin at the axle at full extension
       ├─ carriage (slide "suspension")   stator, axle clamps, slider blocks; spring, damper, end stops
       │    └─ wheel (hinge "axle", y)    tire: a torus, as one convex mesh
       └─ feet (hinge "lean", y)    rigid rider only: hinged at the ankles, ankle motor "ankle"
  └─ rider (free joint)     articulated (default): upper body
       ├─ left_thigh (hinge "left_hip") ── left_shank ("left_knee") ── left_foot ("left_ankle")
       └─ right_thigh ...                  each foot welded to the shell ("on_<side>_pedal"), or for
                                           articulated_contact_v1 a sole-on-pedal pair "<side>_sole_on_pedal"
          articulated_lateral_v1: each leg adds hip ab/adduction and rotation, the tibia's rotation
          and ankle roll (14 joints), and knee, shin and calf geoms that rest on the side pads
     rigid: one body, welded to the feet ("on_pedals") until it steps off
```

Without `[body.suspension]` the carriage is left out and the wheel hinges straight in the shell.

`Physics::mjcf()` returns the generated MJCF for inspection.

- **Tire.** A torus around the axle with the scanned crown radius (tube 0.058 m at 0.1716 m, so
  0.2296 m outside: the free tire, from `geometry.toml` `[collision]`), built as one convex mesh: the
  tube's outer half, 360 points around and 13 across, within 9 µm of the torus around and 0.5 mm
  across. (Until 2026-10 it was a ring of 48 capsules. The ring sat 0.36 mm low between capsules and
  handed its contact from capsule to capsule, so at 15 km/h the shell shook at 145 Hz, 48 times the
  wheel's turn rate: IMU accel x 0.67 m/s² RMS, the pedal welds ±17 N, and ±250 N and accel z
  9.8 m/s² RMS on a wheel without suspension. Holding 15 km/h on stock firmware took 9.5 A instead
  of 5.8 A. On the mesh: accel x 0.07 and z 0.02 m/s², welds 0.6 N.) Without measured shapes the
  tire is an ellipsoid with semi-axes (r, half width, r).
  The tire is rubber at the owner's 25 psi. It meets each ground geom (the floor, each curb, the
  streaming world's window and pool) through its own MuJoCo pair, soft along the normal and stiff
  along the ground; nothing else touches it.
  - Give: a 3.00-12 tire at 25 psi is about 125 kN/m (membrane estimate 2π · p · √(R · r_crown) =
    2π · 172 kPa · √(0.2296 m · 0.058 m)). The pairs' constant impedance (0.5) with
    `contact_time_s` 0.022 s gives that: the loaded Extreme sinks 9.5 mm and its axle rides at
    0.2201 m. At speed it rolls on 0.2247 m (30 km/h on stock firmware), within 0.2 % of the
    firmware's 0.2251 m. (Until 2026-10 the contact was near rigid, 0.4 mm under the loaded wheel,
    about 3 MN/m, and the tire was built 4.5 mm small to stand on the loaded radius.)
  - Grip: `friction` 1.1, the owner's report for dry asphalt (about 1.0-1.2). MuJoCo softens a
    contact's friction with its normal: on the soft contact alone the tread crept 20-30 % under
    drive and brake torque. The pairs' grip reference is stiff (`solreffriction` 0.002 s), which
    needs elliptic friction cones (the model's, for every contact): a full-effort launch and
    `hard-braking-30kmh` slip under 1 %. No brake of the stock firmware slides the tire: it caps the
    brake at about −168 N·m (`hard-braking-30kmh`, and the same with a rider asking 10 m/s²), a
    friction use of about 0.62 under the loaded wheel. The owner reports a rare slide in a very hard
    brake; this firmware cannot reproduce it, and it stays an open item.
  - Rolling resistance: a torque on the wheel against the ground, each tire contact's normal force
    times its rolling coefficient `crr · r` (which the streaming world scales per surface), faded out
    below 0.3 m/s so it never turns a wheel at rest. It is `crr · N · r` exactly: holding 5 km/h
    takes 3.45 N·m against 3.23 N·m for `crr · N · r` (the rest is air). MuJoCo's own rolling
    friction (`condim 6`, until 2026-10) delivered about a third of that near 5 km/h and 90-100 %
    at cruising speeds. `crr` 0.014 is INFERRED from the owner's ride log 2026-10-17 (light rider):
    its steady stretches above 15 km/h, with the slope (GPS altitude over distance) and the
    acceleration taken out, against steady flat cruises of the model with the same rider, per
    5 km/h band of battery power. A fit of `crr` alone gives 0.030-0.038 and a residual that changes
    sign with speed (+50 to +180 W below 40 km/h, −160 W at 52-60 km/h), so it is rejected. `crr`
    with a constant power fits within ±150 W: `crr` 0.014 (±0.01) and 270-400 W at every speed,
    which rolling resistance cannot make (an open item, electrical.md "To measure").
  - Curbs: with the sagittal rider on stock firmware the tire takes 5, 8 and 10 cm curbs at 5, 10
    and 20 km/h, square on and at 30°. From 12 cm the rider falls at the impact: the legs do not
    absorb it yet. 20 cm is not climbed.
- **Wheel.** The spinning part of the hub motor (rotor, magnets, rim, tire). The stator rides on the
  suspension carriage (in the shell without a suspension). The spin inertia is set directly; the other
  two axes are half of it (a ring).
- **Suspension.** The Extreme's motor axle slides up into the frame on two vertical rods, held down
  by a coil-over shock at the rear through a rocker linkage (per side, from the owner's photos: the
  shock runs forward and down from its top eye on the rocker, under the rear roller, to its bottom
  eye on the carriage, a link joins the rocker's lower pin to a second pin on the carriage, and
  the rocker turns about a pin on the frame, `docs/hardware/geometry.md`, "Rear linkage"). In the
  model the carriage (stator, axle clamps, slider blocks: `unsprung_mass_kg`)
  slides on a joint along `axis_forward_deg` from the shell's vertical, and the wheel spins on the
  carriage, so the motor's reaction torque still reaches the shell. The joint counts compression
  from full extension (the scanned state, where the shell's heights are measured). The linkage's
  leverage `r(x)` (axle travel per shock travel) may change with the axle travel `x`: `ratio_curve`
  gives it as points joined by straight lines, or `linkage_ratio` holds it constant. The shock
  compresses `s(x) = ∫ dx / r` (exact on each straight piece), its whole stroke sets the axle
  travel, and each step `Physics::step` pushes the carriage down with
  `(k·(s(x) + preload) + c·ẋ / r(x)) / r(x)` (the coil never pulls). With a constant ratio that is
  a wheel rate `k / r²` and damping `c / r²`. MuJoCo's joint limits are the end stops (top-out and
  bottom-out, time constant 4 ms). The model starts at the static sag under the shell and the
  rider (where that force carries their weight); on the stand the spring presses the tire into the
  ground, so the sag shows once the stand lets go. Pedals, IMU and rider are on the frame (sprung).
  The tire and the shell never touch each other (an explicit contact exclusion: the tire runs inside
  the frame). Telemetry channels: `suspension_mm` (axle travel) and `shock_mm` (shock travel). The
  studio poses the drawn linkage from `suspension_mm`; its shock length follows the drawn linkage,
  not `shock_mm` (they differ by up to 8.4 mm while the physics keeps the constant ratio).
- **Shell.** Mass, centre of mass and inertia from the parameters. The shapes are the scan's collision
  solids (`geometry.toml` `[collision]`: battery packs with their slider channels, seat, handle and
  headlight, rear roller, shock, pedal brackets, and estimated pedal decks), for drawing and for the
  contacts of a fall; without them, generic packs, handle and pedals sized from the tire.
- **Stand.** A weld from the shell to the world holds the wheel upright and still while the firmware
  boots; the scenario releases it. Before time starts, the model settles on the stand for 0.2 s, so
  the first IMU sample already reads a wheel at rest (both the weld and the tire contact are soft).

## The rider

The rider models share one controller (below). A scenario names its model in `[rider] model`.

- **`articulated_sagittal_v1` (the default since 2026-10-22).** An upper body (trunk, head, fixed
  arms) on two legs, each a thigh, a shank and a foot on hip, knee and ankle hinges that turn
  forward and back only. Its feet are welded to the pedals. See
  [the articulated rider](#the-articulated-rider-default) below.
- **`articulated_contact_v1` (opt-in).** The same articulated rider with each sole standing on its
  own pedal through friction contact instead of a weld. It has its own id so that a recording or a
  result that names `articulated_sagittal_v1` keeps meaning welded feet.
- **`articulated_lateral_v1` (opt-in, built in stages from 2026-10-23).** The articulated rider
  with legs that also move sideways and twist, de Leva's segments and soft knees, its feet welded
  to the pedals. See [the lateral rider](#the-lateral-rider-opt-in) below.
- **`rigid_ankle_v1`.** One rigid body: mass, centre of mass height above the pedals and a radius
  of gyration, hinged at the ankles and welded to the feet. It is the controlled baseline of the
  results before 2026-10-22; a scenario selects it with `model = "rigid_ankle_v1"`, and
  `euc sim run --rider-model rigid_ankle_v1` overrides any scenario. Recordings name the model
  that ran (`context.rider_model`).

The rider controller sets the lean the ankles hold:

1. **Target speed to lean.** `a = kp·(v_target − v) + ki·∫(v_target − v)`, limited to the rider's
   strongest acceleration and braking; `lean = atan((a + drag(v_target)/m) / g)`, limited to ±15°
   and ramped at 12°/s. The integral only runs within 1 m/s of the target and while the request is
   not limited, so it trims the steady speed without winding up.
2. **Lean to ankle torque.** `τ = kp_lean·(lean_cmd − lean) − kd_lean·lean_rate`, limited to the
   strongest ankle torque. The pedals take the opposite torque; that is what the firmware's balance
   loop feels when the rider leans.
3. **Tiltback eases the rider off.** When the firmware tips the pedals back (nose up) by more than
   2° while the rider wants to go forward, and the rider is not already slowing down, the rider
   aims 1.5 km/h below the present speed for every degree beyond 2°, braking at most 1 m/s² (it
   stops pushing rather than brakes hard), down to a stop and never backwards, so a speed, battery
   or PWM tiltback (or a limp mode) slows the ride the way it slows a real rider. A stopped rider
   waits for the tiltback to fall back within 2° before setting off again. The tiltback the rider
   feels is the firmware's own (the sum of its speed, battery and PWM offsets) when the simulator can
   read it, as for the stock map and our builds; otherwise it is the pedals' pitch above the riding
   pitch, the angle the rider is used to (a 3 s average that does not rise while a tiltback is felt).
   Either way a tiltback counts in full even when the rider leans hard and the shell rides nose down.
   A tiltback read from the firmware is felt at once, as the firmware commands it, a little before
   the pedals have moved; a real rider feels the pedals.
   Set `[rider.control] tiltback_ease_kmh_per_deg = 0` or `tiltback_feel_deg = 90` to ride through
   tiltback.
4. **Steering.** The rider keeps the line: feet and legs twist the wheel back with
   `τ = −kp_steer·heading − kd_steer·yaw_rate` (2000 N·m/rad and 200 N·m·s/rad, at most 60 N·m; all
   DEFAULT). A rigid rider cannot twist, so the torque acts on the shell about the vertical, like the
   roll spring; it stops when the rider steps off. Without it, a wheel that slows on a steep grade
   turns across the slope and falls. The line is world +x, the direction of every scenario so far; a
   scenario that turns will need a path heading.
5. **Letting go.** A rider leaves a wheel that goes down: once the firmware has cut the drive (the
   session watches the cutoff flags) and the shell's pitch or roll passes 20°, or as soon as either
   passes 45° (a fall, cut or not), the rider lets go of the pedals 0.1 s later. The feet leave the
   pedals as in a step-off, but without the stepping foot's push, so the rider keeps their own
   momentum; ankle torque, steering and the roll aid stop. The timeline records "rider off" with
   the reason. `[rider.let_go]` sets `after_cut_deg`, `fall_deg` and `reaction_s` (DEFAULT 20°, 45°,
   0.1 s), and `enabled = false` keeps the rider on the pedals whatever happens (with contact feet
   the soles can still leave the pedals physically).

Leaning works on a real wheel the same way: the firmware keeps the pedals near its set angle, so a
rider who shifts the centre of mass forward makes it accelerate to stay under them. In a steady
acceleration `a = g·tan(lean)` the ankles need no torque at all, which is why the controller holds a
lean and does not push.

### The articulated rider (default)

Segment masses are fractions of the rider's mass (each foot 2 %, shank 5 %, thigh 10 %, upper body
66 %); segment lengths scale with `height_m`; the upper body's centre of mass sits where the whole
rider's lands at `com_above_pedals_m`. All are generic ESTIMATES, not anyone's body. The hips and
knees hold a posture (`[rider.articulation] joint_kp`, `joint_kd`, at most `joint_max_nm` per joint;
DEFAULT 1000 N·m/rad, 60 N·m·s/rad, 180 N·m), so they give under a bump and come back.

- **Lean from the ankles.** The lean controller turns the upper body's pitch error into one task
  moment, applied equally at the hip, knee and ankle of each standing leg (the Jacobian transpose
  of the upper body's pitch over a supported foot), so the joints keep internal reactions and the
  damping removes energy. A standing ankle has no posture spring: the hips and knees keep the body
  straight and the whole body leans from the ankles, as on a real wheel and as the rigid rider
  does. Until 2026-10-22 the ankles also held a posture spring; the three joints then shared the
  lean, the legs leaned about a third as far as the upper body, and the wheel accelerated about half
  as hard as the speed controller asked. Measured before and after the change:

  | Measure | Ankle spring (before) | No standing ankle spring (now) | Rigid rider |
  |---|---|---|---|
  | `overlean-top-speed`, stock: speed at 15 s | 17.1 km/h | 33.6 km/h | 30.6 km/h |
  | Steady 3.3 m/s, test balance law: upper-body lean, centre-of-mass lean | 1.92°, 1.28° | 0.74°, 0.93° | (one body) |
  | Largest gap between the two leans | 0.69° | 0.19° | 0 |
  | Mechanical braking test: stopping travel, axle work | 7.46 m, −68 J | 5.05 m, −195 J | |

  The centre-of-mass lean is the angle from vertical of the line from the ankles to the rider's
  centre of mass; the rigid rider's hinge angle is exactly that line. A regression test keeps the
  two leans within 0.5° in steady riding.
- **Feet on the pedals.** With `articulated_sagittal_v1` (the default) both feet are welded to
  the pedals until the rider leaves. With `articulated_contact_v1` each sole is a box (24 × 9 × 6 cm,
  from 9 cm behind the ankle to 15 cm ahead) that touches only its own pedal, through an explicit
  contact pair: friction `sole_friction` (DEFAULT 1.0, rubber on grip pins), softness
  `sole_contact_time_s` (DEFAULT 5 ms). A foot can lift, rock onto heel or toe and slip while the
  other keeps its contact; the pedal pushes and never pulls. The soles therefore carry an ankle
  moment only while the centre of pressure stays under them: for the default 80 kg rider about
  70 N·m backward (heel) and 120 N·m forward (toes), both feet together, where a welded foot passes
  the controller's whole 400 N·m. The lean controller was tuned on welded feet and does not keep
  the centre of pressure under the soles: on contact feet the rider comes off the pedals in the hard
  brake of `hard-braking-30kmh` (at 17.7 s), on the 20 % climb of `hill-climb-low-battery` (33.4 s)
  and in the 30° launch of the full-effort runs, while the riderless wheel stays up. Contact feet
  stay opt-in until the controller keeps within what the soles carry. The `rider_on_pedals`
  check, in every scenario without a step-off, fails when this happens.
- **The roll aid holds the upper body too** (`articulated_contact_v1`; the lateral rider's
  upper-body spring leans the whole machine instead, see the lateral rider). The legs have no roll joints and
  contact feet do not hold the body to the shell, so the same roll spring that holds the shell also
  holds the upper body at the wheel's roll target; it stands in for the hip and ankle roll a person
  balances with. Its torque and work add to the reported roll aid. Without it a one-foot stance, or
  the end of a turn, topples the rider sideways off the pedals.
- **Lifting a foot** (`articulated_contact_v1`). `Physics::set_foot_lift(foot, rad)` bends one leg (that much more hip flexion,
  twice that at the knee, the foot kept level) at 3 rad/s; 0.3 rad lifts the foot about 2 cm. One
  foot at a time; the other leg then carries the whole lean task. Its sole sticks to its pedal
  meanwhile (under 1 mm of slip in a quarter-second lift), and the foot comes down where the leg
  puts it, while the other stays within about 1 cm. A contact foot's ankle damping
  (`articulation.joint_kd`) is a joint damping that the implicit integrator handles, not part of
  the motor command: a foot off its deck turns on the ankle alone (about 0.01 kg·m²), and the
  explicit damping chattered between the ±200 N·m caps every step. The soles then hopped for up to
  1 s and slid 2-8 cm back on their decks (2026-10-23).
- **Crouch** (both articulated models; Free Ride's crouch input). `Physics::set_crouch(knee_rad)`,
  0 to 1.2 rad of knee flexion. The knees bend toward it at 3 rad/s (a full crouch in 0.4 s) and
  the hips by 0.58 of it, while the upper body keeps the lean the controller holds. At 0.58 the
  centre of mass stays within 1° of the ankles at any depth; with the hips at half the knee angle
  a full crouch put it 7.8° ahead, and the wheel rode off with the stick at rest. A full crouch
  lowers the centre of mass by 0.15 m. Asked for at once while riding, it pitches the shell by
  1.6° at most (10° without the 3 rad/s ramp). The segment masses behind the 0.58 are estimates.
- **Riding by hand** (Free Ride and Ride's manual mode; `RiderController::set_manual`). The manual
  limits of `[rider.control]` apply instead of the scripted ones. The turn cap is
  `manual_turn_rate_max_rad_s` (2.5 rad/s) at a stop and falls with speed to the rate whose lean into
  the turn is `manual_turn_lean_max_deg` (35°, 0.7 g: 1.4 rad/s at 5 m/s, 0.76 rad/s at 9 m/s). The
  lean into the turn changes at up to 60°/s, and the forward lean at 30°/s. Turn requests do not
  fade at walking pace: a stopped rider turns on the spot, through the twist aid (an external yaw
  torque on the shell, at most 60 N·m) that already held the heading below walking pace. The
  sagittal legs have no twist joint, and the welded rider turns with the wheel (the lateral rider's
  hips and tibiae twist, and its hips turn the body with the shell). The line never runs more than
  0.5 rad ahead of the wheel, so a wheel held back (a wall, the twist aid's cap) winds up no turn
  that would go on after the input stops. On the stock firmware (2026-10-22)
  a full turn input at a stop turned the wheel 3.75 rad in 1.5 s, and at 3.8-4.1 m/s it settled at
  6.8-7.0 m/s² of lateral acceleration with 34° of lean. Scripted riding keeps its limits (1.0 rad/s,
  20°, 30°/s, 12°/s, no turning at a stop), so no scenario changes.
- **Manual drive** (Free Ride's drive mode; `Intent::Accel`, by hand only). The rider is asked for
  an acceleration, net of drag, instead of a lean. It leans for the ask plus the drag at the speed
  now, `tan(lean) = (a + drag) / g`. It also follows a reference speed that the asks add up to,
  adding `manual_drive_kp` (1.2 /s) per m/s that the reference leads. An ask carries the reference at
  most 0.5 m/s past the wheel, so a wheel the firmware holds back does not wind the rider up. Letting
  go of an ask that the wheel still lags stops the reference at the wheel; before, it stayed up to
  the band ahead, and a half throttle let go at 20 km/h settled at 21.3 km/h instead of 20.0. The
  reference never follows the wheel's speed ripple (about ±0.25 m/s at 2.5 Hz at 40 km/h): a
  clamp to the measured speed ratcheted it down and lost 7 km/h in 10 s. As the lean returns
  after a push, the wheel drives forward under the rider and runs on: a half throttle let go at
  20 km/h peaks 4.7 km/h higher (full throttle 8.1 km/h) and settles back at the speed let go, and
  after a half throttle to 40 km/h the speed stays at 39-44 km/h for 10 s. The owner's ride log
  (2026-10-17) shows the same run-on: pushes of about 2.2 m/s² end in about 0.74 s (80 to 20 % of
  the push, a lean rate near 18°/s), and the speed rose 0.6-7.9 km/h (median 3.0) from the
  let-go to the peak. A faster drive lean (100°/s) was tried: built that fast, it kicked the
  pedals nose up at a launch, the wheel rolled back and the rider fell; let go that fast, it did
  not lower the peak. So drive mode keeps `manual_lean_rate_deg_s` (30°/s). A brake after a ride
  eases near the stop (at most
  2.5 /s times the reference) and ends at zero. The rider holds the stop while the brake stays,
  and a brake let go below 0.5 m/s also stops; the wheel then rocks within ±0.6 km/h while the
  pedals settle. Only a new back ask from rest reverses. In a
  firmware tiltback the rider eases off as in speed mode; there is no other hidden assist.
  The lean limit is its own, `manual_drive_lean_max_deg` (22°; `lean_max_deg` stays 15°).
  Measured on stock (2026-10-23, city world, articulated rider): the wheel follows every limit
  from 15° to 22° at low speed, at 0.83 to 0.84 of the ask (2.1 m/s² at 15°, 3.25 m/s² at 22°),
  and no limit made it fall below 50 km/h. 22°, the largest limit allowed, is therefore used;
  overlean stays possible and visible. Normal riding at half throttle (40 km/h and a stop) is fine
  at 100, 50 and 20 % charge. No top-speed result is recorded yet. Above about 55 km/h the motor
  plant saturates: its modulation reaches 100 % at about 68 km/h on a full pack, while the
  firmware's own PWM figure reads about 70 %. The owner's report has the real wheel at 78 km/h on
  an 80 % pack with headroom left, so the plant is being calibrated against the owner's ride log.
- **Foot state.** `Physics::feet()` reports each foot: contact, the pedal's normal load and friction,
  and how far the sole has moved on its deck since the model was built (slip) and above it (lift).
- **Leaving.** A step-off, a let-go and (contact feet) a lost footing release the pedal welds or
  switch the sole-pedal pairs off, so the feet pass the decks, and stop every joint motor and aid.
  A step-off adds
  the stepping foot's push: the right foot presses its pedal down for 0.2 s with the force that
  gives the rigid rider's 30 N·m roll kick, and the pedal pushes the foot up as hard (an internal
  pair, so the rider keeps its horizontal momentum). Without the push the riderless wheel rolled on
  upright, and neither step-off scenario was cut within 3 s; with it stock cuts 2.24 s after the
  15 km/h step-off and 1.50 s after the 5 km/h one (rigid rider: 2.75 s and 0.63 s). When both soles
  have been off their pedals for
  0.5 s with contact feet (longer than the 0.05 s a 2 cm pothole unloads them at 12 km/h), the rider
  counts as off:
  "rider off: both feet off the pedals (the wheel stays up)".

What no rider can do yet: balance sideways or twist without the external aids (the lateral
rider's later stages), step, hop, or use the hips to recover when the soles' centre of pressure
reaches heel or toe. Only the lateral rider's legs touch the shell: they rest and press on the
side pads (stages 2 and 3). The rigid rider also cannot bend its knees and hips.

**A hop with welded feet (feasibility, 2026-10-22; not built).** A hop would be the rider's own
motion: crouch, straighten the legs fast, then tuck so that the feet lift the wheel. Welded feet
can pull the wheel up; a real rider does it with the calves against the side pads, which only the
lateral rider has (this study rode the sagittal rider). A private test rode the Extreme's body on the plain PD balance, without firmware. At
the crouch rate (3 rad/s) the legs never threw the body up, and the wheel stayed down at every joint
cap. Straightening as fast as the joint caps allow, the tire rose 1.2 cm at today's 180 N·m per hip
and knee. It rose 2.4 cm at 300 N·m and 6.2 cm at 400 N·m, and in both the scripted tuck then
landed the rider off the wheel. A usable hop therefore needs four things. The first is a hop
controller: a full-torque extension, a timed tuck and a landing that the lean controller absorbs.
The second is a stronger, still estimated, jump torque. The third is a check of what the stock
firmware does with a wheel in the air. The fourth is a choice between the welds' pull and a
calf-pad contact. This is a study phase of its own, not a control setting.

**The absorb (2026-10-23; task 61, part 1).** The owner rides straight into curbs of about 15 cm on
soft knees that let the wheel come up under him. With legs that held their posture, a curb that
stopped the wheel in 30 ms turned the welded rider and the shell as one rigid body about the axle
(about 200 kg·m²/s of the body's momentum): the shell pitched nose-down at 150-300°/s, more than
the firmware's 210 N·m stops, and the rider fell. Most of that moment went through the ankles: the
lean task and the joints' damping held the body upright over feet that had stopped with the wheel.
A welded rider (`articulated_sagittal_v1` and `articulated_lateral_v1`) now absorbs. When the
axle's speed across the ground falls faster than `absorb_decel_mps2` (30 m/s², the speed and its
fall each averaged over 10 ms), the absorb starts in full, and it fades out over `absorb_time_s`
(0.3 s). In it the knees fold by up to `absorb_fold_rad` (0.6 rad, the hips by the crouch's share),
the hips and knees hold their posture at 0.15 of `joint_kp`, every leg joint keeps 0.2 of
`joint_kd`, and the ankles' lean task gives way. All are ESTIMATES. Two other triggers did not do:
a sole's load (the sagittal rider's soles take peaks of over three times their share in a full
manual turn) and the speed along the heading (it drops by a tenth for a few milliseconds whenever
the shell's heading jumps in a hard turn). Riding to 15 km/h, braking hard to a stop and a full
manual turn at 5 m/s do not start it; a 10 cm curb at 10 km/h does, and the knees fold by about 0.5
rad. `RiderState::absorb` reports it. The absorb alone does not carry a rider over a 15 cm curb
(stock firmware, the convex-mesh tire of bundle P1): the shell stays within −12° while the wheel
climbs, but the body goes on 0.5 m ahead, leaning 0.5 rad, and the recovery overshoots: the brace
(below) is needed too. In the shipped scenarios (stock firmware, the convex-mesh tire) it starts
once each in `pothole-5cm-20kmh`, `world-pothole-15kmh`, `world-curb-up-30deg`,
`world-curb-up-90deg` and `world-stairs-down-5kmh`, and never in `hard-braking-30kmh` or the
full-effort, overlean and step-off runs.

**The brace (2026-10-23; task 61, part 2).** A rider who sees a curb coming leans back a little and
bends the knees. The scripted articulated rider looks ahead every 10 ms: from the ground just in
front of the tire, at points at most 15 cm apart along the heading, out to `brace_lead_s` (0.55 s)
at the speed now (`Physics::ground_ray_m`, the ground only: the strip and its curbs, or the city).
While two neighbouring points rise by more than `brace_step_m` (7 cm), a step and not a slope (that
slope would be steeper than any road), it braces: it leans back by `brace_lean_deg` (5.7°) on top
of the lean its intent asks for, and its knees bend by `brace_crouch_rad` (0.3 rad, or the crouch
asked for if deeper). Once the tire meets the step the ground in front of it is the step's top, the
brace ends and the absorb takes over. 7 cm is a DEFAULT: the convex-mesh tire climbs 8 cm curbs at
every speed and angle without help and 10 cm only at walking pace (posefix's curb table); the lean,
the crouch and the lead are ESTIMATES from the K0 study. No brace below 1 m/s, for a step down, or
by hand: in Free Ride the player braces, or not, as a rider does. `RiderState::brace_lean_rad`
reports it. On the stock firmware with the convex-mesh tire, square on: with the absorb and the
brace, the sagittal rider crosses a 15 cm curb at 10, 12 and 15 km/h (the shell within −17°, −16°
and −21°), the lateral rider at 8, 10, 12 and 15 km/h (−14° to −24°); both cross 10 cm at 10 km/h
(−3°), and 20 cm still throws them. At 8 km/h the sagittal rider's wheel gets only 7 cm up the face
and rolls back: on this rigid mesh the wheel itself cannot climb 15 cm at 8 km/h with that rider.
On posefix's soft 25 psi tire (bundle P1) both riders cross 15 cm at 8, 10 and 15 km/h (−4° to
−15°), and 20 cm still throws them. In the shipped scenarios (stock firmware, the convex-mesh tire)
the rider braces only in `world-curb-up-30deg` and `world-curb-up-90deg`, and with the absorb it
now rides both (each fell without them); it does not brace up the 20 % climb of
`hill-climb-low-battery` or the plaza ramp of `world-ramp-up-7kmh`, which are slopes.

### The lateral rider (opt-in)

A rider's legs also move sideways: they lean the body into a turn, press a calf on a side pad
and twist the wheel at walking pace. `articulated_lateral_v1` is the articulated rider with those
joints. It is built in stages: stage 1 (2026-10-23) gave it its body and its posture, stage 2 the
knees' rotation and the legs on the side pads, stage 3 the turn's posture and an external roll aid
that leans wheel and rider as one; the external aids still steer. Later stages add the balance
made by steering and the twist at walking pace, then take the aids away above walking pace.
The model kept its id while it gained the knees' rotation in stage 2 (12 joints before, 14 now):
its recordings name the joints they had (`context.rider_joints`).

**Owner's report** (two photos of the owner riding the Extreme, straight and in a curve, and his
answers; not in the repository): he rides on the stock pad covers with knee and shin guards.
Riding straight his knees bend about 30° (25-40°: a low wide-angle camera) with the lower legs
about 15° forward, the shoe covers the whole deck, and the knee guard rests on the cover's upper
front corner by the front handle while the calf stays behind the cover's front block. In a curve
the outside knee presses the shell's front-top corner and the inside knee spreads away from the
wheel, both deeply bent; he puts more weight on the inside pedal and countersteers at speed. His
ankle sits about 3-6 cm behind the pedal's centre (the model keeps it over the centre; moving it
is a change in the pitch plane, planned for a later stage). He rides straight into curbs of about
15 cm without lifting the wheel, meeting them on soft knees that let the wheel come up under him
(a knee-absorb response is planned once the tire is rubber, with a 15 cm curb as its test).

- **Segments: de Leva (1996), adult men (ESTIMATES for an average man, not anyone's body).**
  Thigh 14.16 %, shank 4.33 % and foot 1.37 % of the mass, the upper body the other 60.28 %; the
  thigh 0.2425 and the shank 0.2493 of the height (43 and 44 cm at 1.78 m); each segment's centre
  of mass and radii of gyration from the same table. The upper body's centre of mass puts the whole
  rider's at `com_above_pedals_m` in the built, straight-legged stance.
- **Hips at their anatomical width.** The hip joint centres sit 9.0 cm either side of the midline
  at 1.78 m, scaled with the height (Harrington et al. 2007: 0.33 × pelvis width + 7.3 mm, with a
  250 mm pelvis width; ESTIMATE). Each foot stands `foot_offset_m` (DEFAULT 1.5 cm; the photos do
  not show it) outside its pedal's centre, so each leg leans in by about 6° from its pedal up to its
  hip, as a rider's legs stand on a wide wheel. (The other articulated riders' hips sit right above
  the pedals.)
- **Seven hinges per leg.** Hip flexion, hip ab/adduction (about the front-back axis), hip rotation
  (about the thigh), knee, the tibia's rotation about the shank (stage 2), ankle pitch and ankle
  roll (inversion and eversion, about the foot's long axis). Angles are anatomical: in the rest
  stance the hips are abducted and the ankles inverted by the legs' lean. Adduction, internal
  rotation and inversion are positive on both legs. The ranges are the AAOS normal ranges: hip
  flexion 120°, extension 30°, abduction 45°, adduction 30°, rotation 45° each way; knee 0-135°;
  ankle inversion 35°, eversion 15°. The tibia turns out 30° and in 20° (ESTIMATE: about 40° and
  30° at 90° of knee flexion, less on softly bent knees). The ankle pitch keeps the articulated
  rider's −52° to 40°, because a crouch needs more dorsiflexion than the AAOS 20° (weight-bearing
  dorsiflexion goes well past it). With six hinges a leg on its welded foot could not move by
  itself; the tibia's rotation gives each leg one motion of its own, the knee's swing out and in
  about the line from hip to ankle, which the legs need to stand round the pads.
- **Strength (ESTIMATES, per joint).** Hip flexion and extension and the knee 180 N·m, the ankle
  pitch 200 N·m (as the articulated rider). Hip ab/adduction `hip_adduction_max_nm` 150 N·m (about
  1.9 N·m/kg; professional footballers measure 2.6-3.0 N·m/kg eccentric). Hip rotation
  `hip_rotation_max_nm` 50 N·m (about 0.6 N·m/kg; the rotators are the weakest hip muscles). The
  tibia's rotation `knee_rotation_max_nm` 25 N·m. Ankle roll `ankle_roll_max_nm` 35 N·m
  (isokinetic, adult men: inversion 37 N·m, eversion 33 N·m).
- **Soft knees.** At rest the knees bend by `soft_knee_rad` (0.52 rad, 30°: the owner's report;
  riding guides say slightly bent, never locked; stage 1 used 20°) and the hips by 0.60 of it, so
  the centre of mass stays over the ankles. Under the rider's weight the knees hold 31.7° and the
  centre of mass sits 3.0 cm lower than the straight-legged sagittal rider's. A crouch bends on
  from there:
  `set_crouch(0)` is the soft-knee stance and `set_crouch(1.2)` the same full 1.2 rad of knee
  flexion as before. With de Leva's segments the hips need 0.60 of the knee bend (the articulated
  rider's 0.58 left the centre of mass 2.2° ahead in a full crouch); at 0.60 it stays within 0.4° of
  the ankles from the soft knees to a full crouch.
- **The lateral joints hold the settled stance.** Once the model has settled, the hips'
  ab/adduction and rotation and the tibia's rotation hold what they then held (so their posture
  springs carry no load at rest), moved by the knee's swing: the hip turns out, the tibia turns in
  as far, and the hip abducts 0.17 rad per radian of swing (measured on the model). A standing
  ankle's roll, like its pitch, has no posture spring: the legs' A shape sets it.
- **The legs on the side pads** (stage 2; a wheel with measured pads). The pads are the cover's
  measured contact faces (`geometry.toml` `[pad_covers]`), as boxes from the packs' outer face out
  to each face, the right cover 1.0 mm further out. A leg touches the shell with its knee (a 5 cm
  ball at the joint), its shin with the guard (a 3.5 cm capsule on the front of the upper shank)
  and its calf (a 5 cm capsule behind the shin bone over the middle of the shank); all ESTIMATES
  for an adult man with guards. The knee and the shin reach the pads on their side and the front
  and top solids; the calf reaches the pads but not the cover's front block (a face that starts
  more than 5 cm ahead of the pedal's centre), behind which the owner's calf stays. Without that
  rule the calf met the block's rear face as the rider leaned to accelerate, and the wheel took
  10 s to reach 1.9 m/s instead of 4.2. Contact: friction `pad_friction` 0.3 (hard guards and
  trousers on plastic), softness `pad_contact_time_s` 0.03 s, soft at first touch and firm 2 cm
  in (DEFAULTS). The pairs are off while the model settles: the knees bend to the stance and swing
  out until both legs stand 2 mm clear of the shell, or swing as far as they go (0.45 rad, the
  tibia's range) and press in less than 1 cm. Then the pairs come on, and a slow stance loop swings
  each knee out when its leg presses harder than `pad_preload_n` (DEFAULT 20 N) and in when it
  presses less. At rest each calf rests on its pad's upper arm with 20 N, the knees swung out
  0.05-0.06 rad. The pads' depth is INFERRED to ±20 % (about ±1 cm), and the rest stance carries
  that uncertainty. `Physics::leg_pads()` reports
  each leg's load, its gap to the shell and its swing. The pairs go off when the rider leaves.
- **What stage 2 measured** (test balance law, the Extreme's profile with its suspension). Riding
  straight the pads do not hold the rider back: 4.226 m/s after 10 s towards 15 km/h (sagittal
  4.234 m/s), the legs' load at most 56 N. In a 0.4 rad/s turn at 15 km/h the inside leg takes
  280 N for a moment as the wheel rolls in, then both legs carry about 21 N; the aids did 33.8 J
  of roll work and −2.8 J of yaw work in the 4 s (without pads: 32.9 J and −2.8 J). A full manual
  turn at 5 m/s (35° at 60°/s) is beyond stage 2: as the external aid rolls the wheel in, the
  inside pad pushes the inside leg with up to 1.7 kN, and the rider can come off. Stage 3 makes the
  turn with the legs (the inside leg spreads, the body leans in, the outside knee presses), and Free
  Ride stays on the sagittal rider until stage 4. On the stock firmware `ride-20kmh-stop` passes at
  2.14 times real time (sagittal 2.57 in the same session) and `world-ramp-up-7kmh` at 1.85 (1.93).
- **Honest welded soles.** Both feet are welded to the pedals. A real sole rolls its pedal only as
  hard as its centre of pressure can move to the edge of the foot, so each ankle's roll motor is held
  to the sole's load times half its width (4.5 cm), within its strength. The load is the weld's
  constraint force along the deck's normal, averaged over 20 ms, which keeps a step's or a bump's
  load and drops the fast part. (Until 2026-10 the tire was a ring of 48 capsules that rippled the
  wheel at 48 times its turn rate, 144 Hz at 15 km/h: on the suspension-less test wheel each weld's
  load swung by up to ±250 N (sagittal rider) and ±40 N (lateral rider) about half the rider's
  weight, on the Extreme's profile with its suspension by about ±17 N. Stage 1 took the swing for a
  ringing of the welds. On the convex-mesh tire a weld's load varies by 0.6 N RMS with the
  suspension and 41 N without one, most of it the mesh's ripple near 1 kHz.)
  `Physics::sole_loads()` reports each sole's load
  (negative: the foot pulls its pedal, which only a weld can), its ankle roll moment and the limit
  it was held to; `Physics::sole_pulls()` says whether a foot pulls. Riding at 15 km/h the soles
  carry the rider's weight within 1 % and no foot pulls.
- **The roll aid leans wheel and rider as one** (stage 3). The external aids, not the legs, still
  steer the wheel. Into a full manual turn the roll aid rolls the wheel with over 1000 N·m, more
  than legs carry. On the shell alone it pulled the shell away from the body, and the legs had to
  carry it: in stage 1 the hip ab/adductors saturated at 150 N·m and the wheel spun round, so the
  same aids also held the upper body to the shell's roll and heading (external torques on the
  upper body). In stage 2 a full manual turn still pressed the inside leg with 1.7 kN. Since stage
  3 the roll aid acts on the whole machine: the shell's roll spring and the upper body's (the same
  law, 5000 N·m/rad and 500 N·m·s/rad each, so 10000 and 1000 together, about the wheel's roll and
  roll rate) give a torque about the heading through the tire's contact, and each body of wheel
  and rider takes the force and moment of its own part of one rigid lean about that line (its
  mass times its acceleration there, and its inertia times its turn). The aid then loads no
  joint, pad or pedal: in free flight without gravity, with the legs holding their stance, 2000 N·m
  leans the machine 0.07 rad in 0.1 s and moves no joint by more than 0.001 rad beyond what the
  stance alone moves it; the same torque on the shell alone rolls the shell 0.8 rad and bends a
  joint by 2 rad. The upper body's yaw aid is gone: the hips turn the body with the shell, within
  their strength. The torque and work of the lean add to the reported roll aid, as before.
  (Every external wrench is set again each step: MuJoCo keeps an applied wrench until it is
  set again, and a wrench on a leg would otherwise stay on it.)
- **The turn's posture** (stage 3; the owner's report). In a turn the knees bend deeper, by
  `turn_crouch` (0.5) radians per radian of the turn's lean: from 32° to 52° at 35° of lean. The
  inside knee swings out from its pad by `turn_spread` (1.3) radians per radian of lean, as far as
  the knee swings (0.45 rad, from about 20° of lean on), and the outside leg rests on its pad
  harder: the stance loop aims at `pad_preload_n` plus `turn_press_n_per_rad` (100 N) per radian
  of lean. All three are ESTIMATES: the owner's report says what the legs do, not how far or how
  hard. At about 20° of lean the inside knee stands 8.4 cm off the shell, where the owner's photo
  of a turn at roughly 15-20° shows about 10 cm. The inside sole then carries more of the rider, as
  the owner's does: the body leans a little further than the wheel.
- **Where the outside leg rests: a known difference.** In the owner's report the outside knee guard
  rests on the shell's front-top corner by the front handle, riding straight and in a turn. The
  model's knee comes within 2-3 cm of that corner (in a full manual turn 2.0 cm off the side of the
  shell's front block, riding straight 2.3 cm above the side of its top), and its calf rests on the
  cover's upper arm instead and takes the press. One centimetre at the calf decides it: with a
  4 cm calf instead of 5 cm the knee comes within 0.2 cm of the corner, and with 3 cm the knee
  takes the press. The calf's size (ESTIMATE) and the pads' depth (INFERRED to ±1 cm) both cover
  that centimetre, so the model keeps them and records the difference. The stage that moves the
  ankle 3-6 cm behind the pedal's centre revisits it (untested: that moves the shank too), best
  with a measured depth of the cover's upper arm and a measured width of a guarded calf.
- **What stage 3 measured** (test balance law, the Extreme's profile with its suspension). In a
  0.4 rad/s turn at 15 km/h (10° of lean) the aids did 18.3 J of roll work and −3.3 J of yaw work
  in 4 s (stage 2: 33.8 J and −2.8 J; the sagittal rider 23.3 J and −3.1 J). The outside leg
  takes 61 N for a moment as the wheel rolls in (stage 2: 280 N), then rests on its pad with
  40 N (aim 37 N); the inside leg stands 1.6 cm off its pad; the soles carry 407 N inside and
  386 N outside. A full manual turn at 5 m/s (35° at 60°/s), where stage 2 could lose the rider,
  holds within 0.4° of the turn's lean. Its aids did 222 J of roll work and −15.8 J of yaw work
  in 4 s (the sagittal rider 190 J and −25.7 J; stage 2 275-295 J and −197 to −237 J before the
  rider came off). The knees bend from 32° to 52°, the inside leg stands 4.2 cm off its pad, the
  outside calf rests on the cover's upper arm with about 100 N, and the soles carry 544 N inside
  and 380 N outside. Into the turn the heading turns at once and the wheel tips out by 0.2° for
  0.1 s before the aid leans it in: the aids need no countersteer, where the owner countersteers
  at speed. Stage 4 replaces the roll aid with balance by steering and tests for the
  countersteer. On the stock firmware `ride-20kmh-stop` passes at 2.42 times real time (sagittal
  2.57 in the same session) and `world-ramp-up-7kmh` at 1.86 (1.94); a slalom at 12 km/h on the
  city world (seed 42) stays upright at 1.78 (1.87).
- **It rides as the sagittal rider.** On the test balance law, without pads, it reaches 4.22 m/s in
  10 s towards 15 km/h and stops in 11.50 m (sagittal: 4.22 m/s, 11.45 m); a full manual turn at
  5 m/s settles at 34° of lean and 1.12 rad/s (sagittal: 34°, 1.13 rad/s); it rides 30 km/h
  straight with no roll. Stage 1 passed `ride-20kmh-stop` on the stock firmware at 2.45 times real
  time (sagittal 2.51 in the same session).
- **Tests.** `cargo test -p euc-physics --test lateral` (the body, its joints and soft knees) and
  `cargo test -p euc-sim --test lateral_pads` (the legs on the Extreme's pads, its turns); the curb
  answer of both articulated riders in `--test absorb` and `--test brace` (euc-physics).

## Aids (M2)

- **Roll spring.** Neither rider can balance sideways (a real rider steers and twists), so a
  torque about the heading holds the shell upright: `τ = −k·roll − c·roll_rate`, k = 5000 N·m/rad,
  c = 500 N·m·s/rad, far stiffer than the toppling moment of wheel and rider (about 1200 N·m/rad).
  It shows in recordings as a roll that stays at zero. Set `[assist] roll_stiffness = 0` in a
  scenario to see the wheel fall over sideways. Since 2026-10-23 the spring's torque leans wheel and
  rider as one about the heading through the tire's contact for the rigid and welded riders too, as
  for the lateral rider (each body takes its part of one rigid lean; contact feet keep a torque on
  the shell). As a bare torque on the shell it also yawed the machine when the rider's mass lay far
  behind the contact. In the hard brake of `full-effort-70kmh-stop-soc100` in the city (rider 31°
  back, a small roll from the height field) that yaw made the heading loop ask for more lean: the
  roll grew from 0.5° at 22.0 s to 9.7° at 23.5 s and the wheel fell at 15 km/h. Leaned as one, the
  roll stays under 0.5° (rigid and sagittal riders). On the flat plane the roll stays at zero either
  way. Leaned as one the aid turns the whole machine about the contact, so its gains are doubled
  (10000 N·m/rad and 1000 N·m·s/rad, as the lateral rider's two springs together): with the single
  gains the rigid rider's full manual turn at 3 m/s rolled in too slowly, overleaned and the
  firmware cut out. Each body also takes the part of its own turning moment that lies off the heading: a body
  tilted in the lean (a rider leaning back) needs it. With the part along the heading only, 2000 N·m
  on a rigid rider leaning 29° back, in free flight without gravity, still yawed the machine at
  1.5 rad/s after 0.1 s; with the whole moment at 0.003 rad/s (a bare torque on the shell: 17 rad/s).
- **Upper-body roll aid** (`articulated_contact_v1` and `articulated_lateral_v1`). The same law,
  with the same gains, holds the articulated upper body. For contact feet it holds the body at the
  wheel's roll target, because contact feet do not hold the body to the shell and the legs cannot
  roll: an external torque on the upper body, like the shell's roll spring and the yaw aid of the
  steering (point 4 above). For the lateral rider it adds to the shell's spring, and the two lean
  wheel and rider together as one machine (see the lateral rider), because its legs hold its body
  but are only as strong as a person's. Its torque and work add to the reported roll aid.
- **Recorded.** A recording's context (`model_aids`) carries every aid the run had: the shell's roll
  stiffness and damping, the yaw aid's gains and cap, and for `articulated_contact_v1` and
  `articulated_lateral_v1` `upper_body_roll_stiffness` and `upper_body_roll_damping` (absent for
  the models without them). Recordings of `articulated_lateral_v1` from before stage 3 also carry
  `upper_body_steer_kp`, `upper_body_steer_kd` and `upper_body_steer_max_nm`: the yaw aid then held
  its upper body at the shell's heading. For `articulated_lateral_v1` the context also names the
  rider's leg joints (`rider_joints`), because the model gained a joint under the same id.
- **Air drag** acts on the rider's centre of mass: `F = ½·ρ·CdA·v·|v|`, ρ = 1.2 kg/m³,
  CdA = 0.6 m² (sim.py's value, an upright adult).

## Parameters

Wheel (`[body]` in `plant.toml`, Begode Extreme values):

| Parameter | Value | Source |
|---|---|---|
| `tire.radius_m` | 0.2251 m | ESTIMATE: loaded rolling radius, 4.5 mm under the scanned 3.00-12 tire's free radius of 0.2296 m. The firmware's 0.0505 m per electrical turn with 31 pole pairs then shows 1.107 × the true speed, as the owner's ride log does (electrical.md) |
| `tire.half_width_m` | 0.0407 m | MEASURED: 81.4 mm wide (used only without a torus) |
| `geometry.toml` `tire_torus` | tube 0.058 m at 0.1716 m | MEASURED: crown lateral radius; 0.2296 m outside, the free tire |
| `tire.friction` | 1.1 | ESTIMATE: owner's report, dry asphalt about 1.0-1.2 |
| `tire.crr` | 0.014 | INFERRED: owner's ride log 2026-10-17 (light rider), fit with a constant power term, ±0.01 |
| `tire.contact_time_s` | 0.022 s | INFERRED: 25 psi (owner's report), about 125 kN/m with the pairs' impedance 0.5: 9.5 mm under the loaded wheel |
| `wheel.mass_kg` | 16 kg | DEFAULT: rotor, rim and tire |
| `wheel.spin_inertia_kgm2` | 0.487 kg·m² | DEFAULT: 0.6·m·r² |
| `shell.mass_kg` | 20 kg | DEFAULT: 42 kg with the wheel and the carriage |
| `shell.com_above_axle_m` | 0.13 m | DEFAULT: 0.10 m for a 26 kg shell with the stator, the stator taken out |
| `shell.inertia_kgm2` | 0.76, 0.82, 0.45 kg·m² | DEFAULT: a 0.35 × 0.30 × 0.55 m box (0.85, 0.92, 0.46 with the stator), less the stator's own and offset terms |
| `shell.pedal_below_axle_m` | −0.012 m | MEASURED (approx.): deck tops 12 mm above the axle at full extension: the scanned pins at +7.5 mm plus half a 9 mm deck; the owner measured about 240 mm from the floor to the deck top (unloaded), 10.4 mm above the 229.6 mm free radius, within the ±5 mm of the deck estimate |
| `shell.imu_above_axle_m` | 0.15 m | DEFAULT: research sim.py; on the frame (sprung) |
| `suspension.unsprung_mass_kg` | 6 kg | ESTIMATE: stator, axle clamps, slider blocks (the wheel's 16 kg moves too) |
| `suspension.axis_forward_deg` | −0.13° | MEASURED: the two 30 mm slider rods are vertical within 0.15°, top to the rear (scan) |
| `suspension.spring_rate_n_per_mm` | 131.3 N/mm | MEASURED: "DNM - 750 LBS/IN" printed on the owner's spring |
| `suspension.preload_mm` | 2 mm | ESTIMATE |
| `suspension.stroke_mm` | 55 mm | ESTIMATE: 200 mm DNM coil-overs come with 50-57 mm of stroke |
| `suspension.eye_to_eye_mm` | 200 mm | MEASURED (owner; drawing only); the scan's older shock was 204 mm |
| `suspension.linkage_ratio` | 1.7 | ESTIMATE from the owner's sag (1.6-1.9 with 0-2 mm preload); used while `ratio_curve` is empty |
| `suspension.ratio_curve` | none | the rocker's leverage along the travel; if the rocker turns (reading (a), `docs/hardware/suspension-linkage.md`) 2.3 at rest falling to 1.7 at 93.5 mm (INFERRED); if it is fixed (b) a constant near 1.5-1.7; a seat-press video decides |
| `suspension.damping_n_s_per_m` | 2200 N·s/m | ESTIMATE: damping ratio about 0.18 for the sprung 100 kg |

With these values an 80 kg rider sags the suspension 18 mm (19 % of the 93.5 mm axle travel), and
the sprung mass bounces at about 3.4 Hz with a damping ratio of about 0.18.

Rider and aids (scenario `[rider]`, `[rider.control]`, `[assist]`; these are the defaults):

| Parameter | Value | Note |
|---|---|---|
| `model` | `articulated_sagittal_v1` | `rigid_ankle_v1`: the controlled baseline; `articulated_contact_v1`: contact feet (opt-in); `articulated_lateral_v1`: lateral legs (opt-in) |
| `mass_kg` | 80 kg | |
| `com_above_pedals_m` | 1.0 m | about 55 % of a 1.78 m adult's height |
| `ankle_above_pedals_m` | 0.08 m | the hinge |
| `gyration_m` | 0.35 m | pitch and roll; 9.8 kg·m² about the centre of mass (rigid); the articulated upper body uses it for its own inertia |
| `height_m` | 1.78 m | drawing; the articulated segment lengths scale with it |
| `articulation.joint_kp`, `joint_kd`, `joint_max_nm` | 1000 N·m/rad, 60 N·m·s/rad, 180 N·m | ESTIMATE: hip and knee posture (and a lifted foot's ankle), per joint |
| `articulation.sole_friction`, `sole_contact_time_s` | 1.0, 0.005 s | DEFAULT: rubber on grip pins; shoe and pedal give (`articulated_contact_v1`) |
| `articulation.soft_knee_rad` | 0.52 rad | the owner's report: the knees' bend at rest, 30° (`articulated_lateral_v1`) |
| `articulation.foot_offset_m` | 0.015 m | DEFAULT: each foot this far outside its pedal's centre (`articulated_lateral_v1`) |
| `articulation.hip_adduction_max_nm`, `hip_rotation_max_nm`, `knee_rotation_max_nm`, `ankle_roll_max_nm` | 150, 50, 25, 35 N·m | ESTIMATE: per joint, from published strength of adults (`articulated_lateral_v1`; see the lateral rider) |
| `articulation.pad_friction`, `pad_contact_time_s`, `pad_preload_n` | 0.3, 0.03 s, 20 N | DEFAULT: a leg on a side pad, and how hard it rests there (`articulated_lateral_v1` on a wheel with measured pads) |
| `articulation.absorb_decel_mps2`, `absorb_time_s`, `absorb_fold_rad` | 30 m/s², 0.3 s, 0.6 rad | ESTIMATE: the absorb's trigger (the axle's fall in speed over 10 ms; `inf` turns it off), how long it lasts and how far the knees fold (`articulated_sagittal_v1` and `articulated_lateral_v1`) |
| `articulation.turn_crouch`, `turn_spread`, `turn_press_n_per_rad` | 0.5, 1.3, 100 N | ESTIMATE from the owner's report: per radian of a turn's lean, the knees' deeper bend (rad), the inside knee's spread from its pad (rad) and the outside leg's harder rest on its pad (`articulated_lateral_v1`) |
| `cda_m2` | 0.6 m² | air drag |
| `control.lean_kp`, `lean_kd` | 6000 N·m/rad, 900 N·m·s/rad | about 1.3 Hz, damping 0.6 about the ankles |
| `control.ankle_max_nm` | 400 N·m | both legs |
| `control.speed_kp`, `speed_ki` | 0.6 /s, 0.03 /s² | |
| `control.accel_max`, `brake_max` | 1.5, 2.5 m/s² | |
| `control.lean_max_deg`, `lean_rate_deg_s` | 15°, 12°/s | |
| `control.tiltback_feel_deg`, `tiltback_ease_kmh_per_deg`, `tiltback_ease_brake` | 2°, 1.5 km/h per degree, 1 m/s² | DEFAULT: how a rider reacts to tiltback |
| `control.steer_kp`, `steer_kd`, `steer_max_nm` | 2000 N·m/rad, 200 N·m·s/rad, 60 N·m | DEFAULT: how a rider keeps the line |
| `control.manual_turn_rate_max_rad_s`, `manual_turn_lean_max_deg`, `manual_turn_lean_rate_deg_s`, `manual_lean_rate_deg_s` | 2.5 rad/s, 35°, 60°/s, 30°/s | DEFAULT, riding by hand only (Free Ride, Ride's manual mode): the turn cap at a stop, falling with speed to `g·tan(35°)/v`; scripted riding keeps 1.0 rad/s, 20°, 30°/s and 12°/s |
| `control.brace_lead_s`, `brace_step_m`, `brace_lean_deg`, `brace_crouch_rad` | 0.55 s, 0.07 m, 5.7°, 0.3 rad | Scripted riding, articulated riders: how far ahead the rider looks for a step up (0: no brace), the smallest step it braces for (DEFAULT: the mesh tire climbs 8 cm alone), and how far it leans back and bends the knees then (ESTIMATES) |
| `control.manual_drive_lean_max_deg`, `manual_drive_kp` | 22°, 1.2 /s | DEFAULT, manual drive only (`Intent::Accel`): the drive lean limit (stock follows up to 22° at low speed) and the reference-speed gain (tuned on stock at 40 km/h: holds speed, stops cleanly) |
| `let_go.enabled`, `after_cut_deg`, `fall_deg`, `reaction_s` | true, 20°, 45°, 0.1 s | DEFAULT: when a rider leaves a wheel that goes down |
| `assist.roll_stiffness`, `roll_damping` | 5000 N·m/rad, 500 N·m·s/rad | on the shell; for `articulated_contact_v1` also on the upper body; for `articulated_lateral_v1` the shell's and the upper body's springs lean wheel and rider as one |

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
| `rider_on_pedals` | the rider never comes off the pedals while the wheel stays up; every scenario without `step_off_s` has it by default; only `articulated_contact_v1` can fail it |

`--record FILE` writes a recording the studio replays: a JSON header (wheel, firmware, scenario,
bodies with their shapes, channel schema), the telemetry batches of `euc-telemetry`, and the events
(see `crates/euc-sim/src/recording.rs` for the layout).

## To measure on a real wheel

- Rolling radius: mark the tire, roll one turn with a rider on, measure.
- Masses: weigh the wheel; weigh the shell side with the motor out if it is ever apart.
- IMU position and mounting signs: open the shell; check the signs with the wheel tilted.
- Shell inertia: swing the wheel as a pendulum from the handle.
- Rolling resistance: coast-down from 10 km/h with the motor off (free-wheel), log the speed.
- Tire stiffness: the axle's height above the floor at 25 psi with and without the rider, the
  suspension's own sag taken out.
- Suspension: the shock's make, spring rate (stamped on the coil), stroke and preload; the axle
  travel from top-out to bottom-out (zip tie on a rail); the sag with the rider on (pedal height
  above the floor with and without the rider).

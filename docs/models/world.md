# Procedural world (W1)

Status: W2, 2026-10-23. The world is integrated: MuJoCo ground in `euc-physics`, `[world]` scenarios, recordings and replay, the studio's 3D view in daylight (FR4), Free Ride's ground, and since W2 the ground of every shipped scenario. Since then: the rider's window, the ground ray and 1/16 m cells (#13).
W1a (2026-10-22) built the design, the `euc-world` generator (city biome), a PNG preview and a MuJoCo feasibility spike; W1b (the same day) integrated it.
The finite 32 m map of [generated-map-implementation.md](generated-map-implementation.md) is no longer offered in Free Ride (play mode); old test-pad recordings still replay.

The owner asked for a more realistic map with roadside objects, generated progressively like Minecraft, with roads, potholes and stairs.
This document is the design for that world, what W1a and W1b built, and what the spike and the acceptance runs measured.

## Goals

- An infinite world, generated on demand in chunks around the rider.
- One seed gives the same world on every machine, in any generation order.
- A resolution-independent description: physics samples it finely near the wheel, rendering coarsely far away.
- Real city ground: curbs up and down, curb cuts, crosswalks, speed bumps, manholes, cracks, potholes, rough and wet patches, stairs with a ramp, and surfaces with their own grip and rolling resistance.
- Recordings store the seed and generator revision, never meshes.

## World model

### Chunks

A chunk is 16 m × 16 m. Chunk `(cx, cy)` covers `[16 cx, 16 cx + 16] × [16 cy, 16 cy + 16]`.

Why 16 m:

- The physics window (12 m, below) overlaps at most 2 × 2 chunks, so a recentre touches at most four chunks.
- Every local feature fits inside one chunk plus a neighbour: potholes (≤ 1.5 m), crosswalks (3 m), tree pits, curb cuts (≤ 6 m with flares).
- Generation stays small and parallel: 0.94 ms per chunk with the tile cache (measured, below).
- A chunk is a power of two in metres, so the physics lattice (1/16 m) divides it exactly (256 cells).

`Chunk::generate(spec, key)` depends only on the `WorldSpec` (seed, generator revision, biome, densities, incline) and the key.
Two chunks give bit-identical heights and surfaces on their shared border (tested).

### Determinism rules

- Every random decision hashes integer coordinates with the seed and a per-layer salt (`det::hash`, a `SplitMix64` finalizer chain). There is no random state carried between items.
- A feature is placed by its own grid cell or street edge. Conflicts between neighbours are thinned by priority: a candidate is dropped when an overlapping neighbour candidate has a higher priority. Neighbours are judged by the same pure rule, so the result does not depend on which chunk asks first.
- Height contributions are summed in a canonical order (feature kind, then identity), so floating-point sums are identical in every chunk.
- Only IEEE-754 `+ − × ÷`, `sqrt`, `floor`, `abs`, `min`, `max` and integer operations appear in generation and sampling. Rust neither fuses them into FMAs nor reorders them, and they are correctly rounded on every shipped target. `sin`, `cos`, `exp`, `powf`, `hypot` and `atan2` call the platform's libm, which differs in the last bit between macOS, Linux and Windows, so `euc-world` never calls them. Curves are Catmull-Rom splines, profiles are polynomials, noise is value noise with a quintic fade.
- A golden test hashes the chunks around the spawn for a fixed seed. Any change to generated output fails it; the fix is to bump `GENERATOR_REVISION` and record the new value.

### Layers

1. **Terrain relief.** Two octaves of value noise (256 m and 80 m wavelengths, 3.5 m and 0.6 m amplitudes). Per-axis slope is bounded by 5.1 % and 2.8 %; the measured maximum over 5 seeds × 1.6 km² is under 12 % (test `hills_stay_gentle`). The relief is level within 12 m of the spawn and blends back in by 72 m.
2. **Road network.** Intersections sit on a jittered 64 m lattice. Every fourth row and column is an arterial (14 m, four lanes, double centre line, always present). Each 4 × 4 superblock keeps all of its local edges along one axis and 60 % along the other, so every node reaches an arterial along its row or column: the network is connected (test `the_road_network_is_connected`). Local streets are 7–10 m wide, with midpoint bulges of up to 6 m, so streets curve; missing edges leave dead ends and bends. Centrelines are Catmull-Rom splines through nodes and bulged midpoints, flattened to 32 segments per edge, and they join with matching tangents. Street paving is per edge: arterials asphalt; local streets 73 % asphalt, 12 % concrete slabs, 15 % cobblestones.
3. **Road field.** `f(p)` is the signed distance to the curb face line, negative on the carriageway: a hard minimum over the row edges and over the column edges, then a polynomial smooth minimum of the two (blend 4 m). The smooth minimum rounds block corners; it is exactly the plain minimum away from intersections. Every zone and profile reads `f`.
4. **Street profile.** Carriageway: a 2 % crown that rises over 3.5 m from the gutter, so wide streets have a level middle. Gutter pan: 0.3 m of concrete. Curb face: 10–16 cm (a slow noise field along the street). Sidewalk: 2.5–4 m wide, rising 1.5 % away from the curb, concrete with expansion joints every 1.5 m or pavers. Beyond it, the lot zone (grass, dirt, gravel) at the sidewalk's outer height.
5. **Street items** (per edge, at fixed arc-length slots): crosswalks at intersection ends (zebra paint, with a curb cut on both sides), speed bumps or humps mid-block on local streets (6–9 cm), driveways (curb cuts with flares, 3–4.5 m), tree pits with trees, lamp posts, benches, bins, hydrants and bollards.
6. **Grid features** (global jittered grids, one candidate per cell): potholes (4 m cells; 2–12 cm deep, 0.2–1.5 m wide, irregular outlines from 2–4 blended discs, steep walls 4 cm wide, a rough gravel bottom, 30 % hold water), manholes (16 m; 0.6–0.76 m covers, 1–3 cm sunk or 0.8–2 cm raised, 2 cm rim), cracks (3 m; 1–2 cm wide V grooves, 0.5–1.5 cm deep, 3–6 segment random walks), rough patches (12 m; worn asphalt with 3–8 mm roughness) and wet patches (10 m).
7. **Parcels** (20 m jittered grid, resolved per 80 m tile): buildings (setback 0.5–2.5 m, 6–40 m tall), parks (grass, a gravel path, trees), parking lots (painted bays and a driveway) and raised plazas. A parcel faces its nearest street, fronts on the sidewalk's outer edge, shrinks to fit a tight block (scales 1, 0.75, 0.55), and must lie wholly beyond the sidewalks. Exact rectangle overlap (separating axes) with 1 m clearance decides conflicts; plazas win, then buildings, parking, parks.
8. **Plazas: stairs between levels.** A level platform 3–12 risers above the ground at the foot of its stairs (risers 15–18 cm, treads 28–32 cm). The stairs (2–4 m wide) climb from the front. A ramp (1.5 m wide, 8.3–12 %, at most 12.5 %, at most 18 m long) runs along the platform's front edge beside the stairs, from the ground up to the platform. The step count rises until the platform clears the ground everywhere, so every wall is an up face.
9. **Textures in the height:** cobblestone setts (14 cm domes, about 8 mm peak to peak), paver joints (2 mm), concrete joints (6–8 mm grooves).

### Sampler

All on `World` (thread-safe; it caches chunks and parcel tiles) and on `Chunk`:

| Call | Returns | Used by |
| --- | --- | --- |
| `height_at(x, y)` | ground height, faces where they are | rendering, preview |
| `ground_at(x, y, inset)` | height with every face that has a solid moved `inset` behind its solid's face | physics height field |
| `fill_heights(origin, cell, nx, ny, inset, out)` | a lattice of `ground_at`, chunk by chunk | physics window |
| `sample(x, y)` / `surface_at(x, y)` | height, `Zone`, `Surface` (material, wet, friction, rolling resistance) | physics grip, rendering |
| `solids_in(rect)` | oriented boxes (curbs, steps, platforms, ramps, buildings, props), each once | physics box pool, rendering faces |
| `props_in(rect)` | props with ground z | rendering |
| `terrain_at(x, y)` | bare relief | distant scenery |
| `node(i, j)`, `edge(id)` | the road graph | routes, autopilot |
| `spawn()` | origin, heading +x | Free Ride start |

Solids are oriented boxes (`centre`, `half`, unit `axes` rows, right-handed).
A curb box is 15 cm deep from the face into the sidewalk, its top follows the curb top, and it reaches 15 cm below the road.
Curb boxes are traced from the zero contour of `f` by marching squares on a 0.25 m grid, simplified to 1 cm, cut into pieces of at most 2 m (shorter in curb-cut flares), and clipped to the chunk.
Every face of at least 4 cm gets a box; lower faces (manhole rims, tree pits, curb-cut lips, pothole walls) stay in the height field.

### Surfaces

Generic estimates for a pneumatic 3-inch tire from the usual published ranges. None is measured on a wheel; they set the order of the surfaces, not a validated grip model.

| Material | μ dry | μ wet | rolling resistance |
| --- | --- | --- | --- |
| asphalt | 0.90 | 0.60 | 0.010 |
| worn asphalt | 0.80 | 0.50 | 0.012 |
| concrete | 0.85 | 0.60 | 0.009 |
| paving stones | 0.75 | 0.50 | 0.012 |
| cobblestones | 0.70 | 0.40 | 0.018 |
| gravel | 0.55 | 0.50 | 0.035 |
| grass | 0.45 | 0.30 | 0.060 |
| dirt | 0.60 | 0.40 | 0.030 |
| paint | 0.65 | 0.35 | 0.010 |
| metal (manholes) | 0.55 | 0.25 | 0.009 |

Wet ground rolls 10 % harder.

### Spawn

The spawn is the origin, heading +x, 1.5 m right of the centreline of a straight arterial (row 0 is straight from x = −64 to 64 m).
Guarantees (test `the_spawn_is_level_clear_and_on_a_straight_road`, 12 seeds): asphalt road; level within ±1 cm over a 2 m radius; no feature within 10 m (crosswalks and curb cuts measured by their painted band and lowered curb); no prop within 10 m; no solid within 4.5 m (the curb is 5.5 m to the right); a smooth lane 15 m ahead.

### Densities

`Densities` sets every rate: potholes 1.5, manholes 0.6, cracks 12, rough patches 0.8 and wet patches 1.0 per 1000 m² of their zone; 35 % of local edges with a bump; 70 % of intersection ends with a crosswalk; 1.5 driveways per 100 m of street side; plaza weight 0.15 among parcels; props 0.8 of their slots.
Realised pothole density lands at 50–110 % of nominal (sites near the gutter and the spawn are refused) and doubles when the parameter doubles (test `densities_follow_their_parameters`).

`hills` (W2) scales the terrain relief: 1 by default, 0 a level city, up to 2. At 1 the products are exact, so a default world is bit-identical to one without the field (golden digest unchanged); a spec that leaves it out reads 1. The street network, features and parcels do not depend on it, only the heights (test `hills_zero_levels_the_terrain_and_keeps_the_streets`).

### Incline

`WorldSpec::incline` (W2, optional) grades the whole world along one direction for a slope test: from `start_m` past `origin` along the unit `direction`, the terrain rises by `slope_pct` per 100 m, built up over `ramp_m` with a parabola (the profile of a scenario's `[terrain]` slope). Behind the start the terrain is unchanged. Streets, curbs, parcels, plazas and features follow it: on a 20 % grade buildings stand into the slope, never above it, and curbs keep their tops on the sidewalk edge (test `an_incline_grades_the_whole_world_along_its_direction`). A scenario gives the direction by its start heading; at whole quarter turns the unit vector is exact, so the generator sees the same bits on every platform.

## Physics (built in W1b)

`crates/euc-physics/src/world.rs`. `Setup::world` takes a `WorldGround` (a shared `World` and a `WorldStart`); it excludes `height_map` and a non-flat `[terrain]`.
The physics frame is the world moved so that the start lies at the origin, heading +x: `world = start + R(heading) · physics`, `z_world = z_physics + origin_z`, where `origin_z` is the world height under the start.

### Height-field window

- One MuJoCo `hfield` of 193 × 193 samples at 1/16 m (6.25 cm): a 12 m square, elevation range 8 m, base 0.5 m. W1b built it at 1/32 m (385 × 385); [Cell size](#cell-size-13) says why it changed.
- Its geom sits in a **mocap body**. Recentring writes `mocap_pos` and the hfield data. A geom in the world body does not move when `geom_pos` changes at run time in MuJoCo 3.9 (spike A1, B): the wheel falls through.
- The window recentres when the shell is more than 1 m from its centre (per axis), by whole lattice cells, so the wheel always stays at least 5 m from its edge. A recentre keeps the overlap and samples only the new strips.
- Exactness: heights are quantized to 2⁻¹⁶ m, the window's z sits on a whole metre below its lowest sample and the range is a power of two, so `pos.z + f32(data) × range` reproduces every stored height exactly. A recentre therefore changes nothing under the wheel: ray heights at 2000 fixed points moved by at most 1.8 × 10⁻¹⁵ m over four recentres (spike E).
- The window samples `World::fill_points(…, inset = 4.7 cm)`: each curb, step and plaza wall steps up just behind its box's face, inside the box. 4.7 cm is the most the world's thinnest boxes allow (curb segments round a tight corner: at 9.4 cm the step left some of them). With 1/16 m cells the field may therefore start to rise up to 1.6 cm in front of a face, at its foot and at most a quarter of its height up; the tire meets the box's top edge first.

### Box pool

- 128 mocap bodies with one box geom each, parked 1000 m below when unused.
- Every tick, `solids_in` around the shell; the solids within 6 m (nearest first, then by identity) fill the pool. A solid keeps its slot while it stays in reach; new ones take free slots.
- Moving a box is `mocap_pos` and `mocap_quat`. Changing its size also needs `geom_size`, `geom_rbound` and `geom_aabb` written: with stale bounds MuJoCo misses contacts (spike B).
- Window and boxes use `friction="0 0 0" condim="3" conaffinity="3"`, so wheel, shell and rider all touch them.

### Rider's window

A rider who has left the wheel (stepped off, let go after a cut or thrown) can end up farther from the wheel than the window reaches; without ground there it would fall through the world.
- A second `hfield` of 65 × 65 samples at the same cells (a 4 m square) sits in its own mocap body, parked 1000 m below.
- Every tick, once a rider off the wheel has its origin more than 4 m from the window's centre (per axis), the rider's window lies under it, centred on whole cells, and follows it when it drifts more than 0.5 m. It lets go when the rider is back within 3.5 m, or on the wheel.
- It shows the ground only from the window's edge outwards: its samples inside the window sit at its floor, below the window's ground, so no contact is doubled. Its heights are the window's samples (same lattice, same quantum), so the two meet without a step.
- No box pool follows the rider; the faces' step behind each box (above) still gives it curbs and steps.
- Test (`world.rs`): with each rider model, a rider who stepped off stays above the ground for 3 s after the wheel goes 9 m away, and the rider's window takes over.

### Ground ray

`Physics::ground_ray_m(xy)` gives the ground height at a physics-frame point as the tire meets it: one `mj_ray` from 50 m above against the ground geoms only. Every ground geom (the floor or strip, curb boxes, a height map, the world's window, box pool and rider's window) is in geom group 3 (`GROUND_GROUP`), which MuJoCo uses only to filter rays and drawing, never contacts; so the ray never hits the tire, the shell or a rider. It returns `None` where there is no ground (in a world, beyond the windows: the window reaches at least 5 m ahead of the wheel, the box pool 6 m). About 0.9 µs per ray in a release build, so a rider can look ahead with a few samples every step. Test (`tests/world.rs`): under the wheel and the rider, with each rider model, the ray finds the road within 3 mm; on a strip it finds a 15 cm curb's top past its face.

### Ground friction

World geoms have no friction of their own. MuJoCo takes the larger friction of the two geoms in a contact, so the tire's friction decides on every surface.
Every tick, the surface under the shell (`surface_at`) scales the tire geoms' friction: sliding μ = tire μ × surface μ / asphalt μ, rolling = tire rolling × surface rolling resistance / asphalt rolling resistance. MuJoCo applies a run-time change of `geom_friction` from the next step (spike F).
Rider limbs and the shell keep their own friction on every surface.

### Scheduling

The world ticks before every 20th step: a recentre if the shell drifted, the pool refresh, and the friction under the wheel. It runs on state alone, so a replay repeats it exactly.
A prefetch thread keeps the `World`'s chunk cache warm within 40 m of where the wheel will be in 1.5 s. The cache never changes a value, only how soon it is ready, so timing never changes results. The strip samples themselves are taken on the step thread at a recentre.

### Spike results

A throwaway program in `.cache/world-spike` (git-ignored) linked mujoco-rs 5.0.0 against MuJoCo 3.9.0, as `euc-physics` does. Measured on this Mac (Apple silicon), release build.

| Check | Result |
| --- | --- |
| A1: change hfield data under a resting ball | mocap window: ball rose 5.0 cm, as the data did. World-body window: `geom_pos` change ignored. |
| A2: ball rolling 10 m, recentred 10 m window vs one static hfield on the same lattice | mocap: 9 recentres, max deviation 3.2 × 10⁻⁸ m after 4 s. World-body window: fell through at step 800. |
| E: ray heights at fixed points across 4 recentres | max change 1.8 × 10⁻¹⁵ m; 7998 of 8000 bit-identical |
| B: boxes moved between steps | static world geom via `geom_pos`: not seen. Mocap box via `mocap_pos`: contact correct. Mocap box resized: wrong until `geom_rbound` and `geom_aabb` are written, then correct. |
| F: ground friction changed at run time (priority 1) | μ 0.9 → 9.6 m/s² (start transient), 0.2 → 1.80 (expected 1.96), 0.5 → 4.76 (expected 4.91) |
| C: step cost, 20-inch wheel of 48 capsules on a carriage, 5 m/s | plane 2.8 µs; hfield 0.25 m 14.5 µs; 3.125 cm 21–25 µs (window 10, 12 or 16 m: no difference); 2 cm (today's scenario strip) 34.5 µs; 1.5625 cm 37–43 µs |
| C: recentre cost, stand-in terrain | 3.125 cm, 12 m: 0.26–0.41 ms; 1.5625 cm, 10 m: 1.1 ms |
| D: 12 cm curb at 2.5 m/s, straight on | reference box, hfield only, hfield + box: same outcome (min vx −0.75 m/s, peak vertical acceleration 117–125 m/s²) |
| D: the same curb at 30° | reference box: lateral push 11 m/s², drift 2.3 cm. Hfield only: 0 m/s², no drift (wrong). Hfield + box: 12 m/s², 2.3 cm (matches). |
| G: euc-world sampler | 164–229 ns per sample; full 385² window 24–34 ms warm, 72–80 ms with chunk generation; a 1 m strip 1.7–2.1 ms |
| G: chunk generation | 0.94 ms per chunk through `World` (256 m × 256 m); 29 ms for a lone `Chunk::generate` (no tile cache) |
| G: the rig on the generated spawn street, 6 m/s, 7 s | 36 recentres at 4.0 ms each (strip sampling and chunk generation on demand); 29.8 µs per step including recentres; axle stayed within 1.0 mm of ground + radius |

Conclusions: the mocap window and the mocap box pool work. Use 1/32 m cells: 1/64 m doubles the contact cost for no visible gain once boxes carry the faces (since #13 the cells are 1/16 m: below). Boxes are needed for oblique faces. Prefetch strips off the step thread: on demand they cost 4 ms per recentre.

### Cell size (#13)

The soft tire of bundle P1 sinks about 9.5 mm and spans more cells than the stiff contact, so at 1/32 m the collision narrowphase took about 80 % of the physics time and the world runs fell below real time's budget. The faces sit on the box pool, not in the field, so the cell went to 1/16 m (same 12 m and 4 m windows, same 1 m recentre, inset 4.7 cm). Measured (HEAD f83a800, stiff tire, stock firmware), 1/32 m → 1/16 m:

- The ground as the tire meets it (`ground_ray_m` against the drawn ground, every 5 mm along the path):
  - the 2 cm city pothole (W2): depth 19.2 mm both; largest field error 3.5 → 6.5 mm, at its wall; the tire's rolling envelope dips 19.2 mm both;
  - the 5 cm city pothole: depth 48.1 → 48.0 mm; largest error 9.8 → 21 mm, at its steep wall; the envelope dips 47.0 → 47.6 mm;
  - the W1b stairs: the step boxes carry every riser in both (5 × 16.7 cm, sharp); field within 2 mm of the drawn steps;
  - small relief, every road point on lines 2 m apart over 100 m × 100 m of a default-density city (cracks, rough patches, manholes, potholes, hills; 304 618 points): field error RMS 1.15 → 1.16 mm, points off by more than 5 mm 0.017 → 0.036 %; the tire's envelope RMS 0.44 mm both, points off by more than 1 mm 0.61 → 0.64 %. Cracks (1-2 cm wide) were already narrower than a cell.
  - At rest on a level street the tire sits at the same height (shell 224.67 mm both).
- Runs (stock, quanta 4000, 3000 and 2000): every verdict unchanged (curb up straight on and at 30°: fall at 11.27 s and 11.80 s, 0.02 s sooner; the other four W1b runs and both city potholes pass). Impacts are a little softer: the 2 cm pothole costs 20.1 → 18.8 km/h (18.5 before) with Iq 79 A (95), the 5 cm one 16.2 km/h (15.6) with the shell 4.1° nose down (5.2°) and Iq 241 A (247); down the stairs the suspension takes 72 mm (90) and Iq 145 A (172). The ground is the same within 2 mm there, so the change is in the contacts: fewer simultaneous contacts per impact at 1/16 m (on the stairs the step boxes and the field under them both touch). The tire's impact stiffness is therefore calibrated on this cell (P1).
- Speed, with today's stiff tire (a quiet machine, one run at a time, quantum 4000, best of two, wall time per 1/120 s of simulated time): the six W1b world runs and the two city potholes take 4.37 ms on average at 1/32 m (1.91 × real time) and 4.10 ms at 1/16 m (2.03 ×), 5–9 % less per run. In the same window the soft tire of bundle P1 went from 6.1–6.8 to 5.2–5.5 ms of CPU per 1/120 s (1.52–1.59 ×): contacts per step 6.3–7.8 → 5.2–6.0, the collision narrow phase 69–74 % → 59–66 % of the physics time.

### Budgets

- Physics: at most 30 µs per step on average for ground contact and world upkeep together (2 kHz, about 6 % of real time).
- Chunk generation: at most 2 ms amortized (0.94 ms measured), off the step thread.

Measured in W1b (this Mac, Apple silicon), whole runs with the stock firmware on the project's own Cortex-M3 core:

| Run | Result |
| --- | --- |
| `euc sim run` on the six `scenarios/world` runs, release build | real-time factor 1.75–1.99 on an idle Mac; 1.47–1.91 with the dev studio running |
| Studio Free Ride host alone, world seed 42, 30 s at about 6.5 km/h, articulated rider (`stock_articulated_rides_the_streaming_world`) | 4.70 ms of wall time per 1/120 s sample: 56 % of the 8.33 ms budget, real-time factor 1.77 |
| The same, rigid rider | 4.54 ms per sample (54 %), real-time factor 1.84 |

The studio's engine crates build at `opt-level = 3` in development too, so the second and third rows are what the dev studio's run thread does. Drawing runs on other threads.

## Studio (built in W1b)

- **Payloads.** `euc_world::render` encodes chunks for drawing: per chunk and level of detail, a height grid (f32, world z), a material grid (u8 codes, bit 7 wet), the solids as oriented boxes, and the props. The layout is in the module's documentation; `world_chunks` (Tauri, binary IPC, at most 64 chunks per request) sends it. Level 0: 129² heights and 256² material texels per chunk (12.5 cm and 6.25 cm), faces inset 15 cm behind their boxes, every solid and prop. Level 1: 33² and 32², buildings and props. Level 2 (9², 8²) exists but the stream does not use it yet.
- **Worker.** `worldWorker.ts` decodes payloads and builds the ground mesh (positions, normals, a material texture) and one box mesh per chunk off the UI thread, as transferable buffers.
- **Rings.** `WorldStream` keeps level 0 within 30 m of the wheel and level 1 within 104 m (high quality), or 18 m and 56 m (low quality), refreshes the wanted set every 0.25 s, asks for the nearest chunks first (4 per level-0 request, 24 per level-1 request), and drops what falls out of range. Props are one `InstancedMesh` per part and kind (lamp posts, trees, benches, bins, hydrants, bollards).
- **Frame.** The world group sits in the physics frame like the live bodies. Chunk meshes carry chunk-local vertices, so f32 stays precise far from the spawn without a floating origin.
- **View.** On a world the camera sees 300 m. `WorldView` samples the loaded level-0 and level-1 heights for the floor light, the camera and the grade readout.
- **Outdoor look (FR4, `app/src/workspaces/ride/outdoor/`).** A world is outdoors; scenarios without a world and old test-pad recordings keep the studio sweep. After EUC Thrills by VibezZzCoder (MIT), re-implemented in our code (`docs/design/free-ride.md`):
  - *Sky.* An equirectangular sky painted once per world in a worker (1024 × 512): a horizon-to-zenith ramp shaped by y^0.62, warm scatter low toward the sun, value-noise clouds on a flat deck seeded from the world seed (a recording shows the same clouds), and the sun's core and glow. It is not tone mapped (sRGB), and it turns with the world group, so the sun is fixed in the world.
  - *Fog.* Linear, in the horizon colour `#BCD4EC`. three.js mixes fog after tone mapping and colour conversion, so a fully fogged pixel is exactly the sky's horizon. The fog ends before the outer ring does (30 to 90 m on high, 12 to 44 m on low), so the ground's edge is never seen.
  - *Sun and fill.* One sun at azimuth 225° (south-west) and 42° up, colour `#FFF0DE`, intensity 3.1, shared by the painted sky and the shadows; a hemisphere light (`#C2CCD6` over `#B2AB97`, 0.6); the sky's PMREM environment (a 256 × 128 painting with ground bounce below the horizon) at 0.2. Tuned under the Neutral tone mapping at exposure 1.5, which stays: sunlit asphalt renders about (122, 121, 124), its shadow about (29, 47, 69), close to the reference.
  - *Shadows (high tier).* A 2048² map over ±25 m that follows the wheel, 9 m ahead along the view, its centre snapped to whole shadow texels in the sun's frame so edges do not crawl. Buildings, solids, props and the bodies cast; the ground receives.
  - *Contact pools.* The ground's shader darkens softly under the wheel (radius 0.6 m) and under the rider's body (1.05 m, stretched 1.6 times along the shadow), from the poses each frame. They replace `ContactShadows` on a world.
  - *Palette.* Muted mid-values for daylight (asphalt `#72767C`, concrete `#B2AEA4`, grass `#547446`, paint `#E2E0D8`, never pure white) and facade colours chosen per building by hash.
  - Budget, measured in headless WebKit at 960 × 540 with real chunks: 60 frames per second, about 190 draw calls, 605 000 to 630 000 triangles, 53 to 57 textures.
- **Errors.** When the engine refuses chunks, the view keeps what it drew, asks again, and shows a "World view incomplete" pill whose tooltip gives the reason. The physics does not depend on the view.
- **Where it shows.** Free Ride on a world, a live `[world]` scenario (the run sends its `WorldFrame` with the bodies), and the playback of a recording with a world (the studio regenerates it from the recorded seed).
- **Not built:** ring 2 and skirts, a facade shader, wet gloss, a minimap; ground detail and road marking ribbons (FR5), building looks and a skyline (FR7), vegetation and better props (FR8).

## Free Ride

Free Ride always rides a city world. Its panel shows the world as "City, seed N" and offers "New world" (a random seed, ridden at once); the densities are the defaults.
The run starts at the world's spawn. There is no edge: the ground follows the wheel. The status names the ground (`ground: "world"`), and `free_ride_world` gives the view its frame.

## Recordings and replay

- The recording context (`euc_sim::recording::RecordingContext`) has `world` or `height_map`, never both. `world` is a `WorldRecord`: the `WorldSpec` (seed, revision, biome, densities, incline), the start point and heading, and a digest of the 3 × 3 chunks around the start. It never stores meshes or height grids: a world recording's context is a few hundred bytes.
- Free Ride on a world and every `[world]` scenario record it.
- Import and playback check it: a revision other than this build's generator is refused with a clear message, and a different digest says that this build generates a different world for the recorded seed.
- Recordings with the old `generated_heightmap_v1` map keep their `height_map` and replay as before.

## Scenario format

```toml
[world]
seed = 42
revision = 2               # must match the generator (euc_world::GENERATOR_REVISION)
biome = "city"             # optional; the only biome
start = { x_m = 70.0, y_m = 3.05, heading_deg = 0.0 }   # optional; the world's spawn without it
route = [[127.59, -54.24], [133.01, -54.60]]            # optional waypoints, world metres
slope_pct = -2.0           # optional: an incline along the start heading (± 40 %)
slope_start_m = 25.0       # where it starts, metres ahead of the start
slope_ramp_m = 10.0        # over which it builds up (default 10)

[world.densities]          # optional; defaults as above
potholes = 3.0
plazas = 0.3
hills = 0.0                # a level city
```

A scenario has either a non-flat `[terrain]` (the 1-D strip) or `[world]`, never both. A route has at most 256 finite points. After release the rider heads for the next waypoint and moves on within 1 m of it; after the last one it holds its heading. [scenarios/writing-scenarios.md](../scenarios/writing-scenarios.md) has the field table.

## Shipped scenarios in the city (W2)

Every scenario in `scenarios/` has a `[world]` table (seed 42, revision 2); the parser's default is unchanged, so a file without one, and the scenario text inside recordings made before, still rides the plane. Each scenario's note says why its spot fits its test.
The plain flat floor is allowed only on purpose, as `[world] flat = "<reason>"` (no other `[world]` key with it): `full-effort-70kmh-stop-soc100` rides it, because the city exposes the rigid rider's marginal roll stability in its hard brake ([scenarios.md](scenarios.md#city-world-w2-2026-10-23)).

| Scenarios | Spot |
| --- | --- |
| ride, coasting, hard braking, hall loss (2), low battery, steps off (2), full effort (20 % and 50 %), overlean (3) | From the spawn east along the arterial (row 0) on a straight line that stays within 2 m of the centreline, in the carriageway's level middle, for up to 450 m. Ideal road: `hills = 0`, no potholes, manholes, cracks, rough or wet patches. Lane and crosswalk paint stays. |
| balance-60s, ble-service-cut-standing | The middle of the level paver platform of the plaza at (48.1, −108.0), 15.9 × 6.4 m, at least 3 m from every edge; ideal ground. |
| pothole-20kmh | A city pothole 2.0 cm deep and 0.49 m across at (−31.13, −70.91), 50 m south of the start on a level, clean, straight approach. |
| pothole-5cm-20kmh | A city pothole 4.9 cm deep and 0.60 m across at (−28.33, 214.60), 50 m north of the start on a level, clean, straight approach. |
| downhill-cruise-2pct, hill-climb-low-battery | The spawn arterial with an incline: −2 % from 25 m, +20 % from 40 m (10 m ramps); ideal road. |

## W1b acceptance

`scenarios/world/` (seed 42, revision 2, settings `tarik-2026-10-15`, release at 2.5 s), stock firmware GW1825007, quanta 4000, 3000 and 2000:

| Scenario | What it rides | Result |
| --- | --- | --- |
| `world-curb-up-90deg` | a 13.5 cm curb up, straight on | falls at 11.27 s at all three quanta: the tire stops at a curb of 60 % of its radius |
| `world-curb-up-30deg` | the same curb at 30° | falls at 11.80 s |
| `world-curb-down-90deg` | a curb down, straight on | passes: pitch −2.0°, 86–90 A, suspension 76 mm |
| `world-pothole-15kmh` | a 7.2 cm deep, 1.18 m wide generated pothole at 15 km/h | passes: pitch −1.9°, 36–37 A, suspension 54 mm |
| `world-stairs-down-5kmh` | 5 steps of 16.7 cm down a plaza's stairs at walking speed | passes: suspension 72–73 mm, 130–145 A |
| `world-ramp-up-7kmh` | a plaza ramp up, steered by a route | passes: pitch at most 2.1° |

Quanta 4000 and 2000 give identical results; 3000 differs slightly. The numbers are those after the moving-mass fix (a1951a4: the rider senses air drag in a world run too; before it the curb-up runs fell 0.04 s later) and on 1/16 m cells ([Cell size](#cell-size-13); at 1/32 m the curb-up runs fell 0.02 s later, and the stairs took 90–94 mm of suspension and 125–172 A).
The two curb-up runs fail their `upright` check: the wheel stops dead at the curb and the rider falls. The ground works as built (the curb box stops the tire), but a real wheel climbs such a curb: see the open item below. PRELIMINARY: no measured ride over a curb backs these runs, and their `record` checks only report.

**Open item (curbs).** The owner rides straight into curbs of about 15 cm without lifting the wheel; a higher curb needs a jump. So the result above is wrong: the sim throws the rider at a 13.5 cm curb at every quantum. The cause is in the tire and the rider, not the ground: the bundled physics update (tire envelopment and grip, and the rider's knees absorbing the hit) is to fix it. In that update's rerun the acceptance changes to: a 15 cm curb taken straight on at moderate speed is climbed; clearly higher curbs still need a jump. The curb scenarios stay as they are until then.

## Cross-platform determinism

- Generation is bit-identical on macOS, Linux and Windows by construction (the arithmetic rules above). The golden digest checks it: the engine CI's test job runs `euc-world`'s tests in debug on Linux, macOS and Windows, and its `world-golden` job runs them in debug and release on Linux and Windows.
- f32 hfield values are exact (quantized heights, power-of-two range), so the physics receives the same ground everywhere.
- MuJoCo itself is not guaranteed bit-identical across CPUs and compilers. That is today's situation too: a replay is bit-identical on the same build and platform; across platforms the world is identical and the physics can drift at round-off level.

## Hooks (designed, not built)

- **Biomes:** `Biome` selects the layer set. Park (paths, grass, benches, gentle hills), trail (dirt single track, roots, rocks, steep grades), mountain road (switchbacks, guard rails, 10–15 % grades), industrial (wide concrete, rail crossings, loading docks). Biomes blend by a low-frequency field across region borders.
- **Weather:** a `Weather` input (rain intensity, wind vector, temperature). Rain makes every surface wet (the wet columns above) and grows puddles in potholes and low spots; wind is an external force on rider and wheel; temperature feeds battery and tire models.
- **Obstacles and traffic:** static obstacles reuse the solid pool (parked cars, bins, construction barriers). Moving traffic is kinematic mocap bodies on lane centrelines from the road graph, with simple yielding at intersections.
- **Autopilot rider:** follows a route over `edge()` centrelines with a look-ahead point, picks a speed from curvature and surface, and sees the ground through a reaction time (perception delay plus the rider's existing command delay).
- **Campaign runs:** ride N km over M seeds per firmware build; count incidents (falls, cutoffs, alarms, tilt-backs) per km; keep a recording for each incident; compare firmware builds by incidents per km with the same seeds.
- **Real-world import:** OpenStreetMap ways become road edges with real widths and classes; an elevation model (SRTM or a city DEM) replaces the relief. The owner's ride logs (GPS and speed) become routes and validation targets. The sampler interface stays the same.

## Preview

```text
cargo run -p euc-world --features preview --release --bin euc-world-preview -- \
    --seed 42 --area -128,-128,128,128 --px-per-m 4 --out world.png [--grid] [--print]
```

Shaded relief (light from the north-west, relief exaggerated 3×) over surface colours: asphalt dark grey, concrete light grey, pavers tan, cobblestones brown-grey, grass green, dirt brown, gravel beige, paint white, metal blue-grey, wet patches blue.
Curb stones, stair steps (white outlines), ramps and plaza platforms are drawn from their solids, buildings as slate roofs, props as dots (lamp posts yellow, hydrants red, bollards black, benches and bins as their boxes, trees as canopies). The spawn is a magenta ring. `--grid` draws chunk borders; `--print` lists counts and every plaza with its steps and ramp.
The `preview` feature adds two small dependencies (`miniz_oxide` for deflate, `crc32fast` for PNG chunks); the library has only `serde`.

## Tests

How to test the world, from the repository root:

- the generator: `cargo test -p euc-world` (and `--release`, as the `world-golden` CI job does);
- the ground in the physics: `cargo test -p euc-physics --test world` and the `world::tests` unit tests (`cargo test -p euc-physics --lib world`);
- a picture of a world: `euc-world-preview` (above);
- the studio's view on real chunks: `cargo build --release -p euc-world --example payload`, then in `app/`: `WORLD_PAYLOAD_BIN=<target>/release/examples/payload pnpm exec playwright test -c e2e/world-view.config.ts`.

`cargo test -p euc-world` (about 21 s in a debug build, 4 s in release):

- the same seed gives identical chunks, another seed does not;
- chunks do not depend on generation order (standalone generation vs a cached world in scrambled order);
- neighbouring chunks agree bit for bit on their border (drawn and physics heights, zones, surfaces);
- `fill_heights` matches `ground_at`;
- every jump of more than 4.5 cm over 1 cm, drawn or physics, lies under a solid (scan lines over 140 m × 140 m, two seeds);
- curb boxes agree with the sampler: face on the curb line, top on the sidewalk edge, physics step behind the face;
- the spawn guarantees, over 12 seeds;
- feature sizes within their limits (potholes, manholes, bumps, plaza steps, risers, treads, ramp slopes, curbs);
- densities follow their parameters; the network is connected; hills stay under 12 %;
- solid tops match the drawn ground (two seeds);
- a golden digest of the chunks around the spawn;
- `render`: payloads are aligned and complete at every level, and neighbouring chunks share their border heights bit for bit.

W1b added, elsewhere:

- `euc-physics` (`tests/world.rs`): the window follows the wheel down the spawn street with each rider (rigid, articulated, lateral) (ray heights match the world within 3 mm, no ground past the window); a curb box is climbed or stops the wheel, never passed through.
- `euc-sim`: `[world]` scenarios parse, place their start, and refuse `[terrain]` as well, another generator revision, unknown fields, bad routes and densities out of range; a world recording context round-trips and refuses another revision, a different world, a map as well, and densities out of range.
- Studio, Rust: the shared world cache; Free Ride status, spec and recording context with a world; ignored tests with the cached stock firmware: Free Ride rides 30 s on world seed 42 through the real factory with both riders (no edge pause, tire within 1 cm of the drawn ground, the step budget above), and a `[world]` scenario sends its frame to the view and records its world.
- Studio, TypeScript: payload decoding, mesh winding and the view's sampling (`worldMesh.test.ts`), the world choice over IPC (`freeRide.test.ts`), the live engine's world message (`live.test.ts`).
- Studio, browser (`app/e2e/world-view.spec.ts`, Chromium and WebKit): the real stream, worker, mesh builder and `WorldSurface` in WebGL on synthetic chunks: two rings of detail, chunks dropped behind and loaded ahead, errors reported while drawn chunks stay, and `WorldSurface` reporting a refused request to the view and clearing it once chunks arrive. With `WORLD_PAYLOAD_BIN` set to `euc-world`'s `payload` example, a fourth test draws real chunks and saves a screenshot.

## Known limits

- The road field takes the cross slope from the terrain: on the steepest hills a street can tilt sideways up to the terrain's grade.
- Cracks (1–2 cm wide) are narrower than a physics cell; the height field catches them as scattered one-cell dips.
- Plazas face their anchor's nearest street; on a strongly curved street the plaza sits at an angle to the curb.
- Street paving changes at intersections along a line where the nearer street changes; an arterial wins inside the crossing itself.
- Only the city biome exists.
- One surface per tick: the tire's friction follows the surface under the shell, not under each contact point.
- Rider limbs and the shell keep their own friction on every surface.
- A 13.5 cm curb taken straight on throws the rider; the owner climbs about 15 cm curbs without lifting (see the open item under W1b acceptance).
- Not built from the plan: an `euc world preview` wrapper (the `euc-world-preview` binary does the job), ring 2 and skirts, a facade shader, wet gloss, and a Free Ride minimap.

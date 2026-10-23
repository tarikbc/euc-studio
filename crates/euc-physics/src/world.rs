//! The streaming world as MuJoCo ground (`docs/models/world.md`, "Physics plan").
//!
//! A height-field window of 193 × 193 samples at 1/16 m (a 12 m square) sits in a mocap body and
//! follows the wheel by whole cells; a pool of mocap boxes carries every vertical face near the
//! wheel (curbs, steps, plaza walls, ramp sides, buildings, props); a second, 4 m window lies
//! under a rider who has left the wheel once the rider is near the first window's edge. All of
//! them move only between steps, on a fixed cadence and on state alone, so a run repeats exactly.
//! A worker thread keeps the world's chunk cache warm ahead of the wheel; the cache never changes
//! a value, only how soon it is ready.
//!
//! The physics frame is the world frame moved so that the start lies at the origin heading +x:
//! `world = start + R(heading) · physics`, `z_world = z_physics + origin_z`.

// Moving the window and the boxes writes MuJoCo's model between steps (height-field samples,
// box sizes and bounds, tire friction), which mujoco-rs only offers through `model_mut`.
#![allow(unsafe_code)]

use std::collections::BTreeMap;
use std::fmt::{self, Write as _};
use std::sync::{Arc, mpsc};

use euc_world::{Material, Rect, Solid, Surface, World, WorldSpec};
use mujoco_rs::prelude::*;

use crate::PhysicsError;
use crate::mjcf::names;

/// Samples along each side of the window.
pub const WINDOW_SAMPLES: usize = 193;
/// Window cell, metres (a power of two, so every sample position is exact). The faces sit on the
/// box pool, not in the height field, so 1/16 m loses only sub-cell relief (cracks, the wall of a
/// pothole drawn over one cell more) and costs the tire's contacts far less than 1/32 m.
pub const WINDOW_CELL_M: f64 = 1.0 / 16.0;
/// Half the window's side, metres.
pub const WINDOW_HALF_M: f64 = (WINDOW_SAMPLES - 1) as f64 / 2.0 * WINDOW_CELL_M;
/// The window's elevation range, metres (a power of two, so stored heights stay exact).
pub const WINDOW_RANGE_M: f64 = 8.0;
/// How far the wheel may drift from the window's centre before it recentres, metres.
pub const RECENTRE_M: f64 = 1.0;
/// Faces with a solid step up this far behind their solid's face in the height field, metres:
/// 4.7 cm, the most the world's thinnest boxes (curb segments round a tight corner) allow. With
/// 1/16 m cells the field may start to rise up to 1.6 cm in front of a face, at its foot: the
/// tire meets the box's top edge before that (for any face lower than the tire's radius).
pub const FACE_INSET_M: f64 = 1.5 / 32.0;
/// Boxes in the pool.
pub const POOL_SIZE: usize = 128;
/// Solids within this distance of the wheel go into the pool, metres.
pub const POOL_REACH_M: f64 = 6.0;
/// Physics steps between world ticks (recentre check, pool refresh, surface under the wheel).
pub const TICK_STEPS: u64 = 20;
/// Samples along each side of the rider's window: a 4 m square at the window's cells, under a
/// rider who has left the wheel and lies beyond the wheel's window.
pub const RIDER_WINDOW_SAMPLES: usize = 65;
/// Half the rider's window's side, metres.
pub const RIDER_WINDOW_HALF_M: f64 = (RIDER_WINDOW_SAMPLES - 1) as f64 / 2.0 * WINDOW_CELL_M;
/// A rider off the wheel gets its own window once its body's origin is this far from the
/// window's centre along x or y, metres (its body reaches about 1 m from that origin), and keeps
/// it until it is back within [`RIDER_RELEASE_M`].
pub const RIDER_TAKEOVER_M: f64 = WINDOW_HALF_M - 2.0;
/// See [`RIDER_TAKEOVER_M`].
pub const RIDER_RELEASE_M: f64 = WINDOW_HALF_M - 2.5;
/// How far the rider may drift from its window's centre before the window moves, metres.
pub const RIDER_RECENTRE_M: f64 = 0.5;

/// Height quantum, metres: window heights are multiples of it, so `f32` data is exact.
const QUANTUM_M: f64 = 1.0 / 65536.0;
const WINDOW_BASE_M: f64 = 0.5;
const PARKED_Z_M: f64 = -1000.0;
const HALF_CELLS: i64 = (WINDOW_SAMPLES as i64 - 1) / 2;
const RIDER_HALF_CELLS: i64 = (RIDER_WINDOW_SAMPLES as i64 - 1) / 2;
/// Name of the window's mocap body, hfield and geom.
pub(crate) const WINDOW: &str = "world_window";
/// Name of the rider's window's mocap body, hfield and geom.
pub(crate) const RIDER_WINDOW: &str = "world_rider_window";
/// Prefix of the pool's mocap bodies.
pub(crate) const BOX: &str = "world_box_";
/// Contact settings of every world geom: today's floor, without friction of its own, so the
/// tire's friction (scaled per surface) and each body's own friction decide.
const GROUND_CONTACT: &str = r#"friction="0 0 0" condim="3" conaffinity="3" group="3""#;

/// Where a world run starts: the wheel's ground point and heading in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldStart {
    /// Ground point under the wheel, metres.
    pub xy_m: [f64; 2],
    /// Heading, radians, left positive from world +x.
    pub heading_rad: f64,
}

/// A streaming world as the ground of one run.
#[derive(Clone)]
pub struct WorldGround {
    spec: WorldSpec,
    start: WorldStart,
    world: Arc<World>,
    origin_z: f64,
    rotation: [f64; 2],
}

impl fmt::Debug for WorldGround {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WorldGround").field("spec", &self.spec).field("start", &self.start).finish_non_exhaustive()
    }
}

impl PartialEq for WorldGround {
    fn eq(&self, other: &Self) -> bool {
        self.spec == other.spec && self.start == other.start
    }
}

impl WorldGround {
    /// The world of `spec`, starting at its spawn.
    pub fn at_spawn(spec: WorldSpec) -> Result<Self, String> {
        let world = World::new(spec)?;
        let spawn = world.spawn();
        let heading = spawn.heading[1].atan2(spawn.heading[0]);
        Self::with_world(
            Arc::new(world),
            WorldStart { xy_m: [spawn.position[0], spawn.position[1]], heading_rad: heading },
        )
    }

    /// The world of `spec`, starting at `start`.
    pub fn new(spec: WorldSpec, start: WorldStart) -> Result<Self, String> {
        Self::with_world(Arc::new(World::new(spec)?), start)
    }

    /// A run on an existing (cached) world.
    pub fn with_world(world: Arc<World>, start: WorldStart) -> Result<Self, String> {
        if !start.xy_m.iter().all(|v| v.is_finite()) || !start.heading_rad.is_finite() {
            return Err("world start must be finite".into());
        }
        let rotation =
            if start.heading_rad == 0.0 { [1.0, 0.0] } else { [start.heading_rad.cos(), start.heading_rad.sin()] };
        let origin_z = world.height_at(start.xy_m[0], start.xy_m[1]);
        Ok(Self { spec: *world.spec(), start, world, origin_z, rotation })
    }

    /// The world's specification.
    pub const fn spec(&self) -> &WorldSpec {
        &self.spec
    }

    /// The start.
    pub const fn start(&self) -> WorldStart {
        self.start
    }

    /// The world (shared, with its caches).
    pub fn world(&self) -> &Arc<World> {
        &self.world
    }

    /// World height of the physics frame's z = 0, metres.
    pub const fn origin_z(&self) -> f64 {
        self.origin_z
    }

    /// A physics-frame point in world coordinates.
    pub fn to_world(&self, p: [f64; 2]) -> [f64; 2] {
        let [c, s] = self.rotation;
        [self.start.xy_m[0] + c * p[0] - s * p[1], self.start.xy_m[1] + s * p[0] + c * p[1]]
    }

    /// A world point in the physics frame.
    pub fn to_physics(&self, p: [f64; 2]) -> [f64; 2] {
        let [c, s] = self.rotation;
        let d = [p[0] - self.start.xy_m[0], p[1] - self.start.xy_m[1]];
        [c * d[0] + s * d[1], -s * d[0] + c * d[1]]
    }

    /// Drawn ground height at a physics-frame point, in the physics frame, metres.
    pub fn height_m(&self, p: [f64; 2]) -> f64 {
        let w = self.to_world(p);
        self.world.height_at(w[0], w[1]) - self.origin_z
    }

    /// The surface at a physics-frame point.
    pub fn surface(&self, p: [f64; 2]) -> Surface {
        let w = self.to_world(p);
        self.world.surface_at(w[0], w[1])
    }

    /// MJCF `<asset>` line: the window's and the rider's window's height fields, filled at run
    /// time.
    pub(crate) fn asset() -> String {
        format!(
            r#"  <asset><hfield name="{WINDOW}" nrow="{WINDOW_SAMPLES}" ncol="{WINDOW_SAMPLES}" size="{WINDOW_HALF_M} {WINDOW_HALF_M} {WINDOW_RANGE_M} {WINDOW_BASE_M}"/><hfield name="{RIDER_WINDOW}" nrow="{RIDER_WINDOW_SAMPLES}" ncol="{RIDER_WINDOW_SAMPLES}" size="{RIDER_WINDOW_HALF_M} {RIDER_WINDOW_HALF_M} {WINDOW_RANGE_M} {WINDOW_BASE_M}"/></asset>"#
        )
    }

    /// MJCF world-body lines: the window's mocap body, the parked box pool and the parked
    /// rider's window.
    pub(crate) fn bodies() -> String {
        let mut xml = format!(
            r#"    <body name="{WINDOW}" mocap="true"><geom name="floor" type="hfield" hfield="{WINDOW}" {GROUND_CONTACT}/></body>"#
        );
        for k in 0..POOL_SIZE {
            // Writing to a String cannot fail.
            let _ = write!(
                xml,
                "\n    <body name=\"{BOX}{k}\" mocap=\"true\" pos=\"0 0 {PARKED_Z_M}\"><geom name=\"{BOX}{k}\" type=\"box\" size=\"0.1 0.1 0.1\" {GROUND_CONTACT}/></body>"
            );
        }
        let _ = write!(
            xml,
            "\n    <body name=\"{RIDER_WINDOW}\" mocap=\"true\" pos=\"0 0 {PARKED_Z_M}\"><geom name=\"{RIDER_WINDOW}\" type=\"hfield\" hfield=\"{RIDER_WINDOW}\" {GROUND_CONTACT}/></body>"
        );
        xml
    }

    /// The ground geoms a tire can meet: the window's floor and every pool box.
    pub(crate) fn geom_names() -> Vec<String> {
        std::iter::once("floor".to_owned()).chain((0..POOL_SIZE).map(|k| format!("{BOX}{k}"))).collect()
    }
}

/// One pool slot.
#[derive(Debug, Clone, Copy)]
struct Slot {
    mocap: usize,
    geom: usize,
    solid: Option<u64>,
}

/// The window, the pool and the tire friction of one run, inside [`crate::Physics`].
pub(crate) struct Window {
    ground: WorldGround,
    hfield_adr: usize,
    mocap: usize,
    centre: [i64; 2],
    z0: f64,
    heights: Vec<f64>,
    slots: Vec<Slot>,
    /// The tire's ground pairs.
    tires: Vec<usize>,
    /// The tire's own contact friction on asphalt: sliding, spin, rolling (length).
    tire_friction: [f64; 3],
    steps: u64,
    surface: Surface,
    prefetch: Option<mpsc::Sender<[f64; 4]>>,
    rider: RiderWindow,
    /// Recentres so far, and the cost of the last one on the step thread, seconds.
    pub recentres: u64,
    pub last_recentre_s: f64,
}

/// The rider's window: the ground under a rider who has left the wheel and lies beyond the
/// wheel's window.
struct RiderWindow {
    hfield_adr: usize,
    mocap: usize,
    /// Its centre cell while it holds the rider up; `None` while parked.
    centre: Option<[i64; 2]>,
    /// Quantized physics-frame heights at its samples, row by row.
    heights: Vec<f64>,
}

impl Window {
    /// Resolves the model's world bodies and fills the window and pool around the origin.
    pub(crate) fn new(ground: WorldGround, data: &mut MjData<Box<MjModel>>) -> Result<Self, PhysicsError> {
        let model = data.model();
        let missing = |what: &str| PhysicsError::Model(format!("no {what} in the world model"));
        let hfield = model.name_to_id(MjtObj::mjOBJ_HFIELD, WINDOW).ok_or_else(|| missing("window hfield"))?;
        let body = model.name_to_id(MjtObj::mjOBJ_BODY, WINDOW).ok_or_else(|| missing("window body"))?;
        let mocap = usize::try_from(model.body_mocapid()[body]).map_err(|_| missing("window mocap"))?;
        let mut slots = Vec::with_capacity(POOL_SIZE);
        for k in 0..POOL_SIZE {
            let name = format!("{BOX}{k}");
            let body = model.name_to_id(MjtObj::mjOBJ_BODY, &name).ok_or_else(|| missing(&name))?;
            let geom = model.name_to_id(MjtObj::mjOBJ_GEOM, &name).ok_or_else(|| missing(&name))?;
            let mocap = usize::try_from(model.body_mocapid()[body]).map_err(|_| missing(&name))?;
            slots.push(Slot { mocap, geom, solid: None });
        }
        let rider_hfield =
            model.name_to_id(MjtObj::mjOBJ_HFIELD, RIDER_WINDOW).ok_or_else(|| missing("rider window hfield"))?;
        let rider_body =
            model.name_to_id(MjtObj::mjOBJ_BODY, RIDER_WINDOW).ok_or_else(|| missing("rider window body"))?;
        let rider = RiderWindow {
            hfield_adr: model.hfield_adr()[rider_hfield] as usize,
            mocap: usize::try_from(model.body_mocapid()[rider_body]).map_err(|_| missing("rider window mocap"))?,
            centre: None,
            heights: Vec::new(),
        };
        // The tire meets the ground through its own pairs (mjcf.rs `tire_pairs`): their friction is
        // what each surface scales.
        let tire = model.name_to_id(MjtObj::mjOBJ_GEOM, names::TIRE).ok_or_else(|| missing("tire geom"))?;
        let (first, second) = (model.pair_geom1(), model.pair_geom2());
        let tires: Vec<usize> =
            (0..first.len()).filter(|&p| first[p] as usize == tire || second[p] as usize == tire).collect();
        // Every pair carries the tire's friction on asphalt (sliding, sliding, spin, rolling, rolling).
        let Some(&[mu, _, spin, rolling, _]) = tires.first().map(|&p| &model.pair_friction()[p]) else {
            return Err(missing("tire pairs"));
        };
        let tire_friction = [mu, spin, rolling];
        let (sender, receiver) = mpsc::channel::<[f64; 4]>();
        let world = Arc::clone(ground.world());
        let prefetch = std::thread::Builder::new()
            .name("world-prefetch".into())
            .spawn(move || prefetch(&world, &receiver))
            .ok()
            .map(|_| sender);
        let mut window = Self {
            hfield_adr: model.hfield_adr()[hfield] as usize,
            mocap,
            centre: [0, 0],
            z0: 0.0,
            heights: Vec::new(),
            slots,
            tires,
            tire_friction,
            steps: 0,
            surface: Surface::of(Material::Asphalt, false),
            prefetch,
            rider,
            recentres: 0,
            last_recentre_s: 0.0,
            ground,
        };
        window.heights = window.sample(&Self::lattice(WINDOW_SAMPLES, [0, 0], |_, _| true));
        window.upload(data);
        window.refresh(data, [0.0, 0.0]);
        Ok(window)
    }

    /// The ground this window shows.
    pub(crate) const fn ground(&self) -> &WorldGround {
        &self.ground
    }

    /// The surface under the wheel as of the last tick.
    pub(crate) const fn surface(&self) -> Surface {
        self.surface
    }

    /// Runs before a step: every [`TICK_STEPS`] steps, recentres if the wheel drifted, refreshes
    /// the pool and the tire friction, and keeps a rider off the wheel on the ground. `position`
    /// and `velocity` are the shell's, `rider` the origin of a rider who has left the pedals,
    /// physics frame.
    pub(crate) fn tick(
        &mut self,
        data: &mut MjData<Box<MjModel>>,
        position: [f64; 2],
        velocity: [f64; 2],
        rider: Option<[f64; 2]>,
    ) {
        self.steps += 1;
        if !self.steps.is_multiple_of(TICK_STEPS) {
            return;
        }
        let centre = [self.centre[0] as f64 * WINDOW_CELL_M, self.centre[1] as f64 * WINDOW_CELL_M];
        let moved = (position[0] - centre[0]).abs() > RECENTRE_M || (position[1] - centre[1]).abs() > RECENTRE_M;
        if moved {
            let started = std::time::Instant::now();
            self.recentre([(position[0] / WINDOW_CELL_M).round() as i64, (position[1] / WINDOW_CELL_M).round() as i64]);
            self.upload(data);
            self.recentres += 1;
            self.last_recentre_s = started.elapsed().as_secs_f64();
        }
        self.refresh(data, position);
        self.follow_rider(data, rider, moved);
        if let Some(sender) = &self.prefetch {
            let w = self.ground.to_world(position);
            let v = self.ground.to_world([position[0] + velocity[0], position[1] + velocity[1]]);
            // A gone worker only means a colder cache.
            let _ = sender.send([w[0], w[1], v[0] - w[0], v[1] - w[1]]);
        }
    }

    /// Lattice points (physics frame) of a window of `samples` per side centred on cell `centre`,
    /// row by row, where `keep(row, column)` holds.
    fn lattice(samples: usize, centre: [i64; 2], keep: impl Fn(i64, i64) -> bool) -> Vec<(usize, [f64; 2])> {
        let n = samples as i64;
        let half = (n - 1) / 2;
        let mut points = Vec::new();
        for r in 0..n {
            for c in 0..n {
                if keep(r, c) {
                    let x = (centre[0] - half + c) as f64 * WINDOW_CELL_M;
                    let y = (centre[1] - half + r) as f64 * WINDOW_CELL_M;
                    points.push(((r * n + c) as usize, [x, y]));
                }
            }
        }
        points
    }

    /// Quantized physics-frame heights at lattice points, in their order.
    fn sample(&self, points: &[(usize, [f64; 2])]) -> Vec<f64> {
        let world: Vec<[f64; 2]> = points.iter().map(|&(_, p)| self.ground.to_world(p)).collect();
        let mut out = vec![0.0; world.len()];
        self.ground.world().fill_points(&world, FACE_INSET_M, &mut out);
        out.iter().map(|&h| ((h - self.ground.origin_z()) / QUANTUM_M).round() * QUANTUM_M).collect()
    }

    /// Moves the window to cell `to`: keeps the overlap, samples only the new strips.
    fn recentre(&mut self, to: [i64; 2]) {
        let n = WINDOW_SAMPLES as i64;
        let [dx, dy] = [to[0] - self.centre[0], to[1] - self.centre[1]];
        let inside = |r: i64, c: i64| (0..n).contains(&(r + dy)) && (0..n).contains(&(c + dx));
        let fresh = Self::lattice(WINDOW_SAMPLES, to, |r, c| !inside(r, c));
        let values = self.sample(&fresh);
        let mut heights = vec![0.0; self.heights.len()];
        for r in 0..n {
            for c in 0..n {
                if inside(r, c) {
                    heights[(r * n + c) as usize] = self.heights[((r + dy) * n + c + dx) as usize];
                }
            }
        }
        for (&(index, _), value) in fresh.iter().zip(values) {
            heights[index] = value;
        }
        self.heights = heights;
        self.centre = to;
    }

    /// Writes the heights and the window's position into the model and data.
    fn upload(&mut self, data: &mut MjData<Box<MjModel>>) {
        let low = self.heights.iter().copied().fold(f64::INFINITY, f64::min);
        self.z0 = low.floor();
        let pos = [self.centre[0] as f64 * WINDOW_CELL_M, self.centre[1] as f64 * WINDOW_CELL_M, self.z0];
        // SAFETY: only height-field samples change; sizes, counts and the signature stay.
        let model = unsafe { data.model_mut() };
        let field = &mut model.hfield_data_mut()[self.hfield_adr..self.hfield_adr + self.heights.len()];
        for (value, &h) in field.iter_mut().zip(&self.heights) {
            // Exact: h − z0 is a multiple of 2⁻¹⁶ below 8 m, and the range is a power of two.
            *value = ((h - self.z0).clamp(0.0, WINDOW_RANGE_M) / WINDOW_RANGE_M) as f32;
        }
        data.mocap_pos_mut()[self.mocap] = pos;
    }

    /// Keeps a rider who has left the wheel on the ground: once its origin `rider` is
    /// [`RIDER_TAKEOVER_M`] from the window's centre, the rider's window lies under it and follows
    /// it by whole cells. The rider's window shows the ground only outside the wheel's window
    /// (from its edge on), so the two never double a contact.
    fn follow_rider(&mut self, data: &mut MjData<Box<MjModel>>, rider: Option<[f64; 2]>, window_moved: bool) {
        let centre = [self.centre[0] as f64 * WINDOW_CELL_M, self.centre[1] as f64 * WINDOW_CELL_M];
        let reach = if self.rider.centre.is_some() { RIDER_RELEASE_M } else { RIDER_TAKEOVER_M };
        let Some(p) = rider.filter(|p| (p[0] - centre[0]).abs().max((p[1] - centre[1]).abs()) > reach) else {
            if self.rider.centre.take().is_some() {
                data.mocap_pos_mut()[self.rider.mocap] = [0.0, 0.0, PARKED_Z_M];
            }
            return;
        };
        let drifted = self.rider.centre.is_none_or(|c| {
            (p[0] - c[0] as f64 * WINDOW_CELL_M).abs() > RIDER_RECENTRE_M
                || (p[1] - c[1] as f64 * WINDOW_CELL_M).abs() > RIDER_RECENTRE_M
        });
        if drifted {
            let cell = [(p[0] / WINDOW_CELL_M).round() as i64, (p[1] / WINDOW_CELL_M).round() as i64];
            self.rider.heights = self.sample(&Self::lattice(RIDER_WINDOW_SAMPLES, cell, |_, _| true));
            self.rider.centre = Some(cell);
        }
        if drifted || window_moved {
            self.upload_rider(data);
        }
    }

    /// Writes the rider's window into the model and data: its samples inside the wheel's window
    /// (that window's edge excluded) sit at its floor, below the wheel's window's ground.
    fn upload_rider(&self, data: &mut MjData<Box<MjModel>>) {
        let Some(cell) = self.rider.centre else { return };
        let side = RIDER_WINDOW_SAMPLES as i64;
        let hidden = |k: usize| {
            let (row, column) = (k as i64 / side, k as i64 % side);
            let dx = cell[0] - RIDER_HALF_CELLS + column - self.centre[0];
            let dy = cell[1] - RIDER_HALF_CELLS + row - self.centre[1];
            dx.abs() < HALF_CELLS && dy.abs() < HALF_CELLS
        };
        let low = (0..self.rider.heights.len())
            .filter(|&k| !hidden(k))
            .map(|k| self.rider.heights[k])
            .fold(f64::INFINITY, f64::min);
        if !low.is_finite() {
            // The wheel's window covers all of it.
            data.mocap_pos_mut()[self.rider.mocap] = [0.0, 0.0, PARKED_Z_M];
            return;
        }
        let z0 = low.floor();
        // SAFETY: only height-field samples change; sizes, counts and the signature stay.
        let model = unsafe { data.model_mut() };
        let adr = self.rider.hfield_adr;
        let field = &mut model.hfield_data_mut()[adr..adr + self.rider.heights.len()];
        for (k, (value, &h)) in field.iter_mut().zip(&self.rider.heights).enumerate() {
            // Exact, as in `upload`.
            *value = if hidden(k) { 0.0 } else { ((h - z0).clamp(0.0, WINDOW_RANGE_M) / WINDOW_RANGE_M) as f32 };
        }
        data.mocap_pos_mut()[self.rider.mocap] = [cell[0] as f64 * WINDOW_CELL_M, cell[1] as f64 * WINDOW_CELL_M, z0];
    }

    /// Fills the pool with the solids near `position` and sets the tire friction for the surface
    /// under it.
    fn refresh(&mut self, data: &mut MjData<Box<MjModel>>, position: [f64; 2]) {
        let reach = POOL_REACH_M;
        let mut rect = Rect::empty();
        for corner in [[-reach, -reach], [reach, -reach], [-reach, reach], [reach, reach]] {
            rect.include(self.ground.to_world([position[0] + corner[0], position[1] + corner[1]]));
        }
        let anchor = self.ground.to_world(position);
        let mut near: Vec<(f64, Solid)> = self
            .ground
            .world()
            .solids_in(rect)
            .into_iter()
            .map(|s| (s.bounds().distance(anchor), s))
            .filter(|(d, _)| *d <= reach)
            .collect();
        near.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.id.cmp(&b.1.id)));
        near.truncate(POOL_SIZE);
        let wanted: BTreeMap<u64, Solid> = near.into_iter().map(|(_, s)| (s.id, s)).collect();
        // Keep the slots of solids still in reach; park the rest.
        for slot in &mut self.slots {
            if slot.solid.is_some_and(|id| !wanted.contains_key(&id)) {
                slot.solid = None;
                data.mocap_pos_mut()[slot.mocap] = [0.0, 0.0, PARKED_Z_M];
            }
        }
        let placed: Vec<u64> = self.slots.iter().filter_map(|s| s.solid).collect();
        let free: Vec<usize> = (0..self.slots.len()).filter(|&k| self.slots[k].solid.is_none()).collect();
        let mut free = free.into_iter();
        for (id, solid) in wanted.iter().filter(|(id, _)| !placed.contains(id)) {
            let Some(k) = free.next() else { break };
            self.slots[k].solid = Some(*id);
            self.place(data, k, solid);
        }
        let surface = self.ground.surface(position);
        if surface != self.surface || self.steps == 0 {
            self.surface = surface;
            let asphalt = Surface::of(Material::Asphalt, false);
            let grip = surface.friction / asphalt.friction;
            let roll = surface.rolling_resistance / asphalt.rolling_resistance;
            let [mu, spin, rolling] = self.tire_friction;
            // SAFETY: friction coefficients only.
            let model = unsafe { data.model_mut() };
            for &p in &self.tires {
                model.pair_friction_mut()[p] = [mu * grip, mu * grip, spin, rolling * roll, rolling * roll];
            }
        }
    }

    /// Moves pool slot `k` onto `solid`.
    fn place(&self, data: &mut MjData<Box<MjModel>>, k: usize, solid: &Solid) {
        let slot = self.slots[k];
        let [c, s] = self.ground.rotation;
        let rotate = |v: [f64; 3]| [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]];
        let axes = solid.axes.map(rotate);
        let centre = self.ground.to_physics([solid.centre[0], solid.centre[1]]);
        data.mocap_pos_mut()[slot.mocap] = [centre[0], centre[1], solid.centre[2] - self.ground.origin_z()];
        data.mocap_quat_mut()[slot.mocap] = Solid { axes, ..*solid }.quaternion();
        // SAFETY: a box's size and its bounds, kept consistent; the signature stays.
        let model = unsafe { data.model_mut() };
        model.geom_size_mut()[slot.geom] = solid.half;
        model.geom_rbound_mut()[slot.geom] = solid.half.iter().map(|h| h * h).sum::<f64>().sqrt();
        model.geom_aabb_mut()[slot.geom] = [0.0, 0.0, 0.0, solid.half[0], solid.half[1], solid.half[2]];
    }

    /// Solids in the pool now (physics frame), for checks and drawing.
    pub(crate) fn pooled(&self) -> usize {
        self.slots.iter().filter(|s| s.solid.is_some()).count()
    }
}

impl Drop for Window {
    fn drop(&mut self) {
        // Closing the channel ends the worker.
        self.prefetch.take();
    }
}

/// Keeps chunks generated ahead of the wheel: within 40 m of where it will be in 1.5 s.
fn prefetch(world: &World, hints: &mpsc::Receiver<[f64; 4]>) {
    while let Ok(mut hint) = hints.recv() {
        // Only the newest hint matters.
        while let Ok(newer) = hints.try_recv() {
            hint = newer;
        }
        let ahead = [hint[0] + hint[2] * 1.5, hint[1] + hint[3] * 1.5];
        let rect =
            Rect::new([ahead[0].min(hint[0]), ahead[1].min(hint[1])], [ahead[0].max(hint[0]), ahead[1].max(hint[1])])
                .grow(40.0);
        for key in euc_world::keys_in(rect) {
            world.chunk(key);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_of_identity_and_quarter_turn() {
        let solid = |axes| Solid { id: 0, kind: euc_world::SolidKind::Curb, centre: [0.0; 3], half: [1.0; 3], axes };
        let id = solid([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]).quaternion();
        assert!((id[0] - 1.0).abs() < 1e-12 && id[1..].iter().all(|v| v.abs() < 1e-12));
        // x → y: a quarter turn about z.
        let q = solid([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]).quaternion();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!((q[0] - h).abs() < 1e-12 && (q[3] - h).abs() < 1e-12, "{q:?}");
    }

    /// The Begode Extreme without its suspension on world seed 7 (as tests/world.rs).
    fn setup(model: crate::RiderModel) -> crate::Setup {
        use crate::{Shapes, Shell, Tire, Torus, Wheel, WheelBody};
        let mut setup = crate::Setup::new(WheelBody {
            tire: Tire { radius_m: 0.2251, half_width_m: 0.0407, friction: 0.9, crr: 0.012, contact_time_s: 0.02 },
            wheel: Wheel { mass_kg: 16.0, spin_inertia_kgm2: 0.487 },
            shell: Shell {
                mass_kg: 26.0,
                com_above_axle_m: 0.10,
                inertia_kgm2: [0.85, 0.92, 0.46],
                pedal_below_axle_m: -0.012,
                imu_above_axle_m: 0.15,
            },
            suspension: None,
            shapes: Some(Shapes { tire: Some(Torus { major_m: 0.1716, minor_m: 0.058 }), ..Shapes::default() }),
        });
        setup.rider.model = model;
        setup.world = Some(WorldGround::at_spawn(WorldSpec::city(7)).unwrap());
        setup
    }

    /// A rider who has left the wheel stays on the ground when the wheel goes on far beyond the
    /// window's reach: the rider's window takes over under it.
    #[test]
    fn a_rider_off_the_wheel_stays_on_the_ground_beyond_the_window() {
        use crate::RiderModel;
        let steps = |s: f64| (s / crate::TIMESTEP_S).round() as usize;
        for model in [RiderModel::RigidAnkleV1, RiderModel::ArticulatedSagittalV1, RiderModel::ArticulatedLateralV1] {
            let mut physics = crate::Physics::new(setup(model)).unwrap();
            let ground = physics.world_ground().unwrap().clone();
            for _ in 0..steps(0.5) {
                physics.step(0.0);
            }
            physics.step_off();
            for _ in 0..steps(1.5) {
                physics.step(0.0);
            }
            // The wheel goes: 9 m ahead at once, beyond the window's reach from the rider.
            physics.set_held(false);
            let q = physics.ids.root_qpos;
            physics.data.qpos_mut()[q] += 9.0;
            let mut covered = false;
            for _ in 0..steps(3.0) {
                physics.step(0.0);
                let r = physics.rider_position_m();
                let below = ground.height_m([r[0], r[1]]);
                assert!(r[2] > below, "{model:?}: the rider at {r:?} is below the ground ({below:.3} m)");
                covered |= physics.world.as_ref().is_some_and(|w| w.rider.centre.is_some());
            }
            assert!(covered, "{model:?}: the rider's window never took over");
        }
    }

    #[test]
    fn frames_round_trip() {
        let g = WorldGround::new(WorldSpec::city(3), WorldStart { xy_m: [40.0, -12.0], heading_rad: 0.7 }).unwrap();
        let p = [3.25, -1.5];
        let back = g.to_physics(g.to_world(p));
        assert!((back[0] - p[0]).abs() < 1e-12 && (back[1] - p[1]).abs() < 1e-12);
        assert!(g.height_m([0.0, 0.0]).abs() < 1e-12);
    }
}

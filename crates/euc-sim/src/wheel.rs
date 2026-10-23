//! A wheel's files, loaded and checked: profile, board, stock map and plant.

use euc_physics::{Assist, Physics, PhysicsError, Rider, Setup, Shapes, Solid, Terrain, Torus, WheelBody};
use euc_plant::{ElectricalPlant, PlantError, PlantParams};
use euc_profile::{ProfileError, ProfileSource, WheelProfile};
use thiserror::Error;

use crate::config::{BoardFile, StockMap};
use crate::plant::Live;
use crate::probes::ProbesFile;
use crate::settings::SettingsFile;

/// Everything the simulator reads from `profiles/<wheel>/`.
#[derive(Debug, Clone)]
pub struct WheelFiles {
    /// `wheel.toml`.
    pub profile: WheelProfile,
    /// `board.toml`.
    pub board: BoardFile,
    /// `stockmap.toml`.
    pub stockmap: StockMap,
    /// `plant.toml` without `[body]`: the electrical plant.
    pub electrical: PlantParams,
    /// `[body]` of `plant.toml`: the mechanics.
    pub body: WheelBody,
    /// `settings.toml`, if the wheel has settings presets.
    pub settings: Option<SettingsFile>,
    /// `probes.toml`, if the wheel names places in our firmware.
    pub probes: Option<ProbesFile>,
}

/// A wheel's files could not be used.
#[derive(Debug, Error)]
pub enum LoadError {
    /// The profile or one of its files is missing.
    #[error(transparent)]
    Profile(#[from] ProfileError),
    /// A file does not parse or check.
    #[error("{file}: {why}")]
    File {
        /// Which file.
        file: &'static str,
        /// What is wrong.
        why: String,
    },
}

/// A live plant could not be built.
#[derive(Debug, Error)]
pub enum LiveError {
    /// The electrical plant.
    #[error(transparent)]
    Plant(#[from] PlantError),
    /// The body physics.
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    /// The scenario's `[world]`.
    #[error("world: {0}")]
    World(String),
}

impl WheelFiles {
    /// Loads wheel `id` from `source`.
    pub fn load(source: &ProfileSource, id: &str) -> Result<Self, LoadError> {
        let profile = source.load(id)?;
        let parse = |file: &'static str| -> Result<String, LoadError> { Ok(source.read(&profile.id, file)?) };
        let board = toml::from_str(&parse("board.toml")?)
            .map_err(|e: toml::de::Error| LoadError::File { file: "board.toml", why: e.to_string() })?;
        let stockmap = toml::from_str(&parse("stockmap.toml")?)
            .map_err(|e: toml::de::Error| LoadError::File { file: "stockmap.toml", why: e.to_string() })?;
        let (electrical, mut body) =
            split_plant(&parse("plant.toml")?).map_err(|why| LoadError::File { file: "plant.toml", why })?;
        body.shapes = match source.read(&profile.id, "geometry.toml") {
            Ok(text) => collision_shapes(&text).map_err(|why| LoadError::File { file: "geometry.toml", why })?,
            Err(ProfileError::UnknownFile { .. }) => None,
            Err(error) => return Err(error.into()),
        };
        let settings = match source.read(&profile.id, "settings.toml") {
            Ok(text) => {
                Some(SettingsFile::from_toml(&text).map_err(|why| LoadError::File { file: "settings.toml", why })?)
            }
            Err(ProfileError::UnknownFile { .. }) => None,
            Err(error) => return Err(error.into()),
        };
        let probes = match source.read(&profile.id, "probes.toml") {
            Ok(text) => Some(ProbesFile::from_toml(&text).map_err(|why| LoadError::File { file: "probes.toml", why })?),
            Err(ProfileError::UnknownFile { .. }) => None,
            Err(error) => return Err(error.into()),
        };
        Ok(Self { profile, board, stockmap, electrical, body, settings, probes })
    }

    /// The board with settings preset `name` seeded into the settings journal (power-on state).
    pub fn board_with_settings(&self, name: &str) -> Result<BoardFile, LoadError> {
        let file = |why: String| LoadError::File { file: "settings.toml", why };
        let settings =
            self.settings.as_ref().ok_or_else(|| file(format!("{} has no settings presets", self.profile.id)))?;
        let mut board = self.board.clone();
        settings.seed(&mut board, settings.preset(name).map_err(file)?).map_err(file)?;
        Ok(board)
    }

    /// Core clock in Hz.
    pub fn hclk(&self) -> u64 {
        u64::from(self.profile.board.hclk_hz)
    }

    /// A live plant for this wheel at cycle 0, with `rider` and `assist`, on its stand on flat
    /// ground.
    pub fn live(&self, rider: Rider, assist: Assist) -> Result<Live, LiveError> {
        self.live_on(rider, assist, Terrain::flat())
    }

    /// [`WheelFiles::live`] on `terrain` (a scenario's `[terrain]`).
    pub fn live_on(&self, rider: Rider, assist: Assist, terrain: Terrain) -> Result<Live, LiveError> {
        let electrical = ElectricalPlant::new(self.electrical.clone(), self.hclk())?;
        let physics = Physics::new(Setup { rider, assist, terrain, ..Setup::new(self.body.clone()) })?;
        Ok(Live::new(electrical, physics, self.hclk()))
    }

    /// A live plant for `scenario`: on its `[world]` when it has one, else on its `[terrain]`.
    pub fn live_for(&self, scenario: &crate::scenario::Scenario) -> Result<Live, LiveError> {
        match &scenario.world {
            Some(world) => {
                let ground = world.ground().map_err(LiveError::World)?;
                self.live_on_world(scenario.rider, scenario.assist, ground)
            }
            None => self.live_on(scenario.rider, scenario.assist, scenario.terrain.clone()),
        }
    }

    /// A live plant on a streaming world (`docs/models/world.md`).
    pub fn live_on_world(
        &self,
        rider: Rider,
        assist: Assist,
        ground: euc_physics::WorldGround,
    ) -> Result<Live, LiveError> {
        let electrical = ElectricalPlant::new(self.electrical.clone(), self.hclk())?;
        let physics = Physics::new(Setup { rider, assist, world: Some(ground), ..Setup::new(self.body.clone()) })?;
        Ok(Live::new(electrical, physics, self.hclk()))
    }

    /// A live plant on a validated finite generated map. Existing scenario profiles stay unchanged.
    pub fn live_on_map(&self, rider: Rider, assist: Assist, map: euc_physics::HeightMap) -> Result<Live, LiveError> {
        let electrical = ElectricalPlant::new(self.electrical.clone(), self.hclk())?;
        let physics = Physics::new(Setup { rider, assist, height_map: Some(map), ..Setup::new(self.body.clone()) })?;
        Ok(Live::new(electrical, physics, self.hclk()))
    }
}

/// The measured collision shapes of `geometry.toml` (`[collision]`), if it has them: `tire_torus`
/// (`major_m`, `minor_m`), boxes (`centre`, `half`) and capsules (`from`, `to`, `radius`). Other
/// `tire_*` entries are alternatives for people and are skipped.
pub fn collision_shapes(text: &str) -> Result<Option<Shapes>, String> {
    let table: toml::Table = toml::from_str(text).map_err(|e| e.to_string())?;
    let Some(collision) = table.get("collision") else { return Ok(None) };
    let collision = collision.as_table().ok_or("[collision] is not a table")?;
    let vector = |value: Option<&toml::Value>, what: &str| -> Result<[f64; 3], String> {
        let items = value.and_then(toml::Value::as_array).ok_or(format!("{what} must be [x, y, z]"))?;
        let numbers: Vec<f64> =
            items.iter().filter_map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64))).collect();
        <[f64; 3]>::try_from(numbers).map_err(|_| format!("{what} must be [x, y, z]"))
    };
    let number = |value: Option<&toml::Value>, what: &str| -> Result<f64, String> {
        value
            .and_then(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)))
            .ok_or(format!("{what} must be a number"))
    };
    let mut shapes = Shapes::default();
    for (name, value) in collision {
        let entry = value.as_table().ok_or(format!("[collision] {name} is not a table"))?;
        let at = |key: &str| format!("[collision] {name}.{key}");
        if name == "tire_torus" {
            shapes.tire = Some(Torus {
                major_m: number(entry.get("major_m"), &at("major_m"))?,
                minor_m: number(entry.get("minor_m"), &at("minor_m"))?,
            });
        } else if name.starts_with("tire_") {
            // An alternative tire shape.
        } else if entry.contains_key("half") {
            let (centre, half) = (vector(entry.get("centre"), &at("centre"))?, vector(entry.get("half"), &at("half"))?);
            shapes.solids.push(Solid::Box { name: name.clone(), centre, half });
        } else if entry.contains_key("radius") {
            shapes.solids.push(Solid::Capsule {
                name: name.clone(),
                from: vector(entry.get("from"), &at("from"))?,
                to: vector(entry.get("to"), &at("to"))?,
                radius: number(entry.get("radius"), &at("radius"))?,
            });
        } else {
            return Err(format!("[collision] {name}: neither a box (centre, half) nor a capsule (from, to, radius)"));
        }
    }
    if let Some(covers) = table.get("pad_covers") {
        shapes.pads = pad_faces(covers)?;
    }
    Ok(Some(shapes))
}

/// How much further out the right pad cover sits than the left one, metres: each cover sits on its
/// own side's packs, and the right packs' outer face is 1.0 mm further out (`geometry.toml`
/// `[pad_covers]`, which gives the left cover).
const RIGHT_COVER_OFFSET_M: f64 = 0.001;

/// The faces of the side pad covers where a leg can rest (`[pad_covers] contact_faces`), as boxes
/// from the packs' outer face (`inner_y_m`) out to each face, left and mirrored right.
fn pad_faces(covers: &toml::Value) -> Result<Vec<Solid>, String> {
    let number = |value: Option<&toml::Value>, what: &str| -> Result<f64, String> {
        value
            .and_then(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)))
            .ok_or(format!("[pad_covers] {what} must be a number"))
    };
    let range = |value: Option<&toml::Value>, what: &str| -> Result<[f64; 2], String> {
        let items = value.and_then(toml::Value::as_array).ok_or(format!("[pad_covers] {what} must be [low, high]"))?;
        let numbers: Vec<f64> =
            items.iter().filter_map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64))).collect();
        match <[f64; 2]>::try_from(numbers) {
            Ok([low, high]) if low < high => Ok([low, high]),
            _ => Err(format!("[pad_covers] {what} must be [low, high]")),
        }
    };
    let Some(faces) = covers.get("contact_faces") else { return Ok(Vec::new()) };
    let faces = faces.as_array().ok_or("[pad_covers] contact_faces must be a list")?;
    let inner = number(covers.get("inner_y_m"), "inner_y_m")?;
    let mut pads = Vec::new();
    for face in faces {
        let name = face.get("name").and_then(toml::Value::as_str).ok_or("[pad_covers] a contact face needs a name")?;
        let ([x0, x1], [z0, z1]) =
            (range(face.get("x_m"), &format!("{name}.x_m"))?, range(face.get("z_m"), &format!("{name}.z_m"))?);
        let outer = number(face.get("y_m"), &format!("{name}.y_m"))?;
        if outer <= inner {
            return Err(format!("[pad_covers] {name}.y_m must stand out from inner_y_m"));
        }
        for (side, sign, offset) in [("left", 1.0, 0.0), ("right", -1.0, RIGHT_COVER_OFFSET_M)] {
            pads.push(Solid::Box {
                name: format!("pad_{name}_{side}"),
                centre: [f64::midpoint(x0, x1), sign * (f64::midpoint(inner, outer) + offset), f64::midpoint(z0, z1)],
                half: [(x1 - x0) / 2.0, (outer - inner) / 2.0, (z1 - z0) / 2.0],
            });
        }
    }
    Ok(pads)
}

/// Splits `plant.toml` into the electrical plant and the `[body]` mechanics.
pub fn split_plant(text: &str) -> Result<(PlantParams, WheelBody), String> {
    let mut table: toml::Table = toml::from_str(text).map_err(|e| e.to_string())?;
    let body = table.remove("body").ok_or("no [body] tables (the mechanics)")?;
    let body: WheelBody = body.try_into().map_err(|e: toml::de::Error| format!("[body]: {e}"))?;
    let electrical: PlantParams = toml::Value::Table(table).try_into().map_err(|e: toml::de::Error| e.to_string())?;
    electrical.validate().map_err(|e| e.to_string())?;
    Ok((electrical, body))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_begode_extreme_loads_and_builds_a_live_plant() {
        let files = WheelFiles::load(&ProfileSource::Builtin, "begode-extreme").unwrap();
        // The firmware counts 0.0505 m per electrical turn: with this tire and motor it shows 1.107
        // times the true speed, as the owner's ride log does (GPS 0.905 of the display).
        let pole_pairs = f64::from(files.electrical.motor.pole_pairs);
        let display = 0.0505 * pole_pairs / (files.body.tire.radius_m * std::f64::consts::TAU);
        assert!((display - 1.0 / 0.905).abs() < 0.005, "{display}");
        let shapes = files.body.shapes.as_ref().expect("geometry.toml [collision]");
        assert!(shapes.tire.is_some() && shapes.solids.iter().any(|s| s.name() == "pedal_left"));
        let live = files.live(Rider::default(), Assist::default()).unwrap();
        assert!(live.physics().held());
        assert!(files.board_with_settings("neutral").is_ok());
        assert!(files.board_with_settings("nope").unwrap_err().to_string().contains("no settings preset"));
    }

    #[test]
    fn collision_shapes_parse_and_refuse_strangers() {
        let text = "[collision]\ntire_torus = { major_m = 0.17, minor_m = 0.05 }\ntire_alt = { major_m = 1, minor_m = 1 }\n\
                    pack = { centre = [0, 0.07, 0.16], half = [0.12, 0.03, 0.2] }\n\
                    roller = { from = [0, -0.07, 0.45], to = [0, 0.07, 0.45], radius = 0.026 }\n";
        let shapes = collision_shapes(text).unwrap().unwrap();
        assert_eq!(shapes.tire, Some(Torus { major_m: 0.17, minor_m: 0.05 }));
        assert_eq!(shapes.solids.len(), 2);
        assert!(collision_shapes("[collision]\nblob = { size = 1 }").unwrap_err().contains("blob"));
        assert_eq!(collision_shapes("schema = 1").unwrap(), None);
        // Pad faces become boxes from the packs out to each face, mirrored and 1 mm further out
        // on the right.
        let pads = collision_shapes(
            "[collision]\ntire_torus = { major_m = 0.17, minor_m = 0.05 }\n[pad_covers]\ninner_y_m = 0.10\n\
             contact_faces = [{ name = \"arm\", x_m = [-0.1, 0.1], z_m = [0.2, 0.3], y_m = 0.12 }]",
        )
        .unwrap()
        .unwrap()
        .pads;
        assert_eq!(pads.len(), 2);
        let Solid::Box { name, centre, half } = &pads[1] else { panic!("{pads:?}") };
        assert_eq!(name, "pad_arm_right");
        assert!(
            (centre[1] + 0.111).abs() < 1e-12 && (half[1] - 0.01).abs() < 1e-12 && (centre[2] - 0.25).abs() < 1e-12
        );
        assert!(collision_shapes("[collision]\n[pad_covers]\ninner_y_m = 0.1\ncontact_faces = [{ name = \"a\", x_m = [0, 1], z_m = [0, 1], y_m = 0.05 }]").is_err());
    }

    #[test]
    fn plant_toml_needs_its_body() {
        let error = split_plant("schema = 1").unwrap_err();
        assert!(error.contains("[body]"), "{error}");
    }
}

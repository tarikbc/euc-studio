//! A wheel's files, loaded and checked: profile, board, stock map and plant.

use euc_physics::{Assist, Physics, PhysicsError, Rider, Setup, WheelBody};
use euc_plant::{ElectricalPlant, PlantError, PlantParams};
use euc_profile::{ProfileError, ProfileSource, WheelProfile};
use thiserror::Error;

use crate::config::{BoardFile, StockMap};
use crate::plant::Live;

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
        let (electrical, body) =
            split_plant(&parse("plant.toml")?).map_err(|why| LoadError::File { file: "plant.toml", why })?;
        Ok(Self { profile, board, stockmap, electrical, body })
    }

    /// Core clock in Hz.
    pub fn hclk(&self) -> u64 {
        u64::from(self.profile.board.hclk_hz)
    }

    /// A live plant for this wheel at cycle 0, with `rider` and `assist`, on its stand.
    pub fn live(&self, rider: Rider, assist: Assist) -> Result<Live, LiveError> {
        let electrical = ElectricalPlant::new(self.electrical.clone(), self.hclk())?;
        let physics = Physics::new(Setup { rider, assist, ..Setup::new(self.body.clone()) })?;
        Ok(Live::new(electrical, physics, self.hclk()))
    }
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
        assert!((files.body.tire.radius_m * std::f64::consts::TAU - 1.313).abs() < 0.001, "{:?}", files.body.tire);
        let live = files.live(Rider::default(), Assist::default()).unwrap();
        assert!(live.physics().held());
    }

    #[test]
    fn plant_toml_needs_its_body() {
        let error = split_plant("schema = 1").unwrap_err();
        assert!(error.contains("[body]"), "{error}");
    }
}

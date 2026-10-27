//! The repo's wheel profiles, compiled into the binary.

use std::path::PathBuf;

use crate::dir::from_text;
use crate::{Origin, ProfileError, WHEEL_FILE, WheelProfile};

/// `(wheel id, file name, text)` for every file in the repo's `profiles/<id>/`, sorted: the wheels,
/// and the radio-only brand profiles (a `ble.toml` alone, for `euc replay`).
///
/// Embedding them means a plain `euc fetch` trusts only pins that were reviewed and built with
/// the code, never a `profiles/` folder it happens to find on disk. A new wheel or file is added
/// here (the tests below fail until it is).
const FILES: &[(&str, &str, &str)] = &[
    ("begode-extreme", "ble.toml", include_str!("../../../profiles/begode-extreme/ble.toml")),
    ("begode-extreme", "board.toml", include_str!("../../../profiles/begode-extreme/board.toml")),
    ("begode-extreme", "display.toml", include_str!("../../../profiles/begode-extreme/display.toml")),
    ("begode-extreme", "geometry.toml", include_str!("../../../profiles/begode-extreme/geometry.toml")),
    ("begode-extreme", "lights.toml", include_str!("../../../profiles/begode-extreme/lights.toml")),
    ("begode-extreme", "plant.toml", include_str!("../../../profiles/begode-extreme/plant.toml")),
    ("begode-extreme", "probes.toml", include_str!("../../../profiles/begode-extreme/probes.toml")),
    ("begode-extreme", "settings.toml", include_str!("../../../profiles/begode-extreme/settings.toml")),
    ("begode-extreme", "stockmap.toml", include_str!("../../../profiles/begode-extreme/stockmap.toml")),
    ("begode-extreme", "wheel.toml", include_str!("../../../profiles/begode-extreme/wheel.toml")),
    // Radio-only profiles: a brand's BLE module for `euc replay`, without a simulated wheel.
    ("inmotion-v2", "ble.toml", include_str!("../../../profiles/inmotion-v2/ble.toml")),
    ("kingsong", "ble.toml", include_str!("../../../profiles/kingsong/ble.toml")),
    ("leaperkim", "ble.toml", include_str!("../../../profiles/leaperkim/ble.toml")),
];

pub(crate) fn ids() -> Vec<String> {
    FILES.iter().filter(|(_, file, _)| *file == WHEEL_FILE).map(|(id, _, _)| (*id).to_owned()).collect()
}

pub(crate) fn read(id: &str, file: &str) -> Result<&'static str, ProfileError> {
    FILES.iter().find(|(i, f, _)| *i == id && *f == file).map(|(_, _, text)| *text).ok_or_else(|| {
        ProfileError::UnknownFile { id: id.to_owned(), file: file.to_owned(), searched: Origin::Builtin }
    })
}

pub(crate) fn load(id: &str) -> Result<WheelProfile, ProfileError> {
    if !ids().iter().any(|known| known == id) {
        return Err(ProfileError::UnknownWheel { id: id.to_owned(), searched: Origin::Builtin, available: ids() });
    }
    let text = read(id, WHEEL_FILE)?;
    from_text(text, id, PathBuf::from(format!("built-in:{id}/{WHEEL_FILE}")), Origin::Builtin)
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::Path;

    use super::*;

    fn repo_profiles() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../profiles")
    }

    #[test]
    fn every_builtin_profile_loads() {
        for id in ids() {
            let wheel = load(&id).unwrap();
            assert_eq!((wheel.id.as_str(), &wheel.origin), (id.as_str(), &Origin::Builtin));
        }
        let mut sorted = FILES.to_vec();
        sorted.sort_unstable();
        assert_eq!(FILES, sorted, "keep FILES sorted");
    }

    /// The binary must carry exactly what is on disk: a profile file edited, added or removed
    /// without rebuilding (or without registering it in `FILES`) fails here. Every folder counts,
    /// wheels and radio-only brand profiles alike.
    #[test]
    fn embedded_profiles_match_the_repo() {
        let mut on_disk = Vec::new();
        for folder in fs::read_dir(repo_profiles()).unwrap() {
            let folder = folder.unwrap().path();
            if !folder.is_dir() {
                continue;
            }
            let id = folder.file_name().unwrap().to_string_lossy().into_owned();
            for entry in fs::read_dir(&folder).unwrap() {
                let path = entry.unwrap().path();
                if path.extension().is_some_and(|ext| ext == "toml") {
                    let name = path.file_name().unwrap().to_string_lossy().into_owned();
                    on_disk.push((id.clone(), name, fs::read_to_string(&path).unwrap()));
                }
            }
        }
        on_disk.sort();
        let embedded: Vec<_> =
            FILES.iter().map(|(i, f, t)| ((*i).to_owned(), (*f).to_owned(), (*t).to_owned())).collect();
        assert_eq!(
            embedded, on_disk,
            "profiles/ differs from the built-in copy; rebuild or register the file in FILES"
        );
    }

    #[test]
    fn unknown_wheel_or_file_names_the_built_in_set() {
        let message = load("nope").unwrap_err().to_string();
        assert_eq!(message, "no wheel profile \"nope\" in the built-in profiles (available: begode-extreme)");
        let message = read("begode-extreme", "nope.toml").unwrap_err().to_string();
        assert_eq!(message, "no file nope.toml for wheel \"begode-extreme\" in the built-in profiles");
    }
}

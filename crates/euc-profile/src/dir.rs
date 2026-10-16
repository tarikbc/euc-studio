use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use thiserror::Error;

use crate::{Origin, ParseError, WheelProfile};

/// File name of a wheel profile inside its folder.
pub const WHEEL_FILE: &str = "wheel.toml";

/// Why a profile could not be loaded from disk.
#[derive(Debug, Error)]
pub enum ProfileError {
    /// The file or folder could not be read.
    #[error("cannot read {}: {error}", path.display())]
    Io {
        /// What we tried to read.
        path: PathBuf,
        /// The OS error (shown in the message, so not repeated as the error source).
        error: io::Error,
    },
    /// The file is not a valid profile.
    #[error("{}: {error}", path.display())]
    Parse {
        /// The profile file.
        path: PathBuf,
        /// What is wrong with it (shown in the message, so not repeated as the error source).
        error: ParseError,
    },
    /// The profile's `id` differs from its folder name, so `euc fetch <id>` would not find it.
    #[error("{}: id \"{id}\" must match its folder name \"{folder}\"", path.display())]
    IdMismatch {
        /// The profile file.
        path: PathBuf,
        /// The `id` field.
        id: String,
        /// The folder name.
        folder: String,
    },
    /// The wheel's profile has no file of that name.
    #[error("no file {file} for wheel \"{id}\" in {searched}")]
    UnknownFile {
        /// The wheel id.
        id: String,
        /// The file asked for.
        file: String,
        /// The source searched.
        searched: Origin,
    },
    /// The source has no profile for the requested wheel.
    #[error("no wheel profile \"{id}\" in {searched} (available: {})", list_or_none(available))]
    UnknownWheel {
        /// The requested id.
        id: String,
        /// The source searched.
        searched: Origin,
        /// Ids that do exist.
        available: Vec<String>,
    },
}

fn list_or_none(ids: &[String]) -> String {
    if ids.is_empty() { "none".to_owned() } else { ids.join(", ") }
}

/// The `profiles/` folder: one sub-folder per wheel, each with a [`WHEEL_FILE`].
#[derive(Debug, Clone)]
pub struct ProfileDir {
    root: PathBuf,
}

impl ProfileDir {
    /// A profiles folder at `root`. Nothing is read until a method asks for it.
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }

    /// The folder this reads from.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Ids of every folder that holds a [`WHEEL_FILE`], sorted.
    pub fn ids(&self) -> Result<Vec<String>, ProfileError> {
        let io_error = |error| ProfileError::Io { path: self.root.clone(), error };
        let mut ids = Vec::new();
        for entry in fs::read_dir(&self.root).map_err(io_error)? {
            let entry = entry.map_err(io_error)?;
            if entry.path().join(WHEEL_FILE).is_file() {
                ids.push(entry.file_name().to_string_lossy().into_owned());
            }
        }
        ids.sort();
        Ok(ids)
    }

    /// Reads another file of wheel `id`'s profile folder (`board.toml`, `stockmap.toml`).
    pub fn read(&self, id: &str, file: &str) -> Result<String, ProfileError> {
        let path = self.root.join(id).join(file);
        let unknown = || ProfileError::UnknownFile {
            id: id.to_owned(),
            file: file.to_owned(),
            searched: Origin::Override(self.root.clone()),
        };
        if !is_plain(id) || !is_plain_file(file) || !path.is_file() {
            return Err(unknown());
        }
        fs::read_to_string(&path).map_err(|error| ProfileError::Io { path, error })
    }

    /// Loads and validates the profile of wheel `id`. It carries [`Origin::Override`].
    pub fn load(&self, id: &str) -> Result<WheelProfile, ProfileError> {
        let origin = Origin::Override(self.root.clone());
        let path = self.root.join(id).join(WHEEL_FILE);
        if !is_plain(id) || !path.is_file() {
            return Err(ProfileError::UnknownWheel {
                id: id.to_owned(),
                searched: origin,
                available: self.ids().unwrap_or_default(),
            });
        }
        let text = fs::read_to_string(&path).map_err(|error| ProfileError::Io { path: path.clone(), error })?;
        from_text(&text, id, path, origin)
    }
}

/// Ids are kebab case; anything else (`..`, separators) never becomes a path.
fn is_plain(id: &str) -> bool {
    !id.is_empty() && id.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
}

/// A file name inside a profile folder: no separators, no leading dot.
fn is_plain_file(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with('.')
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.'))
}

/// Parses a profile that must have id `id`, and stamps it with `origin`.
/// `path` names the file in error messages.
pub(crate) fn from_text(text: &str, id: &str, path: PathBuf, origin: Origin) -> Result<WheelProfile, ProfileError> {
    let mut profile = WheelProfile::parse(text).map_err(|error| ProfileError::Parse { path: path.clone(), error })?;
    if profile.id != id {
        return Err(ProfileError::IdMismatch { path, id: profile.id, folder: id.to_owned() });
    }
    profile.origin = origin;
    Ok(profile)
}

#[cfg(test)]
mod tests {
    use super::*;

    const WHEEL: &str = r#"
schema = 1
id = "demo"
name = "Demo"
brand = "Demo"

[board]
id = "demo"
mcu = "STM32F103RC-class"
core = "Cortex-M3"
hclk_hz = 72_000_000
app = { base = 0x0800_3000, max_size = 49_152, vectors = 76, info_offset = 0x140 }
ram = { base = 0x2000_0000, size = 0xC000 }
"#;

    fn write_wheel(root: &Path, folder: &str, text: &str) {
        fs::create_dir_all(root.join(folder)).unwrap();
        fs::write(root.join(folder).join(WHEEL_FILE), text).unwrap();
    }

    #[test]
    fn loads_by_folder_name() {
        let tmp = tempfile::tempdir().unwrap();
        write_wheel(tmp.path(), "demo", WHEEL);
        fs::create_dir(tmp.path().join("riders")).unwrap();

        let dir = ProfileDir::new(tmp.path());
        assert_eq!(dir.ids().unwrap(), ["demo"], "folders without wheel.toml are not wheels");
        let wheel = dir.load("demo").unwrap();
        assert_eq!(wheel.name, "Demo");
        assert_eq!(wheel.origin, Origin::Override(tmp.path().to_owned()));
    }

    #[test]
    fn unknown_wheel_lists_what_exists() {
        let tmp = tempfile::tempdir().unwrap();
        write_wheel(tmp.path(), "demo", WHEEL);
        let message = ProfileDir::new(tmp.path()).load("nope").unwrap_err().to_string();
        assert!(message.starts_with("no wheel profile \"nope\" in "), "{message}");
        assert!(message.ends_with("(available: demo)"), "{message}");
    }

    #[test]
    fn ids_never_walk_out_of_the_folder() {
        let tmp = tempfile::tempdir().unwrap();
        write_wheel(tmp.path(), "profiles/demo", WHEEL);
        write_wheel(tmp.path(), "outside", WHEEL);
        let dir = ProfileDir::new(tmp.path().join("profiles"));
        for id in ["../outside", "demo/../demo", "", "DEMO"] {
            let err = dir.load(id).unwrap_err();
            assert!(matches!(err, ProfileError::UnknownWheel { .. }), "{id}: {err}");
        }
    }

    #[test]
    fn id_must_match_the_folder() {
        let tmp = tempfile::tempdir().unwrap();
        write_wheel(tmp.path(), "other", WHEEL);
        let err = ProfileDir::new(tmp.path()).load("other").unwrap_err();
        assert!(matches!(err, ProfileError::IdMismatch { ref id, .. } if id == "demo"), "{err}");
    }

    #[test]
    fn parse_errors_name_the_file() {
        let tmp = tempfile::tempdir().unwrap();
        write_wheel(tmp.path(), "demo", &WHEEL.replace("schema = 1", "schema = 9"));
        let message = ProfileDir::new(tmp.path()).load("demo").unwrap_err().to_string();
        assert!(message.contains(WHEEL_FILE), "{message}");
        assert!(message.ends_with("schema 9 is not supported; this build reads schema 1"), "{message}");
    }
}

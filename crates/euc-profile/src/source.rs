use std::fmt;
use std::path::PathBuf;

use serde::{Serialize, Serializer};

use crate::{ProfileDir, ProfileError, WheelProfile, builtin, sanitize_display};

/// Where wheel profiles are read from.
///
/// Profiles carry the sha256 pins that decide which firmware is official, so the default is the
/// built-in set and a folder on disk is used only when someone names it.
#[derive(Debug, Clone)]
pub enum ProfileSource {
    /// The repo's `profiles/`, compiled into this binary: pins reviewed and built with the code.
    Builtin,
    /// A folder named at run time that replaces the built-in set. Its pins are only as
    /// trustworthy as whoever wrote the folder.
    Override(ProfileDir),
}

impl ProfileSource {
    /// The [`Origin`] that profiles from this source carry.
    pub fn origin(&self) -> Origin {
        match self {
            Self::Builtin => Origin::Builtin,
            Self::Override(dir) => Origin::Override(dir.root().to_owned()),
        }
    }

    /// Ids of every wheel this source has, sorted.
    pub fn ids(&self) -> Result<Vec<String>, ProfileError> {
        match self {
            Self::Builtin => Ok(builtin::ids()),
            Self::Override(dir) => dir.ids(),
        }
    }

    /// Loads and validates the profile of wheel `id`.
    pub fn load(&self, id: &str) -> Result<WheelProfile, ProfileError> {
        match self {
            Self::Builtin => builtin::load(id),
            Self::Override(dir) => dir.load(id),
        }
    }

    /// The text of another file of wheel `id`'s profile (`board.toml`, `stockmap.toml`), from
    /// the same source as its `wheel.toml`.
    pub fn read(&self, id: &str, file: &str) -> Result<String, ProfileError> {
        match self {
            Self::Builtin => builtin::read(id, file).map(str::to_owned),
            Self::Override(dir) => dir.read(id, file),
        }
    }
}

/// Where a loaded profile came from.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum Origin {
    /// Compiled into this binary.
    Builtin,
    /// Read from this folder, replacing the built-in set.
    Override(PathBuf),
    /// Parsed from text by the caller ([`WheelProfile::parse`]), not loaded from a source.
    #[default]
    Unspecified,
}

impl Origin {
    /// Machine label for JSON output: `builtin`, `override:<dir>` or `unspecified`.
    pub fn label(&self) -> String {
        match self {
            Self::Builtin => "builtin".to_owned(),
            Self::Override(dir) => format!("override:{}", sanitize_display(&dir.display().to_string())),
            Self::Unspecified => "unspecified".to_owned(),
        }
    }
}

impl fmt::Display for Origin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Builtin => f.write_str("the built-in profiles"),
            Self::Override(dir) => {
                write!(f, "override profiles at {}", sanitize_display(&dir.display().to_string()))
            }
            Self::Unspecified => f.write_str("a profile of unspecified origin"),
        }
    }
}

impl Serialize for Origin {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.label())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn labels_and_names() {
        let dir = PathBuf::from("/tmp/p\u{1b}[31m");
        assert_eq!(Origin::Builtin.label(), "builtin");
        assert_eq!(Origin::Override(dir.clone()).label(), "override:/tmp/p\\u{1b}[31m");
        assert_eq!(Origin::Builtin.to_string(), "the built-in profiles");
        assert_eq!(Origin::Override(dir).to_string(), "override profiles at /tmp/p\\u{1b}[31m");
    }

    #[test]
    fn sources_stamp_their_origin() {
        let wheel = ProfileSource::Builtin.load("begode-extreme").unwrap();
        assert_eq!(wheel.origin, Origin::Builtin);
        assert_eq!(ProfileSource::Builtin.ids().unwrap(), ["begode-extreme"]);
    }
}

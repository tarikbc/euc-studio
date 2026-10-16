//! Named places in our own firmware (`profiles/<wheel>/probes.toml`), for viewers, scenarios and
//! `euc sim compare`.
//!
//! The stock map names RAM variables of the pinned stock build by address. Our firmware moves
//! from build to build, so a probe names an ELF symbol and an offset into it, resolved against the
//! image's `.elf`. A probe uses the stock map's name when it holds the same quantity, so a
//! comparison pairs them; a `role` works as in the stock map (`cutoff`).

use object::{Object, ObjectSymbol};
use serde::Deserialize;

use crate::config::{RamVar, StockMap};

/// `profiles/<wheel>/probes.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProbesFile {
    /// Format version, 1.
    pub schema: u32,
    /// The probes.
    #[serde(default, rename = "var")]
    pub vars: Vec<Probe>,
}

/// One variable of our firmware.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probe {
    /// Name (the stock map's, for the same quantity).
    pub name: String,
    /// ELF symbol that holds it.
    pub symbol: String,
    /// Byte offset into the symbol.
    #[serde(default)]
    pub offset: u32,
    /// Size in bytes.
    pub size: u32,
    /// `u8`, `s16`, `f32`, ... as in the stock map.
    pub kind: String,
    /// What it is.
    pub note: Option<String>,
    /// As in the stock map (`cutoff`).
    pub role: Option<String>,
}

impl ProbesFile {
    /// Parses `probes.toml`.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let file: Self = toml::from_str(text).map_err(|e| e.to_string())?;
        if file.schema != 1 {
            return Err(format!("schema {} is not supported (expected 1)", file.schema));
        }
        Ok(file)
    }

    /// The probes resolved against `elf` (the image's ELF file), as a map of `label`.
    pub fn resolve(&self, elf: &[u8], label: &str) -> Result<StockMap, String> {
        let file = object::File::parse(elf).map_err(|e| format!("not an ELF file: {e}"))?;
        let symbol =
            |name: &str| file.symbols().find(|s| s.name() == Ok(name)).map(|s| (s.address() as u32, s.size() as u32));
        let mut ram = Vec::new();
        for probe in &self.vars {
            let (addr, size) =
                symbol(&probe.symbol).ok_or_else(|| format!("{}: no symbol {}", probe.name, probe.symbol))?;
            if probe.offset + probe.size > size {
                return Err(format!(
                    "{}: {} bytes at offset {} run past {} ({size} bytes)",
                    probe.name, probe.size, probe.offset, probe.symbol
                ));
            }
            ram.push(RamVar {
                name: probe.name.clone(),
                addr: addr + probe.offset,
                size: probe.size,
                kind: probe.kind.clone(),
                note: probe.note.clone(),
                role: probe.role.clone(),
            });
        }
        Ok(StockMap { schema: 1, build: label.to_owned(), entries: Vec::new(), ram })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_refuses_bad_probes() {
        let file = ProbesFile::from_toml(
            "schema = 1\n[[var]]\nname = \"balance_disabled\"\nsymbol = \"app\"\noffset = 4\nsize = 1\nkind = \"u8\"\nrole = \"cutoff\"\n",
        )
        .unwrap();
        assert_eq!((file.vars[0].offset, file.vars[0].role.as_deref()), (4, Some("cutoff")));
        assert!(file.resolve(b"not an elf", "ours").unwrap_err().starts_with("not an ELF file"));
        assert!(ProbesFile::from_toml("schema = 2").is_err());
    }
}

//! Named places in our own firmware (`profiles/<wheel>/probes.toml`), for viewers, scenarios and
//! `euc sim compare`.
//!
//! The stock map names RAM variables of the pinned stock build by address. Our firmware moves
//! from build to build, so a probe names an ELF symbol and an offset into it, resolved against the
//! image's `.elf`. A probe uses the stock map's name when it holds the same quantity, so a
//! comparison pairs them; a `role` works as in the stock map (`cutoff`).
//!
//! The offsets are only right for the layout they were measured on. `probes.toml` carries that
//! layout's id and the image carries the same id as the ELF symbol [`LAYOUT_SYMBOL`]; resolving
//! refuses a pair that differs (a CLI with built-in profiles older than the image, say), so a stale
//! map never reads the wrong bytes.

use object::{Object, ObjectSymbol};
use serde::Deserialize;

use crate::config::{RamVar, StockMap};

/// The image's absolute ELF symbol whose value is the id of its probed layout.
pub const LAYOUT_SYMBOL: &str = "euc_probe_layout";

/// `profiles/<wheel>/probes.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProbesFile {
    /// Format version, 1.
    pub schema: u32,
    /// Id of the layout the offsets were measured on (CRC-32, `firmware/tools/gen_probes.py`);
    /// the image carries the same value as [`LAYOUT_SYMBOL`].
    pub layout: u32,
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
        match symbol(LAYOUT_SYMBOL) {
            Some((layout, _)) if layout == self.layout => {}
            Some((layout, _)) => {
                return Err(format!(
                    "the probe map does not match this image (map layout {:#010x}, image {layout:#010x}): they \
                     come from different versions of struct app; rebuild the image, or rebuild euc (or pass \
                     --profiles profiles), so both come from the same tree",
                    self.layout
                ));
            }
            None => {
                return Err(format!(
                    "the image has no {LAYOUT_SYMBOL} symbol, so the probe map cannot be checked against it: \
                     rebuild the firmware"
                ));
            }
        }
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

    const PROBES: &str = "schema = 1\nlayout = 0x1234abcd\n[[var]]\nname = \"balance_disabled\"\nsymbol = \"app\"\n\
                          offset = 4\nsize = 1\nkind = \"u8\"\nrole = \"cutoff\"\n";

    /// A little-endian ELF32 file with only a symbol table: `(name, value, size)`, all absolute.
    fn elf(symbols: &[(&str, u32, u32)]) -> Vec<u8> {
        let mut strtab = vec![0u8];
        let mut symtab = vec![0u8; 16];
        for (name, value, size) in symbols {
            let name_at = strtab.len() as u32;
            strtab.extend_from_slice(name.as_bytes());
            strtab.push(0);
            for word in [name_at, *value, *size] {
                symtab.extend_from_slice(&word.to_le_bytes());
            }
            // Global object, absolute (SHN_ABS).
            symtab.extend_from_slice(&[0x11, 0, 0xf1, 0xff]);
        }
        let shstrtab = b"\0.symtab\0.strtab\0.shstrtab\0".to_vec();
        let (symtab_at, strtab_at) = (52, 52 + symtab.len());
        let shstrtab_at = strtab_at + strtab.len();
        let headers_at = (shstrtab_at + shstrtab.len()).next_multiple_of(4);
        let mut out = vec![0x7f, b'E', b'L', b'F', 1, 1, 1];
        out.resize(16, 0);
        for half in [2u16, 40] {
            out.extend_from_slice(&half.to_le_bytes());
        }
        for word in [1u32, 0, 0, headers_at as u32, 0x0500_0000] {
            out.extend_from_slice(&word.to_le_bytes());
        }
        for half in [52u16, 32, 0, 40, 4, 3] {
            out.extend_from_slice(&half.to_le_bytes());
        }
        out.extend_from_slice(&symtab);
        out.extend_from_slice(&strtab);
        out.extend_from_slice(&shstrtab);
        out.resize(headers_at, 0);
        // name, type, flags, addr, offset, size, link, info, align, entsize
        let sections: [[usize; 10]; 4] = [
            [0; 10],
            [1, 2, 0, 0, symtab_at, symtab.len(), 2, 1, 4, 16],
            [9, 3, 0, 0, strtab_at, strtab.len(), 0, 0, 1, 0],
            [17, 3, 0, 0, shstrtab_at, shstrtab.len(), 0, 0, 1, 0],
        ];
        for section in sections {
            for word in section {
                out.extend_from_slice(&(word as u32).to_le_bytes());
            }
        }
        out
    }

    #[test]
    fn parses_and_refuses_bad_probes() {
        let file = ProbesFile::from_toml(PROBES).unwrap();
        assert_eq!((file.layout, file.vars[0].offset, file.vars[0].role.as_deref()), (0x1234_abcd, 4, Some("cutoff")));
        assert!(file.resolve(b"not an elf", "ours").unwrap_err().starts_with("not an ELF file"));
        assert!(ProbesFile::from_toml("schema = 2\nlayout = 1").is_err());
        assert!(ProbesFile::from_toml("schema = 1").is_err(), "a map without its layout id");
    }

    #[test]
    fn resolves_only_against_an_image_of_the_same_layout() {
        let file = ProbesFile::from_toml(PROBES).unwrap();
        let map = file.resolve(&elf(&[("app", 0x2000_0100, 64), (LAYOUT_SYMBOL, 0x1234_abcd, 0)]), "ours").unwrap();
        assert_eq!((map.ram[0].name.as_str(), map.ram[0].addr), ("balance_disabled", 0x2000_0104));

        let stale = file.resolve(&elf(&[("app", 0x2000_0100, 68), (LAYOUT_SYMBOL, 0x0bad_f00d, 0)]), "ours");
        let why = stale.unwrap_err();
        assert!(why.contains("does not match this image") && why.contains("0x0badf00d"), "{why}");

        let unmarked = file.resolve(&elf(&[("app", 0x2000_0100, 64)]), "ours");
        assert!(unmarked.unwrap_err().contains("rebuild the firmware"));
    }
}

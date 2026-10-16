//! Rider settings presets (`profiles/<wheel>/settings.toml`): what the stock firmware's settings
//! journal holds at power-on.
//!
//! A real wheel never boots with blank settings: the rider set them once in the app and the wheel
//! saved them. A preset is packed into the firmware's 8-halfword settings record (the layout of
//! `firmware/core/include/euc/settings/settings.h`) and seeded into journal slot 0 of the
//! settings page before the machine is built.

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::config::BoardFile;

/// Kind of the component that holds programmable flash.
const FLASH_ARRAY_KIND: &str = "stm32f1.flash-array";

/// `profiles/<wheel>/settings.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SettingsFile {
    /// Format version, 1.
    pub schema: u32,
    /// Address of the settings journal page; a preset goes in its first slot.
    pub journal: u32,
    /// Presets by name.
    #[serde(rename = "preset")]
    pub presets: BTreeMap<String, Preset>,
}

/// Pedal stiffness (`h` `f` `s`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PedalMode {
    /// Soft (0).
    Soft,
    /// Medium (1), the blank record's value.
    #[default]
    Medium,
    /// Hard (2).
    Hard,
}

/// Motor control mode (toggled by `+` `-`; applies at the next boot).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DriveMode {
    /// Voltage-mode FOC at 10 kHz (0), the blank record's value.
    #[default]
    Voltage,
    /// Current-mode FOC at 12 kHz (1).
    Current,
}

/// One preset. Every field defaults to the blank record's value.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Preset {
    /// What it is, for people.
    pub description: String,
    /// Odometer in the firmware's units (118 hall steps, about 1 m; frame 04 shows it as metres).
    pub odometer_units: u32,
    /// Pedal mode.
    pub pedal_mode: PedalMode,
    /// Speed-alarm mode, 0..3 (`o` `u` `i` `I`).
    pub speed_alarm_mode: u8,
    /// Tiltback speed, km/h (0: the firmware's 20; 200: off).
    pub tiltback_kmh: u8,
    /// `W U` pedal trim, 0..9 (4 = level).
    pub pedal_trim: u8,
    /// `W X` roll limit step, 0..9.
    pub roll_limit_step: u8,
    /// `W C` value, 0..9.
    pub wc_value: u8,
    /// `W M` value, 0..1.
    pub wm_value: u8,
    /// Roll cut-off mode, 0..2 (`<` `=` `>`).
    pub roll_cutoff_mode: u8,
    /// `W R` turn compensation, 0..9.
    pub turn_compensation: u8,
    /// PWM tiltback (`e` on, `x` off).
    pub pwm_tiltback: bool,
    /// Drive mode.
    pub drive_mode: DriveMode,
    /// Speeds and distances in miles.
    pub miles: bool,
    /// `W P` PWM alarm threshold, % (0: the firmware's 80).
    pub pwm_alarm_percent: u8,
    /// `W B` beeper volume, 0..9.
    pub beeper_volume: u8,
}

impl Default for Preset {
    /// The blank record: medium pedals, turn compensation 3, volume 6, everything else 0.
    fn default() -> Self {
        Self {
            description: String::new(),
            odometer_units: 0,
            pedal_mode: PedalMode::Medium,
            speed_alarm_mode: 0,
            tiltback_kmh: 0,
            pedal_trim: 0,
            roll_limit_step: 0,
            wc_value: 0,
            wm_value: 0,
            roll_cutoff_mode: 0,
            turn_compensation: 3,
            pwm_tiltback: false,
            drive_mode: DriveMode::Voltage,
            miles: false,
            pwm_alarm_percent: 0,
            beeper_volume: 6,
        }
    }
}

impl Preset {
    /// Checks each field against the range its commands allow.
    pub fn validate(&self) -> Result<(), String> {
        let digits = [
            ("speed_alarm_mode", self.speed_alarm_mode, 3),
            ("pedal_trim", self.pedal_trim, 9),
            ("roll_limit_step", self.roll_limit_step, 9),
            ("wc_value", self.wc_value, 9),
            ("wm_value", self.wm_value, 1),
            ("roll_cutoff_mode", self.roll_cutoff_mode, 2),
            ("turn_compensation", self.turn_compensation, 9),
            ("beeper_volume", self.beeper_volume, 9),
        ];
        if let Some((name, value, max)) = digits.iter().find(|(_, value, max)| value > max) {
            return Err(format!("{name} = {value} is above {max}"));
        }
        if self.pwm_alarm_percent != 0 && !(50..=90).contains(&self.pwm_alarm_percent) {
            return Err(format!("pwm_alarm_percent = {} is outside 50..90", self.pwm_alarm_percent));
        }
        Ok(())
    }

    /// The firmware's settings record (journal slot and backup-register layout).
    pub fn record(&self) -> [u16; 8] {
        let n = |v: u8| u16::from(v & 0xF);
        let pedal = match self.pedal_mode {
            PedalMode::Soft => 0,
            PedalMode::Medium => 1,
            PedalMode::Hard => 2,
        };
        [
            (self.odometer_units & 0xFFFF) as u16,
            (self.odometer_units >> 16) as u16,
            pedal << 12 | n(self.speed_alarm_mode) << 8 | u16::from(self.tiltback_kmh),
            n(self.pedal_trim),
            n(self.roll_limit_step) << 12 | n(self.wc_value) << 8 | n(self.wm_value) << 4 | n(self.roll_cutoff_mode),
            u16::from(self.turn_compensation) << 8
                | u16::from(self.pwm_tiltback) << 4
                | u16::from(self.drive_mode == DriveMode::Current),
            u16::from(self.miles),
            u16::from(self.pwm_alarm_percent) << 8 | n(self.beeper_volume),
        ]
    }
}

impl SettingsFile {
    /// Parses and checks `settings.toml`.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let file: Self = toml::from_str(text).map_err(|e| e.to_string())?;
        if file.schema != 1 {
            return Err(format!("schema {} is not supported (expected 1)", file.schema));
        }
        for (name, preset) in &file.presets {
            preset.validate().map_err(|why| format!("preset {name}: {why}"))?;
        }
        Ok(file)
    }

    /// The preset called `name`.
    pub fn preset(&self, name: &str) -> Result<&Preset, String> {
        self.presets.get(name).ok_or_else(|| {
            let known: Vec<&str> = self.presets.keys().map(String::as_str).collect();
            format!("no settings preset \"{name}\" (have: {})", known.join(", "))
        })
    }

    /// Seeds `preset` into journal slot 0 of `board`'s flash before power-on.
    pub fn seed(&self, board: &mut BoardFile, preset: &Preset) -> Result<(), String> {
        seed_flash(board, self.journal, &preset.record())
    }
}

/// Adds `halfwords` at `addr` to the seed of the flash array that holds it.
pub fn seed_flash(board: &mut BoardFile, addr: u32, halfwords: &[u16]) -> Result<(), String> {
    let array = board
        .instances
        .iter_mut()
        .find(|i| {
            i.kind == FLASH_ARRAY_KIND
                && i.base.is_some_and(|base| (base..base.saturating_add(i.size.unwrap_or(0))).contains(&addr))
        })
        .ok_or_else(|| format!("no {FLASH_ARRAY_KIND} holds 0x{addr:08X}"))?;
    let mut entry = toml::Table::new();
    entry.insert("addr".into(), toml::Value::Integer(i64::from(addr)));
    entry.insert(
        "halfwords".into(),
        toml::Value::Array(halfwords.iter().map(|&h| toml::Value::Integer(i64::from(h))).collect()),
    );
    match array.params.entry("seed").or_insert_with(|| toml::Value::Array(Vec::new())) {
        toml::Value::Array(seeds) => seeds.push(toml::Value::Table(entry)),
        _ => return Err(format!("{}: seed is not a list", array.name)),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use euc_profile::ProfileSource;

    use super::*;

    fn file() -> SettingsFile {
        SettingsFile::from_toml(&ProfileSource::Builtin.read("begode-extreme", "settings.toml").unwrap()).unwrap()
    }

    #[test]
    fn neutral_is_the_blank_record_with_level_pedals() {
        let neutral = file().preset("neutral").unwrap().record();
        assert_eq!(neutral, [0, 0, 0x1000, 4, 0, 0x0300, 0, 0x0006]);
    }

    #[test]
    fn the_captured_wheel_packs_to_its_frames() {
        let record = file().preset("tarik-2026-10-15").unwrap().record();
        // Frame 04 bytes 2-5 hold the odometer as 00 16 68 FE; modes 0x4800 = hard, alarm mode 2.
        assert_eq!(record, [0x68FE, 0x0016, 0x2245, 5, 0x0010, 0x0511, 0, 0x5009]);
    }

    #[test]
    fn presets_seed_the_journal_page() {
        let source = ProfileSource::Builtin;
        let mut board: BoardFile = toml::from_str(&source.read("begode-extreme", "board.toml").unwrap()).unwrap();
        let settings = file();
        settings.seed(&mut board, settings.preset("neutral").unwrap()).unwrap();
        let array = board.instances.iter().find(|i| i.kind == FLASH_ARRAY_KIND).unwrap();
        let seeds = array.params["seed"].as_array().unwrap();
        let last = seeds.last().unwrap().as_table().unwrap();
        assert_eq!(last["addr"].as_integer(), Some(0x0800_F000));
        assert!(seed_flash(&mut board, 0x2000_0000, &[1]).is_err());
        assert!(settings.preset("nope").unwrap_err().contains("have: neutral, tarik-2026-10-15"));
    }

    #[test]
    fn out_of_range_values_are_refused() {
        let bad = "schema = 1\njournal = 0\n[preset.x]\npedal_trim = 12\n";
        assert!(SettingsFile::from_toml(bad).unwrap_err().contains("pedal_trim = 12 is above 9"));
        let typo = "schema = 1\njournal = 0\n[preset.x]\npedal_trims = 1\n";
        assert!(SettingsFile::from_toml(typo).is_err());
    }
}

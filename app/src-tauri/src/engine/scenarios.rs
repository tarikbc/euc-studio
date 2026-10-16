//! The scenarios the studio offers: the repository's `scenarios/*.toml`. A dev build reads the
//! folder when the web view connects, so a new scenario shows without a rebuild; the two below are
//! built in and come first.

use std::path::Path;

use euc_sim::scenario::Scenario;
use serde::Serialize;

const BUILT_IN: [(&str, &str, &str); 2] = [
    ("ride-20kmh-stop", "Ride to 20 km/h and stop", include_str!("../../../../scenarios/ride-20kmh-stop.toml")),
    ("balance-60s", "Balance for 60 s", include_str!("../../../../scenarios/balance-60s.toml")),
];

/// A scenario as the run bar lists it.
#[derive(Debug, Clone, Serialize)]
pub struct ScenarioInfo {
    pub id: String,
    pub label: String,
    pub detail: String,
}

/// `(id, label, text)` of every scenario file, built-in ones first.
fn files() -> Vec<(String, String, String)> {
    let mut files: Vec<_> =
        BUILT_IN.iter().map(|&(id, label, text)| (id.to_owned(), label.to_owned(), text.to_owned())).collect();
    if !cfg!(dev) {
        return files;
    }
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let Ok(entries) = std::fs::read_dir(&dir) else { return files };
    let mut found: Vec<_> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    found.sort();
    for path in found {
        let (Some(id), Ok(text)) = (path.file_stem().and_then(|s| s.to_str()), std::fs::read_to_string(&path)) else {
            continue;
        };
        match files.iter_mut().find(|f| f.0 == id) {
            // The file on disk is newer than the copy built in.
            Some(file) => file.2 = text,
            None => files.push((id.to_owned(), label(id), text)),
        }
    }
    files
}

/// "low-battery-tiltback" -> "Low battery tiltback", "hard-braking-30kmh" -> "Hard braking 30 km/h".
fn label(id: &str) -> String {
    let words: Vec<String> = id
        .split('-')
        .map(|w| {
            let digits = w.trim_end_matches(|c: char| c.is_ascii_alphabetic());
            match &w[digits.len()..] {
                "kmh" if !digits.is_empty() => format!("{digits} km/h"),
                "pct" if !digits.is_empty() => format!("{digits} %"),
                _ => w.to_owned(),
            }
        })
        .collect();
    let text = words.join(" ");
    let mut chars = text.chars();
    chars.next().map(|c| c.to_uppercase().chain(chars).collect()).unwrap_or_default()
}

/// Every scenario that parses, built-in ones first. Drafts (description "draft") are left out.
pub fn infos() -> Vec<ScenarioInfo> {
    files()
        .into_iter()
        .filter_map(|(id, label, text)| {
            let scenario = Scenario::from_toml(&text).ok()?;
            (scenario.description != "draft").then_some(ScenarioInfo { id, label, detail: scenario.description })
        })
        .collect()
}

/// Scenario `id`, parsed.
pub fn load(id: &str) -> Result<Scenario, String> {
    let (_, _, text) = files().into_iter().find(|f| f.0 == id).ok_or_else(|| format!("no scenario {id}"))?;
    Scenario::from_toml(&text).map_err(|e| format!("scenario {id}: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_scenarios_parse_and_come_first() {
        let infos = infos();
        assert_eq!(infos[0].id, "ride-20kmh-stop");
        assert_eq!(infos[1].id, "balance-60s");
        for info in &infos {
            assert_eq!(load(&info.id).expect(&info.id).name, info.id);
        }
    }

    #[test]
    fn labels_read_like_words() {
        assert_eq!(label("low-battery-tiltback"), "Low battery tiltback");
        assert_eq!(label("hard-braking-30kmh"), "Hard braking 30 km/h");
        assert_eq!(label("overlean-top-speed-30pct"), "Overlean top speed 30 %");
    }
}

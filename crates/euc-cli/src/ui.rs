//! Terminal output: aligned columns, color only on a TTY, JSON on request.
//!
//! Colors go through `anstream`, which strips them when stdout is not a terminal and honors
//! `NO_COLOR` and `CLICOLOR_FORCE`. Text from outside the program reaches these helpers already
//! sanitized (`euc_fwsource` does it at the network and cache boundary); paths and error chains,
//! which can quote user files, are sanitized here.

use std::fmt::Display;
use std::path::Path;

use anstream::{eprintln, println};
use anstyle::{AnsiColor, Style};
use euc_fwsource::sanitize_display;
use euc_profile::Origin;
use serde::Serialize;

pub(crate) const GOOD: Style = AnsiColor::Green.on_default().bold();
pub(crate) const BAD: Style = AnsiColor::Red.on_default().bold();
pub(crate) const WARN: Style = AnsiColor::Yellow.on_default().bold();
pub(crate) const DIM: Style = Style::new().dimmed();
pub(crate) const STRONG: Style = Style::new().bold();

/// Width of the status column.
const STATUS: usize = 10;

/// `text` in `style`.
pub(crate) fn paint(style: Style, text: impl Display) -> String {
    format!("{style}{text}{style:#}")
}

/// One row: a styled status word padded to the status column, then the rest.
pub(crate) fn row(style: Style, status: &str, rest: impl Display) {
    println!("{style}{status:<STATUS$}{style:#}{rest}");
}

/// A continuation line under a row, aligned with its text.
pub(crate) fn detail(text: impl Display) {
    println!("{:STATUS$}{text}", "");
}

/// A path, safe to print.
pub(crate) fn path(path: &Path) -> String {
    sanitize_display(&path.display().to_string())
}

/// Text from outside the program (firmware output, blocker texts), safe to print.
pub(crate) fn safe(text: &str) -> String {
    sanitize_display(text)
}

/// Text quoted with its control characters visible (`"NAME:EXTREME\r\n"`), safe to print.
pub(crate) fn quoted(text: &str) -> String {
    sanitize_display(&format!("{text:?}"))
}

/// Pretty JSON on stdout.
pub(crate) fn json(value: &impl Serialize) -> anyhow::Result<()> {
    println!("{}", serde_json::to_string_pretty(value)?);
    Ok(())
}

/// Pretty JSON for a command that used wheel profiles: `profiles_source` says where its pins came
/// from (`builtin` or `override:<dir>`), next to the command's own fields.
pub(crate) fn json_with_profiles(origin: &Origin, body: &impl Serialize) -> anyhow::Result<()> {
    #[derive(Serialize)]
    struct WithProfiles<'a, T> {
        profiles_source: &'a Origin,
        #[serde(flatten)]
        body: &'a T,
    }
    json(&WithProfiles { profiles_source: origin, body })
}

/// A warning on stderr.
pub(crate) fn warn(text: impl Display) {
    eprintln!("{} {text}", paint(WARN, "warning:"));
}

/// The error and its causes on stderr. Sanitized: a cause can quote a user's file.
pub(crate) fn error(error: &anyhow::Error) {
    eprintln!("{} {}", paint(BAD, "error:"), sanitize_display(&format!("{error:#}")));
}

/// `41596` as `41,596`.
pub(crate) fn thousands(n: u64) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn groups_thousands() {
        assert_eq!(thousands(0), "0");
        assert_eq!(thousands(999), "999");
        assert_eq!(thousands(41_596), "41,596");
        assert_eq!(thousands(1_234_567), "1,234,567");
    }
}

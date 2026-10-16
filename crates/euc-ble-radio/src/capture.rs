//! Capture files: what a BLE central saw of a wheel, one notification (or write) per line.
//!
//! ```text
//! # euc-capture v1
//! # brand: Begode
//! # wheel: begode-extreme
//! # name: GotWay_71105
//! 0.000 55 aa 19 50 00 00 00 75 00 05 00 b4 f2 5b 14 a9 00 09 00 18
//! 0.000 5a 5a 5a 5a 55 aa 00 50 ff f8 05 10 ff ff 00 17 00 17 02 8b
//! 1.204 write 56
//! ```
//!
//! A data line is `<seconds since the start> [<direction>[:<characteristic>]] <bytes as two-digit
//! hex, space separated>`. The direction is `notify` (the wheel sent it, the default) or `write`
//! (a central wrote it); the characteristic is a UUID of the wheel's profile, by default its
//! `tx` characteristic for notifications and its `rx` one for writes. So the files EUC Remote
//! records (no header, no direction) read as single-characteristic captures.
//!
//! Lines that start with `#` are comments. In a file whose comments include `euc-capture v1`,
//! comments of the form `key: value` are header fields (`brand`, `wheel`, `name`; others are
//! kept). Empty lines are skipped. Lines keep the packet splits of the module that sent them, so
//! a replay sends each line as one notification.

use std::path::Path;

/// The header line that marks the current format.
pub const MAGIC: &str = "euc-capture v1";

/// The optional header.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Header {
    /// Format version: 1 with the `euc-capture v1` line, `None` for a plain capture.
    pub version: Option<u32>,
    /// `brand: Begode`.
    pub brand: Option<String>,
    /// `wheel: begode-extreme`, a wheel profile id.
    pub wheel: Option<String>,
    /// `name: GotWay_71105`, what the wheel advertised.
    pub name: Option<String>,
    /// Other `key: value` fields, in file order.
    pub other: Vec<(String, String)>,
}

/// Who sent a line's bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// The wheel, as a notification.
    Notify,
    /// A central, as a write.
    Write,
}

/// One data line.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    /// Line number in the file, from 1.
    pub number: usize,
    /// Seconds since the capture started.
    pub t: f64,
    /// Who sent it.
    pub direction: Direction,
    /// The characteristic's UUID as written, or `None` for the default.
    pub characteristic: Option<String>,
    /// The payload.
    pub bytes: Vec<u8>,
}

/// A whole capture.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Capture {
    /// The header (empty for a plain capture).
    pub header: Header,
    /// The data lines, in time order.
    pub lines: Vec<Line>,
}

/// A line that cannot be read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("line {line}: {reason}")]
pub struct ParseError {
    /// Line number, from 1.
    pub line: usize,
    /// What is wrong with it.
    pub reason: String,
}

/// A capture file that cannot be read.
#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    /// The file cannot be read.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// A line is wrong.
    #[error(transparent)]
    Parse(#[from] ParseError),
}

/// Reads a capture file.
pub fn load(path: &Path) -> Result<Capture, LoadError> {
    Ok(parse(&std::fs::read_to_string(path)?)?)
}

/// Reads a capture's text. Times must not go back.
pub fn parse(text: &str) -> Result<Capture, ParseError> {
    let mut capture = Capture::default();
    let mut last = 0.0;
    for (index, raw) in text.lines().enumerate() {
        let number = index + 1;
        let fail = |reason: String| ParseError { line: number, reason };
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            header_line(&mut capture.header, comment.trim()).map_err(fail)?;
            continue;
        }
        let mut fields = line.split_whitespace().peekable();
        let stamp = fields.next().unwrap_or_default();
        let t = stamp
            .parse::<f64>()
            .ok()
            .filter(|t| t.is_finite() && *t >= 0.0)
            .ok_or_else(|| fail(format!("\"{stamp}\" is not a time in seconds")))?;
        if t < last {
            return Err(fail(format!("time {t} is before the line above ({last})")));
        }
        last = t;
        let (direction, characteristic) = match fields.next_if(|f| hex_byte(f).is_none()) {
            Some(field) => {
                direction(field).ok_or_else(|| fail(format!("\"{field}\" is not a hex byte or a direction")))?
            }
            None => (Direction::Notify, None),
        };
        let bytes = fields.map(|field| hex_byte(field).ok_or_else(|| fail(format!("\"{field}\" is not a hex byte"))));
        let bytes = bytes.collect::<Result<Vec<u8>, _>>()?;
        if bytes.is_empty() {
            return Err(fail("no bytes after the time".into()));
        }
        capture.lines.push(Line { number, t, direction, characteristic, bytes });
    }
    Ok(capture)
}

/// One comment: the format line, a header field, or a plain comment.
fn header_line(header: &mut Header, comment: &str) -> Result<(), String> {
    if let Some(version) = comment.strip_prefix("euc-capture v") {
        return match version.trim() {
            "1" => {
                header.version = Some(1);
                Ok(())
            }
            other => Err(format!("capture format v{other} is newer than this program reads (v1)")),
        };
    }
    if header.version.is_none() {
        return Ok(());
    }
    let Some((key, value)) = comment.split_once(':') else { return Ok(()) };
    let (key, value) = (key.trim(), value.trim().to_owned());
    match key {
        "brand" => header.brand = Some(value),
        "wheel" => header.wheel = Some(value),
        "name" => header.name = Some(value),
        _ => header.other.push((key.to_owned(), value)),
    }
    Ok(())
}

/// `notify`, `write`, `notify:FFE1`, `write:6E400002-B5A3-F393-E0A9-E50E24DCCA9E`.
fn direction(field: &str) -> Option<(Direction, Option<String>)> {
    let (word, characteristic) = match field.split_once(':') {
        Some((word, uuid)) if !uuid.is_empty() => (word, Some(uuid.to_owned())),
        Some(_) => return None,
        None => (field, None),
    };
    let direction = match word {
        "notify" => Direction::Notify,
        "write" => Direction::Write,
        _ => return None,
    };
    Some((direction, characteristic))
}

/// `5a` or `5A` as a byte; nothing else (no sign, no `0x`, exactly two digits).
fn hex_byte(field: &str) -> Option<u8> {
    let digits = field.as_bytes();
    (digits.len() == 2 && digits.iter().all(u8::is_ascii_hexdigit)).then(|| u8::from_str_radix(field, 16).ok())?
}

#[cfg(test)]
mod tests {
    use super::*;

    fn notify(number: usize, t: f64, bytes: &[u8]) -> Line {
        Line { number, t, direction: Direction::Notify, characteristic: None, bytes: bytes.to_vec() }
    }

    #[test]
    fn plain_captures_are_single_characteristic_notifications() {
        let text = "# eucctl record GotWay 2026-10-16T03:04:33Z\n\n0.000 55 AA 19\n  # note: kept as a comment\n0.029 5a 5a\r\n";
        let capture = parse(text).unwrap();
        assert_eq!(capture.header, Header::default(), "comments of a plain capture are not header fields");
        assert_eq!(capture.lines, [notify(3, 0.0, &[0x55, 0xAA, 0x19]), notify(5, 0.029, &[0x5A, 0x5A])]);
    }

    #[test]
    fn a_v1_header_names_the_wheel_and_lines_can_name_characteristics() {
        let text = "# euc-capture v1\n# brand: Begode\n# wheel: begode-extreme\n# name: GotWay_71105\n# phone: watch\n\
                    0.1 55 aa\n0.2 write 56\n0.3 notify:6E400003-B5A3-F393-E0A9-E50E24DCCA9E 01 02\n0.4 write:ffe1 4e\n";
        let capture = parse(text).unwrap();
        let header = &capture.header;
        assert_eq!(
            (header.version, header.brand.as_deref(), header.wheel.as_deref()),
            (Some(1), Some("Begode"), Some("begode-extreme"))
        );
        assert_eq!(
            (header.name.as_deref(), header.other.clone()),
            (Some("GotWay_71105"), vec![("phone".into(), "watch".into())])
        );
        let kinds: Vec<(Direction, Option<&str>)> =
            capture.lines.iter().map(|l| (l.direction, l.characteristic.as_deref())).collect();
        assert_eq!(
            kinds,
            [
                (Direction::Notify, None),
                (Direction::Write, None),
                (Direction::Notify, Some("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")),
                (Direction::Write, Some("ffe1")),
            ]
        );
        assert_eq!(capture.lines[3].bytes, b"N");
    }

    #[test]
    fn says_which_line_is_wrong() {
        let reason = |text: &str| parse(text).unwrap_err().reason;
        assert_eq!(
            parse("0.1 55\nx 55\n").unwrap_err(),
            ParseError { line: 2, reason: "\"x\" is not a time in seconds".into() }
        );
        assert_eq!(reason("0.1 55 5g"), "\"5g\" is not a hex byte");
        assert_eq!(reason("0.1 5g"), "\"5g\" is not a hex byte or a direction");
        assert_eq!(reason("0.1 +f"), "\"+f\" is not a hex byte or a direction", "no sign");
        assert_eq!(reason("0.1 read:FFE1 55"), "\"read:FFE1\" is not a hex byte or a direction");
        assert_eq!(reason("0.1 write: 55"), "\"write:\" is not a hex byte or a direction");
        assert_eq!(reason("0.1"), "no bytes after the time");
        assert_eq!(reason("0.1 write"), "no bytes after the time");
        assert_eq!(parse("0.2 55\n0.1 55").unwrap_err().line, 2, "time goes back");
        assert!(reason("# euc-capture v2\n0.1 55").contains("newer"));
        assert!(parse("-1 55").is_err() && parse("NaN 55").is_err());
    }

    #[test]
    fn the_wheel_capture_has_456_notifications_of_20_bytes() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/data/begode-extreme/idle-2026-10-15.hex");
        let capture = load(Path::new(path)).unwrap();
        assert_eq!(capture.lines.len(), 456);
        assert!(capture.lines.iter().all(|l| l.bytes.len() == 20 && l.direction == Direction::Notify));
    }
}

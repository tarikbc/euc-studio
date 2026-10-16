//! Commands: the bytes a phone app writes to characteristic `FFE1`.
//!
//! The wheel reads one byte at a time in its USART interrupt. Most commands are one byte; the
//! `W` commands are a letter pair plus one or two ASCII digits; three dangerous functions need
//! two bytes with a minimum quiet gap between them.
//!
//! Timing rules that apply to every command (GW1825020):
//! - Any received byte pauses telemetry for two frame groups (about 0.6 s).
//! - `V`, `N` and `k` are answered by the telemetry scheduler 0.6-0.9 s later, from a one-byte
//!   pending slot: any byte received in between replaces the request (last byte wins).
//! - The UART has no FIFO and shares its interrupt level with the 496 Hz control tick, so bytes
//!   sent back to back can overrun. Send one byte per BLE write, at least [`SAFE_BYTE_GAP`] apart.

use std::time::Duration;

use thiserror::Error;

use crate::settings::{AlarmMode, LightMode, PedalMode, RollCutoff, Units};

/// One tick of the wheel's control loop: 128 × 126 / 8 MHz = 2.016 ms (RTC second interrupt).
pub const TICK: Duration = Duration::from_micros(2016);

/// Gap between bytes that keeps the wheel's UART from overrunning while its control tick runs.
pub const SAFE_BYTE_GAP: Duration = Duration::from_millis(20);

/// How risky a command is to send (the safety classes of `docs/protocol/begode.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Safety {
    /// Asks for information: `V`, `N`. Send alone and wait 1 s for the reply.
    Read,
    /// Safe while riding: horn, lights, units, beeper volume, cancelling a sequence.
    Comfort,
    /// Changes how the wheel rides or warns (pedal mode, tiltback, alarms, cut-off angles).
    /// Send only at standstill and confirm the change in the frames.
    Ride,
    /// A debug function: the wheel stops telemetry and prints a sensor dump instead.
    Debug,
    /// Never send from an app: switches the motor off at once and then erases flash, powers the
    /// wheel off, or enters the bootloader. Bootloader entry belongs to the gated flasher only.
    Never,
}

/// A command with its parameter. [`encode`](Self::encode) turns it into the bytes to send.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Command {
    /// `b`: two short beeps.
    Horn,
    /// `E`, `Q`, `T`: light off, on or strobe.
    Light(LightMode),
    /// `V`: firmware banner, such as `GW1825020` (9 bytes, no line end).
    Version,
    /// `N`: model name line, such as `NAME:EXTREME\r\n`.
    Name,
    /// `s`, `f`, `h`: pedal mode. Ignored while |speed| ≥ 0.5 m/s.
    PedalMode(PedalMode),
    /// `o`, `u`, `i`, `I`: speed alarm mode, with a text reply. Ignored while moving, and for
    /// 0.2 s after the wheel stops; `o`, `u`, `i` then reply "stop the wheel first".
    SpeedAlarms(AlarmMode),
    /// `>`, `=`, `<`: roll (side-tilt) cut-off angle.
    RollCutoff(RollCutoff),
    /// `g`, `m`: metric or imperial units in telemetry.
    Units(Units),
    /// `e`, `x`: PWM tiltback on or off (pitch offset ramps up when the PWM nears the alarm level).
    PwmTiltback(bool),
    /// `"`: tiltback off (stored as 200 km/h). Applies once |speed| < 0.5 m/s, unless another
    /// byte arrives first.
    TiltbackOff,
    /// `W` `Y` d d: tiltback speed 0..=99 km/h. Applies on the second digit; no clamp, so 0 is
    /// accepted.
    Tiltback(u8),
    /// `W` `P` d d: PWM alarm threshold, 50..=90 %. Also the reference for PWM tiltback.
    PwmAlarm(u8),
    /// `W` `B` d: beeper volume 0..=9. The stock firmware computes the duty without a sign check,
    /// so 0 gives the loudest beep.
    BeeperVolume(u8),
    /// `W` `M` d: 0 or 1. Meaning UNKNOWN (the older map calls it LED mode 0..9; the firmware
    /// accepts only 0 and 1).
    Wm(u8),
    /// `W` `R` d: turn compensation gain 0..=9 (lateral acceleration correction of the
    /// attitude filter; default 3).
    TurnCompensation(u8),
    /// `W` `U` d: pedal horizon trim 0..=9 (angle offset `(d - 4) × 1.2`, meaning INFERRED).
    PedalTrim(u8),
    /// `W` `C` d: 0..=9. Stored and echoed in frame 07, but nothing reads it in GW1825020.
    Wc(u8),
    /// `W` `X` d: 0..=9, an angle limit of `40 + 5 × d` degrees (meaning INFERRED).
    AngleLimit(u8),
    /// `k`, `k`: debug dump. The second `k` must be consumed after the first, so the gap covers
    /// the telemetry pause. The wheel then prints sensor values instead of telemetry.
    DebugDump,
    /// `p`: cancels a pending `!` or `c` sequence (only while moving). No other effect.
    CancelSequence,
    /// `c`, then `y`: motor off at once, erase the IMU calibration marker; the wheel recalibrates
    /// its IMU at the next power-on. Only while |speed| ≤ 0.09 m/s.
    CalibrateImu,
    /// `!`, then `@`: motor off at once, erase the bootloader request page, reset into the
    /// bootloader (OTA update). Only while |speed| ≤ 0.09 m/s.
    EnterBootloader,
    /// `+`, then `-`: motor off at once, toggle the drive mode (10 kHz voltage FOC / 12 kHz
    /// current FOC), save settings and power the wheel off. Only while |speed| ≤ 0.09 m/s.
    ToggleDriveMode,
}

/// One byte of a command and how long the sender must wait after the previous byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Step {
    /// The byte to send.
    pub byte: u8,
    /// Minimum quiet time since the previous byte that the wheel requires (zero for the first
    /// byte). Add the transport's own margin; [`SAFE_BYTE_GAP`] is a sensible floor.
    pub min_gap: Duration,
}

/// What the catalogue knows about a command, independent of its parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Info {
    /// Short name for lists and logs.
    pub name: &'static str,
    /// Bytes as they appear on the wire; `d` stands for one ASCII digit, `…` for a wait.
    pub pattern: &'static str,
    /// Risk class.
    pub safety: Safety,
    /// The wheel ignores the command while |speed| exceeds this value, in 0.01 m/s.
    pub max_speed: Option<u16>,
}

/// A command the encoder refuses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[error("{name} takes {min}..={max}, got {value}")]
pub struct RangeError {
    /// The command's name.
    pub name: &'static str,
    /// The value given.
    pub value: u8,
    /// Smallest accepted value.
    pub min: u8,
    /// Largest accepted value.
    pub max: u8,
}

impl Command {
    /// Every command, with a representative parameter, in catalogue order.
    pub const ALL: [Self; 23] = [
        Self::Horn,
        Self::Light(LightMode::On),
        Self::Version,
        Self::Name,
        Self::PedalMode(PedalMode::Medium),
        Self::SpeedAlarms(AlarmMode::AllOn),
        Self::RollCutoff(RollCutoff::Low),
        Self::Units(Units::Metric),
        Self::PwmTiltback(false),
        Self::TiltbackOff,
        Self::Tiltback(30),
        Self::PwmAlarm(80),
        Self::BeeperVolume(5),
        Self::Wm(0),
        Self::TurnCompensation(3),
        Self::PedalTrim(4),
        Self::Wc(0),
        Self::AngleLimit(0),
        Self::DebugDump,
        Self::CancelSequence,
        Self::CalibrateImu,
        Self::EnterBootloader,
        Self::ToggleDriveMode,
    ];

    /// Catalogue entry.
    pub const fn info(&self) -> Info {
        const STOPPED: Option<u16> = Some(9);
        const STANDSTILL: Option<u16> = Some(49);
        let (name, pattern, safety, max_speed) = match self {
            Self::Horn => ("horn", "b", Safety::Comfort, None),
            Self::Light(_) => ("light", "E | Q | T", Safety::Comfort, None),
            Self::Version => ("version", "V", Safety::Read, None),
            Self::Name => ("name", "N", Safety::Read, None),
            Self::PedalMode(_) => ("pedal mode", "s | f | h", Safety::Ride, STANDSTILL),
            Self::SpeedAlarms(_) => ("speed alarms", "o | u | i | I", Safety::Ride, STANDSTILL),
            Self::RollCutoff(_) => ("roll cut-off", "> | = | <", Safety::Ride, None),
            Self::Units(_) => ("units", "g | m", Safety::Comfort, None),
            Self::PwmTiltback(_) => ("PWM tiltback", "e | x", Safety::Ride, None),
            Self::TiltbackOff => ("tiltback off", "\"", Safety::Ride, STANDSTILL),
            Self::Tiltback(_) => ("tiltback", "W Y d d", Safety::Ride, STANDSTILL),
            Self::PwmAlarm(_) => ("PWM alarm", "W P d d", Safety::Ride, STANDSTILL),
            Self::BeeperVolume(_) => ("beeper volume", "W B d", Safety::Comfort, None),
            Self::Wm(_) => ("WM", "W M d", Safety::Ride, None),
            Self::TurnCompensation(_) => ("turn compensation", "W R d", Safety::Ride, None),
            Self::PedalTrim(_) => ("pedal trim", "W U d", Safety::Ride, None),
            Self::Wc(_) => ("WC", "W C d", Safety::Ride, None),
            Self::AngleLimit(_) => ("angle limit", "W X d", Safety::Ride, None),
            Self::DebugDump => ("debug dump", "k … k", Safety::Debug, None),
            Self::CancelSequence => ("cancel sequence", "p", Safety::Comfort, None),
            Self::CalibrateImu => ("calibrate IMU", "c … y", Safety::Never, STOPPED),
            Self::EnterBootloader => ("enter bootloader", "! … @", Safety::Never, STOPPED),
            Self::ToggleDriveMode => ("toggle drive mode", "+ … -", Safety::Never, STOPPED),
        };
        Info { name, pattern, safety, max_speed }
    }

    /// Risk class, see [`Safety`].
    pub const fn safety(&self) -> Safety {
        self.info().safety
    }

    /// The bytes to send, with the gaps the wheel requires.
    ///
    /// This only encodes; it never refuses a command for its [`Safety`] class. The transport
    /// decides what it may send (the BLE layer refuses [`Safety::Never`]).
    pub fn encode(&self) -> Result<Vec<Step>, RangeError> {
        let now = |byte| Step { byte, min_gap: Duration::ZERO };
        let after = |byte, ticks: u32| Step { byte, min_gap: TICK * ticks };
        Ok(match *self {
            Self::Horn => vec![now(b'b')],
            Self::Light(mode) => vec![now(b"EQT"[usize::from(mode.value())])],
            Self::Version => vec![now(b'V')],
            Self::Name => vec![now(b'N')],
            Self::PedalMode(mode) => vec![now(b"sfh"[usize::from(mode.value())])],
            Self::SpeedAlarms(mode) => vec![now(b"ouiI"[usize::from(mode.value())])],
            Self::RollCutoff(mode) => vec![now(b">=<"[usize::from(mode.value())])],
            Self::Units(units) => vec![now(b"gm"[usize::from(units.value())])],
            Self::PwmTiltback(on) => vec![now(if on { b'e' } else { b'x' })],
            Self::TiltbackOff => vec![now(b'"')],
            Self::Tiltback(kmh) => self.two_digits(b'Y', kmh, 0, 99)?,
            Self::PwmAlarm(percent) => self.two_digits(b'P', percent, 50, 90)?,
            Self::BeeperVolume(d) => self.one_digit(b'B', d, 9)?,
            Self::Wm(d) => self.one_digit(b'M', d, 1)?,
            Self::TurnCompensation(d) => self.one_digit(b'R', d, 9)?,
            Self::PedalTrim(d) => self.one_digit(b'U', d, 9)?,
            Self::Wc(d) => self.one_digit(b'C', d, 9)?,
            Self::AngleLimit(d) => self.one_digit(b'X', d, 9)?,
            // Three scheduler runs of 151 ticks: the first `k` is consumed at the latest by then.
            Self::DebugDump => vec![now(b'k'), after(b'k', 3 * 151)],
            Self::CancelSequence => vec![now(b'p')],
            Self::CalibrateImu => vec![now(b'c'), after(b'y', 50)],
            Self::EnterBootloader => vec![now(b'!'), after(b'@', 100)],
            Self::ToggleDriveMode => vec![now(b'+'), after(b'-', 50)],
        })
    }

    fn one_digit(self, letter: u8, value: u8, max: u8) -> Result<Vec<Step>, RangeError> {
        self.check(value, 0, max)?;
        Ok([b'W', letter, b'0' + value].map(|byte| Step { byte, min_gap: Duration::ZERO }).to_vec())
    }

    fn two_digits(self, letter: u8, value: u8, min: u8, max: u8) -> Result<Vec<Step>, RangeError> {
        self.check(value, min, max)?;
        let digits = [b'W', letter, b'0' + value / 10, b'0' + value % 10];
        Ok(digits.map(|byte| Step { byte, min_gap: Duration::ZERO }).to_vec())
    }

    fn check(self, value: u8, min: u8, max: u8) -> Result<(), RangeError> {
        if (min..=max).contains(&value) { Ok(()) } else { Err(RangeError { name: self.info().name, value, min, max }) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes(command: Command) -> Vec<u8> {
        command.encode().unwrap().iter().map(|s| s.byte).collect()
    }

    #[test]
    fn single_byte_commands() {
        assert_eq!(bytes(Command::Horn), b"b");
        assert_eq!(bytes(Command::Light(LightMode::Off)), b"E");
        assert_eq!(bytes(Command::Light(LightMode::Strobe)), b"T");
        assert_eq!(bytes(Command::PedalMode(PedalMode::Hard)), b"h");
        assert_eq!(bytes(Command::SpeedAlarms(AlarmMode::Level3Off)), b"I");
        assert_eq!(bytes(Command::RollCutoff(RollCutoff::High)), b"<");
        assert_eq!(bytes(Command::Units(Units::Imperial)), b"m");
        assert_eq!(bytes(Command::PwmTiltback(true)), b"e");
        assert_eq!(bytes(Command::TiltbackOff), b"\"");
    }

    #[test]
    fn w_commands_spell_their_digits() {
        assert_eq!(bytes(Command::Tiltback(7)), b"WY07");
        assert_eq!(bytes(Command::Tiltback(0)), b"WY00");
        assert_eq!(bytes(Command::PwmAlarm(85)), b"WP85");
        assert_eq!(bytes(Command::BeeperVolume(9)), b"WB9");
        assert_eq!(bytes(Command::Wm(1)), b"WM1");
        assert_eq!(bytes(Command::AngleLimit(2)), b"WX2");
    }

    #[test]
    fn out_of_range_values_are_refused() {
        let err = Command::PwmAlarm(95).encode().unwrap_err();
        assert_eq!(err.to_string(), "PWM alarm takes 50..=90, got 95");
        assert!(Command::Tiltback(100).encode().is_err());
        assert!(Command::Wm(2).encode().is_err());
        assert!(Command::BeeperVolume(10).encode().is_err());
    }

    #[test]
    fn sequences_carry_minimum_gaps() {
        let steps = Command::EnterBootloader.encode().unwrap();
        assert_eq!(steps[0], Step { byte: b'!', min_gap: Duration::ZERO });
        assert_eq!(steps[1], Step { byte: b'@', min_gap: Duration::from_micros(201_600) });
        let steps = Command::CalibrateImu.encode().unwrap();
        assert_eq!((steps[1].byte, steps[1].min_gap), (b'y', Duration::from_micros(100_800)));
        let steps = Command::ToggleDriveMode.encode().unwrap();
        assert_eq!((steps[1].byte, steps[1].min_gap), (b'-', Duration::from_micros(100_800)));
        let steps = Command::DebugDump.encode().unwrap();
        assert!(steps[1].min_gap > Duration::from_millis(900));
    }

    #[test]
    fn never_class_is_exactly_the_three_dangerous_sequences() {
        let never: Vec<_> =
            Command::ALL.iter().filter(|c| c.safety() == Safety::Never).map(|c| c.info().pattern).collect();
        assert_eq!(never, ["c … y", "! … @", "+ … -"]);
        for command in Command::ALL {
            let first = command.encode().unwrap()[0].byte;
            if command.safety() < Safety::Never {
                assert!(!b"+-cy!@".contains(&first), "{command:?} starts with a never-send byte");
            }
        }
    }

    #[test]
    fn every_command_encodes_its_representative() {
        for command in Command::ALL {
            let steps = command.encode().unwrap();
            assert_ne!(steps.len(), 0, "{command:?}");
            assert_eq!(steps[0].min_gap, Duration::ZERO, "{command:?}");
        }
    }
}

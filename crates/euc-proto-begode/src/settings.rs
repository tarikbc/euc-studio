//! Ride settings that appear both in commands and in telemetry.

/// Pedal stiffness: how hard the wheel pushes back against the rider's lean.
///
/// Set with `s`, `f`, `h`; reported in frame 04 bytes 6-7, bits 13-14.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PedalMode {
    /// `s`, value 0.
    Soft,
    /// `f`, value 1.
    Medium,
    /// `h`, value 2.
    Hard,
}

/// Which speed alarms (beeps) are active.
///
/// Set with `o`, `u`, `i`, `I`; reported in frame 04 bytes 6-7, bits 10-11.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AlarmMode {
    /// `o`, value 0: every speed alarm on.
    AllOn,
    /// `u`, value 1: first-level alarm off.
    Level1Off,
    /// `i`, value 2: second-level alarm off.
    Level2Off,
    /// `I`, value 3: third-level alarm off. Not in the published app protocol.
    Level3Off,
}

/// Side-tilt (roll) angle at which the motor cuts out.
///
/// Set with `>`, `=`, `<`; reported in frame 04 bytes 6-7, bits 7-8. The angles are per build:
/// GW1825020 (2025-11) uses 50°, 58° and 66°; the 2025-07 build used 40°, 50° and 60°.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RollCutoff {
    /// `>`, value 0: the smallest angle.
    Low,
    /// `=`, value 1.
    Medium,
    /// `<`, value 2: the largest angle.
    High,
}

/// Distance and speed units. The wheel converts speed, trip, odometer and range itself (× 0.62)
/// before it sends them, so frame values are in the selected unit.
///
/// Set with `g` and `m`; reported in frame 04 bytes 6-7, bit 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Units {
    /// `g`, value 0.
    Metric,
    /// `m`, value 1.
    Imperial,
}

/// Light output state.
///
/// Set with `E`, `Q`, `T`; reported in frame 04 byte 15.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LightMode {
    /// `E`, value 0.
    Off,
    /// `Q`, value 1.
    On,
    /// `T`, value 2.
    Strobe,
}

/// Motor drive mode, reported in frame 00 bytes 14-15, bit 0.
///
/// Toggled only by the `+` `-` sequence, which also powers the wheel off; apps never send it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DriveMode {
    /// Value 0: voltage-mode FOC at 10 kHz PWM.
    Voltage,
    /// Value 1: current-mode FOC at 12 kHz PWM.
    Current,
}

macro_rules! numbered {
    ($($ty:ident { $($variant:ident = $value:literal),+ $(,)? })+) => {$(
        impl $ty {
            /// The value the firmware stores and reports.
            pub const fn value(self) -> u8 {
                match self { $(Self::$variant => $value),+ }
            }

            /// The setting with firmware value `value`, if there is one.
            pub const fn from_value(value: u8) -> Option<Self> {
                match value { $($value => Some(Self::$variant),)+ _ => None }
            }
        }
    )+};
}

numbered! {
    PedalMode { Soft = 0, Medium = 1, Hard = 2 }
    AlarmMode { AllOn = 0, Level1Off = 1, Level2Off = 2, Level3Off = 3 }
    RollCutoff { Low = 0, Medium = 1, High = 2 }
    Units { Metric = 0, Imperial = 1 }
    LightMode { Off = 0, On = 1, Strobe = 2 }
    DriveMode { Voltage = 0, Current = 1 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_round_trip() {
        for value in 0..=3 {
            assert_eq!(AlarmMode::from_value(value).map(AlarmMode::value), Some(value));
        }
        assert_eq!(PedalMode::from_value(2), Some(PedalMode::Hard));
        assert_eq!(PedalMode::from_value(3), None);
        assert_eq!(LightMode::Strobe.value(), 2);
        assert_eq!(Units::from_value(1), Some(Units::Imperial));
        assert_eq!(DriveMode::from_value(1), Some(DriveMode::Current));
        assert_eq!(RollCutoff::from_value(0), Some(RollCutoff::Low));
    }
}

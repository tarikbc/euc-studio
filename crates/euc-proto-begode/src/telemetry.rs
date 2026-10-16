//! Typed telemetry frames.
//!
//! Field map: the Begode Extreme build GW1825020, traced from the firmware code and checked
//! byte for byte against a capture from a real wheel (`docs/protocol/begode.md` describes every
//! field, with the same names as here). It corrects the older community map in
//! several places: frame 00 bytes 12-13 are the board NTC (not the IMU), bytes 14-15 echo
//! settings (not hardware PWM), frame 07 bytes 8-9 are an integer percentage, and frame 07 bytes
//! 6-7 (motor temperature in older apps) are always zero.
//!
//! Every struct keeps every payload byte, so [`Telemetry::encode`] gives back the exact frame
//! [`Telemetry::decode`] read, unknown fields included. Raw fields hold wire values; methods
//! convert them to physical units.

use crate::frame::Frame;
use crate::settings::{AlarmMode, DriveMode, LightMode, PedalMode, RollCutoff, Units};

/// One decoded telemetry frame.
///
/// The wheel sends one group of five frames about every 0.31 s, always in this order: 00, 01,
/// 02 or 03, 04, 07. Frame 01 cycles through BMS blocks 0..3; frames 02/03 cycle through pack A
/// pages 0..3, then pack B pages 0..3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Telemetry {
    /// Type 00.
    Live(Live),
    /// Type 01.
    Bms(Bms),
    /// Types 02 and 03.
    Cells(Cells),
    /// Type 04.
    Status(Status),
    /// Type 07.
    Power(Power),
    /// A type this decoder does not know, kept as is.
    Other(Frame),
}

impl Telemetry {
    /// Decodes `frame` by its type byte.
    pub fn decode(frame: &Frame) -> Self {
        match frame.kind() {
            Live::KIND => Self::Live(Live::decode(frame)),
            Bms::KIND => Self::Bms(Bms::decode(frame)),
            Cells::KIND_A | Cells::KIND_B => Self::Cells(Cells::decode(frame)),
            Status::KIND => Self::Status(Status::decode(frame)),
            Power::KIND => Self::Power(Power::decode(frame)),
            _ => Self::Other(*frame),
        }
    }

    /// The frame on the wire. `Telemetry::decode(&f).encode() == f` for every frame `f`.
    pub fn encode(&self) -> Frame {
        match self {
            Self::Live(live) => live.encode(),
            Self::Bms(bms) => bms.encode(),
            Self::Cells(cells) => cells.encode(),
            Self::Status(status) => status.encode(),
            Self::Power(power) => power.encode(),
            Self::Other(frame) => *frame,
        }
    }
}

/// Frame 00: battery voltage, speed, range, trip, phase current, board temperature, settings echo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Live {
    /// Bytes 2-3: battery voltage in 0.02 V, from the controller's own ADC. The firmware clamps
    /// it at 134.40 V (6720).
    pub voltage: u16,
    /// Bytes 4-5: speed in 0.01 m/s (signed; in 0.01 mph-equivalent units when the wheel is set to
    /// imperial, see [`Units`]).
    pub speed: i16,
    /// Bytes 6-7: estimated remaining range in km or mi, computed from the voltage alone.
    pub range: u16,
    /// Bytes 8-9: trip distance in m (or the imperial equivalent). Wraps at 65,535.
    pub trip: u16,
    /// Bytes 10-11: phase current in 0.01 A (signed).
    pub phase_current: i16,
    /// Bytes 12-13: board (MOSFET) NTC temperature, encoded so that the MPU-6050 formula
    /// `raw / 340 + 36.53` reads within about 1 °C. See [`temperature_c`](Self::temperature_c).
    pub temperature: i16,
    /// Bytes 14-15: settings echo.
    pub settings: LiveSettings,
    /// Bytes 16-17: beeper volume 0..9 (`W` `B` d).
    pub beeper_volume: u16,
    /// Byte 19: sub-index, always 0x18 in GW1825020.
    pub sub: u8,
}

impl Live {
    /// Frame type.
    pub const KIND: u8 = 0x00;

    /// Battery voltage in volts.
    pub fn voltage_v(&self) -> f64 {
        f64::from(self.voltage) / 50.0
    }

    /// Speed in km/h (mph when the wheel is set to imperial).
    pub fn speed_kmh(&self) -> f64 {
        f64::from(self.speed) * 0.036
    }

    /// Phase current in amperes.
    pub fn phase_current_a(&self) -> f64 {
        f64::from(self.phase_current) / 100.0
    }

    /// Board temperature in °C, by the exact inverse of the firmware's encoding
    /// `(T - 21) * 334 - 5280`.
    pub fn temperature_c(&self) -> f64 {
        (f64::from(self.temperature) + 5280.0) / 334.0 + 21.0
    }

    fn decode(f: &Frame) -> Self {
        Self {
            voltage: f.u16_at(2),
            speed: f.i16_at(4),
            range: f.u16_at(6),
            trip: f.u16_at(8),
            phase_current: f.i16_at(10),
            temperature: f.i16_at(12),
            settings: LiveSettings(f.u16_at(14)),
            beeper_volume: f.u16_at(16),
            sub: f.sub(),
        }
    }

    fn encode(&self) -> Frame {
        let mut f = Frame::new(Self::KIND, self.sub);
        f.set_u16(2, self.voltage)
            .set_i16(4, self.speed)
            .set_u16(6, self.range)
            .set_u16(8, self.trip)
            .set_i16(10, self.phase_current)
            .set_i16(12, self.temperature)
            .set_u16(14, self.settings.0)
            .set_u16(16, self.beeper_volume);
        f
    }
}

/// Frame 00 bytes 14-15, the settings echo: `WR << 10 | WU << 5 | e/x << 3 | drive mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LiveSettings(pub u16);

impl LiveSettings {
    /// Turn compensation gain (`W` `R` d), bits 10-13.
    pub const fn turn_compensation(self) -> u8 {
        (self.0 >> 10) as u8 & 0xF
    }

    /// Pedal trim (`W` `U` d), bits 5-8.
    pub const fn pedal_trim(self) -> u8 {
        (self.0 >> 5) as u8 & 0xF
    }

    /// PWM tiltback enabled (`e` / `x`), bit 3.
    pub const fn pwm_tiltback(self) -> bool {
        self.0 & 0x08 != 0
    }

    /// Motor drive mode, bit 0.
    pub const fn drive_mode(self) -> DriveMode {
        if self.0 & 1 == 0 { DriveMode::Voltage } else { DriveMode::Current }
    }
}

/// Frame 01: one smart-BMS block (sub-index 0..3) plus controller values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bms {
    /// Bytes 2-3: PWM alarm threshold in % (`W` `P` dd).
    pub pwm_alarm: u16,
    /// Bytes 4-5: a speed value in 0.1 m/s, latched while the motor command exceeds a limit.
    /// Meaning UNKNOWN.
    pub speed_copy: i16,
    /// Bytes 6-7: battery voltage in 0.1 V, from the controller's ADC (not from the BMS).
    pub voltage: u16,
    /// Bytes 8-9: current of this BMS block in 0.1 A (signed).
    pub current: i16,
    /// Bytes 10-11: highest of the 12 BMS temperatures, °C.
    pub temp_max: i16,
    /// Bytes 12-13: lowest of the 12 BMS temperatures, °C.
    pub temp_min: i16,
    /// Bytes 14-15: the block's last reply word (offset 42). Meaning UNKNOWN; 651 on the
    /// maintainer's wheel.
    pub word42: u16,
    /// Bytes 16-17: the block's status word; 0x7D01 when healthy. See [`healthy`](Self::healthy).
    pub status: u16,
    /// Byte 19: BMS block 0..3.
    pub block: u8,
}

impl Bms {
    /// Frame type.
    pub const KIND: u8 = 0x01;

    /// Battery voltage in volts.
    pub fn voltage_v(&self) -> f64 {
        f64::from(self.voltage) / 10.0
    }

    /// Block current in amperes.
    pub fn current_a(&self) -> f64 {
        f64::from(self.current) / 10.0
    }

    /// The firmware's own health test of the status word. Three failed polls in a row on any
    /// block latch a BMS fault that beeps every 10 s until power-off.
    pub const fn healthy(&self) -> bool {
        let [high, low] = self.status.to_be_bytes();
        high & 0x30 != 0 && high & 0x0C == 0x0C && low & 0x70 == 0
    }

    fn decode(f: &Frame) -> Self {
        Self {
            pwm_alarm: f.u16_at(2),
            speed_copy: f.i16_at(4),
            voltage: f.u16_at(6),
            current: f.i16_at(8),
            temp_max: f.i16_at(10),
            temp_min: f.i16_at(12),
            word42: f.u16_at(14),
            status: f.u16_at(16),
            block: f.sub(),
        }
    }

    fn encode(&self) -> Frame {
        let mut f = Frame::new(Self::KIND, self.block);
        f.set_u16(2, self.pwm_alarm)
            .set_i16(4, self.speed_copy)
            .set_u16(6, self.voltage)
            .set_i16(8, self.current)
            .set_i16(10, self.temp_max)
            .set_i16(12, self.temp_min)
            .set_u16(14, self.word42)
            .set_u16(16, self.status);
        f
    }
}

/// Which battery pack a cell frame belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Pack {
    /// Frame type 02: BMS blocks 0 and 1, 32 cells.
    A,
    /// Frame type 03: BMS blocks 2 and 3, 32 cells.
    B,
}

/// Frames 02 and 03: eight cell voltages of one pack.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cells {
    /// Pack A (type 02) or B (type 03).
    pub pack: Pack,
    /// Byte 19: page 0..3; this frame holds cells `8 * page .. 8 * page + 8` of the pack.
    pub page: u8,
    /// Bytes 2-17: cell voltages in mV.
    pub millivolts: [u16; 8],
}

impl Cells {
    /// Frame type of pack A.
    pub const KIND_A: u8 = 0x02;
    /// Frame type of pack B.
    pub const KIND_B: u8 = 0x03;

    /// Index within the pack (0..32) of the first cell in this frame.
    pub fn first_cell(&self) -> usize {
        usize::from(self.page) * 8
    }

    fn decode(f: &Frame) -> Self {
        Self {
            pack: if f.kind() == Self::KIND_A { Pack::A } else { Pack::B },
            page: f.sub(),
            millivolts: std::array::from_fn(|k| f.u16_at(2 + 2 * k)),
        }
    }

    fn encode(&self) -> Frame {
        let kind = match self.pack {
            Pack::A => Self::KIND_A,
            Pack::B => Self::KIND_B,
        };
        let mut f = Frame::new(kind, self.page);
        for (k, &mv) in self.millivolts.iter().enumerate() {
            f.set_u16(2 + 2 * k, mv);
        }
        f
    }
}

/// Frame 04: odometer, ride settings, auto power-off, tiltback, alerts, lights.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Bytes 2-5: odometer in m (or the imperial equivalent).
    pub odometer: u32,
    /// Bytes 6-7: pedal, speed-alarm, roll cut-off and units modes.
    pub modes: Modes,
    /// Bytes 8-9: seconds left before the idle auto power-off (7200 s after the wheel last moved).
    pub auto_off: u16,
    /// Bytes 10-11: tiltback speed in km/h; 200 means off.
    pub tiltback: u16,
    /// Byte 12: always 0 (high byte of an 8-bit value).
    pub byte12: u8,
    /// Byte 13: the `W` `M` d value, 0 or 1. Meaning UNKNOWN; every third light-button press
    /// also toggles it.
    pub wm: u8,
    /// Byte 14: alert flags.
    pub alerts: Alerts,
    /// Byte 15: light state, see [`light`](Self::light).
    pub light: u8,
    /// Bytes 16-17: constant 0x0012 in GW1825020. Meaning UNKNOWN (model or protocol id?).
    pub tag: u16,
    /// Byte 19: sub-index, always 0x18 in GW1825020.
    pub sub: u8,
}

impl Status {
    /// Frame type.
    pub const KIND: u8 = 0x04;

    /// Tiltback value that means "off".
    pub const TILTBACK_OFF: u16 = 200;

    /// Light state, if the byte holds a known one.
    pub const fn light(&self) -> Option<LightMode> {
        LightMode::from_value(self.light)
    }

    fn decode(f: &Frame) -> Self {
        Self {
            odometer: f.u32_at(2),
            modes: Modes(f.u16_at(6)),
            auto_off: f.u16_at(8),
            tiltback: f.u16_at(10),
            byte12: f.u8_at(12),
            wm: f.u8_at(13),
            alerts: Alerts(f.u8_at(14)),
            light: f.u8_at(15),
            tag: f.u16_at(16),
            sub: f.sub(),
        }
    }

    fn encode(&self) -> Frame {
        let mut f = Frame::new(Self::KIND, self.sub);
        f.set_u32(2, self.odometer)
            .set_u16(6, self.modes.0)
            .set_u16(8, self.auto_off)
            .set_u16(10, self.tiltback)
            .set_u8(12, self.byte12)
            .set_u8(13, self.wm)
            .set_u8(14, self.alerts.0)
            .set_u8(15, self.light)
            .set_u16(16, self.tag);
        f
    }
}

/// Frame 04 bytes 6-7: `pedal << 13 | alarms << 10 | roll << 7 | miles`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Modes(pub u16);

impl Modes {
    /// Pedal mode, bits 13-14.
    pub const fn pedal_mode(self) -> Option<PedalMode> {
        PedalMode::from_value((self.0 >> 13) as u8 & 3)
    }

    /// Speed alarm mode, bits 10-11.
    pub const fn alarm_mode(self) -> Option<AlarmMode> {
        AlarmMode::from_value((self.0 >> 10) as u8 & 3)
    }

    /// Roll cut-off, bits 7-8.
    pub const fn roll_cutoff(self) -> Option<RollCutoff> {
        RollCutoff::from_value((self.0 >> 7) as u8 & 3)
    }

    /// Units, bit 0.
    pub const fn units(self) -> Units {
        if self.0 & 1 == 0 { Units::Metric } else { Units::Imperial }
    }
}

/// Frame 04 byte 14: alert flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Alerts(pub u8);

impl Alerts {
    /// Bit 0: PWM above about 95 % of the `W` `P` threshold (also forces the beeper to full volume).
    pub const HIGH_POWER: u8 = 1 << 0;
    /// Bit 1: phase-current sensor offset fault at power-on; the motor stays off. (The older
    /// community map calls this a speed alarm.)
    pub const CURRENT_SENSOR: u8 = 1 << 1;
    /// Bit 2: the IMU did not answer at power-on; the motor stays off. (The older community map
    /// calls this a speed alarm.)
    pub const IMU: u8 = 1 << 2;
    /// Bit 3: battery below 105.60 V.
    pub const LOW_VOLTAGE: u8 = 1 << 3;
    /// Bit 4: battery at or above 137.93 V.
    pub const OVER_VOLTAGE: u8 = 1 << 4;
    /// Bit 5: board temperature above about 80 °C.
    pub const OVER_TEMPERATURE: u8 = 1 << 5;
    /// Bit 6: hall sensor state 0 or 7 persists.
    pub const HALL: u8 = 1 << 6;
    /// Bit 7: transport (lock) mode, set by five button presses at rest.
    pub const TRANSPORT: u8 = 1 << 7;

    /// True when every bit of `flags` is set.
    pub const fn contains(self, flags: u8) -> bool {
        self.0 & flags == flags
    }

    /// True when no alert is set.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// Frame 07: battery current, settings echo, PWM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Power {
    /// Bytes 2-3: battery (DC bus) current, likely in 0.01 A (signed).
    pub battery_current: i16,
    /// Bytes 4-5: settings echo `(WC & 0xF) << 4 | (WX & 0xF)`.
    pub settings: u16,
    /// Bytes 6-7: older apps read motor temperature here; GW1825020 always sends 0.
    pub motor_temperature: i16,
    /// Bytes 8-9: PWM (motor voltage command) in whole % (signed), not 0.01 %.
    pub pwm: i16,
    /// Bytes 10-17: always 0 in GW1825020.
    pub tail: [u8; 8],
    /// Byte 19: sub-index, always 0x18 in GW1825020.
    pub sub: u8,
}

impl Power {
    /// Frame type.
    pub const KIND: u8 = 0x07;

    /// Battery current in amperes (unit INFERRED).
    pub fn battery_current_a(&self) -> f64 {
        f64::from(self.battery_current) / 100.0
    }

    /// The `W` `X` d value echoed in bits 0-3.
    pub const fn wx(&self) -> u8 {
        self.settings as u8 & 0xF
    }

    /// The `W` `C` d value echoed in bits 4-7.
    pub const fn wc(&self) -> u8 {
        (self.settings >> 4) as u8 & 0xF
    }

    fn decode(f: &Frame) -> Self {
        Self {
            battery_current: f.i16_at(2),
            settings: f.u16_at(4),
            motor_temperature: f.i16_at(6),
            pwm: f.i16_at(8),
            tail: std::array::from_fn(|k| f.u8_at(10 + k)),
            sub: f.sub(),
        }
    }

    fn encode(&self) -> Frame {
        let mut f = Frame::new(Self::KIND, self.sub);
        f.set_i16(2, self.battery_current)
            .set_u16(4, self.settings)
            .set_i16(6, self.motor_temperature)
            .set_i16(8, self.pwm);
        for (k, &byte) in self.tail.iter().enumerate() {
            f.set_u8(10 + k, byte);
        }
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(hex: &str) -> Frame {
        let bytes: Vec<u8> = hex.split_whitespace().map(|h| u8::from_str_radix(h, 16).unwrap()).collect();
        Frame::from_bytes(&bytes).unwrap()
    }

    #[test]
    fn live_frame_from_the_capture() {
        let f = frame("55 aa 19 55 00 00 00 75 00 06 00 be f0 fa 14 a9 00 09 00 18 5a 5a 5a 5a");
        let Telemetry::Live(live) = Telemetry::decode(&f) else { panic!("type 00") };
        assert!((live.voltage_v() - 129.70).abs() < 1e-9);
        assert_eq!(live.speed_kmh(), 0.0);
        assert_eq!((live.range, live.trip), (117, 6));
        assert!((live.phase_current_a() - 1.90).abs() < 1e-9);
        assert!((live.temperature_c() - 25.293).abs() < 1e-3);
        assert_eq!(live.settings.turn_compensation(), 5);
        assert_eq!(live.settings.pedal_trim(), 5);
        assert!(live.settings.pwm_tiltback());
        assert_eq!(live.settings.drive_mode(), DriveMode::Current);
        assert_eq!(live.beeper_volume, 9);
        assert_eq!(Telemetry::Live(live).encode(), f);
    }

    #[test]
    fn bms_frame_and_health_check() {
        let f = frame("55 aa 00 50 00 00 05 11 ff ff 00 17 00 16 02 8b 7d 01 01 01 5a 5a 5a 5a");
        let Telemetry::Bms(bms) = Telemetry::decode(&f) else { panic!("type 01") };
        assert_eq!(bms.pwm_alarm, 80);
        assert!((bms.voltage_v() - 129.7).abs() < 1e-9);
        assert!((bms.current_a() + 0.1).abs() < 1e-9);
        assert_eq!((bms.temp_max, bms.temp_min, bms.word42, bms.block), (23, 22, 651, 1));
        assert!(bms.healthy());
        assert!(!Bms { status: 0x4D01, ..bms }.healthy(), "bits 4-5 of the high byte must not both be 0");
        assert!(!Bms { status: 0x7901, ..bms }.healthy(), "bits 2-3 of the high byte must be 0b11");
        assert!(!Bms { status: 0x7D11, ..bms }.healthy(), "bits 4-6 of the low byte must be 0");
        assert_eq!(Telemetry::Bms(bms).encode(), f);
    }

    #[test]
    fn cell_frames_map_to_packs_and_pages() {
        let f = frame("55 aa 0f ea 0f ea 0f eb 0f eb 0f eb 0f eb 0f ea 0f eb 03 02 5a 5a 5a 5a");
        let Telemetry::Cells(cells) = Telemetry::decode(&f) else { panic!("type 03") };
        assert_eq!(cells.pack, Pack::B);
        assert_eq!(cells.first_cell(), 16);
        assert_eq!(cells.millivolts, [4074, 4074, 4075, 4075, 4075, 4075, 4074, 4075]);
        assert_eq!(Telemetry::Cells(cells).encode(), f);
    }

    #[test]
    fn status_frame_settings_and_alerts() {
        let f = frame("55 aa 00 16 68 fe 48 00 1b a7 00 45 00 01 00 00 00 12 04 18 5a 5a 5a 5a");
        let Telemetry::Status(status) = Telemetry::decode(&f) else { panic!("type 04") };
        assert_eq!(status.odometer, 1_468_670);
        assert_eq!(status.modes.pedal_mode(), Some(PedalMode::Hard));
        assert_eq!(status.modes.alarm_mode(), Some(AlarmMode::Level2Off));
        assert_eq!(status.modes.roll_cutoff(), Some(RollCutoff::Low));
        assert_eq!(status.modes.units(), Units::Metric);
        assert_eq!((status.auto_off, status.tiltback, status.wm, status.tag), (7079, 69, 1, 0x12));
        assert!(status.alerts.is_empty());
        assert_eq!(status.light(), Some(LightMode::Off));
        assert_eq!(Telemetry::Status(status).encode(), f);

        let alerts = Alerts(Alerts::IMU | Alerts::HALL);
        assert!(alerts.contains(Alerts::IMU) && alerts.contains(Alerts::HALL));
        assert!(!alerts.contains(Alerts::IMU | Alerts::LOW_VOLTAGE));
    }

    #[test]
    fn power_frame() {
        let f = frame("55 aa ff 67 00 00 00 00 ff ff 00 00 00 00 00 00 00 00 07 18 5a 5a 5a 5a");
        let Telemetry::Power(power) = Telemetry::decode(&f) else { panic!("type 07") };
        assert!((power.battery_current_a() + 1.53).abs() < 1e-9);
        assert_eq!((power.pwm, power.wc(), power.wx()), (-1, 0, 0));
        assert_eq!(Power { settings: 0x35, ..power }.wc(), 3);
        assert_eq!(Power { settings: 0x35, ..power }.wx(), 5);
        assert_eq!(Telemetry::Power(power).encode(), f);
    }

    #[test]
    fn unknown_types_pass_through() {
        let f = frame("55 aa 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 09 00 5a 5a 5a 5a");
        assert_eq!(Telemetry::decode(&f), Telemetry::Other(f));
        assert_eq!(Telemetry::decode(&f).encode(), f);
    }
}

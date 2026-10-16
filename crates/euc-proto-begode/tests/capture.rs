//! The real 25 s idle capture from a Begode Extreme (`tests/data/begode-extreme/README.md`)
//! must reassemble and decode to the values documented for it.

use euc_proto_begode::{
    AlarmMode, Assembler, DriveMode, Frame, LightMode, Pack, PedalMode, RollCutoff, Segment, Telemetry, Units,
};

/// One BLE notification per line: `<seconds> <hex bytes>`.
fn notifications() -> Vec<Vec<u8>> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/data/begode-extreme/idle-2026-10-15.hex");
    let text = std::fs::read_to_string(path).expect("capture file");
    text.lines()
        .filter(|line| !line.trim().is_empty())
        .map(|line| line.split_whitespace().skip(1).map(|h| u8::from_str_radix(h, 16).expect("hex byte")).collect())
        .collect()
}

fn frames() -> Vec<Frame> {
    let mut assembler = Assembler::new();
    let mut frames = Vec::new();
    for notification in notifications() {
        assembler.push(&notification);
        for segment in assembler.by_ref() {
            match segment {
                Segment::Frame(frame) => frames.push(frame),
                Segment::Text(text) => panic!("unexpected text {text:02x?}"),
            }
        }
    }
    assert_eq!(assembler.finish(), None, "nothing left over");
    assert_eq!(assembler.stats().text_bytes, 0);
    frames
}

#[test]
fn reassembles_380_frames_from_456_notifications() {
    let notifications = notifications();
    assert_eq!(notifications.len(), 456);
    assert!(notifications.iter().all(|n| n.len() <= 20), "BLE notifications carry at most 20 bytes");
    assert_eq!(frames().len(), 380);
}

#[test]
fn frames_come_in_76_groups_in_firmware_order() {
    let frames = frames();
    let mut pack = None;
    for (n, group) in frames.chunks(5).enumerate() {
        let kinds: Vec<u8> = group.iter().map(Frame::kind).collect();
        assert!(kinds == [0, 1, 2, 4, 7] || kinds == [0, 1, 3, 4, 7], "group {n}: {kinds:?}");
        // Frame 01 and the cell frame share the page counter; the pack flips after page 3.
        let page = group[1].sub();
        assert_eq!(group[2].sub(), page);
        assert_eq!(page, (n % 4) as u8);
        if page == 0 {
            assert_ne!(pack, Some(kinds[2]), "group {n}: pack toggles every four groups");
            pack = Some(kinds[2]);
        }
        assert_eq!(pack, Some(kinds[2]));
    }
    assert_eq!(frames.len() / 5, 76);
}

#[test]
fn encode_inverts_decode_for_every_frame() {
    for frame in frames() {
        assert_eq!(Telemetry::decode(&frame).encode(), frame, "{frame}");
    }
}

#[test]
fn decodes_the_documented_values() {
    let mut auto_off = Vec::new();
    for frame in frames() {
        match Telemetry::decode(&frame) {
            Telemetry::Live(live) => {
                assert!((live.voltage_v() - 129.70).abs() < 1e-9);
                assert_eq!(live.speed, 0);
                assert_eq!((live.range, live.trip), (117, 6));
                assert!((0.5..=2.4).contains(&live.phase_current_a()), "{}", live.phase_current_a());
                assert!((25.2..25.4).contains(&live.temperature_c()), "{}", live.temperature_c());
                assert_eq!(live.settings.turn_compensation(), 5);
                assert_eq!(live.settings.pedal_trim(), 5);
                assert!(live.settings.pwm_tiltback());
                assert_eq!(live.settings.drive_mode(), DriveMode::Current);
                assert_eq!((live.beeper_volume, live.sub), (9, 0x18));
            }
            Telemetry::Bms(bms) => {
                assert_eq!(bms.pwm_alarm, 80);
                assert!((bms.voltage_v() - 129.7).abs() < 1e-9);
                assert!(bms.current == 0 || bms.current == -1);
                assert_eq!((bms.temp_max, bms.temp_min, bms.word42, bms.status), (23, 22, 651, 0x7D01));
                assert!(bms.healthy());
                assert!(bms.block < 4);
            }
            Telemetry::Cells(cells) => {
                assert!(matches!(cells.pack, Pack::A | Pack::B));
                assert!(cells.millivolts.iter().all(|mv| (4067..=4076).contains(mv)), "{:?}", cells.millivolts);
            }
            Telemetry::Status(status) => {
                assert_eq!(status.odometer, 1_468_670);
                assert_eq!(status.modes.pedal_mode(), Some(PedalMode::Hard));
                assert_eq!(status.modes.alarm_mode(), Some(AlarmMode::Level2Off));
                assert_eq!(status.modes.roll_cutoff(), Some(RollCutoff::Low));
                assert_eq!(status.modes.units(), Units::Metric);
                assert_eq!((status.tiltback, status.wm, status.tag), (69, 1, 0x0012));
                assert!(status.alerts.is_empty());
                assert_eq!(status.light(), Some(LightMode::Off));
                auto_off.push(status.auto_off);
            }
            Telemetry::Power(power) => {
                assert!((-1.71..=0.74).contains(&power.battery_current_a()), "{}", power.battery_current_a());
                assert!(power.pwm == 0 || power.pwm == -1);
                assert_eq!((power.settings, power.motor_temperature, power.tail), (0, 0, [0; 8]));
            }
            Telemetry::Other(frame) => panic!("unknown frame {frame}"),
        }
    }
    // The idle auto power-off counts down over the 25 s: 7079 s to 7056 s.
    assert_eq!((auto_off.first(), auto_off.last()), (Some(&7079), Some(&7056)));
    assert!(auto_off.windows(2).all(|w| w[1] <= w[0]));
}

#[test]
fn resyncs_when_garbage_and_odd_splits_are_mixed_in() {
    // Garbage (including false starts `55 ..` and `55 AA ..`) between some frames.
    let mut stream = Vec::new();
    let mut real = 0;
    for notification in notifications() {
        stream.extend(&notification);
        real += notification.len();
        if real % 24 == 0 && (real / 24) % 3 == 1 {
            stream.extend([0x55, 0x00, 0x5A, 0x55, 0xAA, 0x01]);
        }
    }
    assert!(stream.len() > real + 100, "the test inserts garbage");
    let mut assembler = Assembler::new();
    let mut frames = Vec::new();
    let mut text = 0;
    let mut lcg: u32 = 12_345;
    let mut rest = stream.as_slice();
    while !rest.is_empty() {
        lcg = lcg.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        let (chunk, tail) = rest.split_at(((lcg >> 16) as usize % 31 + 1).min(rest.len()));
        rest = tail;
        assembler.push(chunk);
        for segment in assembler.by_ref() {
            match segment {
                Segment::Frame(frame) => frames.push(frame),
                Segment::Text(bytes) => text += bytes.len(),
            }
        }
    }
    if let Some(Segment::Text(bytes)) = assembler.finish() {
        text += bytes.len();
    }
    assert_eq!(frames, self::frames(), "every real frame survives, nothing else is taken for one");
    assert_eq!(text, stream.len() - 380 * 24);
}

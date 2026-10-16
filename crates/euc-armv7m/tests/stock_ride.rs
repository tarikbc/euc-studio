//! The stock Begode Extreme firmware rides `ride-20kmh-stop` on our core with the live plant, and
//! the session reports the firmware's own PWM figure next to the plant's true modulation, on one
//! scale.
//!
//! ```text
//! cargo test -p euc-armv7m --release --test stock_ride -- --nocapture
//! ```

use euc_profile::ProfileSource;
use euc_sim::cpu::Backend;
use euc_sim::scenario::{self, Scenario};
use euc_sim::{Machine, MachineConfig, PlantState, Session, WheelFiles};

mod common;

/// Speed above which the ride counts as "at speed", km/h.
const AT_SPEED_KMH: f32 = 15.0;

/// The firmware's figure is its commanded q voltage. The inverter applies |Vd, Vq|, which is at
/// least |Vq|, less what the dead time takes: with the plant's 0.5 µs (a DEFAULT) about 2 % on
/// average and up to 6 % at 20 km/h. With ideal switches the ratio stays at 0.995 or more (the
/// rest is the sample instant falling between a command and its PWM update).
const DEAD_TIME_SHARE: f32 = 0.92;

#[test]
fn reports_the_firmware_pwm_and_the_true_modulation() {
    if cfg!(debug_assertions) {
        eprintln!("skipped: 40 simulated seconds need a release build (--release)");
        return;
    }
    let Some((_, image)) = common::stock() else { return };
    let files = WheelFiles::load(&ProfileSource::Builtin, common::WHEEL).unwrap();
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../scenarios/ride-20kmh-stop.toml");
    let scenario = Scenario::from_toml(&std::fs::read_to_string(path).unwrap()).unwrap();
    let board = files.board_with_settings(scenario.settings.as_deref().unwrap()).unwrap();
    let config = MachineConfig {
        board: &board,
        stockmap: Some(&files.stockmap),
        image: &image,
        load_addr: files.profile.board.app.base,
        hclk: files.hclk(),
        plant: PlantState::default(),
        injections: Vec::new(),
        power_cycles: Vec::new(),
        cpi: 1.25,
        max_quantum: 4000,
        marker_window_s: 4.0,
    };
    let live = files.live_on(scenario.rider, scenario.assist, scenario.terrain.clone()).unwrap();
    let cpu = Backend::Own.factory().unwrap();
    let machine = Machine::new_live(&config, &euc_sim::registry(), cpu, live).unwrap();
    let mut session = Session::new(machine, Some(&files.stockmap), 100.0);

    let (mut at_speed, mut pwm_sum, mut modulation_sum) = (0, 0.0_f32, 0.0_f32);
    let outcome = scenario::run(&scenario, &mut session, |snapshot, _| {
        let (t, s) = (snapshot.t_s, snapshot.signals);
        if t < scenario.release_s {
            return;
        }
        assert!(s.pwm_pct.is_finite() && (0.0..=100.0).contains(&s.pwm_pct), "PWM {} % at {t:.2} s", s.pwm_pct);
        if s.speed_kmh.abs() >= AT_SPEED_KMH {
            at_speed += 1;
            assert!(
                s.modulation_pct >= DEAD_TIME_SHARE * s.pwm_pct,
                "modulation {} % well below the firmware's PWM {} % at {t:.2} s",
                s.modulation_pct,
                s.pwm_pct
            );
            pwm_sum += s.pwm_pct;
            modulation_sum += s.modulation_pct;
        }
    })
    .unwrap();
    let ratio = modulation_sum / pwm_sum;
    eprintln!(
        "{at_speed} samples at speed: PWM {:.2} %, modulation {:.2} % on average (ratio {ratio:.3})",
        pwm_sum / at_speed as f32,
        modulation_sum / at_speed as f32
    );
    assert!(outcome.passed(), "{outcome:#?}");
    assert!(at_speed > 500, "only {at_speed} samples at {AT_SPEED_KMH} km/h or more");
    assert!(pwm_sum > 0.0, "the firmware never reported PWM at speed");
    assert!((0.95..1.1).contains(&ratio), "modulation / PWM averages {ratio}");
}

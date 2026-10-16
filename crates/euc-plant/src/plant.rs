//! The electrical plant: inverter, motor, halls, sensors, battery and heat, on one time grid.

use std::cmp::Ordering;

use serde::Deserialize;
use thiserror::Error;

use crate::battery::{Battery, BatteryError, BatteryParams, PackReading};
use crate::fault::{Faults, PlantFault, SocJump};
use crate::halls::HallParams;
use crate::inverter::{Circuit, InverterParams, LegDrive, high_side_share};
use crate::motor::{Dq, MotorParams, Pmsm, inverse_park, park};
use crate::rng::Rng;
use crate::sensors::{Measurands, Phase, SensorParams};
use crate::thermal::{Node, ThermalParams};

/// Everything in `plant.toml`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlantParams {
    /// File format version; 1.
    pub schema: u32,
    /// Integration step, microseconds.
    #[serde(default = "default_grid_us")]
    pub grid_us: f64,
    /// Seed of the sensor and hall noise.
    #[serde(default)]
    pub seed: u64,
    /// The motor.
    pub motor: MotorParams,
    /// The inverter.
    pub inverter: InverterParams,
    /// The hall sensors.
    pub halls: HallParams,
    /// The analog front end.
    pub sensors: SensorParams,
    /// The battery.
    pub battery: BatteryParams,
    /// Heat.
    pub thermal: ThermalParams,
    /// The mechanics (`[body]`): read by `euc-physics` through the simulator, ignored here.
    #[serde(default)]
    pub body: Option<toml::Table>,
}

const fn default_grid_us() -> f64 {
    5.0
}

/// A `plant.toml` that cannot be used.
#[derive(Debug, Error)]
pub enum PlantError {
    /// Not valid TOML for [`PlantParams`].
    #[error("plant.toml: {0}")]
    Parse(#[from] toml::de::Error),
    /// An unsupported `schema`.
    #[error("plant.toml schema {0} is not supported (expected 1)")]
    Schema(u32),
    /// A value out of its range.
    #[error("plant.toml: {0}")]
    Invalid(String),
    /// The battery section.
    #[error("plant.toml [battery]: {0}")]
    Battery(#[from] BatteryError),
}

impl PlantParams {
    /// Parses and checks `plant.toml` text.
    pub fn from_toml(text: &str) -> Result<Self, PlantError> {
        let params: Self = toml::from_str(text)?;
        params.validate()?;
        Ok(params)
    }

    /// Checks ranges that the types cannot.
    pub fn validate(&self) -> Result<(), PlantError> {
        if self.schema != 1 {
            return Err(PlantError::Schema(self.schema));
        }
        let m = &self.motor;
        let positive = [
            ("grid_us", self.grid_us),
            ("motor.resistance_ohm", m.resistance_ohm),
            ("motor.ld_h", m.ld_h),
            ("motor.lq_h", m.lq_h),
            ("battery.capacity_ah", self.battery.capacity_ah),
            ("battery.r0_ohm", self.battery.r0_ohm),
            ("battery.r1_ohm", self.battery.r1_ohm),
            ("battery.tau_s", self.battery.tau_s),
        ];
        if let Some((name, _)) = positive.iter().find(|(_, v)| v.partial_cmp(&0.0) != Some(Ordering::Greater)) {
            return Err(PlantError::Invalid(format!("{name} must be positive")));
        }
        if m.pole_pairs == 0 || m.direction.abs() != 1 {
            return Err(PlantError::Invalid("motor.pole_pairs must be positive and motor.direction ±1".into()));
        }
        self.battery.validate()?;
        Ok(())
    }
}

/// A snapshot of the plant for telemetry and tests.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantState {
    /// Engine cycle of the snapshot.
    pub time: u64,
    /// Electrical angle, radians (not wrapped).
    pub electrical_angle: f64,
    /// Electrical speed, rad/s.
    pub electrical_speed: f64,
    /// Rotor-frame currents, ampere.
    pub current_dq: Dq,
    /// Phase currents a, b, c, ampere (positive into the motor).
    pub phase_current: [f64; 3],
    /// Torque on the wheel, newton-metre, positive forward.
    pub wheel_torque: f64,
    /// DC bus voltage, volt.
    pub bus_v: f64,
    /// Battery current, ampere (positive = discharge).
    pub bus_a: f64,
    /// Mean state of charge of the packs, 0..=1.
    pub soc: f64,
    /// Power-stage temperature, °C.
    pub stage_c: f64,
    /// Winding temperature, °C.
    pub winding_c: f64,
    /// Hall code (bit 0 = sensor a).
    pub halls: u8,
}

/// Wheel motion as physics last reported it; the plant extrapolates at constant speed.
#[derive(Debug, Clone, Copy)]
struct Wheel {
    at: u64,
    angle: f64,
    speed: f64,
}

/// The electrical plant.
///
/// Time is the engine's cycle count. [`advance`](Self::advance) integrates on a fixed grid
/// (`grid_us`, 5 µs by default) up to the cycle it is given, with the inputs that are in force:
/// the three inverter legs ([`set_leg`](Self::set_leg)) and the wheel motion
/// ([`set_wheel`](Self::set_wheel)). Outputs are read at the last grid point: ADC counts
/// ([`adc_raw`](Self::adc_raw)), the hall code, the wheel torque, the battery as its BMS sees it.
#[derive(Debug, Clone)]
pub struct ElectricalPlant {
    params: PlantParams,
    hclk: f64,
    grid: u64,
    h: f64,
    time: u64,
    motor: Pmsm,
    phase_current: [f64; 3],
    legs: [LegDrive; 3],
    wheel: Wheel,
    battery: Battery,
    bus_a: f64,
    stage: Node,
    winding: Node,
    halls: u8,
    rng: Rng,
    torque_sum: f64,
    torque_time: f64,
    faults: Faults,
    /// Kelvin a `stage_overtemp` fault adds to the power stage.
    stage_extra_k: f64,
}

impl ElectricalPlant {
    /// A plant at rest at cycle 0 on an engine of `hclk` cycles per second, all legs off.
    pub fn new(params: PlantParams, hclk: u64) -> Result<Self, PlantError> {
        params.validate()?;
        let hclk = hclk as f64;
        let grid = (params.grid_us * 1e-6 * hclk).round().max(1.0) as u64;
        let ambient = params.thermal.ambient_c;
        let battery = Battery::new(params.battery.clone(), ambient)?;
        let mut plant = Self {
            motor: Pmsm::new(params.motor),
            hclk,
            grid,
            h: grid as f64 / hclk,
            time: 0,
            phase_current: [0.0; 3],
            legs: [LegDrive::Off; 3],
            wheel: Wheel { at: 0, angle: 0.0, speed: 0.0 },
            battery,
            bus_a: 0.0,
            stage: Node::new(params.thermal.stage, ambient),
            winding: Node::new(params.thermal.motor, ambient),
            halls: 0,
            rng: Rng::new(params.seed),
            torque_sum: 0.0,
            torque_time: 0.0,
            faults: Faults::new(&params),
            stage_extra_k: 0.0,
            params,
        };
        plant.halls = plant.params.halls.code(plant.electrical_angle(0), &mut plant.rng);
        Ok(plant)
    }

    /// The parameters.
    pub const fn params(&self) -> &PlantParams {
        &self.params
    }

    /// The cycle the state is integrated to (a multiple of the grid).
    pub const fn time(&self) -> u64 {
        self.time
    }

    /// Integrates every whole grid step up to `now`.
    pub fn advance(&mut self, now: u64) {
        while self.time + self.grid <= now {
            self.step();
        }
    }

    /// Adds a fault for the cycles `from..until` (`None`: to the end of the run). The plant
    /// applies it from the first grid point at or after `from` to the first at or after `until`;
    /// a fault already due takes effect at once.
    pub fn add_fault(&mut self, from: u64, until: Option<u64>, fault: PlantFault) -> Result<(), PlantError> {
        fault.validate(&self.params).map_err(PlantError::Invalid)?;
        if until.is_some_and(|until| until <= from) {
            return Err(PlantError::Invalid(format!("fault {}: its window ends before it starts", fault.kind())));
        }
        self.faults.add(from, until, fault);
        if self.refresh_faults() {
            let theta = self.electrical_angle(self.time);
            self.halls = self.faults.effects().halls.code(theta, &mut self.rng);
        }
        Ok(())
    }

    /// Changes what drives the leg of `phase`, from `now` on.
    pub fn set_leg(&mut self, now: u64, phase: Phase, drive: LegDrive) {
        self.advance(now);
        self.legs[phase.index()] = drive;
    }

    /// The wheel's angle (radians) and speed (rad/s) at `now`, forward positive, from physics.
    pub fn set_wheel(&mut self, now: u64, angle_rad: f64, speed_rad_s: f64) {
        self.advance(now);
        self.wheel = Wheel { at: now, angle: angle_rad, speed: speed_rad_s };
    }

    /// Torque on the wheel now, newton-metre, positive forward.
    pub fn torque_nm(&self) -> f64 {
        self.params.motor.wheel_torque(self.motor.current())
    }

    /// Mean wheel torque since the previous call (or since creation), newton-metre: what a
    /// physics step should apply so the energy matches.
    pub fn mean_torque_nm(&mut self) -> f64 {
        let mean = if self.torque_time > 0.0 { self.torque_sum / self.torque_time } else { self.torque_nm() };
        (self.torque_sum, self.torque_time) = (0.0, 0.0);
        mean
    }

    /// Hall code (bit 0 = sensor a, bit 1 = b, bit 2 = c).
    pub const fn halls(&self) -> u8 {
        self.halls
    }

    /// The first grid point at which the hall code changes if the wheel keeps its speed, or
    /// `None` while it stands still.
    ///
    /// The firmware polls the hall lines faster than any sensor samples the plant, so an engine
    /// that advances the plant lazily schedules an event here to put each edge on the wires on
    /// time, and asks again after every [`set_wheel`](Self::set_wheel). It also returns the
    /// grid point where a fault starts or ends, since a stuck or lost line changes the code too.
    /// Noise is left out: with it the code may change a little earlier or later.
    pub fn next_hall_edge(&self) -> Option<u64> {
        let fault = self
            .faults
            .next_change()
            .map(|at| self.time + at.saturating_sub(self.time).div_ceil(self.grid) * self.grid);
        let speed = self.params.motor.electrical_speed(self.wheel.speed).to_degrees();
        if speed == 0.0 {
            return fault;
        }
        let sensed = self.electrical_angle(self.time).to_degrees() + self.params.halls.offset_deg;
        let edge = if speed > 0.0 { (sensed / 60.0).floor() + 1.0 } else { (sensed / 60.0).ceil() - 1.0 } * 60.0;
        // A sensed angle on an edge going down leaves its sector at once.
        let edge = if speed < 0.0 && sensed % 60.0 == 0.0 { sensed } else { edge };
        let steps = ((edge - sensed) / speed * self.hclk / self.grid as f64).ceil().max(1.0);
        let rotor = (steps < 1e12).then(|| self.time + steps as u64 * self.grid);
        rotor.into_iter().chain(fault).min()
    }

    /// The count ADC `adc` converts on `channel`, if a sensor of this plant is wired there.
    ///
    /// Noise is drawn from the seed, the time and the channel, so a reading repeats exactly.
    pub fn adc_raw(&self, adc: u8, channel: u8) -> Option<u16> {
        let mut rng = Rng::new(self.params.seed ^ self.time.rotate_left(17) ^ u64::from(adc) << 8 ^ u64::from(channel));
        let errors = self.faults.effects().sensors;
        let mut sensed = |true_a: f64, k: usize| {
            let e = errors[k];
            e.gain * true_a + e.offset_a + rng.gaussian(e.noise_a)
        };
        let measurands = Measurands {
            phase_a: std::array::from_fn(|k| sensed(self.phase_current[k], k)),
            bus_a: sensed(self.bus_a, 3),
            bus_v: self.battery.voltage(),
            board_c: self.stage_c(),
        };
        self.params.sensors.count(adc, channel, &measurands, &mut rng)
    }

    /// Bus voltage, volt.
    pub const fn bus_v(&self) -> f64 {
        self.battery.voltage()
    }

    /// What each pack's smart BMS reads.
    pub fn battery_readings(&self) -> Vec<PackReading> {
        self.battery.readings()
    }

    /// A snapshot of the state.
    pub fn snapshot(&self) -> PlantState {
        let packs = self.battery.packs();
        PlantState {
            time: self.time,
            electrical_angle: self.electrical_angle(self.time),
            electrical_speed: self.params.motor.electrical_speed(self.wheel.speed),
            current_dq: self.motor.current(),
            phase_current: self.phase_current,
            wheel_torque: self.torque_nm(),
            bus_v: self.battery.voltage(),
            bus_a: self.bus_a,
            soc: packs.iter().map(|p| p.soc).sum::<f64>() / packs.len() as f64,
            stage_c: self.stage_c(),
            winding_c: self.winding.temp_c(),
            halls: self.halls,
        }
    }

    /// Power-stage temperature, with any overheating fault.
    fn stage_c(&self) -> f64 {
        self.stage.temp_c() + self.stage_extra_k
    }

    /// Brings the fault effects up to the plant's time. True if they changed.
    fn refresh_faults(&mut self) -> bool {
        let Some(jumps) = self.faults.refresh(self.time) else { return false };
        for jump in jumps {
            match jump {
                SocJump::Shift { pack, delta } => self.battery.shift_soc(pack, delta),
                SocJump::Set { soc } => {
                    for pack in 0..self.battery.packs().len() {
                        let delta = soc - self.battery.packs()[pack].soc;
                        self.battery.shift_soc(pack, delta);
                    }
                }
            }
        }
        self.battery.set_conditions(&self.faults.effects().packs);
        true
    }

    fn electrical_angle(&self, at: u64) -> f64 {
        let dt = (at as f64 - self.wheel.at as f64) / self.hclk;
        self.params.motor.electrical_angle(self.wheel.angle + self.wheel.speed * dt)
    }

    /// One grid step.
    fn step(&mut self) {
        let h = self.h;
        let motor = self.params.motor;
        let inverter = self.params.inverter;
        let theta = self.electrical_angle(self.time + self.grid / 2);
        let omega_e = motor.electrical_speed(self.wheel.speed);
        let vbus = self.battery.voltage();
        let r = motor.resistance_ohm + inverter.switch_resistance_ohm;
        let mut current = inverse_park(self.motor.current(), theta);
        let active: [Option<f64>; 3] = std::array::from_fn(|k| match self.legs[k] {
            LegDrive::Pwm { duty, period_s } => Some(inverter.pole_voltage(duty, period_s, current[k], vbus)),
            LegDrive::Off => None,
        });
        let cut = self.faults.effects().cut;
        if let ([Some(a), Some(b), Some(c)], [false, false, false]) = (active, cut) {
            self.motor.step(park([a, b, c], theta), omega_e, r, h);
        } else {
            let circuit = Circuit { inductance: f64::midpoint(motor.ld_h, motor.lq_h), resistance: r, dt: h };
            inverter.step_with_open_legs(&mut current, active, motor.back_emf(theta, omega_e), vbus, circuit, cut);
            self.motor.set_current(park(current, theta));
        }
        self.time += self.grid;
        let theta_end = self.electrical_angle(self.time);
        self.phase_current = inverse_park(self.motor.current(), theta_end);
        let share = high_side_share(&self.legs, &self.phase_current);
        self.bus_a = (0..3).map(|k| share[k] * self.phase_current[k]).sum();
        let ambient = self.params.thermal.ambient_c;
        self.battery.step(self.bus_a, ambient, h);
        let (stage_w, winding_w) = self.losses(vbus);
        self.stage.step(stage_w, ambient, h);
        self.winding.step(winding_w, ambient, h);
        self.stage_extra_k = match self.faults.effects().overtemp {
            Some((rate, max_c)) => (self.stage_extra_k + rate * h).min((max_c - self.stage.temp_c()).max(0.0)),
            None if self.stage_extra_k == 0.0 => 0.0,
            None => {
                let node = self.params.thermal.stage;
                self.stage_extra_k * (-h / (node.heat_capacity_j_per_k * node.resistance_k_per_w)).exp()
            }
        };
        self.torque_sum += self.torque_nm() * h;
        self.torque_time += h;
        self.refresh_faults();
        self.halls = self.faults.effects().halls.code(theta_end, &mut self.rng);
    }

    /// Power-stage and winding losses, watt.
    fn losses(&self, vbus: f64) -> (f64, f64) {
        let inverter = self.params.inverter;
        let mut stage = 0.0;
        for (leg, i) in self.legs.iter().zip(self.phase_current) {
            match *leg {
                LegDrive::Pwm { duty, period_s } => {
                    stage += inverter.switch_resistance_ohm * i * i;
                    if duty > 0.0 && duty < 1.0 {
                        // Two transitions per period, each losing about ½·V·I·t.
                        stage += vbus * i.abs() * inverter.switching_time_s / period_s;
                    }
                }
                LegDrive::Off => stage += inverter.diode_drop_v * i.abs(),
            }
        }
        let copper: f64 = self.phase_current.iter().map(|i| i * i).sum::<f64>() * self.params.motor.resistance_ohm;
        (stage, copper)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::fault::{CurrentSensor, HallLine, Level};

    const HCLK: u64 = 72_000_000;
    const MS: u64 = HCLK / 1000;

    fn text() -> String {
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../profiles/begode-extreme/plant.toml"))
            .unwrap()
    }

    /// The Begode Extreme's `plant.toml`.
    pub(crate) fn params() -> PlantParams {
        PlantParams::from_toml(&text()).unwrap()
    }

    fn plant(edit: impl FnOnce(&mut PlantParams)) -> ElectricalPlant {
        let mut p = PlantParams::from_toml(&text()).unwrap();
        edit(&mut p);
        ElectricalPlant::new(p, HCLK).unwrap()
    }

    fn ideal_switches(p: &mut PlantParams) {
        p.inverter.dead_time_s = 0.0;
        p.inverter.switching_time_s = 0.0;
    }

    fn all_legs(plant: &mut ElectricalPlant, now: u64, drive: [LegDrive; 3]) {
        for (phase, drive) in [Phase::A, Phase::B, Phase::C].into_iter().zip(drive) {
            plant.set_leg(now, phase, drive);
        }
    }

    fn pwm(duty: f64) -> LegDrive {
        LegDrive::Pwm { duty, period_s: 1.0 / 12_000.0 }
    }

    /// Battery energy drawn and resistive heat over `steps` grid steps, joule.
    fn energy(plant: &mut ElectricalPlant, steps: u64, wheel_speed: f64) -> (f64, f64, f64) {
        let h = plant.h;
        let r = plant.params.motor.resistance_ohm + plant.params.inverter.switch_resistance_ohm;
        let (mut drawn, mut heat, mut work) = (0.0, 0.0, 0.0);
        let start = plant.time();
        for n in 1..=steps {
            plant.advance(start + n * plant.grid);
            let s = plant.snapshot();
            drawn += s.bus_v * s.bus_a * h;
            heat += r * s.phase_current.iter().map(|i| i * i).sum::<f64>() * h;
            work += s.wheel_torque * wheel_speed * h;
        }
        (drawn, heat, work)
    }

    #[test]
    fn at_rest_it_reads_like_the_capture() {
        let plant = plant(|_| {});
        assert_eq!(plant.adc_raw(1, 0), Some(3086), "129.7 V");
        assert_eq!(plant.adc_raw(1, 11), Some(3377), "25 °C");
        let phases = (plant.adc_raw(1, 4), plant.adc_raw(2, 5), plant.adc_raw(2, 10));
        assert_eq!(phases, (Some(2048), Some(2048), Some(2048)));
        assert_eq!(plant.adc_raw(1, 1), None);
        assert_eq!(plant.halls(), 3, "electrical 0° is firmware −90°: sector code 3");
        let readings = plant.battery_readings();
        assert_eq!((readings.len(), readings[0].cells_mv[0]), (2, 4053));
    }

    #[test]
    fn advances_in_whole_grid_steps() {
        let mut plant = plant(|_| {});
        plant.advance(1000);
        assert_eq!(plant.time(), 720, "two 5 µs steps of 360 cycles");
        plant.advance(1000);
        assert_eq!(plant.time(), 720);
        plant.set_leg(MS, Phase::A, LegDrive::Off);
        assert_eq!(plant.time(), MS);
    }

    #[test]
    fn locked_rotor_current_and_energy_balance() {
        let mut plant = plant(ideal_switches);
        // A voltage vector along phase a: 2 % differential duty, rotor held at 0.
        all_legs(&mut plant, 0, [pwm(0.51), pwm(0.49), pwm(0.49)]);
        let (drawn, heat, _) = energy(&mut plant, 4000, 0.0);
        let s = plant.snapshot();
        // 2/3 of the 2 % pole difference lies across phase a, through 0.082 Ω.
        let expected = 2.0 / 3.0 * 0.02 * s.bus_v / 0.082;
        assert!((s.phase_current[0] - expected).abs() < 0.01 * expected, "{} vs {expected}", s.phase_current[0]);
        let stored = plant.params.motor.magnetic_energy(s.current_dq);
        assert!((drawn - heat - stored).abs() < 0.01 * drawn, "drawn {drawn} J, heat {heat} J, stored {stored} J");
    }

    #[test]
    fn spinning_energy_balance_includes_the_wheel_work() {
        let mut plant = plant(ideal_switches);
        plant.set_wheel(0, 0.0, 5.0);
        all_legs(&mut plant, 0, [pwm(0.55), pwm(0.45), pwm(0.5)]);
        let (drawn, heat, work) = energy(&mut plant, 40_000, 5.0);
        let stored = plant.params.motor.magnetic_energy(plant.snapshot().current_dq);
        let balance = drawn - heat - work - stored;
        assert!(balance.abs() < 0.01 * heat, "drawn {drawn}, heat {heat}, work {work}: off by {balance} J");
    }

    #[test]
    fn short_circuit_brake_opposes_motion() {
        let mut plant = plant(|_| {});
        plant.set_wheel(0, 0.0, 10.0);
        all_legs(&mut plant, 0, [pwm(0.0); 3]);
        plant.advance(50 * MS);
        assert!(plant.torque_nm() < -5.0, "{}", plant.torque_nm());
        plant.set_wheel(50 * MS, 0.0, -10.0);
        plant.advance(100 * MS);
        assert!(plant.torque_nm() > 5.0, "{}", plant.torque_nm());
    }

    #[test]
    fn outputs_off_freewheel_below_and_rectify_above_the_bus() {
        let mut plant = plant(|_| {});
        plant.set_wheel(0, 0.0, 100.0);
        plant.advance(20 * MS);
        assert_eq!(plant.snapshot().phase_current, [0.0; 3], "back-EMF below 129.7 V: no current");
        assert_eq!(plant.torque_nm(), 0.0);
        // 170 rad/s: line back-EMF √3·26·170·0.0205 = 157 V peak.
        plant.set_wheel(20 * MS, 0.0, 170.0);
        plant.advance(40 * MS);
        let _ = plant.mean_torque_nm();
        plant.advance(60 * MS);
        assert!(plant.mean_torque_nm() < -1.0, "rectification brakes");
        assert!(plant.snapshot().bus_a < 0.0, "and charges the battery");
    }

    #[test]
    fn halls_follow_the_wheel() {
        let mut plant = plant(|_| {});
        // One electrical turn = 1/26 of a wheel turn; at 2 rad/s it takes 0.12 s.
        plant.set_wheel(0, 0.0, 2.0);
        let mut codes = vec![plant.halls()];
        for n in 1..=150 {
            plant.advance(n * MS);
            if codes.last() != Some(&plant.halls()) {
                codes.push(plant.halls());
            }
        }
        assert_eq!(codes[..7], [3, 1, 5, 4, 6, 2, 3], "forward: the firmware's negative order");
    }

    #[test]
    fn next_hall_edge_is_the_grid_point_where_the_code_changes() {
        for speed in [37.0, -37.0, 0.4] {
            let mut plant = plant(|_| {});
            assert_eq!(plant.next_hall_edge(), None, "standing still");
            plant.set_wheel(0, 0.013, speed);
            for _ in 0..8 {
                let (code, edge) = (plant.halls(), plant.next_hall_edge().unwrap());
                plant.advance(edge - plant.grid);
                assert_eq!(plant.halls(), code, "{speed} rad/s: no change before {edge}");
                plant.advance(edge);
                assert_ne!(plant.halls(), code, "{speed} rad/s: changes at {edge}");
            }
        }
    }

    #[test]
    fn a_stuck_hall_line_holds_for_its_window() {
        let mut plant = plant(|_| {});
        let stuck = PlantFault::HallStuck { line: HallLine::A, level: Level::High };
        plant.add_fault(20 * MS, Some(80 * MS), stuck).unwrap();
        assert_eq!(plant.next_hall_edge(), Some(20 * MS), "standing still, the fault's start is the next edge");
        plant.set_wheel(0, 0.0, 8.0);
        let mut bit_a = Vec::new();
        for ms in 1..=120 {
            plant.advance(ms * MS);
            bit_a.push(plant.halls() & 1);
        }
        assert!(bit_a[19..79].iter().all(|&b| b == 1), "held high from 20 ms to 79 ms");
        assert!(bit_a[..19].contains(&0) && bit_a[79..].contains(&0), "toggles outside the window");
        let err = plant.add_fault(5, Some(5), stuck).unwrap_err();
        assert_eq!(err.to_string(), "plant.toml: fault hall_stuck: its window ends before it starts");
    }

    #[test]
    fn current_sensor_errors_shift_the_counts() {
        let mut plant = plant(|_| {});
        let fault = PlantFault::CurrentSensor { sensor: CurrentSensor::A, offset_a: 10.0, gain: 1.0, noise_a: 0.0 };
        plant.add_fault(0, None, fault).unwrap();
        // Phase a's sensor is inverted (ADC1 ch4): +10 A reads 47 counts below mid-scale.
        assert_eq!(plant.adc_raw(1, 4), Some(2001));
        assert_eq!(plant.adc_raw(2, 5), Some(2048), "phase b untouched");
        let noisy = PlantFault::CurrentSensor { sensor: CurrentSensor::Bus, offset_a: 0.0, gain: 1.0, noise_a: 5.0 };
        plant.add_fault(0, None, noisy).unwrap();
        let counts: std::collections::BTreeSet<u16> = (1..50)
            .map(|n| {
                plant.advance(n * 360);
                plant.adc_raw(2, 10).unwrap()
            })
            .collect();
        assert!(counts.len() > 5, "{counts:?}");
    }

    #[test]
    fn an_open_phase_carries_no_current() {
        let mut plant = plant(ideal_switches);
        plant.add_fault(0, None, PlantFault::OpenPhase { phase: Phase::A }).unwrap();
        all_legs(&mut plant, 0, [pwm(0.55), pwm(0.45), pwm(0.5)]);
        plant.advance(20 * MS);
        let i = plant.snapshot().phase_current;
        assert!(i[0].abs() < 1e-9, "{i:?}");
        // b and c carry the 5 % between them: 6.5 V across two phases.
        assert!((i[1] + i[2]).abs() < 1e-9 && (i[2] - 0.05 * 129.7 / 0.164).abs() < 1.0, "{i:?}");
    }

    #[test]
    fn stage_overtemp_ramps_holds_and_cools() {
        // A 1 ms grid: minutes of heat with nothing else going on.
        let mut plant = plant(|p| p.grid_us = 1000.0);
        plant.add_fault(0, Some(40_000 * MS), PlantFault::StageOvertemp { rate_k_per_s: 2.0, max_c: 70.0 }).unwrap();
        plant.advance(10_000 * MS);
        assert!((plant.snapshot().stage_c - 45.0).abs() < 0.01, "{}", plant.snapshot().stage_c);
        let board = plant.adc_raw(1, 11).unwrap();
        assert_eq!(board, 2712, "the NTC reads 45 °C");
        plant.advance(30_000 * MS);
        assert!((plant.snapshot().stage_c - 70.0).abs() < 0.01, "held at 70 °C");
        // 200 s after the window, one stage time constant (400 J/K × 0.5 K/W): 45 K × e⁻¹ left.
        plant.advance(240_000 * MS);
        let expected = 25.0 + 45.0 * (-1.0f64).exp();
        assert!((plant.snapshot().stage_c - expected).abs() < 0.05, "{} °C", plant.snapshot().stage_c);
    }

    #[test]
    fn battery_faults_reach_the_bms_readings() {
        let mut plant = plant(|_| {});
        plant.add_fault(0, None, PlantFault::CellLow { pack: 1, cell: 30, drop_v: 0.5 }).unwrap();
        plant.add_fault(MS, None, PlantFault::PackImbalance { pack: 0, soc_delta: -0.1 }).unwrap();
        plant.add_fault(0, None, PlantFault::HighResistance { pack: 0, factor: 2.0 }).unwrap();
        plant.advance(2 * MS);
        let r = plant.battery_readings();
        assert!(r[1].cells_mv[29] - r[1].cells_mv[30] > 450, "{:?}", &r[1].cells_mv[28..]);
        // Pack 0 at 0.783 state of charge, pack 1 at 0.883: pack 1 now charges pack 0.
        assert!(r[1].current_a > 1.0 && r[0].current_a < -1.0, "{} {}", r[0].current_a, r[1].current_a);
        plant.advance(4 * MS);
        let soc = plant.snapshot().soc;
        assert!((soc - 0.833).abs() < 0.001, "the jump happens once: mean {soc}");
        plant.add_fault(10 * MS, None, PlantFault::BatterySoc { soc: 0.013 }).unwrap();
        plant.advance(11 * MS);
        assert!(
            (plant.snapshot().soc - 0.013).abs() < 1e-4 && plant.bus_v() < 99.0,
            "nearly empty: {} V",
            plant.bus_v()
        );
    }

    #[test]
    fn rejects_bad_files() {
        let text = text();
        let err = PlantParams::from_toml(&text.replace("schema = 1", "schema = 2")).unwrap_err();
        assert_eq!(err.to_string(), "plant.toml schema 2 is not supported (expected 1)");
        let err = PlantParams::from_toml(&text.replace("resistance_ohm = 0.080", "resistance_ohm = -1.0")).unwrap_err();
        assert_eq!(err.to_string(), "plant.toml: motor.resistance_ohm must be positive");
        assert!(PlantParams::from_toml(&text.replace("pole_pairs", "poles")).is_err());
    }
}

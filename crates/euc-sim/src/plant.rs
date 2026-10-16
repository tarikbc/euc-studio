//! The plant: what the firmware's sensors measure and what its outputs drive.
//!
//! The plant is a board component (`plant` in wiring) with inputs `leg_a`, `leg_b`, `leg_c` (the
//! inverter legs, from the PWM timer's outputs) and outputs `hall_a`, `hall_b`, `hall_c`. It is
//! also the [`PlantView`] the ADC, the IMU and the BMS sample. Two kinds exist:
//! - **static** (M1): fixed values turned into ADC counts by the board's `[[analog]]` wiring;
//! - **live** (M2): the electrical plant (`euc-plant`, on its own 5 µs grid) and the body
//!   physics (`euc-physics`, MuJoCo at 0.5 ms), on the engine's one clock. The electrical plant
//!   is advanced lazily, to the cycle at which a sensor samples, a leg changes or physics
//!   steps; physics steps on a fixed grid of cycles, takes the plant's mean torque since the
//!   previous step and hands back the wheel's angle and speed.

use euc_core::{Ctx, ImuSample, PackReading, Peripheral, PlantView, ResetKind, Signal};
use euc_physics::{Physics, TIMESTEP_S};
use euc_plant::ElectricalPlant;
use euc_plant::inverter::LegDrive;
use euc_plant::sensors::Phase;

use crate::config::{Analog, Ntc};

/// Mid-scale: what an unwired channel, or a current sensor at zero current, reads.
const MID_SCALE: u16 = 2048;

/// Port numbers of [`Plant`].
mod port {
    pub(super) const LEG_A: u8 = 0;
    pub(super) const LEG_B: u8 = 1;
    pub(super) const LEG_C: u8 = 2;
    pub(super) const HALL_A: u8 = 3;
    pub(super) const HALL_B: u8 = 4;
    pub(super) const HALL_C: u8 = 5;
}

/// Event tag of the physics step.
const STEP: u32 = 0;
/// Event tag of the next hall edge.
const HALL: u32 = 1;

/// Physical values the firmware senses when nothing moves them (the static plant), and the
/// values a live plant does not model.
#[derive(Debug, Clone)]
pub struct PlantState {
    /// Battery pack voltage.
    pub battery_v: f64,
    /// Controller board temperature.
    pub board_temp_c: f64,
    /// Motor temperature.
    pub motor_temp_c: f64,
    /// IMU sample (level and still by default).
    pub imu: ImuSample,
    /// Hall code: bit 0, 1, 2 drive the board's `hall_a`, `hall_b`, `hall_c` wires.
    pub hall: u8,
}

impl Default for PlantState {
    /// The reference harness defaults: 130 V, 25 °C everywhere, level and still, halls 5.
    fn default() -> Self {
        Self { battery_v: 130.0, board_temp_c: 25.0, motor_temp_c: 25.0, imu: ImuSample::LEVEL, hall: 5 }
    }
}

/// The electrical plant and the body physics, coupled.
pub struct Live {
    electrical: ElectricalPlant,
    physics: Physics,
    /// Cycles per physics step.
    step_cycles: u64,
    /// Cycle of the next physics step.
    next_step: u64,
    imu: ImuSample,
}

impl Live {
    /// Couples `electrical` and `physics` on an engine of `hclk` cycles per second, at cycle 0.
    pub fn new(electrical: ElectricalPlant, physics: Physics, hclk: u64) -> Self {
        let step_cycles = (TIMESTEP_S * hclk as f64).round() as u64;
        let mut live = Self { electrical, physics, step_cycles, next_step: step_cycles, imu: ImuSample::LEVEL };
        live.imu = live.sample_imu();
        live
    }

    /// The electrical plant.
    pub const fn electrical(&self) -> &ElectricalPlant {
        &self.electrical
    }

    /// The electrical plant, to add faults before a run.
    pub const fn electrical_mut(&mut self) -> &mut ElectricalPlant {
        &mut self.electrical
    }

    /// The body physics.
    pub const fn physics(&self) -> &Physics {
        &self.physics
    }

    /// The body physics, to change what the rider wants or hold the wheel.
    pub const fn physics_mut(&mut self) -> &mut Physics {
        &mut self.physics
    }

    /// Runs every physics step due by `now`.
    fn step_to(&mut self, now: u64) {
        while self.next_step <= now {
            let at = self.next_step;
            self.electrical.advance(at);
            let torque = self.electrical.mean_torque_nm();
            self.physics.step(torque);
            let (angle, speed) = self.physics.wheel();
            self.electrical.set_wheel(at, angle, speed);
            self.imu = self.sample_imu();
            self.next_step += self.step_cycles;
        }
    }

    fn sample_imu(&self) -> ImuSample {
        let imu = self.physics.imu();
        let g = self.physics.gravity();
        ImuSample {
            accel_g: imu.accel_mps2.map(|a| a / g),
            gyro_dps: imu.gyro_rad_s.map(f64::to_degrees),
            temp_c: self.electrical.snapshot().stage_c,
        }
    }
}

/// The plant component. See the module docs.
pub struct Plant {
    /// The static values (and, for a live plant, what it does not model).
    pub state: PlantState,
    analog: Vec<Analog>,
    live: Option<Box<Live>>,
    /// The hall code last put on the wires.
    halls_out: u8,
}

impl Plant {
    /// A static plant whose ADC channels follow `analog` (from `board.toml`).
    pub fn new(state: PlantState, analog: Vec<Analog>) -> Self {
        let halls_out = state.hall;
        Self { state, analog, live: None, halls_out }
    }

    /// A live plant; channels the electrical plant does not wire fall back to `analog`.
    pub fn live(state: PlantState, analog: Vec<Analog>, live: Live) -> Self {
        let halls_out = live.electrical.halls();
        Self { state, analog, live: Some(Box::new(live)), halls_out }
    }

    /// The live part, if any.
    pub fn live_part(&self) -> Option<&Live> {
        self.live.as_deref()
    }

    /// The live part, if any, to steer the rider or the stand.
    pub fn live_part_mut(&mut self) -> Option<&mut Live> {
        self.live.as_deref_mut()
    }

    /// Brings a live plant up to `now` and returns the hall lines that changed since they were
    /// last put on the wires, for the caller to emit.
    pub(crate) fn sync(&mut self, now: u64) -> Vec<(u8, Signal)> {
        let Some(live) = self.live.as_deref_mut() else { return Vec::new() };
        live.electrical.advance(now);
        let (old, new) = (self.halls_out, live.electrical.halls());
        self.halls_out = new;
        hall_lines(new).into_iter().filter(|&(out, _)| (old ^ new) >> (out - port::HALL_A) & 1 == 1).collect()
    }

    /// The hall code on the wires.
    pub const fn halls(&self) -> u8 {
        self.halls_out
    }

    /// Schedules the next physics step and the next hall edge. The firmware polls the hall lines
    /// faster than any sensor samples the plant, so each edge gets an event of its own.
    fn arm(&self, ctx: &mut Ctx<'_>) {
        if let Some(live) = &self.live {
            ctx.schedule(live.next_step, STEP);
            match live.electrical.next_hall_edge() {
                Some(at) => ctx.schedule(at, HALL),
                None => ctx.cancel(HALL),
            }
        }
    }

    fn quantity(&self, signal: &str) -> Option<f64> {
        let live = self.live.as_deref().map(|l| l.electrical.snapshot());
        match signal {
            "battery_v" => Some(live.map_or(self.state.battery_v, |s| s.bus_v)),
            "board_temp_c" => Some(live.map_or(self.state.board_temp_c, |s| s.stage_c)),
            "motor_temp_c" => Some(live.map_or(self.state.motor_temp_c, |s| s.winding_c)),
            _ => None,
        }
    }
}

/// Each hall line of `code` with the port that carries it.
fn hall_lines(code: u8) -> [(u8, Signal); 3] {
    [port::HALL_A, port::HALL_B, port::HALL_C].map(|out| (out, Signal::Level(code >> (out - port::HALL_A) & 1 == 1)))
}

impl Peripheral for Plant {
    fn reset(&mut self, kind: ResetKind, ctx: &mut Ctx<'_>) {
        if let Some(live) = self.live.as_deref_mut() {
            // A reset restarts the controller, not the world: catch up, keep the physics grid.
            live.step_to(ctx.now());
        }
        if kind == ResetKind::Power {
            self.sync(ctx.now());
            for (out, level) in hall_lines(self.halls_out) {
                ctx.emit(out, level);
            }
        }
        self.arm(ctx);
    }

    fn on_event(&mut self, _tag: u32, ctx: &mut Ctx<'_>) {
        if let Some(live) = self.live.as_deref_mut() {
            live.step_to(ctx.now());
        }
        for (out, level) in self.sync(ctx.now()) {
            ctx.emit(out, level);
        }
        self.arm(ctx);
    }

    fn input(&mut self, port: u8, signal: Signal, ctx: &mut Ctx<'_>) {
        let phase = match port {
            port::LEG_A => Phase::A,
            port::LEG_B => Phase::B,
            port::LEG_C => Phase::C,
            _ => return,
        };
        let drive = match signal {
            Signal::Pwm { tick_hz, period, high } => LegDrive::from_wave(tick_hz, period, high),
            Signal::Level(high) => LegDrive::Pwm { duty: if high { 1.0 } else { 0.0 }, period_s: 1.0 },
            Signal::Released => LegDrive::Off,
            Signal::Byte { .. } | Signal::Pulse => return,
        };
        let now = ctx.now();
        if let Some(live) = self.live.as_deref_mut() {
            live.electrical.set_leg(now, phase, drive);
        }
        for (out, level) in self.sync(now) {
            ctx.emit(out, level);
        }
        self.arm(ctx);
    }

    fn port(&self, name: &str) -> Option<u8> {
        Some(match name {
            "leg_a" => port::LEG_A,
            "leg_b" => port::LEG_B,
            "leg_c" => port::LEG_C,
            "hall_a" => port::HALL_A,
            "hall_b" => port::HALL_B,
            "hall_c" => port::HALL_C,
            _ => return None,
        })
    }
}

impl PlantView for Plant {
    fn adc_raw(&self, adc: u8, channel: u8) -> u16 {
        if let Some(count) = self.live.as_deref().and_then(|l| l.electrical.adc_raw(adc, channel)) {
            return count;
        }
        let Some(wire) = self.analog.iter().find(|a| a.adc == adc && a.channel == channel) else { return MID_SCALE };
        let Some(value) = self.quantity(&wire.signal) else { return MID_SCALE };
        match (wire.scale, wire.ntc) {
            (Some([num, den]), _) => counts(value * (f64::from(num) / f64::from(den))),
            (None, Some(ntc)) => ntc_counts(value, ntc),
            (None, None) => MID_SCALE,
        }
    }

    fn imu(&self) -> ImuSample {
        self.live.as_deref().map_or(self.state.imu, |l| l.imu)
    }

    fn battery_v(&self) -> f64 {
        self.live.as_deref().map_or(self.state.battery_v, |l| l.electrical.bus_v())
    }

    fn pack(&self, index: usize) -> Option<PackReading> {
        let pack = self.live.as_deref()?.electrical.battery_readings().into_iter().nth(index)?;
        Some(PackReading { cells_mv: pack.cells_mv, temps_c: pack.temps_c, current_a: pack.current_a })
    }
}

/// A 12-bit count, rounded half to even like the reference harness (Python's `round`).
fn counts(value: f64) -> u16 {
    value.round_ties_even().clamp(0.0, 4095.0) as u16
}

/// Count for an NTC to ground under a pull-up to the ADC reference.
fn ntc_counts(temp_c: f64, ntc: Ntc) -> u16 {
    let r = ntc.r25 * (ntc.beta * (1.0 / (temp_c + 273.15) - 1.0 / 298.15)).exp();
    counts(4096.0 * r / (r + ntc.pullup))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plant() -> Plant {
        let ntc = Ntc { r25: 47_000.0, beta: 4150.0, pullup: 10_000.0 };
        let wire = |adc, channel, signal: &str, scale, ntc| Analog { adc, channel, signal: signal.into(), scale, ntc };
        Plant::new(
            PlantState::default(),
            vec![wire(1, 0, "battery_v", Some([32768, 1377]), None), wire(1, 11, "board_temp_c", None, Some(ntc))],
        )
    }

    #[test]
    fn counts_match_the_reference_harness() {
        let plant = plant();
        // round(130 * 32768 / 1377) and the NTC at 25 °C, as the Python plant computes them.
        assert_eq!(plant.adc_raw(1, 0), 3094);
        assert_eq!(plant.adc_raw(1, 11), 3377);
        assert_eq!(plant.adc_raw(1, 4), 2048, "phase current at zero");
        assert_eq!(plant.adc_raw(3, 1), 2048, "unwired");
    }

    #[test]
    fn rounding_is_half_to_even() {
        assert_eq!((counts(2.5), counts(3.5), counts(-1.0), counts(5000.0)), (2, 4, 0, 4095));
    }

    #[test]
    fn hall_lines_follow_the_code_bits() {
        let lines = hall_lines(0b101).map(|(_, s)| s);
        assert_eq!(lines, [Signal::Level(true), Signal::Level(false), Signal::Level(true)]);
    }
}

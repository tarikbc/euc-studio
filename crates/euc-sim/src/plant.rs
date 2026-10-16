//! The plant for M1: static sensor values, turned into ADC counts by the board's analog wiring.

use euc_core::{ImuSample, PlantView};

use crate::config::{Analog, Ntc};

/// Mid-scale: what an unwired channel, or a current sensor at zero current, reads.
const MID_SCALE: u16 = 2048;

/// Physical values the firmware senses. Physics (M2) will move them; M1 holds them still.
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
        Self {
            battery_v: 130.0,
            board_temp_c: 25.0,
            motor_temp_c: 25.0,
            imu: ImuSample { accel_g: [0.0, 0.0, 1.0], gyro_dps: [0.0; 3], temp_c: 25.0 },
            hall: 5,
        }
    }
}

/// [`PlantState`] seen through the board's sensor wiring.
#[derive(Debug, Clone)]
pub struct Plant {
    /// The physical values.
    pub state: PlantState,
    analog: Vec<Analog>,
}

impl Plant {
    /// A plant whose ADC channels follow `analog` (from `board.toml`).
    pub fn new(state: PlantState, analog: Vec<Analog>) -> Self {
        Self { state, analog }
    }

    fn quantity(&self, signal: &str) -> Option<f64> {
        match signal {
            "battery_v" => Some(self.state.battery_v),
            "board_temp_c" => Some(self.state.board_temp_c),
            "motor_temp_c" => Some(self.state.motor_temp_c),
            _ => None,
        }
    }
}

impl PlantView for Plant {
    fn adc_raw(&self, adc: u8, channel: u8) -> u16 {
        let Some(wire) = self.analog.iter().find(|a| a.adc == adc && a.channel == channel) else { return MID_SCALE };
        let Some(value) = self.quantity(&wire.signal) else { return MID_SCALE };
        match (wire.scale, wire.ntc) {
            (Some([num, den]), _) => counts(value * (f64::from(num) / f64::from(den))),
            (None, Some(ntc)) => ntc_counts(value, ntc),
            (None, None) => MID_SCALE,
        }
    }

    fn imu(&self) -> ImuSample {
        self.state.imu
    }

    fn battery_v(&self) -> f64 {
        self.state.battery_v
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
}

//! Battery: parallel strings of series cell groups, each with an equivalent circuit.
//!
//! One pack = `series` cell groups in series, each group an open-circuit voltage OCV(SOC), an
//! ohmic resistance R0 and one RC pair (R1, τ) for polarisation. Packs sit in parallel on the DC
//! bus and share the load by their impedance. Positive current discharges.

use serde::Deserialize;
use thiserror::Error;

/// Battery parameters (`[battery]` in `plant.toml`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryParams {
    /// Packs in parallel.
    pub packs: u8,
    /// Cell groups in series per pack (the cells a BMS reports).
    pub series: u8,
    /// Capacity of one pack, ampere-hours.
    pub capacity_ah: f64,
    /// Ohmic resistance of one cell group, ohm.
    pub r0_ohm: f64,
    /// Polarisation resistance of one cell group, ohm.
    pub r1_ohm: f64,
    /// Polarisation time constant, seconds.
    pub tau_s: f64,
    /// Open-circuit voltage of a cell group against state of charge: `[soc, volt]` pairs with
    /// `soc` rising from 0 to 1.
    pub ocv: Vec<[f64; 2]>,
    /// State of charge at power-on, 0..=1.
    pub soc: f64,
    /// Temperature sensors per pack (what the BMS reports).
    pub temp_sensors: u8,
    /// Heat capacity of one pack, joule per kelvin.
    pub heat_capacity_j_per_k: f64,
    /// Heat transfer of one pack to ambient, watt per kelvin.
    pub cooling_w_per_k: f64,
}

/// A parameter set that cannot describe a battery.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum BatteryError {
    /// No pack or no cell.
    #[error("a battery needs at least one pack of at least one cell")]
    Empty,
    /// The OCV table is too short or not sorted by state of charge.
    #[error("the OCV table needs two or more points with rising state of charge")]
    OcvTable,
}

impl BatteryParams {
    /// Checks the parameters.
    pub fn validate(&self) -> Result<(), BatteryError> {
        if self.packs == 0 || self.series == 0 {
            return Err(BatteryError::Empty);
        }
        if self.ocv.len() < 2 || self.ocv.windows(2).any(|w| w[1][0] <= w[0][0]) {
            return Err(BatteryError::OcvTable);
        }
        Ok(())
    }

    /// Open-circuit voltage of one cell group at `soc`, linear between table points and held at
    /// the ends.
    pub fn cell_ocv(&self, soc: f64) -> f64 {
        let table = &self.ocv;
        let i = table.partition_point(|p| p[0] < soc).clamp(1, table.len() - 1);
        let ([s0, v0], [s1, v1]) = (table[i - 1], table[i]);
        let t = ((soc - s0) / (s1 - s0)).clamp(0.0, 1.0);
        v0 + t * (v1 - v0)
    }
}

/// What is wrong with one pack (faults); a healthy pack has factor 1 and no cell drops.
#[derive(Debug, Clone, PartialEq)]
pub struct PackCondition {
    /// Multiplies the pack's R0 and R1 (aged cells, a loose connector).
    pub resistance_factor: f64,
    /// Volts each cell group's open-circuit voltage sits below the curve (a weak cell).
    pub cell_drop_v: Vec<f64>,
}

impl PackCondition {
    /// A healthy pack of `series` cell groups.
    pub fn healthy(series: u8) -> Self {
        Self { resistance_factor: 1.0, cell_drop_v: vec![0.0; usize::from(series)] }
    }
}

/// One pack's state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pack {
    /// State of charge, 0..=1.
    pub soc: f64,
    /// Voltage across the polarisation RC of the whole pack, volt.
    pub polarisation_v: f64,
    /// Current, ampere (positive = discharge).
    pub current_a: f64,
    /// Temperature, °C.
    pub temp_c: f64,
}

/// What a smart BMS reads from one pack.
#[derive(Debug, Clone, PartialEq)]
pub struct PackReading {
    /// Cell group voltages, millivolt.
    pub cells_mv: Vec<u16>,
    /// Temperature sensors, °C.
    pub temps_c: Vec<f64>,
    /// Pack current, ampere (positive = discharge).
    pub current_a: f64,
}

/// The battery: packs in parallel on the DC bus.
#[derive(Debug, Clone)]
pub struct Battery {
    params: BatteryParams,
    packs: Vec<Pack>,
    conditions: Vec<PackCondition>,
    terminal_v: f64,
}

impl Battery {
    /// A rested battery at the parameters' state of charge and `ambient_c`.
    pub fn new(params: BatteryParams, ambient_c: f64) -> Result<Self, BatteryError> {
        params.validate()?;
        let pack = Pack { soc: params.soc.clamp(0.0, 1.0), polarisation_v: 0.0, current_a: 0.0, temp_c: ambient_c };
        let packs = vec![pack; usize::from(params.packs)];
        let conditions = vec![PackCondition::healthy(params.series); usize::from(params.packs)];
        let mut battery = Self { params, packs, conditions, terminal_v: 0.0 };
        battery.terminal_v = battery.solve(0.0).0;
        Ok(battery)
    }

    /// Terminal (bus) voltage at the last step's load.
    pub const fn voltage(&self) -> f64 {
        self.terminal_v
    }

    /// The packs.
    pub fn packs(&self) -> &[Pack] {
        &self.packs
    }

    /// The parameters.
    pub const fn params(&self) -> &BatteryParams {
        &self.params
    }

    /// Changes pack `index`'s state of charge by `delta` (clamped to 0..=1).
    pub fn shift_soc(&mut self, index: usize, delta: f64) {
        if let Some(pack) = self.packs.get_mut(index) {
            pack.soc = (pack.soc + delta).clamp(0.0, 1.0);
        }
    }

    /// What is wrong with each pack.
    pub fn conditions(&self) -> &[PackCondition] {
        &self.conditions
    }

    /// Sets what is wrong with each pack (one entry per pack; missing packs stay as they are).
    pub fn set_conditions(&mut self, conditions: &[PackCondition]) {
        for (slot, condition) in self.conditions.iter_mut().zip(conditions) {
            slot.clone_from(condition);
        }
    }

    /// Ohmic resistance of pack `index`, ohm.
    fn r0(&self, index: usize) -> f64 {
        f64::from(self.params.series) * self.params.r0_ohm * self.conditions[index].resistance_factor
    }

    /// Open-circuit voltage of cell group `cell` of pack `index`, volt.
    fn cell_emf(&self, index: usize, cell: usize) -> f64 {
        self.params.cell_ocv(self.packs[index].soc) - self.conditions[index].cell_drop_v[cell]
    }

    /// Terminal voltage and pack currents for a bus load of `load_a`.
    fn solve(&self, load_a: f64) -> (f64, Vec<f64>) {
        let series = usize::from(self.params.series);
        let emf: Vec<f64> = (0..self.packs.len())
            .map(|k| (0..series).map(|c| self.cell_emf(k, c)).sum::<f64>() - self.packs[k].polarisation_v)
            .collect();
        let conductance: f64 = (0..emf.len()).map(|k| 1.0 / self.r0(k)).sum();
        let v = (emf.iter().enumerate().map(|(k, e)| e / self.r0(k)).sum::<f64>() - load_a) / conductance;
        let currents = emf.iter().enumerate().map(|(k, e)| (e - v) / self.r0(k)).collect();
        (v, currents)
    }

    /// Advances `h` seconds with the bus drawing `load_a` (negative = regeneration).
    pub fn step(&mut self, load_a: f64, ambient_c: f64, h: f64) {
        let (v, currents) = self.solve(load_a);
        let series = f64::from(self.params.series);
        let decay = (-h / self.params.tau_s).exp();
        let capacity_as = self.params.capacity_ah * 3600.0;
        let heat_decay = (-h * self.params.cooling_w_per_k / self.params.heat_capacity_j_per_k).exp();
        for (k, current) in currents.into_iter().enumerate() {
            let factor = self.conditions[k].resistance_factor;
            let (r0, r1) = (series * self.params.r0_ohm * factor, series * self.params.r1_ohm * factor);
            let pack = &mut self.packs[k];
            pack.current_a = current;
            pack.polarisation_v = pack.polarisation_v * decay + r1 * current * (1.0 - decay);
            pack.soc = (pack.soc - current * h / capacity_as).clamp(0.0, 1.0);
            let heat = current * current * r0 + pack.polarisation_v * pack.polarisation_v / r1;
            let settle = ambient_c + heat / self.params.cooling_w_per_k;
            pack.temp_c = settle + (pack.temp_c - settle) * heat_decay;
        }
        self.terminal_v = v;
    }

    /// What each pack's BMS reads now. Each cell group shows its open-circuit voltage less its
    /// share of the pack's ohmic and polarisation drops, so a pack's cells add up to the bus.
    pub fn readings(&self) -> Vec<PackReading> {
        let series = usize::from(self.params.series);
        let mv = |v: f64| (v * 1000.0).round_ties_even().clamp(0.0, 65_535.0) as u16;
        (0..self.packs.len())
            .map(|k| {
                let pack = &self.packs[k];
                let drop = (pack.current_a * self.r0(k) + pack.polarisation_v) / series as f64;
                PackReading {
                    cells_mv: (0..series).map(|c| mv(self.cell_emf(k, c) - drop)).collect(),
                    temps_c: vec![pack.temp_c; usize::from(self.params.temp_sensors)],
                    current_a: pack.current_a,
                }
            })
            .collect()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Two 32S2P packs of 5 Ah cells (see `profiles/begode-extreme/plant.toml`).
    pub(crate) fn extreme() -> BatteryParams {
        BatteryParams {
            packs: 2,
            series: 32,
            capacity_ah: 10.0,
            r0_ohm: 0.007,
            r1_ohm: 0.003,
            tau_s: 30.0,
            ocv: vec![
                [0.0, 3.00],
                [0.05, 3.30],
                [0.1, 3.42],
                [0.2, 3.52],
                [0.3, 3.60],
                [0.4, 3.66],
                [0.5, 3.72],
                [0.6, 3.80],
                [0.7, 3.88],
                [0.8, 3.97],
                [0.9, 4.07],
                [1.0, 4.20],
            ],
            soc: 0.9,
            temp_sensors: 6,
            heat_capacity_j_per_k: 6000.0,
            cooling_w_per_k: 3.0,
        }
    }

    #[test]
    fn ocv_interpolates_and_holds_at_the_ends() {
        let p = extreme();
        assert!((p.cell_ocv(0.85) - 4.02).abs() < 1e-12);
        assert_eq!((p.cell_ocv(-0.1), p.cell_ocv(1.5)), (3.00, 4.20));
        let rested = Battery::new(p, 25.0).unwrap();
        assert!((rested.voltage() - 32.0 * 4.07).abs() < 1e-9, "130.24 V at 90 %");
    }

    #[test]
    fn sag_and_recovery() {
        let mut b = Battery::new(extreme(), 25.0).unwrap();
        b.step(100.0, 25.0, 1e-3);
        let sag = 130.24 - b.voltage();
        // Two packs of 32 × 7 mΩ in parallel: 0.112 Ω, 11.2 V at 100 A.
        assert!((sag - 11.2).abs() < 0.01, "sag {sag}");
        assert!(b.packs().iter().all(|p| (p.current_a - 50.0).abs() < 1e-9), "shared equally");
        for _ in 0..10_000 {
            b.step(100.0, 25.0, 1e-2);
        }
        let polarised = b.voltage();
        assert!(polarised < 130.24 - 11.2 - 3.0, "polarisation adds sag: {polarised}");
        b.step(0.0, 25.0, 1e-3);
        assert!(b.voltage() > polarised + 11.0, "ohmic part recovers at once");
    }

    #[test]
    fn charge_balance_and_regeneration() {
        let mut b = Battery::new(extreme(), 25.0).unwrap();
        // 20 A for 180 s draws 1 Ah: 5 % of each 10 Ah pack.
        for _ in 0..18_000 {
            b.step(20.0, 25.0, 1e-2);
        }
        assert!(b.packs().iter().all(|p| (p.soc - 0.85).abs() < 1e-9));
        let before = b.voltage();
        b.step(-40.0, 25.0, 1e-3);
        assert!(b.voltage() > before + 4.0, "regeneration lifts the bus");
        assert!(b.packs()[0].current_a < 0.0);
    }

    #[test]
    fn unequal_packs_share_unequally() {
        let mut b = Battery::new(extreme(), 25.0).unwrap();
        b.packs[1].soc = 0.5;
        b.step(30.0, 25.0, 1e-3);
        let [a, low] = [b.packs()[0].current_a, b.packs()[1].current_a];
        assert!(a > low && (a + low - 30.0).abs() < 1e-9, "{a} {low}");
    }

    #[test]
    fn a_weak_cell_and_a_resistive_pack() {
        let mut b = Battery::new(extreme(), 25.0).unwrap();
        let mut weak = PackCondition::healthy(32);
        weak.cell_drop_v[5] = 0.4;
        let resistive = PackCondition { resistance_factor: 3.0, ..PackCondition::healthy(32) };
        b.set_conditions(&[weak, resistive]);
        b.step(60.0, 25.0, 1e-3);
        let r = b.readings();
        // The weak pack's cell 5 reads about 0.4 V low; its other cells carry the rest of the bus.
        assert!(f64::from(r[0].cells_mv[4]) - f64::from(r[0].cells_mv[5]) > 390.0, "{:?}", &r[0].cells_mv[..6]);
        for pack in &r {
            let sum: f64 = pack.cells_mv.iter().map(|&mv| f64::from(mv)).sum();
            assert!((sum / 1000.0 - b.voltage()).abs() < 0.02, "cells add up to the bus");
        }
        // Three times the resistance: about a quarter of the load.
        assert!((r[0].current_a + r[1].current_a - 60.0).abs() < 1e-9);
        assert!(r[1].current_a < 30.0, "{} A", r[1].current_a);
        b.shift_soc(1, -0.3);
        assert!((b.packs()[1].soc - 0.6).abs() < 1e-4);
    }

    #[test]
    fn heating_and_readings() {
        let mut b = Battery::new(extreme(), 25.0).unwrap();
        for _ in 0..60_000 {
            b.step(150.0, 25.0, 1e-2);
        }
        let readings = b.readings();
        assert_eq!((readings.len(), readings[0].cells_mv.len(), readings[0].temps_c.len()), (2, 32, 6));
        assert!(readings[0].temps_c[0] > 30.0, "{}", readings[0].temps_c[0]);
        assert!((f64::from(readings[0].cells_mv[0]) - b.voltage() / 32.0 * 1000.0).abs() <= 0.5);
        assert_eq!(
            Battery::new(BatteryParams { ocv: vec![[0.0, 3.0]], ..extreme() }, 25.0).err(),
            Some(BatteryError::OcvTable)
        );
    }
}

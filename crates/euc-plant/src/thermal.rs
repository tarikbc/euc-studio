//! Lumped thermal models: one heat capacity with a thermal resistance to ambient.

use serde::Deserialize;

/// One thermal node (`plant.toml`: `[thermal.stage]`, `[thermal.motor]`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeParams {
    /// Heat capacity, joule per kelvin.
    pub heat_capacity_j_per_k: f64,
    /// Thermal resistance to ambient, kelvin per watt.
    pub resistance_k_per_w: f64,
}

/// Thermal parameters (`[thermal]` in `plant.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalParams {
    /// Ambient temperature, °C (also the starting temperature of everything).
    pub ambient_c: f64,
    /// Power stage (MOSFETs on their heat sink), where the board NTC sits.
    pub stage: NodeParams,
    /// Motor windings.
    pub motor: NodeParams,
}

/// A node's temperature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    params: NodeParams,
    temp_c: f64,
}

impl Node {
    /// A node at `temp_c`.
    pub const fn new(params: NodeParams, temp_c: f64) -> Self {
        Self { params, temp_c }
    }

    /// Temperature, °C.
    pub const fn temp_c(&self) -> f64 {
        self.temp_c
    }

    /// Advances `h` seconds with `power_w` heating the node, exactly for a constant power.
    pub fn step(&mut self, power_w: f64, ambient_c: f64, h: f64) {
        let settle = ambient_c + power_w * self.params.resistance_k_per_w;
        let tau = self.params.heat_capacity_j_per_k * self.params.resistance_k_per_w;
        self.temp_c = settle + (self.temp_c - settle) * (-h / tau).exp();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_order_rise_and_cool_down() {
        let params = NodeParams { heat_capacity_j_per_k: 500.0, resistance_k_per_w: 0.4 };
        let mut node = Node::new(params, 25.0);
        // 100 W through 0.4 K/W settles 40 K up, with τ = 200 s.
        node.step(100.0, 25.0, 200.0);
        assert!((node.temp_c() - (65.0 - 40.0 * (-1.0f64).exp())).abs() < 1e-9);
        let mut fine = Node::new(params, 25.0);
        for _ in 0..200_000 {
            fine.step(100.0, 25.0, 1e-3);
        }
        assert!((fine.temp_c() - node.temp_c()).abs() < 1e-6, "step size does not matter");
        node.step(0.0, 25.0, 1e5);
        assert!((node.temp_c() - 25.0).abs() < 1e-9);
    }
}

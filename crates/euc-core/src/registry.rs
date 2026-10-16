//! Building components from `board.toml`: kinds, instances and their parameters.
//!
//! A board file lists instances (`name`, `kind`, `base`, `irq`, parameters). Model crates
//! register a constructor per kind; the engine looks each instance up here. New hardware of a
//! known kind is therefore data, and a new kind is one `register` call.

use std::collections::BTreeMap;
use std::fmt;

use serde::de::DeserializeOwned;
use thiserror::Error;

use crate::{DeviceId, Peripheral};

/// One component instance as `board.toml` describes it.
#[derive(Debug, Clone, PartialEq)]
pub struct InstanceSpec {
    /// Instance name, unique on the board (`USART1`, `imu`).
    pub name: String,
    /// Registered kind (`stm32f1.usart`).
    pub kind: String,
    /// Base address on the bus; `None` for off-chip devices.
    pub base: Option<u32>,
    /// Size of the register window in bytes.
    pub size: u32,
    /// Interrupt lines by role (`"up" = 44`, `"irq" = 37`).
    pub irqs: BTreeMap<String, u16>,
    /// Every other key of the instance table, for the constructor to deserialize.
    pub params: toml::Table,
}

/// What a constructor gets: the instance and a way to resolve other instances' ids.
pub struct Build<'a> {
    spec: &'a InstanceSpec,
    ids: &'a dyn Fn(&str) -> Option<DeviceId>,
}

impl<'a> Build<'a> {
    /// A build request; `ids` maps instance names to their ids (all ids exist before any
    /// constructor runs, so instances may refer to each other in any order).
    pub fn new(spec: &'a InstanceSpec, ids: &'a dyn Fn(&str) -> Option<DeviceId>) -> Self {
        Self { spec, ids }
    }

    /// The instance being built.
    pub const fn spec(&self) -> &InstanceSpec {
        self.spec
    }

    /// The instance parameters as the constructor's own config type. Unknown keys are the
    /// type's business (`#[serde(deny_unknown_fields)]` to reject typos).
    pub fn params<T: DeserializeOwned>(&self) -> Result<T, ConfigError> {
        toml::Value::Table(self.spec.params.clone())
            .try_into()
            .map_err(|e: toml::de::Error| self.error(e.message().to_owned()))
    }

    /// The interrupt line with role `role`.
    pub fn irq(&self, role: &str) -> Result<u16, ConfigError> {
        self.spec.irqs.get(role).copied().ok_or_else(|| self.error(format!("needs an irq.{role}")))
    }

    /// The id of the instance named `name` (from a parameter such as `clock = "RCC"`).
    pub fn id_of(&self, name: &str) -> Result<DeviceId, ConfigError> {
        (self.ids)(name).ok_or_else(|| self.error(format!("refers to unknown instance \"{name}\"")))
    }

    /// An error about this instance.
    pub fn error(&self, message: impl Into<String>) -> ConfigError {
        ConfigError { instance: self.spec.name.clone(), message: message.into() }
    }
}

/// An instance that cannot be built.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("instance {instance}: {message}")]
pub struct ConfigError {
    /// Instance name.
    pub instance: String,
    /// What is wrong.
    pub message: String,
}

/// Builds one kind of component.
pub type Constructor = fn(&Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError>;

/// Kind name → constructor.
#[derive(Default)]
pub struct Registry {
    kinds: BTreeMap<&'static str, Constructor>,
}

impl Registry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers `kind`.
    ///
    /// # Panics
    ///
    /// If `kind` is already registered: two models for one kind is a wiring bug.
    pub fn register(&mut self, kind: &'static str, constructor: Constructor) {
        assert!(self.kinds.insert(kind, constructor).is_none(), "kind {kind} registered twice");
    }

    /// Builds `spec` with its kind's constructor.
    pub fn build(&self, build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let kind = build.spec().kind.as_str();
        let constructor = self.kinds.get(kind).ok_or_else(|| build.error(format!("unknown kind \"{kind}\"")))?;
        constructor(build)
    }

    /// Registered kinds, sorted.
    pub fn kinds(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.kinds.keys().copied()
    }
}

impl fmt::Debug for Registry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.kinds()).finish()
    }
}

#[cfg(test)]
mod tests {
    use serde::Deserialize;

    use super::*;
    use crate::{Ctx, ResetKind};

    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct UartConfig {
        pclk_hz: u64,
        clock: String,
    }

    struct Uart {
        pclk_hz: u64,
        clock: DeviceId,
        irq: u16,
    }

    impl Peripheral for Uart {
        fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {}
    }

    fn build_uart(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: UartConfig = build.params()?;
        Ok(Box::new(Uart { pclk_hz: config.pclk_hz, clock: build.id_of(&config.clock)?, irq: build.irq("irq")? }))
    }

    fn spec(params: &str) -> InstanceSpec {
        InstanceSpec {
            name: "USART1".into(),
            kind: "test.uart".into(),
            base: Some(0x4001_3800),
            size: 0x400,
            irqs: BTreeMap::from([("irq".to_owned(), 37)]),
            params: params.parse().unwrap(),
        }
    }

    fn ids(name: &str) -> Option<DeviceId> {
        (name == "RCC").then_some(3)
    }

    #[test]
    fn builds_a_registered_kind() {
        let mut registry = Registry::new();
        registry.register("test.uart", build_uart);
        let spec = spec("pclk_hz = 72_000_000\nclock = \"RCC\"");
        let device = registry.build(&Build::new(&spec, &ids)).unwrap();
        let uart = (device as Box<dyn std::any::Any>).downcast::<Uart>().unwrap();
        assert_eq!((uart.pclk_hz, uart.clock, uart.irq), (72_000_000, 3, 37));
    }

    #[test]
    fn errors_name_the_instance() {
        let mut registry = Registry::new();
        registry.register("test.uart", build_uart);
        let error = |params: &str| registry.build(&Build::new(&spec(params), &ids)).err().unwrap().to_string();
        assert_eq!(error("pclk_hz = 1\nclock = \"RCC2\""), "instance USART1: refers to unknown instance \"RCC2\"");
        assert!(error("pclk_hz = 1\nclock = \"RCC\"\nbaud = 9").contains("unknown field `baud`"));
        let mut other = spec("");
        other.kind = "test.spi".into();
        let message = registry.build(&Build::new(&other, &ids)).err().unwrap().to_string();
        assert_eq!(message, "instance USART1: unknown kind \"test.spi\"");
    }
}

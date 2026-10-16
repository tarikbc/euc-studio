//! The CPU backend a machine runs the firmware on.
//!
//! [`Backend::Own`] is our Cortex-M3 interpreter (`euc-armv7m`): every build has it and it is
//! the default. [`Backend::Unicorn`] is QEMU's core through `euc-unicorn`, GPL-2.0, a development
//! and CI oracle only: it exists in builds with this crate's `unicorn` feature, which no shipped
//! build enables.

use std::fmt;
use std::str::FromStr;

use crate::CpuFactory;

/// Which CPU core runs the firmware.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Backend {
    /// Our own interpreter: exact instruction budgets, verified against Unicorn.
    #[default]
    Own,
    /// Unicorn (builds with the `unicorn` feature). With `exact` every quantum ends after
    /// exactly its budget, as in the Python reference harness and on [`Backend::Own`]; without
    /// it, at the next translated-block boundary (faster, slightly different quanta).
    Unicorn {
        /// End quanta on the exact budget.
        exact: bool,
    },
}

impl Backend {
    /// `own` or `unicorn`, for reports and status bars.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Own => "own",
            Self::Unicorn { .. } => "unicorn",
        }
    }

    /// True when this build has the backend.
    pub const fn available(self) -> bool {
        matches!(self, Self::Own) || cfg!(feature = "unicorn")
    }

    /// A factory that builds the backend for a machine's memory layout, or why this build cannot.
    pub fn factory(self) -> Result<Box<CpuFactory<'static>>, String> {
        match self {
            Self::Own => Ok(Box::new(|regions| {
                // SAFETY: `Machine` calls the factory with regions whose memory it owns in boxed
                // buffers that never move and outlive the CPU it stores next to them, and touches
                // that memory during a run only from inside the run's bus callbacks.
                #[allow(unsafe_code)]
                let cpu = unsafe { euc_armv7m::Armv7m::new(regions) }.map_err(|e| e.to_string())?;
                Ok(Box::new(cpu) as Box<dyn euc_core::Cpu>)
            })),
            #[cfg(feature = "unicorn")]
            Self::Unicorn { exact } => Ok(unicorn(exact)),
            #[cfg(not(feature = "unicorn"))]
            Self::Unicorn { .. } => Err("this build has no Unicorn backend: it is GPL-2.0 and for development only \
                 (build with the `unicorn` feature), and `--cpu own` needs nothing extra"
                .to_owned()),
        }
    }
}

#[cfg(feature = "unicorn")]
fn unicorn(exact: bool) -> Box<CpuFactory<'static>> {
    use euc_unicorn::{Quantum, UnicornCpu};
    let quantum = if exact { Quantum::Exact } else { Quantum::Block };
    Box::new(move |regions| {
        // SAFETY: as for `Backend::Own` above.
        #[allow(unsafe_code)]
        let cpu = unsafe { UnicornCpu::with_quantum(regions, quantum) }.map_err(|e| e.to_string())?;
        Ok(Box::new(cpu) as Box<dyn euc_core::Cpu>)
    })
}

impl fmt::Display for Backend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for Backend {
    type Err = String;

    /// `own` or `unicorn` (block quanta).
    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text {
            "own" => Ok(Self::Own),
            "unicorn" => Ok(Self::Unicorn { exact: false }),
            other => Err(format!("unknown CPU backend `{other}` (own, unicorn)")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_parsing() {
        assert_eq!("own".parse(), Ok(Backend::Own));
        assert_eq!("unicorn".parse(), Ok(Backend::Unicorn { exact: false }));
        assert!("qemu".parse::<Backend>().is_err());
        assert_eq!(Backend::default().to_string(), "own");
        assert!(Backend::Own.available());
        assert_eq!(Backend::Unicorn { exact: true }.available(), cfg!(feature = "unicorn"));
        assert!(Backend::Own.factory().is_ok());
    }
}

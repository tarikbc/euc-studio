//! The CPU backend this build runs firmware on.
//!
//! Until our own ARMv7-M core lands (M3), the only backend is Unicorn, which is GPL-2.0: it may
//! run on a developer's machine but must never ship. `pnpm tauri dev` enables the `unicorn`
//! feature; every bundle (`tauri build`, release or `--debug`) compiles with Tauri's custom
//! protocol, which clears the `dev` cfg, so the guard below stops such a build outright.

use euc_sim::CpuFactory;

#[cfg(all(feature = "unicorn", not(dev)))]
compile_error!(
    "the `unicorn` feature (GPL-2.0) is for `tauri dev` only and must never be part of a bundle; \
     build without it"
);

/// Name of the backend, for the status bar.
pub const BACKEND: &str = if cfg!(feature = "unicorn") { "unicorn" } else { "none" };

/// A factory for the build's CPU backend, or why there is none.
#[cfg(feature = "unicorn")]
pub fn factory() -> Result<Box<CpuFactory<'static>>, String> {
    use euc_unicorn::{Quantum, UnicornCpu};
    Ok(Box::new(|regions| {
        // SAFETY: `Machine::new` calls this with regions whose memory it owns in boxed buffers that
        // never move and outlive the CPU it stores next to them; it touches that memory during a
        // run only from inside the run's bus callbacks.
        #[allow(unsafe_code)]
        let cpu = unsafe { UnicornCpu::with_quantum(regions, Quantum::Block) }.map_err(|e| e.to_string())?;
        Ok(Box::new(cpu) as Box<dyn euc_core::Cpu>)
    }))
}

/// A factory for the build's CPU backend, or why there is none.
#[cfg(not(feature = "unicorn"))]
pub fn factory() -> Result<Box<CpuFactory<'static>>, String> {
    Err("This build has no CPU backend yet: our own core arrives in M3, and the Unicorn backend \
         is for development only (run `pnpm tauri dev`)."
        .to_owned())
}

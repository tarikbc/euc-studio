//! The CPU backend this build runs firmware on.
//!
//! Our own Cortex-M3 core (`euc-armv7m`) runs the firmware in every build, bundles included.
//! Unicorn is GPL-2.0, a development oracle that must never ship: `pnpm tauri dev` enables the
//! `unicorn` feature, and `EUC_CPU=unicorn` then selects it. Every bundle (`tauri build`, release
//! or `--debug`) compiles with Tauri's custom protocol, which clears the `dev` cfg, so the guard
//! below stops a bundle with the feature outright.

use euc_sim::CpuFactory;
use euc_sim::cpu::Backend;

#[cfg(all(feature = "unicorn", not(dev)))]
compile_error!(
    "the `unicorn` feature (GPL-2.0) is for `tauri dev` only and must never be part of a bundle; \
     build without it"
);

/// The backend: Unicorn when a development build with the feature asks for it with
/// `EUC_CPU=unicorn`, our own core otherwise.
pub fn backend() -> Backend {
    let unicorn = cfg!(feature = "unicorn") && std::env::var("EUC_CPU").is_ok_and(|v| v == "unicorn");
    if unicorn { Backend::Unicorn { exact: false } } else { Backend::Own }
}

/// A factory for the build's CPU backend, or why there is none.
pub fn factory() -> Result<Box<CpuFactory<'static>>, String> {
    backend().factory()
}

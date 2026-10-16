fn main() {
    // The engine's physics links MuJoCo dynamically from MUJOCO_DYNAMIC_LINK_DIR (the repository's
    // .cargo/config.toml points it at .cache/mujoco; see tools/fetch-mujoco). On Unix the loader
    // also needs an rpath to it; Windows finds mujoco.dll on PATH. Bundling the library into the
    // app is release work (M7).
    println!("cargo::rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    if let Ok(dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR")
        && std::env::var_os("CARGO_CFG_UNIX").is_some()
    {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
    tauri_build::build()
}

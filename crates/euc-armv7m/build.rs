//! Lets this crate's tests find the MuJoCo library at run time (see `tools/fetch-mujoco`). The
//! library itself never links MuJoCo; the whole-firmware tests do, through `euc-sim`.

fn main() {
    println!("cargo::rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    // mujoco-rs links the library from MUJOCO_DYNAMIC_LINK_DIR; on Unix the loader also needs an rpath to it.
    // Windows finds mujoco.dll on PATH instead.
    if let Ok(dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR")
        && std::env::var_os("CARGO_CFG_UNIX").is_some()
    {
        println!("cargo::rustc-link-arg-tests=-Wl,-rpath,{dir}");
    }
}

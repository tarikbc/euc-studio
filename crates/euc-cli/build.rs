//! Lets this crate's binaries and tests find the MuJoCo library at run time (see `tools/fetch-mujoco`),
//! and gives the `euc` binary on macOS the Info.plist that Bluetooth access needs.

use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    // mujoco-rs links the library from MUJOCO_DYNAMIC_LINK_DIR; on Unix the loader also needs an rpath to it.
    // Windows finds mujoco.dll on PATH instead.
    if let Ok(dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR")
        && std::env::var_os("CARGO_CFG_UNIX").is_some()
    {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        embed_info_plist();
    }
}

/// `euc sim serve --ble` and `euc replay --ble` use CoreBluetooth, and macOS ends a program that
/// asks for Bluetooth without a usage description. A command-line tool carries its Info.plist in
/// a `__TEXT,__info_plist` section; when it runs from a terminal, macOS asks the user on behalf
/// of the terminal app instead (see docs/ble-radio.md).
fn embed_info_plist() {
    let version = std::env::var("CARGO_PKG_VERSION").unwrap_or_default();
    let plist = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleIdentifier</key>
    <string>com.tarikbc.euc-studio.euc</string>
    <key>CFBundleName</key>
    <string>euc</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleShortVersionString</key>
    <string>{version}</string>
    <key>NSBluetoothAlwaysUsageDescription</key>
    <string>euc presents a simulated electric unicycle over Bluetooth, so phone and watch apps can connect to it.</string>
</dict>
</plist>
"#
    );
    let path = PathBuf::from(std::env::var_os("OUT_DIR").expect("cargo sets OUT_DIR")).join("Info.plist");
    std::fs::write(&path, plist).expect("write Info.plist");
    println!("cargo::rustc-link-arg-bin=euc=-Wl,-sectcreate,__TEXT,__info_plist,{}", path.display());
}

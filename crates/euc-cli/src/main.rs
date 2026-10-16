//! `euc`: the headless EUC Studio command line.

mod ble;
mod compare;
mod fetch;
mod firmware;
mod image;
mod paths;
mod replay;
mod serve;
mod sim;
mod tcp;
mod ui;

use std::cell::Cell;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};

/// EUC Studio engine: fetch, verify and inspect electric unicycle firmware.
#[derive(Debug, Parser)]
#[command(name = "euc", version, propagate_version = true)]
struct Cli {
    /// Print JSON instead of text.
    #[arg(long, global = true)]
    json: bool,

    /// Wheel profiles folder that replaces the built-in profiles; its pins decide what counts as
    /// official firmware, so every run that uses it warns [default: the built-in profiles]
    #[arg(long, global = true, env = "EUC_PROFILES", value_name = "DIR")]
    profiles: Option<PathBuf>,

    /// Firmware cache folder [default: the per-user cache, e.g. ~/Library/Caches/euc-studio/firmware]
    #[arg(long, global = true, env = "EUC_CACHE_DIR", value_name = "DIR")]
    cache_dir: Option<PathBuf>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Download a wheel's official firmware and check it against the pinned sha256.
    Fetch(fetch::Args),
    /// Manage the firmware cache.
    #[command(subcommand)]
    Firmware(firmware::Command),
    /// Inspect firmware images.
    #[command(subcommand)]
    Image(image::Command),
    /// Run firmware in the simulator.
    #[command(subcommand)]
    Sim(sim::Command),
    /// Play a BLE capture back as a wheel, on this Mac's Bluetooth or a TCP bridge (docs/ble-radio.md).
    Replay(replay::Args),
}

/// Whether a command's verdict was positive. Errors are separate: they abort with a message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    Ok,
    Refused,
}

/// Settings every command shares.
struct Context {
    json: bool,
    profiles: Option<PathBuf>,
    cache_dir: Option<PathBuf>,
    /// Set once the profiles override warning has been printed.
    warned: Cell<bool>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let ctx = Context { json: cli.json, profiles: cli.profiles, cache_dir: cli.cache_dir, warned: Cell::new(false) };
    let result = match cli.command {
        Command::Fetch(args) => fetch::run(&ctx, &args),
        Command::Firmware(command) => firmware::run(&ctx, command),
        Command::Image(command) => image::run(&ctx, command),
        Command::Sim(command) => sim::run(&ctx, command),
        Command::Replay(args) => replay::run(&ctx, &args),
    };
    match result {
        Ok(Verdict::Ok) => ExitCode::SUCCESS,
        Ok(Verdict::Refused) => ExitCode::FAILURE,
        Err(error) => {
            ui::error(&error);
            ExitCode::FAILURE
        }
    }
}

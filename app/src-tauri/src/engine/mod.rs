//! The live engine behind the studio: Tauri commands, the sim thread and the stream.
//!
//! The web view calls `engine_connect` with two channels: a binary one for telemetry batches
//! (`euc_telemetry::batch`) and a JSON one for status, bodies, the display layout, events, phone frames and board outputs. The
//! engine starts on the first connect and keeps running across web view reloads.

#[cfg(dev)]
pub mod control;
mod cpu;
mod display;
mod firmware;
mod host;
mod lights;
mod phone;
mod runner;
mod scenarios;

use std::sync::Mutex;

use serde::Serialize;
use tauri::State;
use tauri::ipc::{Channel, InvokeResponseBody};

use runner::{Message, Runner, Sinks};

#[derive(Default)]
pub struct EngineState(Mutex<Option<Runner>>);

/// What the web view needs to set up its live client.
#[derive(Serialize)]
pub struct EngineInfo {
    backend: &'static str,
    scenarios: Vec<scenarios::ScenarioInfo>,
    commands: Vec<phone::CommandSpec>,
    /// `EUC_STUDIO_PERF=<seconds>`: the web view measures its frame times for that long and logs
    /// them (to the terminal through `frontend_log`).
    perf_seconds: Option<f64>,
}

fn with_runner<T>(state: &State<'_, EngineState>, f: impl FnOnce(&Runner) -> T) -> Result<T, String> {
    let guard = state.0.lock().map_err(|_| "the engine state is poisoned".to_owned())?;
    guard.as_ref().map(f).ok_or_else(|| "the engine is not running; connect first".to_owned())
}

/// Attaches the web view's channels and starts the engine if it is not running yet.
#[tauri::command]
pub fn engine_connect(
    state: State<'_, EngineState>,
    stream: Channel<InvokeResponseBody>,
    messages: Channel<Message>,
) -> Result<EngineInfo, String> {
    let scenarios = scenarios::infos();
    let first = scenarios.first().map(|s| s.id.clone()).unwrap_or_default();
    let mut guard = state.0.lock().map_err(|_| "the engine state is poisoned".to_owned())?;
    let runner =
        guard.get_or_insert_with(|| Runner::start(Box::new(firmware::build_host), first, cpu::backend().name()));
    runner.attach(Sinks { stream, messages });
    // A reloaded web view needs the bodies and a clean slate even though the run goes on.
    runner.announce();
    Ok(EngineInfo {
        backend: cpu::backend().name(),
        scenarios,
        commands: phone::catalogue(),
        perf_seconds: std::env::var("EUC_STUDIO_PERF").ok().and_then(|v| v.parse().ok()),
    })
}

#[tauri::command]
pub fn engine_play(state: State<'_, EngineState>, playing: bool) -> Result<(), String> {
    with_runner(&state, |r| r.play(playing))
}

#[tauri::command]
pub fn engine_step(state: State<'_, EngineState>) -> Result<(), String> {
    with_runner(&state, Runner::step)
}

#[tauri::command]
pub fn engine_set_rtf(state: State<'_, EngineState>, rtf: f64) -> Result<(), String> {
    if !rtf.is_finite() || rtf <= 0.0 {
        return Err(format!("real-time factor must be positive, got {rtf}"));
    }
    with_runner(&state, |r| r.set_rtf(rtf))
}

#[tauri::command]
pub fn engine_restart(state: State<'_, EngineState>, scenario: Option<String>) -> Result<(), String> {
    if let Some(s) = &scenario
        && !scenarios::infos().iter().any(|i| i.id == *s)
    {
        return Err(format!("unknown scenario \"{s}\""));
    }
    with_runner(&state, |r| r.restart(scenario))
}

/// Sends a phone command. Never-send commands do not exist here (see `phone`).
#[tauri::command]
pub fn engine_send(state: State<'_, EngineState>, id: String, value: Option<u8>) -> Result<String, String> {
    let command = phone::command(&id, value)?;
    let bytes = phone::encode(&command)?;
    let shown: String = bytes.iter().map(|&(_, b)| char::from(b)).collect();
    let label = format!("Phone sent {} ({shown})", command.info().name);
    let answer = command.safety() == euc_proto_begode::Safety::Read;
    with_runner(&state, |r| r.send(bytes, label.clone(), answer))?;
    Ok(shown)
}

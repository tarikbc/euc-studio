//! Dev control file: drive a run from a script or an agent without clicking.
//!
//! With `EUC_STUDIO_CONTROL=<file>` (dev builds only), the studio reads the file every 200 ms, runs
//! each line as a command and empties it:
//!
//! ```text
//! restart [scenario]      play | pause      rtf <factor>      send <command id> [value]
//! eval <javascript>
//! ```
//!
//! Each result is printed to the terminal as `[control] <line>: <result>`; `eval` runs in the web
//! view, whose `console.warn` and `console.error` reach the terminal too.

use std::path::PathBuf;
use std::thread;
use std::time::Duration;

use tauri::{AppHandle, Manager};

use super::EngineState;

/// Starts the control thread if `EUC_STUDIO_CONTROL` names a file.
pub fn spawn(app: AppHandle) {
    let Some(path) = std::env::var_os("EUC_STUDIO_CONTROL").map(PathBuf::from) else { return };
    let _ = std::fs::write(&path, "");
    eprintln!("[control] reading commands from {}", path.display());
    let spawned = thread::Builder::new().name("euc-control".into()).spawn(move || {
        loop {
            thread::sleep(Duration::from_millis(200));
            let Ok(text) = std::fs::read_to_string(&path) else { continue };
            if text.trim().is_empty() {
                continue;
            }
            let _ = std::fs::write(&path, "");
            for line in text.lines().map(str::trim).filter(|l| !l.is_empty()) {
                let result = run(&app, line);
                eprintln!("[control] {line}: {}", result.unwrap_or_else(|e| format!("error: {e}")));
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("[control] cannot start: {e}");
    }
}

fn run(app: &AppHandle, line: &str) -> Result<String, String> {
    if let Some(js) = line.strip_prefix("eval ") {
        let webview = app.webview_windows().into_values().next().ok_or("no web view")?;
        return webview.eval(js).map(|()| "evaluated".into()).map_err(|e| e.to_string());
    }
    let state = app.state::<EngineState>();
    let mut words = line.split_whitespace();
    match (words.next(), words.next(), words.next()) {
        (Some("restart"), scenario, None) => {
            super::engine_restart(state, scenario.map(str::to_owned)).map(|()| "restarted".into())
        }
        (Some("play"), None, None) => super::engine_play(state, true).map(|()| "playing".into()),
        (Some("pause"), None, None) => super::engine_play(state, false).map(|()| "paused".into()),
        (Some("rtf"), Some(rtf), None) => {
            let rtf = rtf.parse().map_err(|_| format!("not a number: {rtf}"))?;
            super::engine_set_rtf(state, rtf).map(|()| format!("{rtf}x"))
        }
        (Some("send"), Some(id), value) => {
            let value = value.map(|v| v.parse().map_err(|_| format!("not a byte: {v}"))).transpose()?;
            super::engine_send(state, id.to_owned(), value).map(|sent| format!("sent {sent}"))
        }
        _ => Err("expected restart [scenario], play, pause, rtf <x>, send <id> [value] or eval <js>".into()),
    }
}

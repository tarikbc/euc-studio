//! EUC Studio desktop shell: the web UI plus the live engine (`engine`), which runs the real
//! firmware in the simulator on its own thread and streams telemetry to the web view.

mod engine;

/// Web view errors and warnings, printed where the app was started (WKWebView and WebView2 keep
/// their consoles to themselves).
#[tauri::command]
fn frontend_log(level: String, message: String) {
    eprintln!("[webview {level}] {message}");
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(engine::EngineState::default())
        .setup(|_app| {
            // EUC_STUDIO_CONTROL: drive the run from a file (dev builds only, see engine::control).
            #[cfg(dev)]
            engine::control::spawn(_app.handle().clone());
            Ok(())
        })
        .on_page_load(|webview, payload| {
            eprintln!("[page] {:?} {}", payload.event(), webview.url().map(|u| u.to_string()).unwrap_or_default());
        })
        .invoke_handler(tauri::generate_handler![
            frontend_log,
            engine::engine_connect,
            engine::engine_play,
            engine::engine_step,
            engine::engine_set_rtf,
            engine::engine_restart,
            engine::engine_send,
        ])
        .run(tauri::generate_context!())
        .expect("error while running EUC Studio");
}

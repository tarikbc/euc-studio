//! Writes the telemetry schema as TypeScript, and the batch fixture the studio's tests decode:
//! `cargo run -p euc-telemetry --example gen_ts`.

use std::path::Path;

fn main() -> std::io::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let ts = root.join(euc_telemetry::TS_PATH);
    std::fs::write(&ts, euc_telemetry::typescript())?;
    println!("wrote {}", ts.display());
    let fixture = root.join(euc_telemetry::batch::FIXTURE_PATH);
    if let Some(dir) = fixture.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&fixture, euc_telemetry::batch::fixture())?;
    println!("wrote {}", fixture.display());
    Ok(())
}

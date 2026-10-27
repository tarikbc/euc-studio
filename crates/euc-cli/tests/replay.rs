//! `euc replay` over its TCP bridge: the capture at its own pace, the stock answers to writes,
//! the test controls (TCP and stdin), and the link log on stdout. The radio itself (`--ble`)
//! needs a second device; docs/ble-radio.md has the checklist for it.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, Instant};

use serde_json::Value;

const CAPTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/data/begode-extreme/idle-2026-10-15.hex");

/// A running `euc replay`, its stdout read line by line on a thread.
struct Replay {
    child: Child,
    stdin: ChildStdin,
    stdout: Receiver<String>,
}

impl Replay {
    fn start(args: &[&str]) -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_euc"))
            .arg("replay")
            .args(args)
            .env_remove("EUC_PROFILES")
            .env("NO_COLOR", "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let stdin = child.stdin.take().unwrap();
        let out = BufReader::new(child.stdout.take().unwrap());
        let (sender, stdout) = mpsc::channel();
        std::thread::spawn(move || {
            for line in out.lines().map_while(Result::ok) {
                if sender.send(line).is_err() {
                    break;
                }
            }
        });
        Self { child, stdin, stdout }
    }

    /// The next stdout line that starts with `prefix`.
    fn line(&self, prefix: &str) -> String {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let line = self.stdout.recv_timeout(left).unwrap_or_else(|_| panic!("no line starting with {prefix:?}"));
            if line.starts_with(prefix) {
                return line;
            }
        }
    }

    fn type_line(&mut self, text: &str) {
        writeln!(self.stdin, "{text}").unwrap();
    }
}

impl Drop for Replay {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// A bridge client that reads JSON lines with a short timeout.
struct Client {
    writer: TcpStream,
    reader: BufReader<TcpStream>,
}

impl Client {
    fn connect(bridge_row: &str) -> Self {
        let url = bridge_row.split_whitespace().find(|w| w.starts_with("tcp://")).unwrap().trim_end_matches(',');
        let stream = TcpStream::connect(url.trim_start_matches("tcp://")).unwrap();
        stream.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
        Self { writer: stream.try_clone().unwrap(), reader: BufReader::new(stream) }
    }

    fn send(&mut self, json: &str) {
        writeln!(self.writer, "{json}").unwrap();
    }

    /// Every message that arrives within `seconds`.
    fn messages(&mut self, seconds: f64) -> Vec<Value> {
        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        let mut out = Vec::new();
        let mut line = String::new();
        while Instant::now() < deadline {
            match self.reader.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) if line.ends_with('\n') => out.push(serde_json::from_str(&std::mem::take(&mut line)).unwrap()),
                _ => {}
            }
        }
        out
    }
}

/// The payload of a `notify` message.
fn bytes(message: &Value) -> Option<Vec<u8>> {
    (message["type"] == "notify").then(|| hex(message["hex"].as_str().unwrap()))
}

fn hex(text: &str) -> Vec<u8> {
    (0..text.len()).step_by(2).map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap()).collect()
}

/// A notification that starts a telemetry group (frame 00).
fn group_start(bytes: &[u8]) -> bool {
    bytes.starts_with(&[0x55, 0xAA]) && bytes.get(18) == Some(&0)
}

#[test]
fn replays_answers_v_like_the_stock_firmware_and_obeys_the_controls() {
    let mut replay = Replay::start(&[CAPTURE, "--bridge", "tcp://127.0.0.1:0", "--seconds", "20"]);
    let mut client = Client::connect(&replay.line("bridge"));

    let first = client.messages(0.8);
    assert_eq!(first[0]["type"], "hello");
    assert_eq!((first[0]["version"].as_u64(), first[0]["notify"].as_str()), (Some(1), Some("split")));
    let notifications: Vec<Vec<u8>> = first.iter().filter_map(bytes).collect();
    assert!(notifications.len() >= 6, "a group or more in 0.8 s: {}", notifications.len());
    assert!(notifications.iter().all(|n| n.len() == 20), "the capture's own 20-byte packets");

    // V: the next two group times send nothing, the third sends the banner instead of a group.
    client.send(r#"{"type":"write","hex":"56"}"#);
    let after: Vec<Vec<u8>> = client.messages(2.0).iter().filter_map(bytes).collect();
    let banner = after.iter().position(|n| n == b"GW1825020").expect("the banner within 2 s");
    assert!(!after[..banner].iter().any(|n| group_start(n)), "no group starts between the write and the reply");
    assert!(after[banner + 1..].iter().any(|n| group_start(n)), "telemetry resumes after the reply");
    assert!(replay.line("ble write ").ends_with(" central=tcp char=FFE1 len=1 data=V"));
    assert!(replay.line("ble reply ").ends_with(" char=FFE1 centrals=0 len=9 text=GW1825020"));

    // silent over TCP: nothing at all; resume on stdin: telemetry again.
    client.send(r#"{"type":"control","command":"silent"}"#);
    replay.line("ble silent t=");
    let _ = client.messages(0.1);
    assert_eq!(client.messages(0.8).iter().filter_map(bytes).count(), 0, "silent sends nothing");
    replay.type_line("resume");
    replay.line("ble resume t=");
    assert!(client.messages(0.8).iter().filter_map(bytes).count() >= 6);

    // off: no notifications, writes refused with a warning; a bad control is refused too.
    client.send(r#"{"type":"control","command":"off"}"#);
    replay.line("ble off t=");
    let _ = client.messages(0.1);
    client.send(r#"{"type":"write","hex":"4e"}"#);
    client.send(r#"{"type":"control","command":"reboot"}"#);
    let off = client.messages(0.8);
    assert_eq!(off.iter().filter_map(bytes).count(), 0);
    let warnings: Vec<&str> = off.iter().filter(|m| m["type"] == "log").filter_map(|m| m["text"].as_str()).collect();
    assert!(warnings.iter().any(|w| w.contains("off")), "{warnings:?}");
    assert!(warnings.iter().any(|w| w.contains("unknown control")), "{warnings:?}");

    replay.type_line("on");
    replay.line("ble on t=");
    replay.type_line("quit");
    replay.line("ble quit t=");
    assert!(replay.child.wait().unwrap().success());
}

#[test]
fn reads_captures_with_a_recorder_header_and_ends_with_the_capture() {
    let dir = tempfile::tempdir().unwrap();
    let lines = std::fs::read_to_string(CAPTURE).unwrap().lines().take(12).collect::<Vec<_>>().join("\n");
    for header in [
        "# eucctl record GotWay 2026-10-16T03:04:33Z\n\n",
        "# euc-capture v1\n# brand: Begode\n# wheel: begode-extreme\n# name: GotWay_71105\n0.000 write:FFE1 4e\n",
    ] {
        let path = dir.path().join("idle.hex");
        std::fs::write(&path, format!("{header}{lines}\n")).unwrap();
        let mut replay = Replay::start(&[path.to_str().unwrap(), "--bridge", "tcp://127.0.0.1:0"]);
        let row = replay.line("replay ");
        assert!(row.contains("12 notifications"), "{row}");
        replay.line("ble end t=");
        assert!(replay.child.wait().unwrap().success());
    }
}

#[test]
fn replays_a_brand_through_its_radio_only_profile() {
    // A synthetic LeaperKim-style capture (2 s, 20-byte notifications): no simulated wheel exists
    // for the brand, only profiles/leaperkim/ble.toml.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("leaperkim.hex");
    let lines = (0..40)
        .map(|i| {
            format!("{:.3} dc 5a 5c 20 {i:02x} 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\n", f64::from(i) * 0.05)
        })
        .collect::<Vec<_>>()
        .concat();
    let header = "# euc-capture v1\n# brand: LeaperKim\n# wheel: leaperkim\n# name: LK12345\n";
    std::fs::write(&path, format!("{header}{lines}")).unwrap();

    let mut replay = Replay::start(&[path.to_str().unwrap(), "--bridge", "tcp://127.0.0.1:0"]);
    let mut client = Client::connect(&replay.line("bridge"));
    let messages = client.messages(1.0);
    assert_eq!((messages[0]["type"].as_str(), messages[0]["wheel"].as_str()), (Some("hello"), Some("leaperkim")));
    let notifications: Vec<Vec<u8>> = messages.iter().filter_map(bytes).collect();
    assert!(!notifications.is_empty(), "the capture plays");
    assert!(notifications.iter().all(|n| n.len() == 20 && n.starts_with(&[0xDC, 0x5A, 0x5C])), "bytes as captured");
    replay.line("ble end t=");
    assert!(replay.child.wait().unwrap().success());

    // A name that is neither a wheel nor a radio profile is still refused.
    let output = Command::new(env!("CARGO_BIN_EXE_euc"))
        .args(["replay", path.to_str().unwrap(), "--wheel", "nope", "--bridge", "tcp://127.0.0.1:0"])
        .env_remove("EUC_PROFILES")
        .env("NO_COLOR", "1")
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("no wheel profile \"nope\""));
}

#[test]
fn needs_somewhere_to_replay_and_a_valid_capture() {
    let output =
        Command::new(env!("CARGO_BIN_EXE_euc")).args(["replay", CAPTURE]).env("NO_COLOR", "1").output().unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("--ble"));

    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.hex");
    std::fs::write(&path, "0.000 55 aa\n0.010 zz\n").unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_euc"))
        .args(["replay", path.to_str().unwrap(), "--bridge", "tcp://127.0.0.1:0"])
        .env("NO_COLOR", "1")
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("line 2: \"zz\" is not a hex byte"));
}

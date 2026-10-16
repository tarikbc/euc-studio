//! The TCP side of the BLE bridge (`docs/bridge.md`): one client, newline-delimited JSON, never
//! blocking the wheel. `euc sim serve` and `euc replay` serve it.

use std::io::{self, ErrorKind, Read, Write};
use std::net::TcpStream;

use anyhow::Context as _;
use euc_sim::bridge::ServerMessage;

/// The `HOST:PORT` of a `tcp://HOST:PORT` URL.
pub(crate) fn address(url: &str) -> anyhow::Result<&str> {
    url.strip_prefix("tcp://").with_context(|| format!("\"{url}\" is not tcp://HOST:PORT"))
}

/// One connected client: lines in, lines out, never blocking the simulation.
pub(crate) struct Client {
    stream: TcpStream,
    inbox: Vec<u8>,
    outbox: Vec<u8>,
}

impl Client {
    /// A client on a stream that is already non-blocking.
    pub(crate) const fn new(stream: TcpStream) -> Self {
        Self { stream, inbox: Vec::new(), outbox: Vec::new() }
    }

    /// Queues one message.
    pub(crate) fn send(&mut self, message: &ServerMessage) {
        if let Ok(line) = serde_json::to_vec(message) {
            self.outbox.extend_from_slice(&line);
            self.outbox.push(b'\n');
        }
    }

    /// Writes what the socket takes now. `Err` when the client is gone.
    pub(crate) fn flush(&mut self) -> io::Result<()> {
        while !self.outbox.is_empty() {
            match self.stream.write(&self.outbox) {
                Ok(0) => return Err(ErrorKind::WriteZero.into()),
                Ok(n) => {
                    self.outbox.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Complete lines received so far. `Err` when the client is gone.
    pub(crate) fn lines(&mut self) -> io::Result<Vec<String>> {
        let mut buf = [0u8; 4096];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => return Err(ErrorKind::UnexpectedEof.into()),
                Ok(n) => self.inbox.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        let mut lines = Vec::new();
        while let Some(end) = self.inbox.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.inbox.drain(..=end).collect();
            lines.push(String::from_utf8_lossy(&line).trim().to_owned());
        }
        Ok(lines)
    }
}

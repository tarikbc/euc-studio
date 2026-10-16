//! The module between a wheel's electronics and the radio. The radio only sees notifications and
//! writes per characteristic; a module model turns what the wheel's electronics do into them.
//!
//! - [`UartBridge`]: an HM-10 style transparent bridge on a controller UART (Begode). Bytes in,
//!   bytes out, cut into notifications by the module's rules.
//! - A BLE chip with firmware of its own ([`Module::Node`](crate::Module::Node)) runs as a node of
//!   the simulated wheel and hands notifications per characteristic to the
//!   [`Link`](crate::Link) directly; it needs no model here.

use std::collections::VecDeque;

use crate::{BleProfile, CharId, Module};

/// A notification the wheel's module sends.
#[derive(Debug, Clone, PartialEq)]
pub struct Notification {
    /// When it goes out, seconds on the caller's clock.
    pub at: f64,
    /// On which characteristic.
    pub characteristic: CharId,
    /// The payload.
    pub bytes: Vec<u8>,
}

/// An HM-10 style transparent UART bridge, the module of Begode wheels.
///
/// The controller's UART output goes out on the profile's `tx` characteristic: packed into
/// notifications of `chunk` bytes, and a shorter one once the line has been idle for `idle_s`
/// (a lone 24-byte frame goes out as 20 + 4), at most one per `interval_s`. Whatever a central
/// writes on an `rx` characteristic goes into the UART as it is.
#[derive(Debug, Clone)]
pub struct UartBridge {
    tx: CharId,
    rx: Vec<CharId>,
    chunk: usize,
    idle_s: f64,
    interval_s: f64,
    /// Bytes not sent yet, and when the last one came.
    pending: Vec<u8>,
    last_byte: f64,
    /// Notifications cut, waiting for their time.
    queue: VecDeque<Notification>,
    /// When the next notification may go out at the earliest (pacing).
    next_free: f64,
}

impl UartBridge {
    /// The bridge a profile describes; `None` when its module is not a UART bridge.
    pub fn new(profile: &BleProfile) -> Option<Self> {
        let Module::UartBridge { chunk, idle_s, interval_s } = profile.module else { return None };
        Some(Self {
            tx: profile.tx(),
            rx: profile.characteristics().filter(|(_, _, c)| c.rx).map(|(id, _, _)| id).collect(),
            chunk: chunk.max(1),
            idle_s,
            interval_s,
            pending: Vec::new(),
            last_byte: f64::NEG_INFINITY,
            queue: VecDeque::new(),
            next_free: f64::NEG_INFINITY,
        })
    }

    /// A byte left the controller's UART at `t` seconds (times never go back).
    pub fn uart(&mut self, t: f64, byte: u8) {
        if !self.pending.is_empty() && t - self.last_byte > self.idle_s {
            self.cut(self.last_byte);
        }
        self.pending.push(byte);
        self.last_byte = t;
        if self.pending.len() == self.chunk {
            self.cut(t);
        }
    }

    /// Notifications due by `now`, in order.
    pub fn poll(&mut self, now: f64) -> Vec<Notification> {
        if !self.pending.is_empty() && now - self.last_byte > self.idle_s {
            self.cut(self.last_byte);
        }
        let due = self.queue.iter().take_while(|n| n.at <= now).count();
        self.queue.drain(..due).collect()
    }

    /// What reaches the controller's UART when a central writes `bytes` on `characteristic`:
    /// all of it on an `rx` characteristic, nothing elsewhere.
    pub fn write<'a>(&self, characteristic: CharId, bytes: &'a [u8]) -> Option<&'a [u8]> {
        self.rx.contains(&characteristic).then_some(bytes)
    }

    fn cut(&mut self, at: f64) {
        let at = at.max(self.next_free);
        self.next_free = at + self.interval_s;
        self.queue.push_back(Notification { at, characteristic: self.tx, bytes: std::mem::take(&mut self.pending) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One byte at 115 200 baud, seconds.
    const BYTE: f64 = 10.0 / 115_200.0;

    fn frame(bridge: &mut UartBridge, start: f64) {
        for i in 0..24u8 {
            bridge.uart(start + f64::from(i) * BYTE, i);
        }
    }

    fn sizes(notifications: &[Notification]) -> Vec<usize> {
        notifications.iter().map(|n| n.bytes.len()).collect()
    }

    #[test]
    fn a_frame_goes_out_as_20_plus_4_like_the_module() {
        let mut bridge = UartBridge::new(&BleProfile::hm10("T")).unwrap();
        frame(&mut bridge, 1.0);
        let end = 1.0 + 23.0 * BYTE;
        let first = bridge.poll(end);
        assert_eq!(sizes(&first), [20]);
        assert_eq!(first[0].characteristic, CharId(0));
        assert_eq!(sizes(&bridge.poll(end + 0.0009)), Vec::<usize>::new(), "the line is not idle yet");
        let rest = bridge.poll(end + 0.002);
        assert_eq!(rest[0].bytes, (20..24).collect::<Vec<u8>>());
        assert!((rest[0].at - end).abs() < 1e-12, "stamped with its last byte");
    }

    #[test]
    fn a_burst_of_frames_packs_into_20_byte_notifications() {
        // The stock firmware starts the frames of a burst 2.22 ms apart; five make six notifications.
        let mut bridge = UartBridge::new(&BleProfile::hm10("T")).unwrap();
        for k in 0..5 {
            frame(&mut bridge, f64::from(k) * 0.00222);
        }
        assert_eq!(sizes(&bridge.poll(1.0)), [20; 6]);
    }

    #[test]
    fn pacing_spaces_notifications_out() {
        let mut profile = BleProfile::hm10("T");
        profile.module = Module::UartBridge { chunk: 20, idle_s: 0.001, interval_s: 0.0075 };
        let mut bridge = UartBridge::new(&profile).unwrap();
        for k in 0..5 {
            frame(&mut bridge, f64::from(k) * 0.00222);
        }
        let all = bridge.poll(1.0);
        let gaps: Vec<f64> = all.windows(2).map(|w| w[1].at - w[0].at).collect();
        assert!(gaps.iter().all(|gap| *gap >= 0.0075 - 1e-12), "{gaps:?}");
        let mut bridge = UartBridge::new(&profile).unwrap();
        frame(&mut bridge, 0.0);
        frame(&mut bridge, 0.0025);
        assert_eq!(bridge.poll(0.003).len(), 1, "the second notification waits for its slot");
    }

    #[test]
    fn writes_on_rx_characteristics_reach_the_uart() {
        let bridge = UartBridge::new(&BleProfile::hm10("T")).unwrap();
        assert_eq!(bridge.write(CharId(0), b"V"), Some(&b"V"[..]));
        assert_eq!(bridge.write(CharId(1), b"V"), None);
        let mut node = BleProfile::hm10("T");
        node.module = Module::Node { chunk: 20 };
        assert!(UartBridge::new(&node).is_none());
    }
}

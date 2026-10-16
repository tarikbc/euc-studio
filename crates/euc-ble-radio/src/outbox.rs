//! The radio's own transmit queue. CoreBluetooth takes only a few notifications at a time:
//! `updateValue` returns false when its queue is full, and calls
//! `peripheralManagerIsReadyToUpdateSubscribers` once it has room. Notifications wait here
//! meanwhile, in order; only when this queue overflows does the oldest go.

use std::collections::VecDeque;

use crate::CharId;

/// Most notifications that wait (about 13 s of Begode telemetry); beyond that the oldest are
/// dropped, as a module with a full buffer would: an app wants fresh telemetry, not old.
pub(crate) const CAPACITY: usize = 256;

/// A busy radio that has not said it is ready again after this long gets another try, seconds:
/// a lost "ready" callback must not stop the link for good.
pub(crate) const RETRY_S: f64 = 1.0;

/// Shortest time between two reports of dropped notifications, seconds.
pub(crate) const REPORT_S: f64 = 1.0;

/// Notifications waiting for the radio.
#[derive(Debug)]
pub(crate) struct Outbox {
    queue: VecDeque<(CharId, Vec<u8>)>,
    capacity: usize,
    /// Since when the radio has refused a notification, waiting for its "ready".
    busy_since: Option<f64>,
    /// Notifications dropped and not reported yet.
    dropped: usize,
    /// Most notifications that waited since the last report.
    deepest: usize,
    /// When the last report went out.
    reported: f64,
}

/// Dropped notifications to log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DropReport {
    /// Dropped since the last report.
    pub(crate) count: usize,
    /// Most notifications that waited meanwhile (the queue's depth when it overflowed).
    pub(crate) queued: usize,
}

impl Outbox {
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            queue: VecDeque::new(),
            capacity: capacity.max(1),
            busy_since: None,
            dropped: 0,
            deepest: 0,
            reported: f64::NEG_INFINITY,
        }
    }

    /// Notifications waiting.
    pub(crate) fn len(&self) -> usize {
        self.queue.len()
    }

    /// Queues one notification; when the queue is full, the oldest one goes.
    pub(crate) fn push(&mut self, characteristic: CharId, bytes: Vec<u8>) {
        if self.queue.len() == self.capacity {
            self.queue.pop_front();
            self.dropped += 1;
        }
        self.queue.push_back((characteristic, bytes));
        self.deepest = self.deepest.max(self.queue.len());
    }

    /// The next notification to send at `now`, unless the radio is busy (and not for so long
    /// that it deserves another try).
    pub(crate) fn next(&mut self, now: f64) -> Option<(CharId, Vec<u8>)> {
        if let Some(since) = self.busy_since {
            if now - since < RETRY_S {
                return None;
            }
            self.busy_since = None;
        }
        self.queue.pop_front()
    }

    /// The radio refused `item` (its queue is full): it goes back to the front, and nothing more
    /// is sent until [`ready`](Self::ready).
    pub(crate) fn refused(&mut self, now: f64, item: (CharId, Vec<u8>)) {
        self.queue.push_front(item);
        self.busy_since = Some(now);
    }

    /// The radio has room again.
    pub(crate) fn ready(&mut self) {
        self.busy_since = None;
    }

    /// Whether the radio refused and has not said it is ready again.
    #[cfg(test)]
    const fn busy(&self) -> bool {
        self.busy_since.is_some()
    }

    /// Forgets what waits for `characteristic` (nobody is subscribed to it any more), or
    /// everything (`None`: nobody at all, so the radio is not busy either).
    pub(crate) fn clear(&mut self, characteristic: Option<CharId>) {
        if let Some(id) = characteristic {
            self.queue.retain(|(c, _)| *c != id);
        } else {
            self.queue.clear();
            self.busy_since = None;
        }
    }

    /// Dropped notifications to report at `now`: at most one report per [`REPORT_S`].
    pub(crate) fn report(&mut self, now: f64) -> Option<DropReport> {
        if self.dropped == 0 || now - self.reported < REPORT_S {
            return None;
        }
        let report = DropReport { count: self.dropped, queued: self.deepest };
        (self.dropped, self.deepest, self.reported) = (0, self.queue.len(), now);
        Some(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: CharId = CharId(0);
    const B: CharId = CharId(1);

    fn bytes(n: u8) -> Vec<u8> {
        vec![n; 20]
    }

    #[test]
    fn notifications_go_in_order() {
        let mut outbox = Outbox::new(8);
        for n in 0..3 {
            outbox.push(A, bytes(n));
        }
        let sent: Vec<u8> = std::iter::from_fn(|| outbox.next(0.0)).map(|(_, b)| b[0]).collect();
        assert_eq!(sent, [0, 1, 2]);
        assert_eq!(outbox.report(0.0), None, "nothing dropped");
    }

    #[test]
    fn a_refused_notification_waits_for_ready_and_keeps_its_place() {
        let mut outbox = Outbox::new(8);
        outbox.push(A, bytes(0));
        outbox.push(A, bytes(1));
        let first = outbox.next(0.0).unwrap();
        outbox.refused(0.0, first);
        outbox.push(A, bytes(2));
        assert!(outbox.busy());
        assert_eq!(outbox.next(0.5), None, "busy: wait for the radio");
        assert_eq!(outbox.len(), 3, "nothing is lost while the radio is busy");
        outbox.ready();
        let sent: Vec<u8> = std::iter::from_fn(|| outbox.next(0.5)).map(|(_, b)| b[0]).collect();
        assert_eq!(sent, [0, 1, 2]);
    }

    #[test]
    fn a_busy_radio_without_ready_gets_another_try() {
        let mut outbox = Outbox::new(8);
        outbox.push(A, bytes(0));
        let first = outbox.next(0.0).unwrap();
        outbox.refused(10.0, first);
        assert_eq!(outbox.next(10.9), None);
        assert_eq!(outbox.next(11.0).map(|(_, b)| b[0]), Some(0));
        assert!(!outbox.busy());
    }

    #[test]
    fn only_an_overflow_drops_and_the_oldest_go() {
        let mut outbox = Outbox::new(4);
        let first = {
            outbox.push(A, bytes(0));
            outbox.next(0.0).unwrap()
        };
        outbox.refused(0.0, first);
        for n in 1..=6 {
            outbox.push(A, bytes(n));
        }
        assert_eq!(outbox.len(), 4);
        assert_eq!(outbox.report(0.2), Some(DropReport { count: 3, queued: 4 }));
        outbox.ready();
        let sent: Vec<u8> = std::iter::from_fn(|| outbox.next(0.3)).map(|(_, b)| b[0]).collect();
        assert_eq!(sent, [3, 4, 5, 6], "the freshest stay");
    }

    #[test]
    fn drops_are_reported_at_most_once_a_second_with_the_depth() {
        let mut outbox = Outbox::new(2);
        outbox.push(A, bytes(0));
        outbox.push(A, bytes(1));
        outbox.push(A, bytes(2));
        assert_eq!(outbox.report(5.0), Some(DropReport { count: 1, queued: 2 }));
        outbox.push(A, bytes(3));
        outbox.push(A, bytes(4));
        assert_eq!(outbox.report(5.5), None, "too soon");
        assert_eq!(outbox.report(6.0), Some(DropReport { count: 2, queued: 2 }));
        assert_eq!(outbox.report(9.0), None, "nothing new");
    }

    #[test]
    fn clearing_a_characteristic_keeps_the_others() {
        let mut outbox = Outbox::new(8);
        outbox.push(A, bytes(0));
        outbox.push(B, bytes(1));
        let first = outbox.next(0.0).unwrap();
        outbox.refused(0.0, first);
        outbox.clear(Some(A));
        assert_eq!((outbox.len(), outbox.busy()), (1, true));
        outbox.clear(None);
        assert_eq!((outbox.len(), outbox.busy()), (0, false), "nobody left: not busy either");
    }
}

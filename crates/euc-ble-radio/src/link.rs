use crate::{BleProfile, CharId, Control, Entry, Log, Protocol, Radio, RadioEvent};

/// What the source behind the link must do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// A central wrote these bytes on this characteristic: they go to the wheel, in order.
    Write {
        /// The characteristic written.
        characteristic: CharId,
        /// The bytes.
        bytes: Vec<u8>,
    },
    /// A `quit` control: end the program.
    Quit,
}

/// The wheel's BLE link, whatever drives it (the simulator's module, a node of the wheel, a
/// replay) and wherever it goes (the radio, a TCP bridge, or both).
///
/// It deals in notifications and writes per characteristic of a [`BleProfile`] and never looks
/// inside the bytes, except through the brand's [`Protocol`] to log text replies. The source
/// hands every notification to [`notify`](Self::notify) and every TCP write to
/// [`tcp_write`](Self::tcp_write), and calls [`poll`](Self::poll) often. The link applies the
/// test [`Control`]s (`silent` and `off` stop notifications everywhere), passes notifications to
/// the [`Radio`], and prints the [`Entry`] lines.
pub struct Link {
    profile: BleProfile,
    protocol: Box<dyn Protocol>,
    radio: Option<Radio>,
    log: Log,
    silent: bool,
    off: bool,
    /// Subscriptions: central and characteristic.
    subscriptions: Vec<(String, CharId)>,
}

impl std::fmt::Debug for Link {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut debug = f.debug_struct("Link");
        debug.field("radio", &self.radio.is_some()).field("silent", &self.silent).field("off", &self.off);
        debug.finish_non_exhaustive()
    }
}

impl Link {
    /// A link that serves `profile` on `radio` (none: a TCP bridge only), reads text with
    /// `protocol`, and prints on `log`.
    pub fn new(profile: BleProfile, protocol: Box<dyn Protocol>, radio: Option<Radio>, log: Log) -> Self {
        Self { profile, protocol, radio, log, silent: false, off: false, subscriptions: Vec::new() }
    }

    /// The log, for lines the source prints itself ([`Entry::Lap`], [`Entry::End`]).
    pub const fn log(&self) -> &Log {
        &self.log
    }

    /// The layout the link serves.
    pub const fn profile(&self) -> &BleProfile {
        &self.profile
    }

    /// Whether notifications go out (neither `silent` nor `off`).
    pub const fn open(&self) -> bool {
        !self.silent && !self.off
    }

    /// Centrals subscribed to anything.
    fn centrals(&self) -> usize {
        let mut ids: Vec<&str> = self.subscriptions.iter().map(|(central, _)| central.as_str()).collect();
        ids.sort_unstable();
        ids.dedup();
        ids.len()
    }

    /// The wheel sent one notification on `characteristic`. Returns whether it goes out, so a TCP
    /// bridge sends what the radio sends.
    pub fn notify(&mut self, characteristic: CharId, bytes: &[u8]) -> bool {
        let open = self.open();
        let uuid = self.profile.uuid(characteristic).to_owned();
        for text in self.protocol.texts(self.log.elapsed(), &uuid, bytes) {
            self.reply(open, characteristic, &text);
        }
        if !open {
            return false;
        }
        self.log.line(&Entry::Notify { characteristic: &uuid, bytes });
        if let Some(radio) = &self.radio {
            radio.notify(characteristic, bytes);
        }
        true
    }

    /// A TCP client wrote bytes as a phone would, on the first `rx` characteristic. Returns
    /// whether they reach the wheel: not while it is `off`.
    pub fn tcp_write(&mut self, bytes: &[u8]) -> bool {
        if self.off {
            return false;
        }
        self.log.line(&Entry::Write { central: "tcp", characteristic: self.profile.uuid(self.profile.rx()), bytes });
        true
    }

    /// Applies a test control and logs it. Returns [`Action::Quit`] for `quit`.
    pub fn control(&mut self, control: Control) -> Option<Action> {
        self.log.line(&Entry::Control(control));
        match control {
            Control::Silent => self.silent = true,
            Control::Resume => self.silent = false,
            Control::Off | Control::On => {
                self.off = control == Control::Off;
                if self.off {
                    self.subscriptions.clear();
                }
                if let Some(radio) = &self.radio {
                    radio.set_on(!self.off);
                }
            }
            Control::Quit => return Some(Action::Quit),
        }
        None
    }

    /// Handles what the radio saw since the last call (logs it) and finishes quiet text. Returns
    /// the writes for the source, in order.
    pub fn poll(&mut self) -> Vec<Action> {
        let open = self.open();
        if let Some(text) = self.protocol.quiet(self.log.elapsed()) {
            self.reply(open, self.profile.tx(), &text);
        }
        let events = self.radio.as_ref().map(Radio::events).unwrap_or_default();
        let mut actions = Vec::new();
        for event in events {
            match event {
                RadioEvent::Bluetooth(state) => {
                    self.log.line(&Entry::Bluetooth { state });
                    if state == "unauthorized" {
                        self.log.line(&Entry::Error { text: &crate::RadioError::Denied.to_string() });
                    }
                }
                RadioEvent::Advertising => {
                    let list = |role: fn(&crate::Characteristic) -> bool| {
                        let uuids: Vec<&str> = self
                            .profile
                            .characteristics()
                            .filter(|(_, _, c)| role(c))
                            .map(|(_, _, c)| c.uuid.as_str())
                            .collect();
                        uuids.join(",")
                    };
                    let (tx, rx) = (list(|c| c.tx), list(|c| c.rx));
                    let services = self.profile.advertising.services.join(",");
                    let name = &self.profile.advertising.name;
                    self.log.line(&Entry::Ready { services: &services, tx: &tx, rx: &rx, name });
                }
                RadioEvent::Subscribed { central, characteristic, max } => {
                    if !self.subscriptions.iter().any(|(c, id)| *c == central && *id == characteristic) {
                        self.subscriptions.push((central.clone(), characteristic));
                    }
                    let uuid = self.profile.uuid(characteristic);
                    self.log.line(&Entry::Subscribe {
                        central: &central,
                        characteristic: uuid,
                        centrals: self.centrals(),
                        max,
                    });
                }
                RadioEvent::Unsubscribed { central, characteristic } => {
                    self.subscriptions.retain(|(c, id)| !(*c == central && *id == characteristic));
                    let uuid = self.profile.uuid(characteristic);
                    self.log.line(&Entry::Unsubscribe {
                        central: &central,
                        characteristic: uuid,
                        centrals: self.centrals(),
                    });
                }
                RadioEvent::Write { central, characteristic, bytes } => {
                    let uuid = self.profile.uuid(characteristic);
                    self.log.line(&Entry::Write { central: &central, characteristic: uuid, bytes: &bytes });
                    actions.push(Action::Write { characteristic, bytes });
                }
                RadioEvent::Busy { queued } => self.log.line(&Entry::Busy { queued }),
                RadioEvent::Dropped { count } => self.log.line(&Entry::Dropped { count }),
                RadioEvent::Error(text) => self.log.line(&Entry::Error { text: &text }),
            }
        }
        actions
    }

    fn reply(&self, open: bool, characteristic: CharId, text: &[u8]) {
        if open {
            let uuid = self.profile.uuid(characteristic);
            self.log.line(&Entry::Reply { characteristic: uuid, centrals: self.centrals(), bytes: text });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Opaque;

    #[test]
    fn silent_and_off_stop_notifications_and_off_stops_writes() {
        let mut link = Link::new(BleProfile::hm10("T"), Box::new(Opaque), None, Log::new(false));
        let tx = link.profile().tx();
        assert!(link.notify(tx, b"frame"));
        assert_eq!(link.control(Control::Silent), None);
        assert!(!link.notify(tx, b"frame"));
        assert!(link.tcp_write(b"V"), "a silent wheel still listens");
        link.control(Control::Resume);
        assert!(link.notify(tx, b"frame"));
        link.control(Control::Off);
        assert!(!link.notify(tx, b"frame"));
        assert!(!link.tcp_write(b"V"));
        link.control(Control::On);
        assert!(link.open() && link.tcp_write(b"V"));
        assert_eq!(link.control(Control::Quit), Some(Action::Quit));
        assert_eq!(link.poll(), Vec::<Action>::new());
    }
}

use std::fmt;
use std::str::FromStr;

/// A test control: what a device test does to the wheel's BLE link. Given as a line on stdin or
/// as a `control` message on the TCP bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    /// Keep every connection and send nothing (a wheel whose controller hangs).
    Silent,
    /// Send again after `silent`.
    Resume,
    /// Remove the service and stop advertising, as when the wheel is switched off.
    Off,
    /// Serve and advertise again after `off`.
    On,
    /// End the program.
    Quit,
}

impl Control {
    /// Every control, in the order the help lists them.
    pub const ALL: [Self; 5] = [Self::Silent, Self::Resume, Self::Off, Self::On, Self::Quit];

    /// The word that names it on stdin and in the log.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Silent => "silent",
            Self::Resume => "resume",
            Self::Off => "off",
            Self::On => "on",
            Self::Quit => "quit",
        }
    }
}

impl fmt::Display for Control {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A word that is not a [`Control`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("unknown control \"{0}\"; use silent, resume, off, on or quit")]
pub struct UnknownControl(pub String);

impl FromStr for Control {
    type Err = UnknownControl;

    /// One word, surrounding blanks ignored.
    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let word = text.trim();
        Self::ALL.into_iter().find(|c| c.name() == word).ok_or_else(|| UnknownControl(word.to_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controls_are_single_words() {
        for control in Control::ALL {
            assert_eq!(control.name().parse(), Ok(control));
        }
        assert_eq!(" silent\r\n".parse(), Ok(Control::Silent));
        assert_eq!("SILENT".parse::<Control>(), Err(UnknownControl("SILENT".into())));
        assert!("off now".parse::<Control>().is_err());
    }
}

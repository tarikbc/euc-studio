//! Text the wheel sends between frames.
//!
//! Replies to `V` and `N` are ASCII. Everything the firmware prints with `printf` (power-on log,
//! speed-alarm replies, the `k` debug dump) goes to the same UART, and its Chinese messages are
//! GBK-encoded.

use crate::settings::AlarmMode;

/// A text segment, interpreted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// Answer to `V`: the firmware banner, such as `GW1825020`.
    Version(String),
    /// Answer to `N`: the model name, such as `EXTREME` (sent as `NAME:EXTREME\r\n`).
    Name(String),
    /// Answer to `o`, `u`, `i`, `I`.
    SpeedAlarms(AlarmReply),
    /// Power-on log: whether the IMU answered (`MPU6500初始化成功` / `MPU6500初始化失败`).
    ImuInit {
        /// The IMU answered its identity check.
        ok: bool,
    },
    /// Power-on log: initialisation finished (`初始化完成..启动次数:N次`).
    InitDone {
        /// Power-on count, from a backup register.
        boot_count: u32,
    },
    /// Any other text (debug dump lines, unknown messages, line noise), decoded.
    Text(String),
}

/// The wheel's answer to a speed alarm command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmReply {
    /// The mode was set (`速度警报已打开` "speed alarm on", or `N级速度警报已关闭` "level N alarm off").
    Set(AlarmMode),
    /// Refused while moving (`本操作须要停车进行` "stop the wheel first").
    MustStop,
}

/// Messages recognised in [`Reply::parse`], with what they mean.
const ALARM_MESSAGES: [(&str, AlarmReply); 5] = [
    ("速度警报已打开", AlarmReply::Set(AlarmMode::AllOn)),
    ("一级速度警报已关闭", AlarmReply::Set(AlarmMode::Level1Off)),
    ("二级速度警报已关闭", AlarmReply::Set(AlarmMode::Level2Off)),
    ("三级速度警报已关闭", AlarmReply::Set(AlarmMode::Level3Off)),
    ("本操作须要停车进行", AlarmReply::MustStop),
];
const IMU_OK: &str = "MPU6500初始化成功";
const IMU_FAILED: &str = "MPU6500初始化失败";
const BOOT_COUNT: &str = "启动次数:";

impl Reply {
    /// Interprets one text segment from the [`Assembler`](crate::Assembler).
    pub fn parse(bytes: &[u8]) -> Self {
        let text = decode_text(bytes);
        let line = text.trim();
        if let Some(name) = line.strip_prefix("NAME:") {
            return Self::Name(name.to_owned());
        }
        if line.len() > 2 && line.starts_with("GW") && line.bytes().all(|b| b.is_ascii_alphanumeric()) {
            return Self::Version(line.to_owned());
        }
        if let Some(&(_, reply)) = ALARM_MESSAGES.iter().find(|(message, _)| line.contains(message)) {
            return Self::SpeedAlarms(reply);
        }
        if line.contains(IMU_FAILED) {
            return Self::ImuInit { ok: false };
        }
        if line.contains(IMU_OK) {
            return Self::ImuInit { ok: true };
        }
        if let Some((_, rest)) = line.split_once(BOOT_COUNT) {
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            if let Ok(boot_count) = digits.parse() {
                return Self::InitDone { boot_count };
            }
        }
        Self::Text(text)
    }
}

/// Decodes wheel text: ASCII as is, Chinese messages from GBK, invalid bytes as U+FFFD.
pub fn decode_text(bytes: &[u8]) -> String {
    encoding_rs::GBK.decode_without_bom_handling(bytes).0.into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gbk(text: &str) -> Vec<u8> {
        encoding_rs::GBK.encode(text).0.into_owned()
    }

    #[test]
    fn banner_and_name() {
        assert_eq!(Reply::parse(b"GW1825020"), Reply::Version("GW1825020".into()));
        assert_eq!(Reply::parse(b"NAME:EXTREME\r\n"), Reply::Name("EXTREME".into()));
        assert_eq!(Reply::parse(b"GW"), Reply::Text("GW".into()));
    }

    #[test]
    fn speed_alarm_replies() {
        let reply = |text: &str| Reply::parse(&gbk(text));
        assert_eq!(reply("速度警报已打开.....\r\n"), Reply::SpeedAlarms(AlarmReply::Set(AlarmMode::AllOn)));
        assert_eq!(reply("二级速度警报已关闭.....\r\n"), Reply::SpeedAlarms(AlarmReply::Set(AlarmMode::Level2Off)));
        assert_eq!(reply("三级速度警报已关闭.....\r\n"), Reply::SpeedAlarms(AlarmReply::Set(AlarmMode::Level3Off)));
        assert_eq!(reply("本操作须要停车进行.....\r\n"), Reply::SpeedAlarms(AlarmReply::MustStop));
    }

    #[test]
    fn power_on_log() {
        assert_eq!(Reply::parse(&gbk(" MPU6500初始化成功 ")), Reply::ImuInit { ok: true });
        assert_eq!(Reply::parse(&gbk(" MPU6500初始化失败")), Reply::ImuInit { ok: false });
        assert_eq!(Reply::parse(&gbk("初始化完成..启动次数:12次\r\n")), Reply::InitDone { boot_count: 12 });
    }

    #[test]
    fn other_text_is_decoded() {
        assert_eq!(Reply::parse(b" AZ =  8192\r\n"), Reply::Text(" AZ =  8192\r\n".into()));
        assert_eq!(decode_text(&[0x41, 0xFF, 0x42]), "A\u{FFFD}B");
    }
}

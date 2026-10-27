//! A wheel's BLE module as data: what it advertises, its GATT table, and how it turns the
//! wheel's output into notifications. Each wheel profile has one, `profiles/<wheel>/ble.toml`.

use serde::Deserialize;

/// The schema version of `ble.toml` this build reads.
pub const SCHEMA: u32 = 1;

/// Longest local name that fits an advertisement next to a 16-bit service UUID. CoreBluetooth
/// shortens longer names.
pub const MAX_NAME: usize = 29;

/// The Bluetooth base UUID that 16- and 32-bit UUIDs abbreviate.
const BASE_TAIL: &str = "-0000-1000-8000-00805F9B34FB";

/// A characteristic of a [`BleProfile`]: its place in [`BleProfile::characteristics`] (services
/// in file order, each service's characteristics in file order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CharId(pub usize);

/// What a wheel's BLE module advertises and serves, and how it sends.
#[derive(Debug, Clone, PartialEq)]
pub struct BleProfile {
    /// The protocol of the bytes on the link (`begode`): which protocol crate finds text in the
    /// stream and answers writes in a replay. The radio itself never looks inside the bytes.
    pub protocol: String,
    /// The advertisement.
    pub advertising: Advertising,
    /// The GATT table.
    pub services: Vec<Service>,
    /// What sits between the wheel's electronics and the radio.
    pub module: Module,
}

/// What the module advertises.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Advertising {
    /// Local name the simulator advertises (`--name` overrides it).
    pub name: String,
    /// What real wheels of the model advertise, `*` for any text (`GotWay_*`). Documentation and
    /// matching; never advertised.
    pub name_pattern: Option<String>,
    /// Service UUIDs in the advertisement.
    pub services: Vec<String>,
    /// Manufacturer data (company id first, little-endian); empty: none.
    pub manufacturer: Vec<u8>,
    /// Advertised TX power, dBm.
    pub tx_power: Option<i8>,
}

/// One GATT service.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Service {
    /// UUID: 16-bit (`FFE0`), 32-bit or 128-bit (8-4-4-4-12).
    pub uuid: String,
    /// Primary (the usual) or secondary.
    pub primary: bool,
    /// Its characteristics.
    pub characteristics: Vec<Characteristic>,
}

/// One characteristic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Characteristic {
    /// UUID.
    pub uuid: String,
    /// GATT properties.
    pub properties: Properties,
    /// The wheel's output goes out on it, as notifications (or indications).
    pub tx: bool,
    /// What a central writes on it goes into the wheel.
    pub rx: bool,
    /// A fixed value served for reads (a model string, say); such a characteristic is read only.
    pub value: Option<Vec<u8>>,
}

/// GATT characteristic properties a peripheral can serve.
// Independent flags, one per property, as the Bluetooth specification lists them.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Properties {
    /// `read`.
    pub read: bool,
    /// `write`: a write request, answered.
    pub write: bool,
    /// `write-without-response`: a write command, not answered.
    pub write_without_response: bool,
    /// `notify`.
    pub notify: bool,
    /// `indicate`: a notification the central confirms.
    pub indicate: bool,
}

/// What sits between the wheel's electronics and the radio.
#[derive(Debug, Clone, PartialEq)]
pub enum Module {
    /// An HM-10 style transparent bridge on a controller UART (Begode): the UART's output goes out
    /// on the one `tx` characteristic, writes on `rx` characteristics go into the UART. See
    /// [`UartBridge`](crate::UartBridge).
    UartBridge {
        /// Largest notification: the UART stream is packed into notifications this long...
        chunk: usize,
        /// ...and a shorter one goes out once the line has been idle this long, seconds (a
        /// lone 24-byte frame goes out as 20 + 4).
        idle_s: f64,
        /// Shortest time between two notifications, seconds; 0: back to back.
        interval_s: f64,
    },
    /// A BLE chip with firmware of its own, a node of the simulated wheel: it sends and takes
    /// notifications and writes per characteristic itself, so there is nothing to model here.
    Node {
        /// Largest notification it sends.
        chunk: usize,
    },
}

/// A `ble.toml` the radio cannot serve.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ProfileError {
    /// Not valid TOML for the schema.
    #[error("ble.toml: {0}")]
    File(String),
    /// Something in it is wrong.
    #[error("ble.toml: {0}")]
    Invalid(String),
}

impl BleProfile {
    /// Reads and checks a wheel profile's `ble.toml`.
    pub fn from_toml(text: &str) -> Result<Self, ProfileError> {
        let file: File = toml::from_str(text).map_err(|e| ProfileError::File(e.to_string()))?;
        if file.schema != SCHEMA {
            return Err(ProfileError::File(format!("schema {} is not {SCHEMA}", file.schema)));
        }
        let profile = file.into_profile()?;
        profile.validate()?;
        Ok(profile)
    }

    /// The common HM-10 style module with no known protocol: service `FFE0`, one characteristic
    /// `FFE1` both ways, 20-byte notifications.
    pub fn hm10(name: &str) -> Self {
        Self {
            protocol: String::new(),
            advertising: Advertising {
                name: name.to_owned(),
                name_pattern: None,
                services: vec!["FFE0".into()],
                manufacturer: Vec::new(),
                tx_power: None,
            },
            services: vec![Service {
                uuid: "FFE0".into(),
                primary: true,
                characteristics: vec![Characteristic {
                    uuid: "FFE1".into(),
                    properties: Properties {
                        notify: true,
                        write: true,
                        write_without_response: true,
                        ..Properties::default()
                    },
                    tx: true,
                    rx: true,
                    value: None,
                }],
            }],
            module: Module::UartBridge { chunk: 20, idle_s: 0.001, interval_s: 0.0 },
        }
    }

    /// Every characteristic with its service, in [`CharId`] order.
    pub fn characteristics(&self) -> impl Iterator<Item = (CharId, &Service, &Characteristic)> {
        self.services
            .iter()
            .flat_map(|service| service.characteristics.iter().map(move |c| (service, c)))
            .enumerate()
            .map(|(index, (service, characteristic))| (CharId(index), service, characteristic))
    }

    /// One characteristic.
    pub fn characteristic(&self, id: CharId) -> Option<&Characteristic> {
        self.characteristics().nth(id.0).map(|(_, _, c)| c)
    }

    /// Its UUID as the profile writes it, or `?`.
    pub fn uuid(&self, id: CharId) -> &str {
        self.characteristic(id).map_or("?", |c| c.uuid.as_str())
    }

    /// The characteristic with this UUID (any form: `ffe1`, `0000FFE1-0000-1000-8000-00805F9B34FB`).
    pub fn find(&self, uuid: &str) -> Option<CharId> {
        let wanted = full_uuid(uuid)?;
        self.characteristics().find(|(_, _, c)| full_uuid(&c.uuid).as_deref() == Some(&wanted)).map(|(id, _, _)| id)
    }

    /// The first `tx` characteristic: where the wheel's output goes when nothing says otherwise.
    pub fn tx(&self) -> CharId {
        self.characteristics().find(|(_, _, c)| c.tx).map_or(CharId(0), |(id, _, _)| id)
    }

    /// The first `rx` characteristic: where a TCP bridge client's writes go.
    pub fn rx(&self) -> CharId {
        self.characteristics().find(|(_, _, c)| c.rx).map_or(CharId(0), |(id, _, _)| id)
    }

    /// The largest notification the module sends.
    pub const fn chunk(&self) -> usize {
        match self.module {
            Module::UartBridge { chunk, .. } | Module::Node { chunk } => chunk,
        }
    }

    /// Checks what the radio relies on (CoreBluetooth raises an exception on a bad UUID).
    pub fn validate(&self) -> Result<(), ProfileError> {
        let fail = |text: String| Err(ProfileError::Invalid(text));
        if self.protocol.bytes().any(|b| !(b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')) {
            return fail(format!("protocol \"{}\" is not a lowercase name", self.protocol));
        }
        let name = &self.advertising.name;
        if name.is_empty() || name.len() > MAX_NAME || name.chars().any(char::is_control) {
            return fail(format!("advertised name must be 1 to {MAX_NAME} bytes without control characters"));
        }
        for uuid in &self.advertising.services {
            check_uuid("advertised service", uuid)?;
        }
        if self.services.is_empty() {
            return fail("no service".into());
        }
        for service in &self.services {
            check_uuid("service", &service.uuid)?;
            if service.characteristics.is_empty() {
                return fail(format!("service {} has no characteristic", service.uuid));
            }
            for (index, c) in service.characteristics.iter().enumerate() {
                check_uuid("characteristic", &c.uuid)?;
                if service.characteristics[..index].iter().any(|other| full_uuid(&other.uuid) == full_uuid(&c.uuid)) {
                    return fail(format!("characteristic {} twice in service {}", c.uuid, service.uuid));
                }
                let p = c.properties;
                if c.value.is_some() && (p != Properties { read: true, ..Properties::default() } || c.tx || c.rx) {
                    return fail(format!("characteristic {} has a fixed value, so it must be read only", c.uuid));
                }
                if c.tx && !(p.notify || p.indicate) {
                    return fail(format!("tx characteristic {} needs notify or indicate", c.uuid));
                }
                if c.rx && !(p.write || p.write_without_response) {
                    return fail(format!("rx characteristic {} needs write or write-without-response", c.uuid));
                }
                if p == Properties::default() {
                    return fail(format!("characteristic {} has no property", c.uuid));
                }
            }
        }
        let tx = self.characteristics().filter(|(_, _, c)| c.tx).count();
        if tx == 0 || !self.characteristics().any(|(_, _, c)| c.rx) {
            return fail("the link needs a tx and an rx characteristic".into());
        }
        if !(1..=512).contains(&self.chunk()) {
            return fail(format!("notification size {} is out of range 1..=512", self.chunk()));
        }
        if let Module::UartBridge { idle_s, interval_s, .. } = self.module {
            if tx != 1 {
                return fail("a UART bridge sends on exactly one tx characteristic".into());
            }
            if !(idle_s > 0.0 && idle_s.is_finite() && interval_s >= 0.0 && interval_s.is_finite()) {
                return fail("idle_ms must be positive and interval_ms 0 or more".into());
            }
        }
        Ok(())
    }
}

fn check_uuid(what: &str, uuid: &str) -> Result<(), ProfileError> {
    match full_uuid(uuid) {
        Some(_) => Ok(()),
        None => Err(ProfileError::Invalid(format!(
            "{what} \"{uuid}\" is not a Bluetooth UUID (FFE0, or 8-4-4-4-12 hex digits)"
        ))),
    }
}

/// The 128-bit form of a UUID, upper case, or `None` when `uuid` is not one.
pub(crate) fn full_uuid(uuid: &str) -> Option<String> {
    let hex = |s: &str| s.bytes().all(|b| b.is_ascii_hexdigit());
    let upper = uuid.to_ascii_uppercase();
    match upper.len() {
        4 if hex(&upper) => Some(format!("0000{upper}{BASE_TAIL}")),
        8 if hex(&upper) => Some(format!("{upper}{BASE_TAIL}")),
        36 => {
            let groups: Vec<&str> = upper.split('-').collect();
            let lengths: Vec<usize> = groups.iter().map(|g| g.len()).collect();
            (lengths == [8, 4, 4, 4, 12] && groups.iter().all(|g| hex(g))).then_some(upper)
        }
        _ => None,
    }
}

/// Bytes from hex, either case, no separators.
pub(crate) fn parse_hex(text: &str) -> Option<Vec<u8>> {
    if !text.len().is_multiple_of(2) || !text.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    (0..text.len()).step_by(2).map(|i| u8::from_str_radix(&text[i..i + 2], 16).ok()).collect()
}

/// `ble.toml` as written.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    schema: u32,
    #[serde(default)]
    protocol: String,
    advertising: AdvertisingFile,
    #[serde(rename = "service")]
    services: Vec<ServiceFile>,
    module: ModuleFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AdvertisingFile {
    name: String,
    name_pattern: Option<String>,
    #[serde(default)]
    services: Vec<String>,
    #[serde(default)]
    manufacturer: String,
    tx_power: Option<i8>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ServiceFile {
    uuid: String,
    #[serde(default = "primary")]
    primary: bool,
    #[serde(rename = "characteristic")]
    characteristics: Vec<CharacteristicFile>,
}

const fn primary() -> bool {
    true
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CharacteristicFile {
    uuid: String,
    properties: Vec<String>,
    #[serde(default)]
    roles: Vec<String>,
    value: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
enum ModuleFile {
    UartBridge {
        chunk: usize,
        idle_ms: f64,
        #[serde(default)]
        interval_ms: f64,
    },
    Node {
        chunk: usize,
    },
}

impl File {
    fn into_profile(self) -> Result<BleProfile, ProfileError> {
        let invalid = ProfileError::Invalid;
        let advertising = Advertising {
            manufacturer: parse_hex(&self.advertising.manufacturer).ok_or_else(|| {
                invalid(format!("manufacturer \"{}\" is not hex bytes", self.advertising.manufacturer))
            })?,
            name: self.advertising.name,
            name_pattern: self.advertising.name_pattern,
            services: self.advertising.services,
            tx_power: self.advertising.tx_power,
        };
        let mut services = Vec::new();
        for service in self.services {
            let mut characteristics = Vec::new();
            for c in service.characteristics {
                let mut properties = Properties::default();
                for property in &c.properties {
                    let flag = match property.as_str() {
                        "read" => &mut properties.read,
                        "write" => &mut properties.write,
                        "write-without-response" => &mut properties.write_without_response,
                        "notify" => &mut properties.notify,
                        "indicate" => &mut properties.indicate,
                        other => {
                            return Err(invalid(format!("characteristic {}: unknown property \"{other}\"", c.uuid)));
                        }
                    };
                    *flag = true;
                }
                let (mut tx, mut rx) = (false, false);
                for role in &c.roles {
                    match role.as_str() {
                        "tx" => tx = true,
                        "rx" => rx = true,
                        other => return Err(invalid(format!("characteristic {}: unknown role \"{other}\"", c.uuid))),
                    }
                }
                let value = c
                    .value
                    .map(|hex| {
                        parse_hex(&hex).ok_or_else(|| invalid(format!("characteristic {}: value is not hex", c.uuid)))
                    })
                    .transpose()?;
                characteristics.push(Characteristic { uuid: c.uuid, properties, tx, rx, value });
            }
            services.push(Service { uuid: service.uuid, primary: service.primary, characteristics });
        }
        let module = match self.module {
            ModuleFile::UartBridge { chunk, idle_ms, interval_ms } => {
                Module::UartBridge { chunk, idle_s: idle_ms / 1000.0, interval_s: interval_ms / 1000.0 }
            }
            ModuleFile::Node { chunk } => Module::Node { chunk },
        };
        Ok(BleProfile { protocol: self.protocol, advertising, services, module })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NUS: &str = r#"
        schema = 1
        protocol = "example"
        [advertising]
        name = "EXAMPLE-1"
        services = ["6E400001-B5A3-F393-E0A9-E50E24DCCA9E"]
        manufacturer = "ffff0102"
        [[service]]
        uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
        [[service.characteristic]]
        uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
        properties = ["notify"]
        roles = ["tx"]
        [[service.characteristic]]
        uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
        properties = ["write", "write-without-response"]
        roles = ["rx"]
        [[service]]
        uuid = "180A"
        [[service.characteristic]]
        uuid = "2A24"
        properties = ["read"]
        value = "4558414d504c45"
        [module]
        kind = "node"
        chunk = 244
    "#;

    #[test]
    fn the_extreme_profile_is_an_hm10_style_bridge() {
        let profile = BleProfile::from_toml(include_str!("../../../profiles/begode-extreme/ble.toml")).unwrap();
        let mut expected = BleProfile::hm10("GotWay_EMU");
        expected.protocol = "begode".into();
        expected.advertising.name_pattern = Some("GotWay_*".into());
        assert_eq!(profile, expected);
        assert_eq!((profile.tx(), profile.rx(), profile.chunk()), (CharId(0), CharId(0), 20));
    }

    #[test]
    fn the_ffe0_brand_radios_are_the_same_bridge_under_their_own_names() {
        for (text, name, protocol, pattern) in [
            (include_str!("../../../profiles/kingsong/ble.toml"), "KS-EMU", "kingsong", "KS*"),
            (include_str!("../../../profiles/leaperkim/ble.toml"), "LK_EMU", "leaperkim", "LK*"),
            (include_str!("../../../profiles/ninebot-one/ble.toml"), "NOE_EMU", "ninebot-one", "NO[CEP]*"),
        ] {
            let profile = BleProfile::from_toml(text).unwrap();
            let mut expected = BleProfile::hm10(name);
            expected.protocol = protocol.into();
            expected.advertising.name_pattern = Some(pattern.into());
            assert_eq!(profile, expected, "{protocol}");
        }
    }

    /// A brand radio that notifies on `tx` and takes writes on `rx` (with response, and also
    /// without when `without_response`), on a chip of its own.
    fn assert_split_radio(text: &str, name: &str, tx: &str, rx: &str, without_response: bool) {
        let profile = BleProfile::from_toml(text).unwrap();
        assert_eq!(profile.advertising.name, name);
        assert_eq!((profile.uuid(profile.tx()), profile.uuid(profile.rx())), (tx, rx), "{name}");
        let write = profile.characteristic(profile.rx()).unwrap().properties;
        assert!(write.write, "{name}: writes with response");
        assert_eq!(write.write_without_response, without_response, "{name}");
        assert_eq!(profile.module, Module::Node { chunk: 20 }, "{name}");
    }

    #[test]
    fn the_split_brand_radios_notify_on_one_characteristic_and_take_writes_on_another() {
        // InMotion V1: notifications in one service (FFE0), writes in another (FFE5).
        assert_split_radio(include_str!("../../../profiles/inmotion-v1/ble.toml"), "V10F-EMU", "FFE4", "FFE9", false);
        assert_split_radio(
            include_str!("../../../profiles/inmotion-v2/ble.toml"),
            "V11-EMU",
            "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
            "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
            false,
        );
        assert_split_radio(
            include_str!("../../../profiles/vesc/ble.toml"),
            "VESC_EMU",
            "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
            "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
            true,
        );
    }

    #[test]
    fn a_nordic_uart_layout_has_split_characteristics_and_more_services() {
        let profile = BleProfile::from_toml(NUS).unwrap();
        assert_eq!(profile.characteristics().count(), 3);
        assert_eq!((profile.tx(), profile.rx()), (CharId(0), CharId(1)));
        assert_eq!(profile.find("6e400002-b5a3-f393-e0a9-e50e24dcca9e"), Some(CharId(1)));
        assert_eq!(profile.find("2a24"), Some(CharId(2)));
        assert_eq!(profile.find("FFE1"), None);
        assert_eq!(profile.uuid(CharId(2)), "2A24");
        assert_eq!(profile.characteristic(CharId(2)).unwrap().value.as_deref(), Some(&b"EXAMPLE"[..]));
        assert_eq!(profile.advertising.manufacturer, [0xFF, 0xFF, 1, 2]);
        assert_eq!(profile.module, Module::Node { chunk: 244 });
    }

    #[test]
    fn short_and_long_forms_are_the_same_uuid() {
        assert_eq!(full_uuid("ffe1"), full_uuid("0000FFE1-0000-1000-8000-00805f9b34fb"));
        assert_eq!(full_uuid("0000ffe1").as_deref(), Some("0000FFE1-0000-1000-8000-00805F9B34FB"));
        for bad in
            ["FFE", "FFEG", "", "6E400001B5A3F393E0A9E50E24DCCA9E", "6E400001-B5A3-F393-E0A9-E50E24DCCA9", "+FFE"]
        {
            assert_eq!(full_uuid(bad), None, "{bad}");
        }
    }

    #[test]
    fn mistakes_are_named() {
        let error = |from: &str, to: &str| match BleProfile::from_toml(&NUS.replacen(from, to, 1)) {
            Err(ProfileError::Invalid(text) | ProfileError::File(text)) => text,
            Ok(_) => panic!("{from} -> {to} was accepted"),
        };
        assert!(error("\"180A\"", "\"180\"").contains("is not a Bluetooth UUID"));
        assert!(error("[\"notify\"]", "[\"notify\", \"sing\"]").contains("unknown property"));
        assert!(error("[\"notify\"]", "[\"read\"]").contains("needs notify or indicate"));
        assert!(error("roles = [\"rx\"]", "roles = []").contains("the link needs a tx and an rx"));
        assert!(error("\"ffff0102\"", "\"ffff010\"").contains("is not hex bytes"));
        assert!(error("name = \"EXAMPLE-1\"", "name = \"\"").contains("advertised name"));
        assert!(error("chunk = 244", "chunk = 0").contains("out of range"));
        assert!(error("schema = 1", "schema = 2").contains("schema 2"));
        assert!(error("kind = \"node\"", "kind = \"modem\"").contains("modem"));
        assert!(error("[module]", "colour = 1\n[module]").contains("colour"), "unknown keys are refused");
        let read_only = error("properties = [\"read\"]", "properties = [\"read\", \"notify\"]");
        assert!(read_only.contains("read only"), "{read_only}");
    }

    #[test]
    fn a_uart_bridge_has_one_tx_characteristic() {
        let two_tx = NUS
            .replace(
                "kind = \"node\"\n        chunk = 244",
                "kind = \"uart-bridge\"\n        chunk = 20\n        idle_ms = 1",
            )
            .replace("properties = [\"write\", \"write-without-response\"]", "properties = [\"write\", \"notify\"]")
            .replace("roles = [\"rx\"]", "roles = [\"rx\", \"tx\"]");
        let Err(ProfileError::Invalid(text)) = BleProfile::from_toml(&two_tx) else { panic!("accepted") };
        assert!(text.contains("exactly one tx"), "{text}");
    }
}

//! The radio on macOS: a CoreBluetooth peripheral manager on its own serial dispatch queue.
//!
//! Every CoreBluetooth call happens on that queue: the delegate's callbacks run there, and the
//! [`Radio`] handle sends its work there with `exec_async`. The delegate's state (its ivars) is
//! therefore only ever touched by one thread at a time, which is what makes [`OnQueue`] sound.

// objc2's CoreBluetooth bindings are unsafe to call. Every call here passes objects this module
// created (or that CoreBluetooth handed to the delegate), on the queue CoreBluetooth was given.
#![allow(unsafe_code)]

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, Sender};

use dispatch2::{DispatchQueue, DispatchRetained};
use objc2::rc::Retained;
use objc2::runtime::{AnyObject, NSObject, NSObjectProtocol, ProtocolObject};
use objc2::{AnyThread, DefinedClass, define_class, msg_send};
use objc2_core_bluetooth::{
    CBATTError, CBATTRequest, CBAdvertisementDataLocalNameKey, CBAdvertisementDataServiceUUIDsKey,
    CBAttributePermissions, CBCentral, CBCharacteristic, CBCharacteristicProperties, CBManager, CBManagerAuthorization,
    CBManagerState, CBMutableCharacteristic, CBMutableService, CBPeripheralManager, CBPeripheralManagerDelegate,
    CBService, CBUUID,
};
use objc2_foundation::{NSArray, NSData, NSDictionary, NSError, NSString};

use super::{RadioError, RadioEvent};
use crate::profile::full_uuid;
use crate::{BleProfile, CharId, Properties};

/// Most notifications that wait for the radio (about ten seconds of Begode telemetry); beyond
/// that the oldest are dropped, as a module with a full buffer would.
const BACKLOG: usize = 256;

/// The dispatch queue's label, as Console and crash reports show it.
const QUEUE_LABEL: &str = "com.tarikbc.euc-studio.ble-radio";

/// A value moved to the radio queue. Sound because it is only used inside work items that run on
/// that one serial queue, never on two threads at once; objc2's reference counts are atomic, so
/// cloning or dropping the `Retained`s elsewhere is fine too.
struct OnQueue<T>(T);

// SAFETY: see the type's documentation.
unsafe impl<T> Send for OnQueue<T> {}

impl<T> OnQueue<T> {
    /// The value. A method, so a closure captures the whole wrapper, not its fields.
    fn into_inner(self) -> T {
        self.0
    }
}

/// The delegate's state, touched only on the radio queue.
struct Ivars {
    profile: BleProfile,
    /// Full service and characteristic UUIDs of every characteristic, in [`CharId`] order.
    keys: Vec<(String, String)>,
    events: Sender<RadioEvent>,
    /// Set on the queue right after the manager is created, before any callback runs.
    manager: RefCell<Option<Retained<CBPeripheralManager>>>,
    /// The published characteristics, in [`CharId`] order.
    characteristics: RefCell<Vec<Retained<CBMutableCharacteristic>>>,
    /// The last value sent on each characteristic, for reads.
    values: RefCell<Vec<Vec<u8>>>,
    /// Subscriptions: central, characteristic, the central's largest notification.
    subscriptions: RefCell<Vec<(String, CharId, usize)>>,
    /// Notifications waiting for the radio.
    backlog: RefCell<VecDeque<(CharId, Vec<u8>)>>,
    /// The wheel is on: serve and advertise whenever Bluetooth is.
    on: Cell<bool>,
    /// Bluetooth is powered on.
    powered: Cell<bool>,
    /// `updateValue` found the transmit queue full; wait for `peripheralManagerIsReadyToUpdateSubscribers`.
    busy: Cell<bool>,
}

define_class!(
    // SAFETY: NSObject has no subclassing requirements, and Delegate does not implement Drop.
    #[unsafe(super(NSObject))]
    #[name = "EucStudioBleRadioDelegate"]
    #[ivars = Ivars]
    struct Delegate;

    unsafe impl NSObjectProtocol for Delegate {}

    unsafe impl CBPeripheralManagerDelegate for Delegate {
        #[unsafe(method(peripheralManagerDidUpdateState:))]
        fn did_update_state(&self, manager: &CBPeripheralManager) {
            let state = unsafe { manager.state() };
            self.send(RadioEvent::Bluetooth(state_name(state)));
            let powered = state == CBManagerState::PoweredOn;
            self.ivars().powered.set(powered);
            if powered {
                if self.ivars().on.get() {
                    self.publish(manager);
                }
            } else {
                // Below poweredOn every connection is gone; below poweredOff the database is too.
                self.forget_centrals();
            }
        }

        #[unsafe(method(peripheralManager:didAddService:error:))]
        fn did_add_service(&self, manager: &CBPeripheralManager, service: &CBService, error: Option<&NSError>) {
            if let Some(error) = error {
                self.send(RadioEvent::Error(format!("cannot add a service: {}", describe(error))));
                return;
            }
            // Advertise once the last service of the table is in.
            let last = self.ivars().profile.services.last().and_then(|s| full_uuid(&s.uuid));
            if self.ivars().on.get() && last == full_uuid(&uuid_text(&*unsafe { service.UUID() })) {
                self.advertise(manager);
            }
        }

        #[unsafe(method(peripheralManagerDidStartAdvertising:error:))]
        fn did_start_advertising(&self, _manager: &CBPeripheralManager, error: Option<&NSError>) {
            self.send(match error {
                Some(error) => RadioEvent::Error(format!("cannot advertise: {}", describe(error))),
                None => RadioEvent::Advertising,
            });
        }

        #[unsafe(method(peripheralManager:central:didSubscribeToCharacteristic:))]
        fn did_subscribe(
            &self,
            _manager: &CBPeripheralManager,
            central: &CBCentral,
            characteristic: &CBCharacteristic,
        ) {
            let Some(id) = self.find(characteristic) else { return };
            let central_id = identifier(central);
            let max = unsafe { central.maximumUpdateValueLength() };
            {
                let mut subscriptions = self.ivars().subscriptions.borrow_mut();
                subscriptions.retain(|(c, char_id, _)| !(*c == central_id && *char_id == id));
                subscriptions.push((central_id.clone(), id, max));
            }
            self.send(RadioEvent::Subscribed { central: central_id, characteristic: id, max });
        }

        #[unsafe(method(peripheralManager:central:didUnsubscribeFromCharacteristic:))]
        fn did_unsubscribe(
            &self,
            _manager: &CBPeripheralManager,
            central: &CBCentral,
            characteristic: &CBCharacteristic,
        ) {
            let Some(id) = self.find(characteristic) else { return };
            let central_id = identifier(central);
            let nobody = {
                let mut subscriptions = self.ivars().subscriptions.borrow_mut();
                subscriptions.retain(|(c, char_id, _)| !(*c == central_id && *char_id == id));
                !subscriptions.iter().any(|(_, char_id, _)| *char_id == id)
            };
            if nobody {
                self.ivars().backlog.borrow_mut().retain(|(char_id, _)| *char_id != id);
            }
            self.send(RadioEvent::Unsubscribed { central: central_id, characteristic: id });
        }

        #[unsafe(method(peripheralManager:didReceiveReadRequest:))]
        fn did_receive_read(&self, manager: &CBPeripheralManager, request: &CBATTRequest) {
            // Characteristics with a fixed value are answered by CoreBluetooth itself; the others
            // answer with the last value sent on them.
            let id = self.find(&*unsafe { request.characteristic() });
            let readable = id.and_then(|id| self.ivars().profile.characteristic(id)).is_some_and(|c| c.properties.read);
            let result = match id.filter(|_| readable) {
                None => CBATTError::ReadNotPermitted,
                Some(id) => {
                    let values = self.ivars().values.borrow();
                    let value = values.get(id.0).map_or(&[][..], Vec::as_slice);
                    match value.get(unsafe { request.offset() }..) {
                        Some(rest) => {
                            unsafe { request.setValue(Some(&NSData::with_bytes(rest))) };
                            CBATTError::Success
                        }
                        None => CBATTError::InvalidOffset,
                    }
                }
            };
            unsafe { manager.respondToRequest_withResult(request, result) };
        }

        #[unsafe(method(peripheralManager:didReceiveWriteRequests:))]
        fn did_receive_writes(&self, manager: &CBPeripheralManager, requests: &NSArray<CBATTRequest>) {
            // Requests come in the order the central wrote them; every byte goes on, in order.
            let requests = requests.to_vec();
            for request in &requests {
                let central = identifier(&*unsafe { request.central() });
                let bytes = unsafe { request.value() }.map(|data| data.to_vec()).unwrap_or_default();
                match self.find(&*unsafe { request.characteristic() }) {
                    Some(characteristic) if !bytes.is_empty() => {
                        self.send(RadioEvent::Write { central, characteristic, bytes });
                    }
                    Some(_) => {}
                    None => {
                        self.send(RadioEvent::Error("a write on a characteristic the profile does not have".into()));
                    }
                }
            }
            // One answer for the whole batch, to its first request (CoreBluetooth's rule).
            if let Some(first) = requests.first() {
                unsafe { manager.respondToRequest_withResult(first, CBATTError::Success) };
            }
        }

        #[unsafe(method(peripheralManagerIsReadyToUpdateSubscribers:))]
        fn ready_to_update(&self, manager: &CBPeripheralManager) {
            self.ivars().busy.set(false);
            self.pump(manager);
        }
    }
);

impl Delegate {
    fn new(profile: BleProfile, events: Sender<RadioEvent>) -> Retained<Self> {
        let keys = profile
            .characteristics()
            .map(|(_, service, c)| {
                (full_uuid(&service.uuid).unwrap_or_default(), full_uuid(&c.uuid).unwrap_or_default())
            })
            .collect::<Vec<_>>();
        let count = keys.len();
        let this = Self::alloc().set_ivars(Ivars {
            profile,
            keys,
            events,
            manager: RefCell::new(None),
            characteristics: RefCell::new(Vec::new()),
            values: RefCell::new(vec![Vec::new(); count]),
            subscriptions: RefCell::new(Vec::new()),
            backlog: RefCell::new(VecDeque::new()),
            on: Cell::new(true),
            powered: Cell::new(false),
            busy: Cell::new(false),
        });
        unsafe { msg_send![super(this), init] }
    }

    fn send(&self, event: RadioEvent) {
        // The receiver is gone only while the program ends.
        let _ = self.ivars().events.send(event);
    }

    fn manager(&self) -> Option<Retained<CBPeripheralManager>> {
        self.ivars().manager.borrow().clone()
    }

    /// The profile's characteristic a CoreBluetooth one stands for, by service and characteristic UUID.
    fn find(&self, characteristic: &CBCharacteristic) -> Option<CharId> {
        let uuid = full_uuid(&uuid_text(&*unsafe { characteristic.UUID() }))?;
        let service = unsafe { characteristic.service() }.and_then(|s| full_uuid(&uuid_text(&*unsafe { s.UUID() })));
        let keys = &self.ivars().keys;
        keys.iter().position(|(s, c)| *c == uuid && service.as_ref().is_none_or(|service| service == s)).map(CharId)
    }

    /// Builds the GATT table from the profile and adds it; advertising starts once it is in.
    fn publish(&self, manager: &CBPeripheralManager) {
        let profile = &self.ivars().profile;
        let mut published = Vec::new();
        let mut services = Vec::new();
        for service in &profile.services {
            let mut characteristics: Vec<Retained<CBCharacteristic>> = Vec::new();
            for c in &service.characteristics {
                let built = characteristic(&c.uuid, c.properties, c.value.as_deref());
                characteristics.push(Retained::into_super(built.clone()));
                published.push(built);
            }
            let built = unsafe {
                CBMutableService::initWithType_primary(CBMutableService::alloc(), &uuid(&service.uuid), service.primary)
            };
            unsafe { built.setCharacteristics(Some(&NSArray::from_retained_slice(&characteristics))) };
            services.push(built);
        }
        *self.ivars().characteristics.borrow_mut() = published;
        unsafe { manager.removeAllServices() };
        for service in &services {
            unsafe { manager.addService(service) };
        }
    }

    /// Advertises the name and the service UUIDs (all CoreBluetooth lets a peripheral advertise).
    fn advertise(&self, manager: &CBPeripheralManager) {
        let advertising = &self.ivars().profile.advertising;
        let name = NSString::from_str(&advertising.name);
        let uuids: Vec<Retained<CBUUID>> = advertising.services.iter().map(|s| uuid(s)).collect();
        let services = NSArray::from_retained_slice(&uuids);
        let keys: [&NSString; 2] = unsafe { [CBAdvertisementDataLocalNameKey, CBAdvertisementDataServiceUUIDsKey] };
        let values: [&AnyObject; 2] = [&name, &services];
        let data = NSDictionary::from_slices(&keys, &values);
        unsafe { manager.startAdvertising(Some(&data)) };
    }

    /// The wheel's module powers on or off.
    fn switch(&self, on: bool) {
        self.ivars().on.set(on);
        let Some(manager) = self.manager() else { return };
        if !self.ivars().powered.get() {
            return;
        }
        if on {
            self.publish(&manager);
        } else {
            unsafe {
                manager.stopAdvertising();
                manager.removeAllServices();
            }
            // Removing the services ends every subscription; CoreBluetooth may not say so.
            self.forget_centrals();
        }
    }

    fn forget_centrals(&self) {
        self.ivars().subscriptions.borrow_mut().clear();
        self.ivars().backlog.borrow_mut().clear();
        self.ivars().busy.set(false);
    }

    /// Queues one notification for the centrals subscribed to `id`, cut to the smallest limit.
    fn enqueue(&self, id: CharId, bytes: &[u8]) {
        let ivars = self.ivars();
        if let Some(value) = ivars.values.borrow_mut().get_mut(id.0) {
            value.clear();
            value.extend_from_slice(bytes);
        }
        let max = ivars.subscriptions.borrow().iter().filter(|(_, c, _)| *c == id).map(|&(_, _, max)| max).min();
        let Some(max) = max else { return };
        if !ivars.on.get() || !ivars.powered.get() {
            return;
        }
        let dropped = {
            let mut backlog = ivars.backlog.borrow_mut();
            backlog.extend(crate::split(bytes, max).map(|part| (id, part.to_vec())));
            let excess = backlog.len().saturating_sub(BACKLOG);
            backlog.drain(..excess);
            excess
        };
        if dropped > 0 {
            self.send(RadioEvent::Dropped { count: dropped });
        }
        if let Some(manager) = self.manager() {
            self.pump(&manager);
        }
    }

    /// Sends waiting notifications until the radio's transmit queue is full.
    fn pump(&self, manager: &CBPeripheralManager) {
        let ivars = self.ivars();
        while !ivars.busy.get() {
            let Some((id, bytes)) = ivars.backlog.borrow_mut().pop_front() else { break };
            let Some(characteristic) = ivars.characteristics.borrow().get(id.0).cloned() else { continue };
            let data = NSData::with_bytes(&bytes);
            let sent =
                unsafe { manager.updateValue_forCharacteristic_onSubscribedCentrals(&data, &characteristic, None) };
            if !sent {
                let queued = {
                    let mut backlog = ivars.backlog.borrow_mut();
                    backlog.push_front((id, bytes));
                    backlog.len()
                };
                ivars.busy.set(true);
                self.send(RadioEvent::Busy { queued });
            }
        }
    }

    /// Stops advertising and serving; the manager lets go of the delegate.
    fn shut_down(&self) {
        if let Some(manager) = self.ivars().manager.borrow_mut().take() {
            unsafe {
                manager.stopAdvertising();
                manager.removeAllServices();
                manager.setDelegate(None);
            }
        }
    }
}

pub(super) fn authorization() -> &'static str {
    match unsafe { CBManager::authorization_class() } {
        CBManagerAuthorization::AllowedAlways => "allowed",
        CBManagerAuthorization::Denied => "denied",
        CBManagerAuthorization::Restricted => "restricted",
        _ => "notDetermined",
    }
}

/// The radio handle: lives on the caller's thread, sends its work to the radio queue.
pub(super) struct Radio {
    queue: DispatchRetained<DispatchQueue>,
    delegate: OnQueue<Retained<Delegate>>,
}

impl Radio {
    pub(super) fn start(profile: &BleProfile) -> Result<(Self, Receiver<RadioEvent>), RadioError> {
        if matches!(authorization(), "denied" | "restricted") {
            return Err(RadioError::Denied);
        }
        let (events, receiver) = mpsc::channel();
        let advertising = &profile.advertising;
        if !advertising.manufacturer.is_empty() || advertising.tx_power.is_some() {
            let _ = events.send(RadioEvent::Error(
                "CoreBluetooth advertises only the name and service UUIDs; manufacturer data and TX power left out"
                    .into(),
            ));
        }
        let queue = DispatchQueue::new(QUEUE_LABEL, None);
        let delegate = Delegate::new(profile.clone(), events);
        let work = OnQueue((delegate.clone(), queue.clone()));
        // Created on the queue, so the delegate holds the manager before its first callback runs.
        queue.exec_sync(move || {
            let (delegate, queue) = work.into_inner();
            let manager = unsafe {
                CBPeripheralManager::initWithDelegate_queue_options(
                    CBPeripheralManager::alloc(),
                    Some(ProtocolObject::from_ref(&*delegate)),
                    Some(&queue),
                    None,
                )
            };
            *delegate.ivars().manager.borrow_mut() = Some(manager);
        });
        Ok((Self { queue, delegate: OnQueue(delegate) }, receiver))
    }

    fn on_queue(&self, work: impl FnOnce(&Delegate) + 'static) {
        let work = OnQueue((self.delegate.0.clone(), work));
        self.queue.exec_async(move || {
            let (delegate, work) = work.into_inner();
            work(&delegate);
        });
    }

    pub(super) fn notify(&self, characteristic: CharId, bytes: &[u8]) {
        let bytes = bytes.to_vec();
        self.on_queue(move |delegate| delegate.enqueue(characteristic, &bytes));
    }

    pub(super) fn set_on(&self, on: bool) {
        self.on_queue(move |delegate| delegate.switch(on));
    }
}

impl Drop for Radio {
    fn drop(&mut self) {
        let delegate = OnQueue(self.delegate.0.clone());
        self.queue.exec_sync(move || delegate.into_inner().shut_down());
    }
}

fn uuid(text: &str) -> Retained<CBUUID> {
    // `BleProfile::validate` checked the syntax: CBUUID raises an exception on anything else.
    unsafe { CBUUID::UUIDWithString(&NSString::from_str(text)) }
}

fn uuid_text(uuid: &CBUUID) -> String {
    unsafe { uuid.UUIDString() }.to_string()
}

fn characteristic(uuid_text: &str, p: Properties, value: Option<&[u8]>) -> Retained<CBMutableCharacteristic> {
    let mut properties = CBCharacteristicProperties::empty();
    let mut permissions = CBAttributePermissions::empty();
    for (on, property) in [
        (p.read, CBCharacteristicProperties::Read),
        (p.write, CBCharacteristicProperties::Write),
        (p.write_without_response, CBCharacteristicProperties::WriteWithoutResponse),
        (p.notify, CBCharacteristicProperties::Notify),
        (p.indicate, CBCharacteristicProperties::Indicate),
    ] {
        if on {
            properties |= property;
        }
    }
    if p.read {
        permissions |= CBAttributePermissions::Readable;
    }
    if p.write || p.write_without_response {
        permissions |= CBAttributePermissions::Writeable;
    }
    // A fixed value is cached and served by CoreBluetooth; without one the value is dynamic, so
    // reads and writes reach the delegate.
    let value = value.map(NSData::with_bytes);
    unsafe {
        CBMutableCharacteristic::initWithType_properties_value_permissions(
            CBMutableCharacteristic::alloc(),
            &uuid(uuid_text),
            properties,
            value.as_deref(),
            permissions,
        )
    }
}

fn identifier(central: &CBCentral) -> String {
    unsafe { central.identifier() }.UUIDString().to_string()
}

fn describe(error: &NSError) -> String {
    error.localizedDescription().to_string()
}

fn state_name(state: CBManagerState) -> &'static str {
    match state {
        CBManagerState::Resetting => "resetting",
        CBManagerState::Unsupported => "unsupported",
        CBManagerState::Unauthorized => "unauthorized",
        CBManagerState::PoweredOff => "poweredOff",
        CBManagerState::PoweredOn => "poweredOn",
        _ => "unknown",
    }
}

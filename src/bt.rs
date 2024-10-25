use std::sync::{Arc, Condvar, Mutex};
use enumset::enum_set;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::bt::ble::gap::{AdvConfiguration, BleGapEvent, EspBleGap};
use esp_idf_svc::bt::ble::gatt::server::{ConnectionId, EspGatts, GattsEvent, TransferId};
use esp_idf_svc::bt::ble::gatt::{
    AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface, GattResponse,
    GattServiceId, GattStatus, Handle, Permission, Property,
};
use esp_idf_svc::bt::{BdAddr, Ble, BtDriver, BtStatus, BtUuid};
use esp_idf_svc::sys::{EspError, ESP_FAIL};
use queues::{IsQueue, Queue};
use log::{info, warn};

const APP_ID: u16 = 0;
const MAX_CONNECTIONS: usize = 2;

// UUIDs for the service and characteristics
pub const SERVICE_UUID: u128 = 0xad91b201734740479e173bed82d75f9d;
pub const RECV_CHARACTERISTIC_UUID: u128 = 0xb6fccb5087be44f3ae22f85485ea42c4;
pub const IND_CHARACTERISTIC_UUID: u128 = 0x503de214868246c4828fd59144da41be;
pub const HOST_NAME: &str = "5P00K-800";

type ExBtDriver = BtDriver<'static, Ble>;
type ExEspBleGap = Arc<EspBleGap<'static, Ble, Arc<ExBtDriver>>>;
type ExEspGatts = Arc<EspGatts<'static, Ble, Arc<ExBtDriver>>>;

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
    subscribed: bool,
    mtu: Option<u16>,
}

#[derive(Default)]
struct State {
    gatt_if: Option<GattInterface>,
    service_handle: Option<Handle>,
    recv_handle: Option<Handle>,
    ind_handle: Option<Handle>,
    ind_cccd_handle: Option<Handle>,
    connections: heapless::Vec<Connection, MAX_CONNECTIONS>,
    response: GattResponse,
    ind_confirmed: Option<BdAddr>,
}

#[derive(Clone)]
pub struct BtMessage {
    pub peer: BdAddr,
    pub data: Vec<u8>
}

#[derive(Clone)]
pub struct BtServer {
    gap: ExEspBleGap,
    gatts: ExEspGatts,
    state: Arc<Mutex<State>>,
    condvar: Arc<Condvar>,
    messages: Arc<Mutex<Queue<BtMessage>>>
}

impl BtServer {
    pub fn new(gap: ExEspBleGap, gatts: ExEspGatts) -> Self {
        Self {
            gap,
            gatts,
            state: Arc::new(Mutex::new(Default::default())),
            condvar: Arc::new(Condvar::new()),
            messages: Arc::new(Mutex::new(Queue::new()))
        }
    }

    pub fn init(&mut self) -> Result<(), EspError> {
        let gap_server = self.clone();

        self.gap.subscribe(move |event| {
            gap_server.check_esp_status(gap_server.on_gap_event(event));
        })?;

        let gatts_server = self.clone();

        self.gatts.subscribe(move |(gatt_if, event)| {
            gatts_server.check_esp_status(gatts_server.on_gatts_event(gatt_if, event))
        })?;

        info!("BLE Gap and Gatts subscriptions initialized");

        self.gatts.register_app(APP_ID)?;

        info!("Gatts BTP app registered");
        Ok(())
    }

    pub fn get_message(&mut self) -> Option<BtMessage> {
        let queue = self.messages.try_lock();
        if queue.is_err() {
            return None;
        }

        return queue.unwrap().remove().ok();
    }

    fn on_gap_event(&self, event: BleGapEvent) -> Result<(), EspError> {
        info!("Got event: {event:?}");

        if let BleGapEvent::AdvertisingConfigured(status) = event {
            self.check_bt_status(status)?;
            self.gap.start_advertising()?;
        }

        Ok(())
    }

    fn on_gatts_event(
        &self,
        gatt_if: GattInterface,
        event: GattsEvent,
    ) -> Result<(), EspError> {
        info!("Got event: {event:?}");

        match event {
            GattsEvent::ServiceRegistered { status, app_id } => {
                self.check_gatt_status(status)?;
                if APP_ID == app_id {
                    self.create_service(gatt_if)?;
                }
            }
            GattsEvent::ServiceCreated {
                status,
                service_handle,
                ..
            } => {
                self.check_gatt_status(status)?;
                self.configure_and_start_service(service_handle)?;
            }
            GattsEvent::CharacteristicAdded {
                status,
                attr_handle,
                service_handle,
                char_uuid,
            } => {
                self.check_gatt_status(status)?;
                self.register_characteristic(service_handle, attr_handle, char_uuid)?;
            }
            GattsEvent::Write {
                conn_id,
                trans_id,
                addr,
                handle,
                offset,
                need_rsp,
                is_prep,
                value,
            } => {
                let handled = self.recv(
                    gatt_if, conn_id, trans_id, addr, handle, offset, need_rsp, is_prep, value,
                )?;

                if handled {
                    self.send_write_response(
                        gatt_if, conn_id, trans_id, handle, offset, need_rsp, is_prep, value,
                    )?;
                }
            }
            _ => (),
        }

        Ok(())
    }

    fn create_service(&self, gatt_if: GattInterface) -> Result<(), EspError> {
        self.state.lock().unwrap().gatt_if = Some(gatt_if);

        self.gap.set_device_name(HOST_NAME)?;
        self.gap.set_adv_conf(&AdvConfiguration {
            include_name: true,
            include_txpower: true,
            flag: 2,
            service_uuid: Some(BtUuid::uuid128(SERVICE_UUID)),
            ..Default::default()
        })?;
        self.gatts.create_service(
            gatt_if,
            &GattServiceId {
                id: GattId {
                    uuid: BtUuid::uuid128(SERVICE_UUID),
                    inst_id: 0,
                },
                is_primary: true,
            },
            8,
        )?;

        Ok(())
    }

    fn configure_and_start_service(&self, service_handle: Handle) -> Result<(), EspError> {
        self.state.lock().unwrap().service_handle = Some(service_handle);

        self.gatts.start_service(service_handle)?;
        self.add_characteristics(service_handle)?;

        Ok(())
    }

    fn add_characteristics(&self, service_handle: Handle) -> Result<(), EspError> {
        self.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: BtUuid::uuid128(RECV_CHARACTERISTIC_UUID),
                permissions: enum_set!(Permission::Write),
                properties: enum_set!(Property::Write),
                max_len: 200, // Max recv data
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        self.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: BtUuid::uuid128(IND_CHARACTERISTIC_UUID),
                permissions: enum_set!(Permission::Write | Permission::Read),
                properties: enum_set!(Property::Indicate),
                max_len: 200, // Max indicate data
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        Ok(())
    }

    fn register_characteristic(
        &self,
        service_handle: Handle,
        attr_handle: Handle,
        char_uuid: BtUuid,
    ) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();

        if state.service_handle == Some(service_handle) {
            if char_uuid == BtUuid::uuid128(RECV_CHARACTERISTIC_UUID) {
                state.recv_handle = Some(attr_handle);
            } else if char_uuid == BtUuid::uuid128(IND_CHARACTERISTIC_UUID) {
                state.ind_handle = Some(attr_handle);
            }
        }

        Ok(())
    }

    fn recv(
        &self,
        _gatt_if: GattInterface,
        conn_id: ConnectionId,
        _trans_id: TransferId,
        addr: BdAddr,
        handle: Handle,
        offset: u16,
        _need_rsp: bool,
        _is_prep: bool,
        value: &[u8],
    ) -> Result<bool, EspError> {
        let mut state = self.state.lock().unwrap();
        let recv_handle = state.recv_handle;

        if Some(handle) == recv_handle {
            // Process received data
            info!("Received data from {addr}: {:?}", value);
            // This probably doesn't deadlock
            let mut attempts = 0;
            loop {
                if attempts >= 100 {
                    // I don't know either
                    return Ok(false);
                }

                let queue = self.messages.try_lock();
                if queue.is_err() {
                    attempts += 1;
                    FreeRtos::delay_ms(1);
                    continue;
                }

                let _ = queue.unwrap().add(BtMessage {
                    peer: addr,
                    data: value.to_vec()
                });

                break;
            }
            // Add your custom logic here to handle the received data
            return Ok(true);
        }

        Ok(false)
    }

    fn send_write_response(
        &self,
        gatt_if: GattInterface,
        conn_id: ConnectionId,
        trans_id: TransferId,
        handle: Handle,
        offset: u16,
        need_rsp: bool,
        is_prep: bool,
        value: &[u8],
    ) -> Result<(), EspError> {
        if !need_rsp {
            return Ok(());
        }

        if is_prep {
            let mut state = self.state.lock().unwrap();

            state
                .response
                .attr_handle(handle)
                .auth_req(0)
                .offset(offset)
                .value(value)
                .map_err(|_| EspError::from_infallible::<ESP_FAIL>())?;

            self.gatts.send_response(
                gatt_if,
                conn_id,
                trans_id,
                GattStatus::Ok,
                Some(&state.response),
            )?;
        } else {
            self.gatts
                .send_response(gatt_if, conn_id, trans_id, GattStatus::Ok, None)?;
        }

        Ok(())
    }

    fn check_esp_status(&self, status: Result<(), EspError>) {
        if let Err(e) = status {
            warn!("Got status: {:?}", e);
        }
    }

    fn check_bt_status(&self, status: BtStatus) -> Result<(), EspError> {
        if !matches!(status, BtStatus::Success) {
            warn!("Got status: {:?}", status);
            Err(EspError::from_infallible::<ESP_FAIL>())
        } else {
            Ok(())
        }
    }

    fn check_gatt_status(&self, status: GattStatus) -> Result<(), EspError> {
        if !matches!(status, GattStatus::Ok) {
            warn!("Got status: {:?}", status);
            Err(EspError::from_infallible::<ESP_FAIL>())
        } else {
            Ok(())
        }
    }
}
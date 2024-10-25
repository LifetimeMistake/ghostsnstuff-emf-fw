#![feature(const_float_methods)]
#![allow(non_snake_case)]

pub mod vector;
pub mod utils;
pub mod imu;
pub mod led;
pub mod emf;
pub mod speaker;
pub mod bt;

use std::fmt::Debug;
use std::sync::Arc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use bmi160::interface::{ReadData, WriteData};
use bmi160::{AccelerometerRange, Bmi160, Error, GyroscopeRange, SlaveAddr};
use bt::BtServer;
use emf::{EMFReader, Ghost, User};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::ledc::{LedcTimer, Resolution};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::TxRmtDriver;

use esp_idf_hal::units::FromValueType;
use esp_idf_svc::bt::ble::gap::EspBleGap;
use esp_idf_svc::bt::ble::gatt::server::EspGatts;
use esp_idf_svc::bt::BtDriver;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use imu::{Axis, AxisMapping, IMU};
use led::{fill_colors, get_emf_colors, LEDOrder, Ws2812, LEVEL_1_COLOR, LEVEL_2_COLOR, LEVEL_3_COLOR, LEVEL_4_COLOR, LEVEL_5_COLOR};
use speaker::{get_emf_pcm, PWMSpeaker};
use utils::{ledc_resolution_to_u32, vectors_almost_equal, ERROR_TUNE_NOTES, SLEEP_TUNE_NOTES, SUCCESS_TUNE_NOTES, WAKEUP_TUNE_NOTES};
use vector::Vector3;

// Constants
const SPEAKER_RESOLUTION: Resolution = Resolution::Bits8;
const LED_ORDER: LEDOrder = LEDOrder::Reverse;
const SAMPLE_RATE: u32 = 8000;
const SAMPLE_DURATION_MS: u32 = 20;
const IMU_I2C_BAUDRATE_KHZ: u32 = 100;
const IMU_ACCEL_RANGE: AccelerometerRange = AccelerometerRange::G4;
const IMU_ACCEL_MAP: AxisMapping = AxisMapping {
    x: (Axis::X, false),
    y: (Axis::Y, true),
    z: (Axis::Z, false),
};
const IMU_GYRO_RANGE: GyroscopeRange = GyroscopeRange::Scale1000;
const IMU_GYRO_MAP: AxisMapping = AxisMapping {
    x: (Axis::X, true),
    y: (Axis::Y, true),
    z: (Axis::Z, true),
};
const IMU_ALPHA: f32 = 0.9;
const IMU_CALIBRATION_SAMPLES: u32 = 1000;
const IMU_CALIBRATION_INTERVAL: Duration = Duration::from_secs(600);
const SLEEP_TIMEOUT: Duration = Duration::from_secs(30);
const WAKE_CHECK_INTERVAL: Duration = Duration::from_secs(1);

const COMMAND_RESET: u8 = 0x00;
const COMMAND_SLEEP: u8 = 0x01;
const COMMAND_SET_ACTIVITY: u8 = 0x02;
const COMMAND_GLITCH: u8 = 0x03;

fn calibrate<T, W, E>(led: &mut Ws2812, speaker: &mut PWMSpeaker<T>, imu: &mut IMU<W, E>)
where T : LedcTimer,
W: ReadData<Error = Error<E>> + WriteData<Error = Error<E>>,
E: Debug
{
    loop {
        // Attempt to calibrate IMU
        println!("Calibrating IMU");
        led.set_colors(&fill_colors(LEVEL_3_COLOR, 5, 5, LED_ORDER)).unwrap();
        let result = imu.calibrate(IMU_CALIBRATION_SAMPLES);

        if let Err(err) = result {
            println!("Calibration aborted, error encountered: {:?}", err);
            led.set_colors(&fill_colors(LEVEL_5_COLOR, 5, 5, LED_ORDER)).unwrap();
            speaker.play_tune(&ERROR_TUNE_NOTES, SAMPLE_RATE).unwrap();
            sleep(Duration::from_millis(1000));
            // Retry calibration
            continue;
        }

        let (bias_x, bias_y, bias_z) = result.unwrap();
        println!(
            "Computed sensor bias: {:.2}, {:.2}, {:.2}",
            bias_x, bias_y, bias_z
        );

        println!("Calibration complete");
        break;
    }

    // Success message
    led.set_colors(&fill_colors(LEVEL_2_COLOR, 5, 5, LED_ORDER)).unwrap();
    speaker.play_tune(&SUCCESS_TUNE_NOTES, SAMPLE_RATE).unwrap();

    led.turn_off().unwrap();
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();

    // GPIO Constants
    let SPEAKER_PIN = peripherals.pins.gpio8;
    let LED_PIN = peripherals.pins.gpio5;
    let IMU_I2C_DEVICE = peripherals.i2c0;
    let IMU_I2C_SDA_PIN = peripherals.pins.gpio6;
    let IMU_I2C_SCL_PIN = peripherals.pins.gpio7;

    // Speaker init code
    let mut speaker_timer = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &(TimerConfig::new().resolution(SPEAKER_RESOLUTION)),
    )
    .unwrap();
    let mut speaker_driver =
        LedcDriver::new(peripherals.ledc.channel0, &speaker_timer, SPEAKER_PIN).unwrap();
    let mut speaker = PWMSpeaker::new(
        &mut speaker_driver,
        &mut speaker_timer,
        ledc_resolution_to_u32(SPEAKER_RESOLUTION),
    );

    // LED init code
    let mut led_tx = TxRmtDriver::new(
        peripherals.rmt.channel0,
        LED_PIN,
        &TransmitConfig::new().clock_divider(1),
    )
    .unwrap();
    let mut led = Ws2812::new(&mut led_tx, 5);

    // IMU init code
    let imu_i2c = I2cDriver::new(
        IMU_I2C_DEVICE,
        IMU_I2C_SDA_PIN,
        IMU_I2C_SCL_PIN,
        &I2cConfig::new().baudrate(IMU_I2C_BAUDRATE_KHZ.kHz().into()),
    )
    .unwrap();
    let mut imu_driver = Bmi160::new_with_i2c(imu_i2c, SlaveAddr::Alternative(true));
    imu_driver.set_accel_power_mode(bmi160::AccelerometerPowerMode::Normal).unwrap();
    imu_driver.set_accel_range(IMU_ACCEL_RANGE).unwrap();
    imu_driver.set_gyro_power_mode(bmi160::GyroscopePowerMode::Normal).unwrap();
    imu_driver.set_gyro_range(IMU_GYRO_RANGE).unwrap();

    let mut imu = IMU::new(
        imu_driver, 
        Some(IMU_ACCEL_MAP), 
        Some(IMU_GYRO_MAP), 
        Some(IMU_ALPHA)
    );

    // Bluetooth init code
    let nvs = EspDefaultNvsPartition::take().unwrap();
    let bt_driver = Arc::new(
        BtDriver::new(peripherals.modem, 
        Some(nvs.clone())
    ).unwrap());

    let mut bt_server = BtServer::new(
        Arc::new(EspBleGap::new(bt_driver.clone()).unwrap()),
        Arc::new(EspGatts::new(bt_driver.clone()).unwrap())
    );

    bt_server.init().unwrap();
    println!("BT init complete");

    let mut emf = EMFReader::new(
        Ghost::new(
            None,
            None,
            7.5,
            0.02
        ),
        User::new(),
    1
    );

    // Pre-compute values
    let SOUNDS = [
        None,
        Some(get_emf_pcm(2, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap()),
        Some(get_emf_pcm(3, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap()),
        Some(get_emf_pcm(4, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap()),
        Some(get_emf_pcm(5, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap()),
        Some(get_emf_pcm(6, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap())
    ];

    let COLORS = [
        get_emf_colors(1, LED_ORDER),
        get_emf_colors(2, LED_ORDER),
        get_emf_colors(3, LED_ORDER),
        get_emf_colors(4, LED_ORDER),
        get_emf_colors(5, LED_ORDER),
        get_emf_colors(6, LED_ORDER),
    ];

    // Perform initial IMU calibration
    calibrate(
        &mut led,
        &mut speaker,
        &mut imu
    );

    let mut prev_level = 0;
    let mut last_heartbeat = Instant::now();
    let mut last_orientation_change = Instant::now();
    let mut last_calibration = Instant::now();
    let mut is_sleeping = false;
    let mut last_orientation = Vector3::new(0.0, 0.0, 0.0);
    let mut wakeup_requested = false;
    let mut sleep_requested = false;

    loop {
        let now = Instant::now();
        let measurement = imu.data().unwrap();
        let orientation = Vector3::new(
            measurement.pitch, 
            measurement.roll, 
            measurement.yaw
        );

        if is_sleeping {
            if !wakeup_requested && (vectors_almost_equal(&last_orientation, &orientation) ||
            now.duration_since(last_heartbeat) < Duration::from_secs(2)) {
                // Compensate for drift
                last_orientation = orientation;
                sleep(WAKE_CHECK_INTERVAL);
                continue;
            }

            // Wake up device
            println!("Waking up");
            led.set_colors(&fill_colors(LEVEL_1_COLOR, 5, 5, LED_ORDER)).unwrap();
            speaker.play_tune(&WAKEUP_TUNE_NOTES, SAMPLE_RATE).unwrap();
            last_orientation_change = now;
            is_sleeping = false;
            prev_level = 0;
            sleep_requested = false;
            wakeup_requested = false;

            if now.duration_since(last_calibration) > IMU_CALIBRATION_INTERVAL {
                // Prompt for re-calibration
                calibrate(
                    &mut led,
                    &mut speaker,
                    &mut imu
                );

                last_calibration = now;
            }
        }

        emf.update_user(
            None,
            Some(orientation),
        );

        let emf_level = emf.simulate_step();
        let pcm_data = SOUNDS.get(emf_level as usize - 1);
        let led_data = COLORS.get(emf_level as usize - 1);

        if prev_level != emf_level && led_data.is_some() {
            led.set_colors(led_data.unwrap()).unwrap();
        }

        if pcm_data.is_some() && pcm_data.as_ref().unwrap().is_some() {
            let pcm_data = pcm_data.unwrap().as_ref().unwrap();
            speaker.play_pcm(pcm_data, SAMPLE_RATE).unwrap();
        }

        let duration= now.duration_since(last_heartbeat);
        if prev_level != emf_level || (emf_level == 1 && duration.as_millis() > 100) || (duration.as_secs() >= 1) {
            let current_orientation = Vector3::new(
                measurement.pitch, 
                measurement.roll, 
                measurement.yaw
            );

            if vectors_almost_equal(&last_orientation, &current_orientation) || sleep_requested {
                let idle_duration = now.duration_since(last_orientation_change);
                if idle_duration > SLEEP_TIMEOUT || sleep_requested {
                    // Sleep device
                    println!("Entering sleep");
                    led.set_colors(&fill_colors(LEVEL_1_COLOR, 5, 5, LED_ORDER)).unwrap();
                    speaker.play_tune(&SLEEP_TUNE_NOTES, SAMPLE_RATE).unwrap();
                    led.turn_off().unwrap();
                    is_sleeping = true;
                }
            } else {
                last_orientation = orientation;
                last_orientation_change = now;
            }

            // Check bluetooth messages
            loop {
                let bt_message = bt_server.get_message();
                if bt_message.is_none() {
                    break;
                }
            
                let data = bt_message.unwrap().data;
                let command = data[0];
                match command {
                    COMMAND_RESET => {
                        prev_level = 0;
                        emf.activity_level = 1;
                        wakeup_requested = true;
                    },
                    COMMAND_SLEEP => {
                        sleep_requested = true;
                    },
                    COMMAND_SET_ACTIVITY => {
                        emf.activity_level = data[1];
                        // Handle activity 0 later
                    },
                    COMMAND_GLITCH => {
                        println!("Glitch command received");
                    }
                    e => println!("Unknown command received: {}", e)
                }
            }

            last_heartbeat = now;
        }

        prev_level = emf_level;
    }
}

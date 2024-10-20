#![feature(const_float_methods)]
#![allow(non_snake_case)]

pub mod imu;
pub mod led;
pub mod speaker;
pub mod utils;

use std::thread::sleep;
use std::time::Duration;

use bmi160::{AccelerometerRange, Bmi160, GyroscopeRange, SlaveAddr};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::TxRmtDriver;

use esp_idf_hal::units::FromValueType;
use imu::{Axis, AxisMapping, IMU};
use led::{fill_colors, get_emf_colors, LEDOrder, Ws2812, LEVEL_1_COLOR};
use speaker::{get_emf_pcm, PWMSpeaker};
use utils::ledc_resolution_to_u32;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();

    // Constants
    let SPEAKER_PIN = peripherals.pins.gpio8;
    let SPEAKER_RESOLUTION = ledc::Resolution::Bits8;
    let SAMPLE_RATE = 8000;
    let SAMPLE_DURATION_MS = 3000;
    let LED_PIN = peripherals.pins.gpio5;
    let LED_ORDER = LEDOrder::Reverse;
    let IMU_I2C_DEVICE = peripherals.i2c0;
    let IMU_I2C_SDA_PIN = peripherals.pins.gpio6;
    let IMU_I2C_SCL_PIN = peripherals.pins.gpio7;
    let IMU_I2C_BAUDRATE = 100_u32.kHz();
    let IMU_ACCEL_RANGE = AccelerometerRange::G4;
    let IMU_ACCEL_MAP = AxisMapping {
        x: (Axis::Y, false),
        y: (Axis::X, true),
        z: (Axis::Z, false),
    };
    let IMU_GYRO_RANGE = GyroscopeRange::Scale1000;
    let IMU_GYRO_MAP = AxisMapping {
        x: (Axis::Y, false),
        y: (Axis::X, true),
        z: (Axis::Z, true),
    };
    let IMU_ALPHA: f32 = 0.9;
    let IMU_CALIBRATION_SAMPLES: u32 = 1000;

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
        &I2cConfig::new().baudrate(IMU_I2C_BAUDRATE.into()),
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

    println!("Calibrating IMU");
    led.set_colors(&fill_colors(LEVEL_1_COLOR, 5, 5, LED_ORDER)).unwrap();

    let (bias_x, bias_y, bias_z) = imu.calibrate(IMU_CALIBRATION_SAMPLES).unwrap();
    println!(
        "Computed sensor bias: {:.2}, {:.2}, {:.2}",
        bias_x, bias_y, bias_z
    );

    led.turn_off().unwrap();

    for level in 1..7 {
        let colors = get_emf_colors(level, LED_ORDER);
        led.set_colors(&colors).unwrap();

        if level >= 2 {
            let pcm_data = get_emf_pcm(level, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap();
            speaker.play_pcm(&pcm_data, SAMPLE_RATE).unwrap();
        }
    }

    led.turn_off().unwrap();
}

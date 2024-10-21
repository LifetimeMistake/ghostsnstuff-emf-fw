#![feature(const_float_methods)]
#![allow(non_snake_case)]

pub mod vector;
pub mod utils;
pub mod imu;
pub mod led;
pub mod emf;
pub mod speaker;

use std::thread::sleep;
use std::time::{Duration, Instant};

use bmi160::{AccelerometerRange, Bmi160, GyroscopeRange, SlaveAddr};
use emf::{EMFReader, Ghost, User};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::TxRmtDriver;

use esp_idf_hal::units::FromValueType;
use imu::{Axis, AxisMapping, IMU};
use led::{fill_colors, get_emf_colors, LEDOrder, Ws2812, LEVEL_1_COLOR, LEVEL_2_COLOR, LEVEL_3_COLOR, LEVEL_4_COLOR, LEVEL_5_COLOR};
use speaker::{get_emf_pcm, PWMSpeaker};
use utils::{ledc_resolution_to_u32, vectors_almost_equal, ERROR_TUNE_NOTES, SLEEP_TUNE_NOTES, SUCCESS_TUNE_NOTES, WAKEUP_TUNE_NOTES};
use vector::Vector3;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();

    // Constants
    let SPEAKER_PIN = peripherals.pins.gpio8;
    let SPEAKER_RESOLUTION = ledc::Resolution::Bits8;
    let SAMPLE_RATE = 8000;
    let SAMPLE_DURATION_MS = 10;
    let LED_PIN = peripherals.pins.gpio5;
    let LED_ORDER = LEDOrder::Reverse;
    let IMU_I2C_DEVICE = peripherals.i2c0;
    let IMU_I2C_SDA_PIN = peripherals.pins.gpio6;
    let IMU_I2C_SCL_PIN = peripherals.pins.gpio7;
    let IMU_I2C_BAUDRATE = 100_u32.kHz();
    let IMU_ACCEL_RANGE = AccelerometerRange::G4;
    let IMU_ACCEL_MAP = AxisMapping {
        x: (Axis::X, false),
        y: (Axis::Y, true),
        z: (Axis::Z, false),
    };
    let IMU_GYRO_RANGE = GyroscopeRange::Scale1000;
    let IMU_GYRO_MAP = AxisMapping {
        x: (Axis::X, true),
        y: (Axis::Y, true),
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

    let mut emf = EMFReader::new(
        Ghost::new(
            None,
            None,
            7.5,
            0.01
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

    emf.activity_level = 4;
    let mut prev_level = 0;
    let mut last_heartbeat = Instant::now();
    let mut last_orientation_change = Instant::now();
    let mut is_sleeping = false;
    let mut last_orientation = Vector3::new(0.0, 0.0, 0.0);
    let sleep_timeout = Duration::from_secs(30);
    let wake_check_interval = Duration::from_secs(1);

    loop {
        let now = Instant::now();
        let measurement = imu.data().unwrap();
        let orientation = Vector3::new(
            measurement.pitch, 
            measurement.roll, 
            measurement.yaw
        );

        if is_sleeping {
            if vectors_almost_equal(&last_orientation, &orientation) {
                // Compensate for drift
                last_orientation = orientation;
                sleep(wake_check_interval);
                continue;
            }

            // Wake up device
            println!("Waking up");
            led.set_colors(&fill_colors(LEVEL_1_COLOR, 5, 5, LED_ORDER)).unwrap();
            speaker.play_tune(&WAKEUP_TUNE_NOTES, SAMPLE_RATE).unwrap();
            last_orientation_change = now;
            is_sleeping = false;
            prev_level = 0;
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
        if prev_level != emf_level || (emf_level == 1 && duration.as_secs() > 1) || (duration.as_secs() > 5) {
            let current_orientation = Vector3::new(
                measurement.pitch, 
                measurement.roll, 
                measurement.yaw
            );

            if vectors_almost_equal(&last_orientation, &current_orientation) {
                let idle_duration = now.duration_since(last_orientation_change);
                if idle_duration > sleep_timeout {
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

            last_heartbeat = now;
        }

        prev_level = emf_level;
    }
}

#![feature(const_float_methods)]
#![allow(non_snake_case)]

pub mod led;
pub mod speaker;
pub mod utils;

use esp_idf_hal::ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::TxRmtDriver;

use led::{get_emf_colors, LEDOrder, Ws2812};
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
    let SAMPLE_DURATION_MS = 1000;
    let LED_PIN = peripherals.pins.gpio5;
    let LED_ORDER = LEDOrder::Reverse;

    // Speaker init code
    let mut speaker_timer = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &(TimerConfig::new()
        .resolution(SPEAKER_RESOLUTION))
    ).unwrap();
    let mut speaker_driver = LedcDriver::new(
        peripherals.ledc.channel0,
        &speaker_timer,
        SPEAKER_PIN
    ).unwrap();
    let mut speaker = PWMSpeaker::new(
        &mut speaker_driver, 
        &mut speaker_timer, 
        ledc_resolution_to_u32(SPEAKER_RESOLUTION)
    );

    // LED init code
    let mut led_tx = TxRmtDriver::new( 
        peripherals.rmt.channel0, 
        LED_PIN, 
        &TransmitConfig::new().clock_divider(1)
    ).unwrap();
    let mut led = Ws2812::new(&mut led_tx, 5);

    for _ in 0..3 {
        for level in 1..6 {
            let mut colors = get_emf_colors(level);
            if LED_ORDER == LEDOrder::Reverse {
                colors.reverse();
            }
            let pcm_data = get_emf_pcm(level, SAMPLE_RATE, SAMPLE_DURATION_MS).unwrap();
            led.set_colors(&colors).unwrap();
            speaker.play_pcm(&pcm_data, SAMPLE_RATE).unwrap();
        }
    }

    led.turn_off().unwrap();
}

use std::{thread::sleep, time::Duration};
use esp_idf_hal::{ledc::{LedcDriver, LedcTimer, LedcTimerDriver}, sys::EspError, units::FromValueType};

pub const LEVEL_1_FREQ: u32 = 350;
pub const LEVEL_2_FREQ: u32 = 400;
pub const LEVEL_3_FREQ: u32 = 450;
pub const LEVEL_4_FREQ: u32 = 550;
pub const LEVEL_5_FREQ: u32 = 700;

pub fn generate_square_wave(frequency: u32, duration_ms: u32, sample_rate: u32) -> Vec<u8> {
    let mut pcm_data = Vec::new();
    let num_samples = (duration_ms as u32 * sample_rate) / 1000;
    let samples_per_cycle = sample_rate / frequency;

    for i in 0..num_samples {
        let sample_value = if (i % samples_per_cycle) < (samples_per_cycle / 2) {
            255
        } else {
            0
        };
        pcm_data.push(sample_value);
    }

    pcm_data
}

pub fn get_emf_pcm(level: u8, sample_rate: u32, duration_ms: u32) -> Result<Vec<u8>, String> {
    let frequency = match level {
        1 => LEVEL_1_FREQ,
        2 => LEVEL_2_FREQ,
        3 => LEVEL_3_FREQ,
        4 => LEVEL_4_FREQ,
        5 => LEVEL_5_FREQ,
        _ => {
            return Err("Unknown level".to_string())
        }
    };

    return Ok(generate_square_wave(frequency, duration_ms, sample_rate));
}

pub struct PWMSpeaker<'a, 'b, T> 
where T: LedcTimer 
{
    driver: &'a mut LedcDriver<'b>,
    timer: &'a mut LedcTimerDriver<'b, T>,
    pwm_resolution: u32
}

impl<'a, 'b, T> PWMSpeaker<'a, 'b, T> 
where T : LedcTimer
{
    pub fn new(driver: &'a mut LedcDriver<'b>, timer: &'a mut LedcTimerDriver<'b, T>, pwm_resolution: u32) -> Self {
        Self {
            driver,
            timer,
            pwm_resolution
        }
    }

    pub fn play_pcm(&mut self, pcm_data: &[u8], sample_rate: u32) -> Result<(), EspError> {
        let pwm_frequency = (sample_rate * 10).Hz();
        self.timer.set_frequency(pwm_frequency)?;

        for &sample in pcm_data {
            let duty_cycle = ((sample as u32 * self.pwm_resolution) / 255) as u32;
            let sample_period = Duration::from_micros(1_000_000 as u64 / sample_rate as u64);
            
            self.driver.set_duty(duty_cycle)?;
            sleep(sample_period);
        }

        self.driver.set_duty(0)?;
        Ok(())
    }

    pub fn play_frequency(&mut self, frequency: u32, sample_rate: u32, duration_ms: u32) -> Result<(), EspError> {
        let pcm_data = generate_square_wave(frequency, duration_ms, sample_rate);
        self.play_pcm(&pcm_data, sample_rate)
    }

    pub fn turn_off(&mut self) -> Result<(), EspError> {
        self.driver.set_duty(0)
    }
}
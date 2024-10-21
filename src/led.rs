use esp_idf_hal::{
    rmt::{PinState, Pulse, TxRmtDriver, VariableLengthSignal},
    sys::EspError,
};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LEDOrder {
    Normal,
    Reverse,
}

pub const LEVEL_1_COLOR: Rgb = Rgb::from_hsv(190, 100, 3);
pub const LEVEL_2_COLOR: Rgb = Rgb::from_hsv(111, 80, 3);
pub const LEVEL_3_COLOR: Rgb = Rgb::from_hsv(30, 100, 5);
pub const LEVEL_4_COLOR: Rgb = Rgb::from_hsv(20, 100, 3);
pub const LEVEL_5_COLOR: Rgb = Rgb::from_hsv(0, 100, 3);
pub const OFF_COLOR: Rgb = Rgb::new(0, 0, 0);

pub fn get_emf_colors(level: u8, order: LEDOrder) -> [Rgb; 5] {
    let mut colors = [OFF_COLOR; 5];

    if level >= 6 {
        colors[0] = LEVEL_5_COLOR;
        colors[1] = LEVEL_5_COLOR;
        colors[2] = LEVEL_5_COLOR;
        colors[3] = LEVEL_5_COLOR;
        colors[4] = LEVEL_5_COLOR;
        return colors;
    }

    if level >= 1 {
        colors[0] = LEVEL_1_COLOR;
    }
    if level >= 2 {
        colors[1] = LEVEL_2_COLOR;
    }
    if level >= 3 {
        colors[2] = LEVEL_3_COLOR;
    }
    if level >= 4 {
        colors[3] = LEVEL_4_COLOR;
    }
    if level >= 5 {
        colors[4] = LEVEL_5_COLOR;
    }

    if order == LEDOrder::Reverse {
        colors.reverse();
    }

    colors
}

pub fn fill_colors(color: Rgb, on_count: u32, total: u32, order: LEDOrder) -> Vec<Rgb> {
    if on_count > total {
        panic!("Invalid amount of LEDs specified");
    }

    let mut colors = Vec::new();
    for _ in 0..on_count {
        colors.push(color);
    }

    for _ in 0..(total - on_count) {
        colors.push(OFF_COLOR);
    }

    if order == LEDOrder::Reverse {
        colors.reverse();
    }

    colors
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn from_hsv(h: u32, s: u32, v: u32) -> Self {
        if h > 360 || s > 100 || v > 100 {
            panic!("The given HSV values are not in valid range");
        }
        let s = s as f64 / 100.0;
        let v = v as f64 / 100.0;
        let c = s * v;
        let x = c * (1.0 - (((h as f64 / 60.0) % 2.0) - 1.0).abs());
        let m = v - c;
        let (r, g, b) = match h {
            0..=59 => (c, x, 0.0),
            60..=119 => (x, c, 0.0),
            120..=179 => (0.0, c, x),
            180..=239 => (0.0, x, c),
            240..=299 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };
        Self {
            r: ((r + m) * 255.0) as u8,
            g: ((g + m) * 255.0) as u8,
            b: ((b + m) * 255.0) as u8,
        }
    }
}

impl From<Rgb> for u32 {
    fn from(rgb: Rgb) -> Self {
        ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32
    }
}

impl From<&Rgb> for u32 {
    fn from(rgb: &Rgb) -> Self {
        ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32
    }
}

pub struct Ws2812<'a, 'b> {
    tx: &'a mut TxRmtDriver<'b>,
    length: usize,
}

impl<'a, 'b> Ws2812<'a, 'b> {
    pub fn new(tx: &'a mut TxRmtDriver<'b>, length: usize) -> Self {
        Ws2812 { tx, length }
    }

    // Function to send color data for the entire strip
    pub fn set_colors(&mut self, colors: &[Rgb]) -> Result<(), EspError> {
        if colors.len() != self.length {
            panic!("no");
        }

        let ticks_hz = self.tx.counter_clock()?;
        let (t0h, t0l, t1h, t1l) = (
            Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(350))?,
            Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(800))?,
            Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(700))?,
            Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(600))?,
        );

        let mut signal = VariableLengthSignal::new();
        for (_, color) in colors.iter().enumerate() {
            let color_value: u32 = color.into();
            for j in (0..24).rev() {
                let p = 2_u32.pow(j);
                let bit: bool = p & color_value != 0;
                let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
                signal.push(&[high_pulse, low_pulse])?;
            }
        }

        self.tx.start_blocking(&signal)?;
        Ok(())
    }

    pub fn turn_off(&mut self) -> Result<(), EspError> {
        let mut colors = Vec::new();
        for _ in 0..self.length {
            colors.push(OFF_COLOR);
        }

        self.set_colors(&colors)?;
        Ok(())
    }
}

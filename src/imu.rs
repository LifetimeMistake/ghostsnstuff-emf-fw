use std::{f32::consts::PI, fmt::Debug};

use bmi160::{
    interface::{ReadData, WriteData},
    Bmi160, Error, Sensor3DDataScaled, SensorSelector,
};

#[derive(Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Debug)]
pub struct IMUMeasurement {
    pub raw_accel_x: f32,
    pub raw_accel_y: f32,
    pub raw_accel_z: f32,
    pub raw_gyro_x: f32,
    pub raw_gyro_y: f32,
    pub raw_gyro_z: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub gravity_x: f32,
    pub gravity_y: f32,
    pub gravity_z: f32,
    pub acceleration_x: f32,
    pub acceleration_y: f32,
    pub acceleration_z: f32,
}

pub struct AxisMapping {
    pub x: (Axis, bool), // (Physical Axis, Invert)
    pub y: (Axis, bool),
    pub z: (Axis, bool),
}

const GRAVITY: f32 = 9.81;

fn get_mapped_axis(data: &Sensor3DDataScaled, mapping: &(Axis, bool)) -> f32 {
    let value = match mapping.0 {
        Axis::X => data.x,
        Axis::Y => data.y,
        Axis::Z => data.z,
    };

    if mapping.1 {
        -value
    } else {
        value
    }
}

fn remap_axes(data: &Sensor3DDataScaled, mapping: &AxisMapping) -> Sensor3DDataScaled {
    Sensor3DDataScaled {
        x: get_mapped_axis(data, &mapping.x),
        y: get_mapped_axis(data, &mapping.y),
        z: get_mapped_axis(data, &mapping.z),
    }
}

// Function to rotate a vector using pitch, roll, and yaw
fn rotate_vector(x: f32, y: f32, z: f32, pitch: f32, roll: f32, yaw: f32) -> (f32, f32, f32) {
    let cos_pitch = pitch.cos();
    let sin_pitch = pitch.sin();
    let cos_roll = roll.cos();
    let sin_roll = roll.sin();
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    // Rotation matrix components
    let r11 = cos_yaw * cos_pitch;
    let r12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    let r13 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    let r21 = sin_yaw * cos_pitch;
    let r22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    let r23 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    let r31 = -sin_pitch;
    let r32 = cos_pitch * sin_roll;
    let r33 = cos_pitch * cos_roll;

    // Apply rotation
    let new_x = r11 * x + r12 * y + r13 * z;
    let new_y = r21 * x + r22 * y + r23 * z;
    let new_z = r31 * x + r32 * y + r33 * z;

    (new_x, new_y, new_z)
}

pub struct IMU<T, CommE>
where
    T: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
    CommE: Debug,
{
    imu: Bmi160<T>,
    accel_mapping: Option<AxisMapping>,
    gyro_mapping: Option<AxisMapping>,
    prev_time: u32,
    pitch: f32,
    roll: f32,
    yaw: f32,
    accel_bias_x: f32,
    accel_bias_y: f32,
    accel_bias_z: f32,
    alpha: f32,
    is_calibrated: bool
}

impl<T, CommE> IMU<T, CommE>
where
    T: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
    CommE: Debug,
{
    pub fn new(
        imu: Bmi160<T>,
        accel_mapping: Option<AxisMapping>,
        gyro_mapping: Option<AxisMapping>,
        alpha: Option<f32>,
    ) -> Self {
        Self {
            imu,
            accel_mapping,
            gyro_mapping,
            prev_time: 0,
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            accel_bias_x: 0.0,
            accel_bias_y: 0.0,
            accel_bias_z: 0.0,
            alpha: alpha.unwrap_or(0.9),
            is_calibrated: false
        }
    }

    fn read_data(&mut self, apply_bias: bool) -> Result<IMUMeasurement, Error<CommE>> {
        let selector = SensorSelector::new().accel().gyro().time();
        let data = self.imu.data_scaled(selector)?;
        let accel_raw = data.accel.unwrap();
        let gyro_raw = data.gyro.unwrap();
        let current_time = data.time.unwrap();

        let accel;
        let gyro;

        if self.accel_mapping.is_some() {
            accel = remap_axes(&accel_raw, self.accel_mapping.as_ref().unwrap());
        } else {
            accel = accel_raw;
        }

        if self.gyro_mapping.is_some() {
            gyro = remap_axes(&gyro_raw, self.gyro_mapping.as_ref().unwrap());
        } else {
            gyro = gyro_raw;
        }

        // Calculate delta time (dt) in seconds
        let dt_us = if current_time < self.prev_time {
            (current_time as i64 + (1 << 24) - self.prev_time as i64) * 39
        } else {
            (current_time as i64 - self.prev_time as i64) * 39
        };
        let dt = (dt_us as f32) / 1_000_000.0;
        self.prev_time = current_time;

        let gyro_x = gyro.x * (PI / 180.0);
        let gyro_y = gyro.y * (PI / 180.0);
        let gyro_z = gyro.z * (PI / 180.0);

        let accel_x = accel.x * GRAVITY;
        let accel_y = accel.y * GRAVITY;
        let accel_z = accel.z * GRAVITY;

        // Calculate pitch and roll from accelerometer
        let pitch_acc = (accel_y / (accel_x.powi(2) + accel_z.powi(2)).sqrt()).atan();
        let roll_acc = (-accel_x / (accel_z.powi(2) + accel_y.powi(2)).sqrt()).atan();

        // Apply complementary filter
        self.pitch = self.alpha * (self.pitch + gyro_y * dt) + (1.0 - self.alpha) * pitch_acc;
        self.roll = self.alpha * (self.roll + gyro_x * dt) + (1.0 - self.alpha) * roll_acc;
        self.yaw += gyro_z * dt; // yaw is accumulated without filtering

        // Rotate the gravity vector according to the orientation
        let cos_pitch = self.pitch.cos();
        let sin_pitch = self.pitch.sin();
        let cos_roll = self.roll.cos();
        let sin_roll = self.roll.sin();

        let gravity_comp_x = GRAVITY * sin_pitch;
        let gravity_comp_y = GRAVITY * sin_roll;
        let gravity_comp_z = GRAVITY * (cos_pitch * cos_roll);

        let (bias_x, bias_y, bias_z);
        if apply_bias {
            (bias_x, bias_y, bias_z) = rotate_vector(
                self.accel_bias_x, 
                self.accel_bias_y, 
                self.accel_bias_z, 
                self.pitch, 
                self.roll, 
                self.yaw
            );
        } else {
            (bias_x, bias_y, bias_z) = (0.0, 0.0, 0.0);
        }

        // Apply gravity compensation to accelerometer readings
        let acceleration_x = (accel_x - gravity_comp_x - bias_x) * dt;
        let acceleration_y = (accel_y - gravity_comp_y - bias_y) * dt;
        let acceleration_z = (accel_z - gravity_comp_z - bias_z) * dt;

        let result = IMUMeasurement {
            raw_accel_x: accel_x,
            raw_accel_y: accel_y,
            raw_accel_z: accel_z,
            raw_gyro_x: gyro_x,
            raw_gyro_y: gyro_y,
            raw_gyro_z: gyro_z,
            pitch: self.pitch,
            yaw: self.yaw,
            roll: self.roll,
            gravity_x: gravity_comp_x,
            gravity_y: gravity_comp_y,
            gravity_z: gravity_comp_z,
            acceleration_x: acceleration_x,
            acceleration_y: acceleration_y,
            acceleration_z: acceleration_z
        };

        Ok(result)
    }

    pub fn data(&mut self) -> Result<IMUMeasurement, Error<CommE>> {
        if !self.is_calibrated {
            return Err(Error::InvalidInputData);
        }
        self.read_data(true)
    }

    pub fn calibrate(&mut self, samples: u32) -> Result<(f32, f32, f32), Error<CommE>> {
        let mut accel_measurements: Vec<(f32, f32, f32)> = Vec::new();
        let mut gyro_measurements: Vec<(f32, f32, f32)> = Vec::new();

        for _ in 0..samples {
            let m = self.read_data(false)?;
            accel_measurements.push((m.raw_accel_x, m.raw_accel_y, m.raw_accel_z - GRAVITY));
            gyro_measurements.push((m.pitch, m.yaw, m.roll));
        }

        let (abx, aby, abz): (f32, f32, f32) = accel_measurements
            .iter()
            .cloned()
            .fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), (x, y, z)| {
                (sum_x + x, sum_y + y, sum_z + z)
            });

        let (gbx, gby, gbz): (f32, f32, f32) = gyro_measurements
            .iter()
            .cloned()
            .fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), (x, y, z)| {
                (sum_x + x, sum_y + y, sum_z + z)
            });

        let accel_bias_x: f32 = abx / accel_measurements.len() as f32;
        let accel_bias_y: f32 = aby / accel_measurements.len() as f32;
        let accel_bias_z: f32 = abz / accel_measurements.len() as f32;
        let avg_gyro_x: f32 = gbx / gyro_measurements.len() as f32;
        let avg_gyro_y: f32 = gby / gyro_measurements.len() as f32;
        let avg_gyro_z: f32 = gbz / gyro_measurements.len() as f32;

        (self.accel_bias_x, self.accel_bias_y, self.accel_bias_z) = rotate_vector(
            accel_bias_x,
            accel_bias_y,
            accel_bias_z,
            -avg_gyro_x,
            -avg_gyro_y,
            -avg_gyro_z
        );

        self.is_calibrated = true;
        Ok((self.accel_bias_x, self.accel_bias_y, self.accel_bias_z))
    }
}
use std::{f32::consts::PI, fmt::Debug};
use crate::vector::Vector3;

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
    pub raw_accel: Vector3,
    pub raw_gyro: Vector3,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub gravity: Vector3,
    pub acceleration: Vector3,
}

pub struct AxisMapping {
    pub x: (Axis, bool), // (Physical Axis, Invert)
    pub y: (Axis, bool),
    pub z: (Axis, bool),
}

const GRAVITY: f32 = 9.81;
const CALIBRATION_ROT_THRESHOLD: f32 = 1.0;
const CALIBRATION_ACCEL_THRESHOLD: f32 = GRAVITY / 2.0;

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

fn wrap_angle(angle: f32) -> f32 {
    if angle > PI {
        angle - 2.0 * PI
    } else if angle < -PI {
        angle + 2.0 * PI
    } else {
        angle
    }
}

// Function to rotate a vector using pitch, roll, and yaw
fn rotate_vector(v: &Vector3, pitch: f32, roll: f32, yaw: f32) -> Vector3 {
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
    Vector3::new(
        r11 * v.x + r12 * v.y + r13 * v.z,
        r21 * v.x + r22 * v.y + r23 * v.z,
        r31 * v.x + r32 * v.y + r33 * v.z,
    )
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
    orientation: Vector3,
    accel_bias: Vector3,
    alpha: f32,
    is_calibrated: bool,
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
            orientation: Vector3::new(0.0, 0.0, 0.0),
            accel_bias: Vector3::new(0.0, 0.0, 0.0),
            alpha: alpha.unwrap_or(0.9),
            is_calibrated: false,
        }
    }

    fn read_data(&mut self, apply_bias: bool) -> Result<IMUMeasurement, Error<CommE>> {
        let selector = SensorSelector::new().accel().gyro().time();
        let data = self.imu.data_scaled(selector)?;
        let accel_raw = data.accel.unwrap();
        let gyro_raw = data.gyro.unwrap();
        let current_time = data.time.unwrap();

        let accel = if let Some(mapping) = &self.accel_mapping {
            remap_axes(&accel_raw, mapping)
        } else {
            accel_raw
        };

        let gyro = if let Some(mapping) = &self.gyro_mapping {
            remap_axes(&gyro_raw, mapping)
        } else {
            gyro_raw
        };

        // Calculate delta time (dt) in seconds
        let dt_us = if current_time < self.prev_time {
            (current_time as i64 + (1 << 24) - self.prev_time as i64) * 39
        } else {
            (current_time as i64 - self.prev_time as i64) * 39
        };
        let dt = (dt_us as f32) / 1_000_000.0;
        self.prev_time = current_time;

        let gyro_rad = Vector3::new(
            gyro.x * (PI / 180.0),
            gyro.y * (PI / 180.0),
            gyro.z * (PI / 180.0),
        );

        let bias = if apply_bias {
            rotate_vector(&self.accel_bias, self.orientation.x, self.orientation.y, self.orientation.z)
        } else {
            Vector3::new(0.0, 0.0, 0.0)
        };

        let accel_scaled = Vector3::new(
            accel.x * GRAVITY - bias.x,
            accel.y * GRAVITY - bias.y,
            accel.z * GRAVITY - bias.z,
        );

        // Calculate pitch and roll from accelerometer
        let pitch_acc = (accel_scaled.y / (accel_scaled.x.powi(2) + accel_scaled.z.powi(2)).sqrt()).atan();
        let roll_acc = (-accel_scaled.x / (accel_scaled.z.powi(2) + accel_scaled.y.powi(2)).sqrt()).atan();

        // Apply complementary filter
        self.orientation.y = self.alpha * (self.orientation.y + gyro_rad.y * dt) + (1.0 - self.alpha) * pitch_acc;
        self.orientation.x = self.alpha * (self.orientation.x + gyro_rad.x * dt) + (1.0 - self.alpha) * roll_acc;
        self.orientation.z = self.orientation.z + gyro_rad.z * dt;
        self.orientation.z = wrap_angle(self.orientation.z);

        // Rotate the gravity vector according to the orientation
        let gravity_comp = Vector3::new(
            GRAVITY * self.orientation.y.sin(),
            GRAVITY * self.orientation.x.sin(),
            GRAVITY * (self.orientation.y.cos() * self.orientation.x.cos()),
        );

        // Apply gravity compensation to accelerometer readings
        let acceleration = Vector3::new(
            (accel_scaled.x - gravity_comp.x) * dt,
            (accel_scaled.y - gravity_comp.y) * dt,
            (accel_scaled.z - gravity_comp.z) * dt,
        );

        let result = IMUMeasurement {
            raw_accel: accel_scaled,
            raw_gyro: gyro_rad,
            pitch: self.orientation.x,
            yaw: self.orientation.z,
            roll: self.orientation.y,
            gravity: gravity_comp,
            acceleration,
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
        let mut accel_measurements: Vec<Vector3> = Vec::new();
        let mut gyro_measurements: Vec<Vector3> = Vec::new();

        for _ in 0..samples {
            let m = self.read_data(false)?;
            if m.raw_gyro.x.abs() > CALIBRATION_ROT_THRESHOLD || 
                m.raw_gyro.y.abs() > CALIBRATION_ROT_THRESHOLD || 
                m.raw_gyro.z.abs() > CALIBRATION_ROT_THRESHOLD {
                return Err(Error::InvalidInputData); // Abort if rotation exceeds threshold
            }

            if m.raw_accel.x.abs() > CALIBRATION_ACCEL_THRESHOLD || 
                m.raw_accel.y.abs() > CALIBRATION_ACCEL_THRESHOLD || 
                m.raw_accel.z.abs() - GRAVITY > CALIBRATION_ACCEL_THRESHOLD {
                return Err(Error::InvalidInputData); // Abort if rotation exceeds threshold
            }

            accel_measurements.push(Vector3::new( 
                m.raw_accel.x,
                m.raw_accel.y,
                m.raw_accel.z - GRAVITY,
            ));
            gyro_measurements.push(Vector3::new(
                m.pitch,
                m.yaw,
                m.roll,
            ));
        }

        let (abx, aby, abz) = accel_measurements.iter().fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), v| {
            (sum_x + v.x, sum_y + v.y, sum_z + v.z)
        });

        let (gbx, gby, gbz) = gyro_measurements.iter().fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), v| {
            (sum_x + v.x, sum_y + v.y, sum_z + v.z)
        });

        let accel_bias = Vector3::new(
            abx / accel_measurements.len() as f32,
            aby / accel_measurements.len() as f32,
            abz / accel_measurements.len() as f32,
        );
        let avg_gyro = Vector3::new(
            gbx / gyro_measurements.len() as f32,
            gby / gyro_measurements.len() as f32,
            gbz / gyro_measurements.len() as f32,
        );

        self.accel_bias = rotate_vector(&accel_bias, -avg_gyro.x, -avg_gyro.y, -avg_gyro.z);
        self.orientation.x = 0.0;
        self.orientation.y = 0.0;
        self.orientation.z = 0.0;
        self.is_calibrated = true;
        Ok((self.accel_bias.x, self.accel_bias.y, self.accel_bias.z))
    }
}
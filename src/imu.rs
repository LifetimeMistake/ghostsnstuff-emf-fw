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
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
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
    Vector3 {
        x: r11 * v.x + r12 * v.y + r13 * v.z,
        y: r21 * v.x + r22 * v.y + r23 * v.z,
        z: r31 * v.x + r32 * v.y + r33 * v.z,
    }
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
            orientation: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            accel_bias: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
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

        let gyro_rad = Vector3 {
            x: gyro.x * (PI / 180.0),
            y: gyro.y * (PI / 180.0),
            z: gyro.z * (PI / 180.0),
        };

        let accel_scaled = Vector3 {
            x: accel.x * GRAVITY,
            y: accel.y * GRAVITY,
            z: accel.z * GRAVITY,
        };

        // Calculate pitch and roll from accelerometer
        let pitch_acc = (accel_scaled.y / (accel_scaled.x.powi(2) + accel_scaled.z.powi(2)).sqrt()).atan();
        let roll_acc = (-accel_scaled.x / (accel_scaled.z.powi(2) + accel_scaled.y.powi(2)).sqrt()).atan();

        // Apply complementary filter
        self.orientation.y = self.alpha * (self.orientation.y + gyro_rad.y * dt) + (1.0 - self.alpha) * pitch_acc;
        self.orientation.x = self.alpha * (self.orientation.x + gyro_rad.x * dt) + (1.0 - self.alpha) * roll_acc;
        self.orientation.z = self.orientation.z + gyro_rad.z * dt;
        self.orientation.z = wrap_angle(self.orientation.z);

        // Rotate the gravity vector according to the orientation
        let gravity_comp = Vector3 {
            x: GRAVITY * self.orientation.y.sin(),
            y: GRAVITY * self.orientation.x.sin(),
            z: GRAVITY * (self.orientation.y.cos() * self.orientation.x.cos()),
        };

        let bias = if apply_bias {
            rotate_vector(&self.accel_bias, self.orientation.x, self.orientation.y, self.orientation.z)
        } else {
            Vector3 { x: 0.0, y: 0.0, z: 0.0 }
        };

        // Apply gravity compensation to accelerometer readings
        let acceleration = Vector3 {
            x: (accel_scaled.x - gravity_comp.x - bias.x) * dt,
            y: (accel_scaled.y - gravity_comp.y - bias.y) * dt,
            z: (accel_scaled.z - gravity_comp.z - bias.z) * dt,
        };

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
            accel_measurements.push(Vector3 {
                x: m.raw_accel.x,
                y: m.raw_accel.y,
                z: m.raw_accel.z - GRAVITY,
            });
            gyro_measurements.push(Vector3 {
                x: m.pitch,
                y: m.yaw,
                z: m.roll,
            });
        }

        let (abx, aby, abz) = accel_measurements.iter().fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), v| {
            (sum_x + v.x, sum_y + v.y, sum_z + v.z)
        });

        let (gbx, gby, gbz) = gyro_measurements.iter().fold((0.0, 0.0, 0.0), |(sum_x, sum_y, sum_z), v| {
            (sum_x + v.x, sum_y + v.y, sum_z + v.z)
        });

        let accel_bias = Vector3 {
            x: abx / accel_measurements.len() as f32,
            y: aby / accel_measurements.len() as f32,
            z: abz / accel_measurements.len() as f32,
        };
        let avg_gyro = Vector3 {
            x: gbx / gyro_measurements.len() as f32,
            y: gby / gyro_measurements.len() as f32,
            z: gbz / gyro_measurements.len() as f32,
        };

        self.accel_bias = rotate_vector(&accel_bias, -avg_gyro.x, -avg_gyro.y, -avg_gyro.z);
        self.is_calibrated = true;
        Ok((self.accel_bias.x, self.accel_bias.y, self.accel_bias.z))
    }
}
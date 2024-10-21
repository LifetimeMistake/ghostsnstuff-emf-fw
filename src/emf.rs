use std::time::{Duration, Instant};
use crate::vector::Vector3;

pub struct User {
    pub orientation: Vector3,  // Orientation (yaw, pitch, roll) from IMU
    pub position: Vector3 // Position from IMU
}

impl User {
    // Constructor for User
    pub fn new() -> Self {
        User {
            orientation: Vector3::new(0.0, 0.0, 0.0),
            position: Vector3::new(0.0, 0.0, 0.0)
        }
    }
}

fn round_emf_level(level: f32) -> f32 {
    let decimal_part = level.fract(); // Efficiently get the fractional part

    if decimal_part >= 0.75 {
        level.floor() + 1.0 // Round up
    } else {
        level.floor() // Round down
    }
}

// Represents the ghost
pub struct Ghost {
    pub position: Vector3,
    pub radius: f32,
    pub last_update: Instant,
    pub speed: f32,
    pub direction: Vector3,
    pub last_direction_change: Instant,
}

// Represents the EMF reader logic
pub struct EMFReader {
    pub ghost: Ghost,
    pub user: User,
    pub activity_level: u8, // Spirit box activity level, range: [1, 6]
}

impl Ghost {
    pub fn new(position: Option<Vector3>, direction: Option<Vector3>, max_wander_radius: f32, speed: f32) -> Self {
        let mut self_ = Ghost {
            position: *position.as_ref().unwrap_or(&Vector3::new(0.0, 0.0, 0.0)),
            radius: max_wander_radius,
            last_update: Instant::now(),
            speed,
            direction: *direction.as_ref().unwrap_or(&Vector3::new(0.0, 0.0, 0.0)),
            last_direction_change: Instant::now()
        };

        if position.is_none() {
            self_.set_random_position();
        }
        if direction.is_none() {
            self_.set_random_direction();
        }

        self_
    }

    pub fn set_random_position(&mut self) {
        self.position = Vector3 {
            x: (rand::random::<f32>() - 0.5) * self.radius,
            y: (rand::random::<f32>() - 0.5) * self.radius,
            z: 0.0,
        };
    }

    pub fn set_random_direction(&mut self) {
        self.direction = Vector3 {
            x: (rand::random::<f32>() - 0.5) * 2.0,
            y: (rand::random::<f32>() - 0.5) * 2.0,
            z: 0.0,
        }.normalize();
        self.last_direction_change = Instant::now();
    }

    pub fn simulate_step(&mut self) {
        let now = Instant::now();
        let elapsed_time = now.duration_since(self.last_update);
        let direction_change_elapsed = now.duration_since(self.last_direction_change);

        if direction_change_elapsed > Duration::from_secs(5) {
            self.set_random_direction();
        }

        if elapsed_time > Duration::from_millis(100) {
            self.last_update = now;

            // Move in the current direction
            self.position.x = (self.position.x + self.direction.x * self.speed).clamp(-self.radius, self.radius);
            self.position.y = (self.position.y + self.direction.y * self.speed).clamp(-self.radius, self.radius);
            self.position.z = (self.position.z + self.direction.z * self.speed).clamp(-self.radius, self.radius);
        }
    }
}

impl EMFReader {
    pub fn new(ghost: Ghost, user: User, activity_level: u8) -> Self {
        EMFReader {
            ghost,
            user,
            activity_level,
        }
    }

    // Calculate EMF level based on user orientation, ghost position, and ghost activity
    fn calculate_emf_level(&self) -> u8 {
        let user_orientation = &self.user.orientation; // Yaw (Z axis) represents facing direction
        let yaw = user_orientation.z; // Ensure yaw is in radians
    
        // Calculate the vector pointing to the ghost from the user's position
        let direction_to_ghost = Vector3 {
            x: self.ghost.position.x - self.user.position.x,
            y: self.ghost.position.y - self.user.position.y,
            z: self.ghost.position.z - self.user.position.z,
        }.normalize();
    
        // User's forward direction based on yaw
        let user_forward = Vector3 {
            x: yaw.cos(),
            y: yaw.sin(),
            z: 0.0,
        };
    
        // Calculate the alignment (dot product)
        let alignment = user_forward.dot(&direction_to_ghost); // dot product, range -1 to 1
        // Scale dot product result to [0, 1] and then map it to [0.1, 1.0]
        let normalized_alignment = ((alignment + 1.0) / 2.0).clamp(0.0, 1.0);

        // Apply exponential scaling and map to [0.1, 1.0]
        let angle_factor = normalized_alignment.powf(2.0);

        // Calculate the distance to the ghost and prevent division by zero
        let distance = self.ghost.position.distance(&self.user.position);
        let max_distance = self.ghost.radius;
        let distance_factor = ((max_distance - distance + 1.0) / max_distance).clamp(0.1, 1.0);
    
        // Combine angle factor and distance factor with activity level
        let base_emf = angle_factor * distance_factor; // Combine angle and distance factors
        let activity_multiplier = match self.activity_level {
            4 | 5 => 1.0, // High activity guarantees high readings
            _ => self.activity_level as f32 / 3.0, // Activity level as a multiplier (0 to 1)
        };

        // Calculate raw EMF level
        let emf_raw = (base_emf * activity_multiplier * 6.0).clamp(1.0, self.activity_level as f32 + 1.0);
    
        // Print debug information
        // println!(
        //     "al: {}, af: {}, df: {}, emf_r: {}, emf: {}", 
        //     self.activity_level, 
        //     angle_factor, 
        //     distance_factor, 
        //     emf_raw,
        //     round_emf_level(emf_raw).clamp(1.0, 5.0)
        // );
    
        // Determine final EMF level based on activity level
        let emf_level = if self.activity_level == 6 {
            6 // Maximum activity forces level 6
        } else {
            // Scale raw EMF to range [1, 5], considering activity level
            let scaled_emf = round_emf_level(emf_raw).clamp(1.0, 5.0);
            if scaled_emf < 2.0 && self.activity_level < 3 {
                1 // Ensure low activity results in low EMF readings
            } else {
                scaled_emf as u8
            }
        };
    
        emf_level
    }    

    // Function to update both user and ghost
    pub fn simulate_step(&mut self) -> u8 {
        // Update ghost's position (slow wandering)
        self.ghost.simulate_step();
        // Return activity level
        self.calculate_emf_level()
    }

    pub fn update_user(&mut self, position: Option<Vector3>, orientation: Option<Vector3>) {
        if orientation.is_some() {
            self.user.orientation = orientation.unwrap();
        }
        if position.is_some() {
            self.user.position = position.unwrap();
        }
    }
}

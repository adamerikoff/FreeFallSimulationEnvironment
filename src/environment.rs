use macroquad::prelude::*;
use ::rand::{prelude::*, random_range};

// RL-related types
#[derive(Debug, Clone)]
pub struct Observation {
    drone_position: Vec3,
    target_position: Vec3,
    drone_target_angle: f32, 
    is_released: bool,
    total_time_elapsed: f32,
    wind_vector: Vec3,
}

#[derive(Debug)]
pub enum DroneAction {
    Left,
    Right,
    Forward,
    Backward,
    Release,
}

// Camera state management
pub struct CameraState {
    angle: f32,
    distance: f32,
    height: f32,
    target: Vec3,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            angle: 0.0,
            distance: 100.0,
            height: 150.0,
            target: Vec3::new(0.0, 0.0, 0.0),
        }
    }
}

impl CameraState {
    fn get_camera(&self) -> Camera3D {
        let position = Vec3::new(
            self.target.x + self.distance * self.angle.sin(),
            self.height,
            self.target.z + self.distance * self.angle.cos(),
        );
        
        Camera3D {
            position,
            target: self.target,
            up: Vec3::Y,
            ..Default::default()
        }
    }
    
    fn update(&mut self, target: Vec3) {
        self.target = target;
    }
    

    pub fn set_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    pub fn set_distance(&mut self, distance: f32) {
        self.distance = distance;
    }
}

struct Drone {
    position: Vec3,
    size: f32,
    color: Color,
}

struct Grenade {
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    color: Color,
    is_released: bool,
}

struct Target {
    position: Vec3,
    radius: f32,
    color: Color,
}

pub struct Environment {
    drone: Drone,
    grenade: Grenade,
    target: Target,
    pub camera: CameraState,
    wind_vector: Vec3,
    episode_complete: bool,
    total_time_elapsed: f32,
    free_fall_time_elapsed: f32,
}

impl Environment {
    pub fn new() -> Self {
        Self {
            drone: Drone {
                position: Vec3::ZERO,
                size: 3.0,
                color: BLUE,
            },
            grenade: Grenade {
                position: Vec3::ZERO,
                velocity: Vec3::ZERO,
                radius: 2.0,
                color: RED,
                is_released: false,
            },
            target: Target {
                position: Vec3::ZERO,
                radius: 10.0,
                color: PINK,
            },
            camera: CameraState::default(),
            wind_vector: Vec3::ZERO,
            episode_complete: false,
            total_time_elapsed: 0.0,
            free_fall_time_elapsed: 0.0,
        }
    }

    pub fn reset(&mut self) {
        let target_position_x = random_range(-150.0..150.0);
        let target_position_z = random_range(-150.0..150.0);

        let epsilon = random_range(-5.0..5.0);
        let drone_position_y = random_range(100.0..300.0);
        let drone_position_x = target_position_x + epsilon;
        let drone_position_z = target_position_z + epsilon;
        
        self.drone.position = Vec3::new(drone_position_x, drone_position_y, drone_position_z);
        
        self.grenade.position = Vec3::new(drone_position_x, drone_position_y, drone_position_z);
        self.grenade.velocity = Vec3::ZERO;
        self.grenade.is_released = false;
        
        self.target.position = Vec3::new(target_position_x, 0.0, target_position_z);
        
        self.wind_vector = self.generate_wind_vector();

        self.total_time_elapsed = 0.0;
        self.free_fall_time_elapsed = 0.0;
        self.episode_complete = false;
    }

    pub fn step(&mut self, action: &DroneAction) -> (Observation, f32, bool) {
        // 1. Process action
        self.process_action(action);
        
        // 2. Update physics
        self.update();
        
        // 3. Get observation
        let observation = self.get_observation();
        
        // 4. Calculate reward
        let reward = self.calculate_reward();
        
        // 5. Check if episode is done
        let done = self.episode_complete;
        
        (observation, reward, done)
    }

    fn process_action(&mut self, action: &DroneAction) {
        match action {
            DroneAction::Left => self.drone.position.x -= 1.0,
            DroneAction::Right => self.drone.position.x += 1.0,
            DroneAction::Forward => self.drone.position.z += 1.0,
            DroneAction::Backward => self.drone.position.z -= 1.0,
            DroneAction::Release => {
                if !self.grenade.is_released {
                    self.grenade.is_released = true;
                }
            }
        }
    }

    fn update(&mut self) {
        // Increment total time
        self.total_time_elapsed += get_frame_time();
        
        // Update camera to follow drone
        self.camera.update(self.drone.position);

        // If grenade is released, update its physics
        if self.grenade.is_released && !self.episode_complete {
            self.free_fall_time_elapsed += get_frame_time();

            // Constants for physics simulation
            const GRAVITY: f32 = 9.8;
            const DRAG_COEFFICIENT: f32 = 0.47; // For a sphere
            const AIR_DENSITY: f32 = 1.2; // kg/m^3

            // Calculate forces
            let gravity_force = Vec3::new(0.0, -GRAVITY, 0.0);

            // Calculate drag force (Fd = 0.5 * ρ * v² * Cd * A)
            let velocity_magnitude = self.grenade.velocity.length();
            let cross_sectional_area = std::f32::consts::PI * self.grenade.radius.powi(2);
            let drag_magnitude = 0.5 * AIR_DENSITY * velocity_magnitude.powi(2) * DRAG_COEFFICIENT * cross_sectional_area;
            let drag_direction = if velocity_magnitude > 0.0 {
                -self.grenade.velocity / velocity_magnitude
            } else {
                Vec3::ZERO
            };
            let drag_force = drag_direction * drag_magnitude;
            
            // Calculate wind force (simplified)
            let wind_force = self.wind_vector * 0.1;

            // Total acceleration (F = ma, a = F/m)
            let mass = 1.0; // Assume mass of 1kg for simplicity
            let total_acceleration = (gravity_force + drag_force + wind_force) / mass;
            
            // Update velocity and position using Euler integration
            self.grenade.velocity += total_acceleration * get_frame_time();
            self.grenade.position += self.grenade.velocity * get_frame_time();
            
            // Check if grenade has hit the ground
            if self.grenade.position.y <= 0.0 {
                self.grenade.position.y = 0.0;
                self.episode_complete = true;
            }
        } else if !self.grenade.is_released {
            // If grenade is not released, keep it attached to the drone
            self.grenade.position = self.drone.position;
        }
    }
    
    fn get_observation(&self) -> Observation {
        Observation { 
            drone_position: self.drone.position, 
            target_position: self.target.position, 
            drone_target_angle: self.calculate_drone_target_angle(), 
            is_released: self.grenade.is_released, 
            total_time_elapsed: self.total_time_elapsed, 
            wind_vector: self.get_wind_vector()
        }
    }

    fn calculate_drone_target_angle(&self) -> f32 {
        let drone_magnitude = self.drone.position.length();
        let target_magnitude = self.target.position.length();
        let cos_angle = Vec3::dot(self.drone.position, self.target.position) / (drone_magnitude * target_magnitude);
        let angle = cos_angle.acos();
        angle
    }

    fn generate_wind_vector(&self) -> Vec3 {
        let wind_x = random_range(-10.0..10.0);
        let wind_z = random_range(-10.0..10.0);
        return Vec3 { x: wind_x, y: 0.0, z: wind_z }
    }

    fn get_wind_vector(&self) -> Vec3 {
        Vec3 { 
            x: self.wind_vector.x + random_range(-0.5..0.5), 
            y: self.wind_vector.y, 
            z: self.wind_vector.z + random_range(-0.5..0.5) 
        }
    }

    fn calculate_reward(&self) -> f32 {
        let mut reward = -0.5;

        if self.episode_complete {
            // Calculate difference in each dimension
            let dx = self.grenade.position.x - self.target.position.x;
            let dy = self.grenade.position.y - self.target.position.y;
            let dz = self.grenade.position.z - self.target.position.z;

            // PROPER Euclidean distance calculation
            let distance_squared = dx.powi(2) + dy.powi(2) + dz.powi(2);
            let distance = distance_squared.sqrt();
            
            // Base reward when hitting ground
            let ground_reward = 100.0;

            // Reduce reward based on distance to target
            if distance <= self.target.radius {
                reward = reward + ground_reward - (20.0 * distance) + (0.2 * self.drone.position.y);
            } else {
                reward = reward - ground_reward - distance - self.drone.position.y;
            }
        }
        reward
    }

    pub fn render(&self) {
        clear_background(LIGHTGRAY);
        set_camera(&self.camera.get_camera());
        
        // Draw ground grid
        draw_grid(450, 10.0, BLACK, GRAY);
        
        // Draw drone
        draw_cube(self.drone.position, Vec3 { x: 5.0, y: 1.0, z: 10.0 }, None, self.drone.color);
        
        // Draw grenade
        draw_sphere(self.grenade.position, self.grenade.radius, None, self.grenade.color);
        
        // Draw target
        draw_sphere(self.target.position, self.target.radius, None, self.target.color);
        
        // Switch to 2D for UI
        set_default_camera();
        self.render_ui();
    }

    fn render_ui(&self) {
        // Render UI information
        let info_text = format!(
            "Drone Position: ({:.1}, {:.1}, {:.1})\n\
            Target Position: ({:.1}, {:.1}, {:.1})\n\
            Grenade Position: ({:.1}, {:.1}, {:.1})\n\
            Wind Vector: ({:.1}, {:.1}, {:.1})\n\
            Angle: ({:.1})\n\
            Ball Status: {}\n\
            FreeFall Time: {:.2}s\n\
            Total Episode Time: {:.2}s",
            self.drone.position.x, self.drone.position.y, self.drone.position.z,
            self.target.position.x, self.target.position.y, self.target.position.z,
            self.grenade.position.x, self.grenade.position.y, self.grenade.position.z,
            self.wind_vector.x, self.wind_vector.y, self.wind_vector.z,
            self.calculate_drone_target_angle(),
            if !self.grenade.is_released { "Attached" } else if self.episode_complete { "Landed" } else { "Falling" },
            self.free_fall_time_elapsed,
            self.total_time_elapsed
        );
        
        draw_text(&info_text, 20.0, 20.0, 20.0, BLACK);
    }
}
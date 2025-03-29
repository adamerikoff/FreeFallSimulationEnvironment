use macroquad::prelude::*;
use ::rand::{prelude::*, random_range};

use crate::main;

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
    None
}

struct Drone {
    position: Vec3,
    size: Vec3,
    color: Color,
}

struct Grenade {
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    color: Color,
    is_released: bool,
    weight: f32
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
                size: Vec3 { x: 10.0, y: 1.0, z: 10.0 },
                color: BLUE,
            },
            grenade: Grenade {
                position: Vec3::ZERO,
                velocity: Vec3::ZERO,
                radius: 0.07,
                weight: 0.4,
                color: RED,
                is_released: false,
            },
            target: Target {
                position: Vec3::ZERO,
                radius: 5.0,
                color: GREEN,
            },
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
            },
            DroneAction::None => (),
        }
    }

    fn update(&mut self) {
        // Increment total time
        self.total_time_elapsed += get_frame_time();

        // If grenade is released, update its physics
        if self.grenade.is_released && !self.episode_complete {
            self.free_fall_time_elapsed += get_frame_time();

            // Constants for physics simulation
            const GRAVITY: f32 = 9.8;
            const DRAG_COEFFICIENT: f32 = 0.47; // For a sphere
            const AIR_DENSITY: f32 = 1.293; // kg/m^3
            const MASS: f32 = 1.0; // Assume mass of 1kg for simplicity

            // Calculate gravitational force (downward)
            let gravity_force = Vec3::new(0.0, -GRAVITY * MASS, 0.0);

            // Calculate relative velocity (object velocity - wind velocity)
            let relative_velocity = self.grenade.velocity - self.wind_vector;
            let speed = relative_velocity.length();

            // Calculate drag force (Fd = 0.5 * ρ * v² * Cd * A)
            let cross_sectional_area = std::f32::consts::PI * self.grenade.radius.powi(2);
            let drag_magnitude = if speed > 0.0 {
                0.5 * AIR_DENSITY * speed.powi(2) * DRAG_COEFFICIENT * cross_sectional_area
            } else {
                0.0
            };

            let drag_force = if speed > 0.0 {
                -relative_velocity.normalize() * drag_magnitude
            } else {
                Vec3::ZERO
            };

            // Total acceleration (F = ma)
            let total_force = gravity_force + drag_force;
            let acceleration = total_force / MASS;
            
            // Update velocity and position using Euler integration
            self.grenade.velocity += acceleration * get_frame_time();
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
        // Vector from drone to target (in world space)
        let drone_to_target = self.target.position - self.drone.position;

        // Assume drone's forward direction is along its local z-axis
        // (If your drone has rotation, use its actual forward vector)
        let drone_forward = Vec3::new(0.0, 0.0, 1.0); // Default forward (adjust if needed)

        // Project drone_to_target onto the horizontal plane (XZ plane)
        let horizontal_target = Vec3::new(drone_to_target.x, 0.0, drone_to_target.z);
        
        // Calculate the angle between drone's forward and the target direction
        let cos_angle = Vec3::dot(drone_forward, horizontal_target.normalize());
        let angle = cos_angle.acos(); // Angle in radians

        // Determine if the target is to the left or right
        let cross = Vec3::cross(drone_forward, horizontal_target);
        if cross.y < 0.0 {
            -angle // Target is to the left
        } else {
            angle // Target is to the right
        }
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
        let mut reward = -0.1;

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
                reward = reward + ground_reward - (20.0 * distance);
            } else {
                reward = reward - ground_reward - distance;
            }
        }
        reward
    }

    pub fn render(&self) {
        clear_background(LIGHTGRAY);
        
        // Calculate viewport dimensions
        let screen_width = screen_width() as i32;
        let screen_height = screen_height() as i32;
        let main_width = (screen_width as f32 * 0.6) as i32;
        let side_width = screen_width - main_width;
        let side_height = screen_height / 2;
    
        // 1. Main view (left 60% of screen)
        let main_camera = Camera3D {
            position: Vec3::new(
                (self.drone.position.x + self.target.position.x) * 0.5,
                (self.drone.position.y + self.target.position.y) * 0.5 + 50.0,
                (self.drone.position.z + self.target.position.z) * 0.5 + 
                    (self.drone.position - self.target.position).length().max(350.0),
            ),
            target: (self.drone.position + self.target.position) * 0.5,
            up: Vec3::Y,
            viewport: Some((0, 0, main_width, screen_height)),
            ..Camera3D::default()
        };
        set_camera(&main_camera);
        draw_grid(200, 10.0, BLACK, DARKGRAY);
        draw_cube(self.drone.position, self.drone.size, None, self.drone.color);
        draw_sphere(self.grenade.position, 3.0, None, self.grenade.color);
        draw_sphere(self.target.position, self.target.radius, None, self.target.color);
    
        // 2. Top-down view (top-right quadrant)
        let top_camera = Camera3D {
            position: Vec3::new(
                (self.drone.position.x + self.target.position.x) * 0.5,
                (self.drone.position.y + self.target.position.y) * 0.5 + 200.0,  // High above
                (self.drone.position.z + self.target.position.z) * 0.5,
            ),
            target: Vec3::new(
                (self.drone.position.x + self.target.position.x) * 0.5,
                (self.drone.position.y + self.target.position.y) * 0.5,
                (self.drone.position.z + self.target.position.z) * 0.5,
            ),
            up: Vec3::Z,  // Z-up for proper top-down view
            viewport: Some((main_width, 0, side_width, side_height)),
            ..Camera3D::default()
        };
        set_camera(&top_camera);
        draw_grid(200, 10.0, BLACK, DARKGRAY);
        draw_cube(self.drone.position, self.drone.size, None, self.drone.color);
        draw_sphere(self.grenade.position, 3.0, None, self.grenade.color);
        draw_sphere(self.target.position, self.target.radius, None, self.target.color);
    
        // 3. Side view (bottom-right quadrant)
        let bot_camera = Camera3D {
            position: Vec3::new(
                self.drone.position.x + 200.0,  // Offset to the side
                self.drone.position.y + 100.0,  // Slightly above
                self.drone.position.z,
            ),
            target: self.target.position,
            up: Vec3::Y,
            viewport: Some((main_width, side_height, side_width, side_height)),
            ..Camera3D::default()
        };
        set_camera(&bot_camera);
        draw_grid(200, 10.0, BLACK, DARKGRAY);
        draw_cube(self.drone.position, self.drone.size, None, self.drone.color);
        draw_sphere(self.grenade.position, 3.0, None, self.grenade.color);
        draw_sphere(self.target.position, self.target.radius, None, self.target.color);
    
        // Switch to 2D for UI and borders
        set_default_camera();
        self.render_ui();
        self.draw_viewport_borders(main_width, side_width, side_height);
    }
    
    fn draw_viewport_borders(&self, main_width: i32, side_width: i32, side_height: i32) {
        // Draw borders between viewports
        let screen_height = screen_height() as i32;
        
        // Vertical line between main and side views
        draw_line(
            main_width as f32, 0.0, 
            main_width as f32, screen_height as f32, 
            2.0, BLACK
        );
        
        // Horizontal line between top and bottom side views
        draw_line(
            main_width as f32, side_height as f32, 
            (main_width + side_width) as f32, side_height as f32, 
            2.0, BLACK
        );
        
        // Add labels for each viewport
        draw_text("Main View", 10.0, 20.0, 20.0, BLACK);
        draw_text("Top View", (main_width + 10) as f32, 20.0, 20.0, BLACK);
        draw_text("Drone View", (main_width + 10) as f32, (side_height + 20) as f32, 20.0, BLACK);
    }

    fn render_ui(&self) {
        let font_size = 20.0;
        let line_height = font_size * 1.2;
        let mut y_pos = 20.0;
        
        // Draw each piece of information on separate lines
        draw_text(
            &format!("Drone Position: ({:.1}, {:.1}, {:.1})", 
                self.drone.position.x, self.drone.position.y, self.drone.position.z),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("Target Position: ({:.1}, {:.1}, {:.1})", 
                self.target.position.x, self.target.position.y, self.target.position.z),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("Grenade Position: ({:.1}, {:.1}, {:.1})", 
                self.grenade.position.x, self.grenade.position.y, self.grenade.position.z),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;

        draw_text(
            &format!("Grenade Velocity: ({:.1}, {:.1}, {:.1})", 
                self.grenade.velocity.x, self.grenade.velocity.y, self.grenade.velocity.z),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("Wind Vector: ({:.1}, {:.1}, {:.1})", 
                self.wind_vector.x, self.wind_vector.y, self.wind_vector.z),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("Angle: {:.1}°", self.calculate_drone_target_angle()),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        let status = if !self.grenade.is_released {
            "Attached"
        } else if self.episode_complete {
            "Landed"
        } else {
            "Falling"
        };
        draw_text(
            &format!("Grenade Status: {}", status),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("FreeFall Time: {:.2}s", self.free_fall_time_elapsed),
            20.0, y_pos, font_size, BLACK
        );
        y_pos += line_height;
        
        draw_text(
            &format!("Total Episode Time: {:.2}s", self.total_time_elapsed),
            20.0, y_pos, font_size, BLACK
        );
    }
}
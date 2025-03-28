use macroquad::prelude::*;
use ::rand::{prelude::*, random_range};

mod environment;
use environment::{Environment, DroneAction};

#[macroquad::main("Drone Grenade Simulation")]
async fn main() {
    let mut env = Environment::new();
    env.reset();

    let mut camera_angle = 0.0;
    let mut camera_distance = 100.0;
    let mut paused = false;

    loop {
        // Handle input
        if is_key_pressed(KeyCode::Space) {
            env.reset();
        }

        if is_key_pressed(KeyCode::P) {
            paused = !paused;
        }

        // Camera controls
        if is_key_down(KeyCode::Q) {
            camera_angle -= 0.05;
        }
        if is_key_down(KeyCode::E) {
            camera_angle += 0.05;
        }
        if is_key_down(KeyCode::K) {
            camera_distance -= 2.0;
        }
        if is_key_down(KeyCode::L) {
            camera_distance += 2.0;
        }

        // Drone controls
        let mut action = None;
        if is_key_down(KeyCode::Left) || is_key_down(KeyCode::A) {
            action = Some(DroneAction::Left);
        } else if is_key_down(KeyCode::Right) || is_key_down(KeyCode::D) {
            action = Some(DroneAction::Right);
        } else if is_key_down(KeyCode::Up) || is_key_down(KeyCode::W) {
            action = Some(DroneAction::Forward);
        } else if is_key_down(KeyCode::Down) || is_key_down(KeyCode::S) {
            action = Some(DroneAction::Backward);
        } else if is_key_pressed(KeyCode::R) {
            action = Some(DroneAction::Release);
        }

        // Step the environment if not paused
        if !paused {
            if let Some(act) = action {
                let (obs, reward, done) = env.step(&act);
                println!("Reward: {:.2}, Done: {}", reward, done);
            }
        }

        // Update camera parameters
        env.camera.set_angle(camera_angle);
        env.camera.set_distance(camera_distance);

        // Render
        env.render();

        // Draw controls help
        draw_text(
            "CONTROLS:\n\
            WASD/Arrows: Move drone\n\
            R: Release grenade\n\
            Space: Reset\n\
            P: Pause\n\
            Q/E: Rotate camera\n\
            PageUp/PageDown: Zoom",
            20.0,
            screen_height() - 150.0,
            20.0,
            BLACK,
        );

        // Draw pause indicator
        if paused {
            draw_text(
                "PAUSED",
                screen_width() / 2.0 - 50.0,
                screen_height() / 2.0 - 50.0,
                50.0,
                RED,
            );
        }

        next_frame().await;
    }
}
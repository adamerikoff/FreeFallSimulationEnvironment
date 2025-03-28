use macroquad::prelude::*;
use ::rand::{prelude::*, random_range};

mod environment;
use environment::{Environment, DroneAction};

fn window_conf() -> Conf {
    Conf {
        window_title: "Drone Grenade Simulation".to_owned(),
        window_width: 1280,
        window_height: 720,
        high_dpi: true,  // For retina/HiDPI displays
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut env = Environment::new();
    env.reset();

    loop {
        // Process input (if any)
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

        // Always step the environment, passing None when no input
        let (obs, reward, done) = if let Some(act) = action {
            env.step(&act)
        } else {
            // Step with no action - physics still updates
            env.step(&DroneAction::None)
        };

        if done {
            println!("Observation: {:?}", obs);
            println!("Reward: {}", reward);
        }

        // Reset if needed
        if is_key_pressed(KeyCode::Space) || done {
            env.reset();
        }

        // Render
        env.render();

        // Draw controls help
        draw_text(
            "CONTROLS:\n\
            WASD/Arrows: Move drone\n\
            R: Release grenade\n\
            Space: Reset\n",
            20.0,
            screen_height() - 150.0,
            20.0,
            BLACK,
        );

        draw_text(&format!("FPS: {}", get_fps()), 20.0, screen_height() - 180.0, 20.0, BLACK);
        next_frame().await;
    }

}
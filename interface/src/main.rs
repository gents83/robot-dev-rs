mod communication;

use brain::RobotBrain;
use communication::CommunicationLayer;
use kinematics::JointState;
use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("Initializing Humanoid Robot Control System...");

    // Initialize Communication Layer
    let comms = match CommunicationLayer::new().await {
        Ok(c) => Arc::new(c),
        Err(e) => {
            log::error!("Failed to initialize communication layer: {}", e);
            return Err(e);
        }
    };
    log::info!("Communication layer initialized.");

    // Subscribe to JointState (Sensor feedback)
    // We clone comms to keep it alive if needed, or just use the reference before moving
    // The subscribe methods spawn a task, so they need 'static or Arc.
    // CommunicationLayer is Arc-ed. But wait, subscribe takes &self.
    // The callback must be Send + Sync + 'static.

    comms
        .subscribe_joint_state(|joints| {
            // In a real system, we'd update internal state here
            // For now, just log
            log::info!("Received joint state update: {} joints", joints.len());
            if let Some(first) = joints.first() {
                log::info!("Joint 0 angle: {:.2}", first.angle);
            }
        })
        .await?;

    // Subscribe to CameraImage
    comms
        .subscribe_camera_image(|image| {
            log::info!(
                "Received image: {}x{} {}",
                image.width, image.height, image.encoding
            );
        })
        .await?;

    let mut brain = RobotBrain::new();

    // Simulate a target
    let target = JointState {
        angle: 1.57,
        ..Default::default()
    };
    log::info!("Planning motion to: {:?}", target);
    brain.plan_motion(target);

    // Control Loop
    loop {
        if let Some(state) = brain.execute_next_step() {
            log::info!("Executing step: {:?}", state);

            // Publish command
            // brain manages a single joint in this simplified example
            let command = vec![state];
            if let Err(e) = comms.publish_joint_command(&command).await {
                log::error!("Failed to publish command: {}", e);
            }

            // Simulate hardware execution time
            sleep(Duration::from_millis(100)).await;
        } else {
            log::info!("Motion complete. Waiting for new commands...");
            sleep(Duration::from_secs(1)).await;
        }
    }
}

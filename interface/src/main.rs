use brain::RobotBrain;
use kinematics::JointState;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() {
    env_logger::init();

    println!("Initializing Humanoid Robot Control System...");

    let mut brain = RobotBrain::new();

    // Simulate a target
    let target = JointState { angle: 1.57, ..Default::default() };
    println!("Planning motion to: {:?}", target);
    brain.plan_motion(target);

    // Simulation loop
    loop {
        if let Some(state) = brain.execute_next_step() {
            println!("Executing step: {:?}", state);
            // Simulate hardware execution time
            sleep(Duration::from_millis(100)).await;
        } else {
            println!("Motion complete.");
            break;
        }
    }
}

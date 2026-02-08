use kinematics::JointState;
use std::collections::VecDeque;

#[derive(Debug)]
pub struct Planner {
    trajectory: VecDeque<JointState>,
}

impl Planner {
    pub fn new() -> Self {
        Self {
            trajectory: VecDeque::new(),
        }
    }

    pub fn add_waypoint(&mut self, state: JointState) {
        self.trajectory.push_back(state);
    }

    pub fn next_step(&mut self) -> Option<JointState> {
        self.trajectory.pop_front()
    }
}

pub struct RobotBrain {
    planner: Planner,
}

impl RobotBrain {
    pub fn new() -> Self {
        Self {
            planner: Planner::new(),
        }
    }

    pub fn plan_motion(&mut self, target: JointState) {
        // Simple planner: Just move directly to target
        self.planner.add_waypoint(target);
    }

    pub fn execute_next_step(&mut self) -> Option<JointState> {
        self.planner.next_step()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_planner() {
        let mut brain = RobotBrain::new();
        let target = JointState { angle: 1.0, ..Default::default() };
        brain.plan_motion(target);

        let next_step = brain.execute_next_step();
        assert!(next_step.is_some());
        assert_eq!(next_step.unwrap().angle, 1.0);
    }
}

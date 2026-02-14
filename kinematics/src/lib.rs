use nalgebra::{Isometry3, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct JointState {
    pub angle: f64,
    pub velocity: f64,
    pub effort: f64,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            angle: 0.0,
            velocity: 0.0,
            effort: 0.0,
        }
    }
}

pub type Position = Vector3<f64>;

pub mod opw_kinematics;

pub trait ForwardKinematics {
    fn forward_kinematics(&self, joints: &[JointState]) -> Isometry3<f64>;
}

pub struct SimpleArm {
    pub link_length: f64,
}

impl SimpleArm {
    pub fn new(link_length: f64) -> Self {
        Self { link_length }
    }
}

impl ForwardKinematics for SimpleArm {
    fn forward_kinematics(&self, joints: &[JointState]) -> Isometry3<f64> {
        // Simple 1-DOF arm for demonstration
        if joints.is_empty() {
            return Isometry3::identity();
        }

        let angle = joints[0].angle;
        let x = self.link_length * angle.cos();
        let y = self.link_length * angle.sin();
        let z = 0.0;

        let translation = Vector3::new(x, y, z);
        let rotation = nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);

        Isometry3::from_parts(translation.into(), rotation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_simple_arm_fk() {
        let arm = SimpleArm::new(1.0);
        let joints = vec![JointState { angle: PI / 2.0, ..Default::default() }];
        let pose = arm.forward_kinematics(&joints);

        assert!((pose.translation.vector.x).abs() < 1e-6);
        assert!((pose.translation.vector.y - 1.0).abs() < 1e-6);
    }
}

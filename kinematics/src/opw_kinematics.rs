use nalgebra::Isometry3;
use rs_opw_kinematics::kinematic_traits::Kinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;

pub struct OpwKinematicsSolver {
    parameters: Parameters,
}

impl OpwKinematicsSolver {
    pub fn new(c1: f64, c2: f64, c3: f64, c4: f64, a1: f64, a2: f64, b: f64) -> Self {
        let parameters = Parameters {
            c1, c2, c3, c4, a1, a2, b,
            offsets: [0.0; 6],
            sign_corrections: [1; 6],
            dof: 6,
        };
        Self { parameters }
    }

    pub fn inverse_kinematics(&self, pose: &Isometry3<f64>) -> Vec<[f64; 6]> {
        let solver = OPWKinematics::new(self.parameters);
        solver.inverse(pose)
    }

    pub fn forward_kinematics(&self, joints: &[f64; 6]) -> Isometry3<f64> {
        let solver = OPWKinematics::new(self.parameters);
        solver.forward(joints)
    }

    /// Verifies that the calculated joint angles result in the target pose
    /// using the transformation product T_{base}^{tip} = \prod_{i=1}^{6} A_{i}(\theta_i).
    /// This is effectively what forward_kinematics computes.
    pub fn verify_solution(&self, target_pose: &Isometry3<f64>, solution: &[f64; 6]) -> bool {
        let fk_pose = self.forward_kinematics(solution);
        let translation_diff = (target_pose.translation.vector - fk_pose.translation.vector).norm();
        let rotation_diff = target_pose.rotation.angle_to(&fk_pose.rotation);

        translation_diff < 1e-4 && rotation_diff < 1e-4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ik_solver() {
        // Parameters for a generic robot (similar to Kuka KR6)
        let solver = OpwKinematicsSolver::new(
            0.550, // c1
            0.550, // c2
            0.600, // c3
            0.110, // c4
            0.150, // a1
            0.000, // a2
            0.000  // b
        );

        // Define a target joint configuration
        let joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

        // Compute FK to get the target pose
        let target_pose = solver.forward_kinematics(&joints);

        // Compute IK
        let solutions = solver.inverse_kinematics(&target_pose);

        // Verify we have solutions
        assert!(!solutions.is_empty(), "No IK solutions found");

        // Verify each solution
        for solution in &solutions {
            assert!(solver.verify_solution(&target_pose, solution), "Solution verification failed: {:?}", solution);
        }

    }
}

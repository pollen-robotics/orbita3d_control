use nalgebra::{Matrix3, Vector3};

use crate::Orbita3dKinematicsModel;

impl Orbita3dKinematicsModel {
    pub fn compute_output_velocity_from_disks(
        &self,
        thetas: [f64; 3],
        input_velocity: [f64; 3],
    ) -> [f64; 3] {
        let rot = self.compute_forward_kinematics(thetas);

        let j_inv = self.jacobian_inverse(rot, thetas);
        let res = self.compute_output_velocity_from_j_inv(j_inv, input_velocity.into());

        [res[0], res[1], res[2]]
    }

    pub fn compute_input_velocity_from_disks(
        &self,
        thetas: [f64; 3],
        output_velocity: [f64; 3],
    ) -> [f64; 3] {
        let rot = self.compute_forward_kinematics([thetas[0], thetas[1], thetas[2]]);
        let j_inv = self.jacobian_inverse(rot, thetas);
        self.compute_input_velocity_from_j_inv(j_inv, output_velocity.into())
            .into()
    }

    fn compute_input_velocity_from_j_inv(
        &self,
        j_inv: Matrix3<f64>,
        output_vel: Vector3<f64>,
    ) -> Vector3<f64> {
        let j = j_inv.pseudo_inverse(0.000001).unwrap();
        j * output_vel
    }

    fn compute_output_velocity_from_j_inv(
        &self,
        j_inv: Matrix3<f64>,
        input_vel: Vector3<f64>,
    ) -> Vector3<f64> {
        j_inv * input_vel
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        conversion::{self},
        Orbita3dKinematicsModel,
    };

    use rand::Rng;

    const ROLL_RANGE: f64 = 30.0;
    const PITCH_RANGE: f64 = 30.0;
    const YAW_RANGE: f64 = 90.0;

    fn random_rpy() -> [f64; 3] {
        let mut rng = rand::thread_rng();

        let roll = rng.gen_range(-ROLL_RANGE..ROLL_RANGE).to_radians();
        let pitch = rng.gen_range(-PITCH_RANGE..PITCH_RANGE).to_radians();
        let yaw = rng.gen_range(-YAW_RANGE..YAW_RANGE).to_radians();

        [roll, pitch, yaw]
    }

    #[test]
    fn inverse_forward_vel() {
        let orb = Orbita3dKinematicsModel::default();

        let rpy = random_rpy();

        let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
        let disks = orb.compute_inverse_kinematics(rot).unwrap();

        let mut rng = rand::thread_rng();
        let input_vel = [
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        ];

        let output_vel = orb.compute_output_velocity_from_disks(disks, input_vel);
        let reconstructed = orb.compute_input_velocity_from_disks(disks, output_vel);

        assert!(
            (input_vel[0] - reconstructed[0]).abs() < 1e-2,
            "Fail for {:?}",
            input_vel
        );
        assert!(
            (input_vel[1] - reconstructed[1]).abs() < 1e-2,
            "Fail for {:?}",
            input_vel
        );
        assert!(
            (input_vel[2] - reconstructed[2]).abs() < 1e-2,
            "Fail for {:?}",
            input_vel
        );
    }
}

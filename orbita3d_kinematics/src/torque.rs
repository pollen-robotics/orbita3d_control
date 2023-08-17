use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::Orbita3dKinematicsModel;

impl Orbita3dKinematicsModel {
    pub fn compute_output_torque_from_disks(
        &self,
        thetas: [f64; 3],
        input_torque: [f64; 3],
    ) -> Rotation3<f64> {
        let rot = self.compute_forward_kinematics(thetas);

        let j_inv = self.jacobian_inverse(rot, thetas);
        let rpy = self.compute_output_torque_from_j_inv(j_inv, input_torque.into());

        Rotation3::from_euler_angles(rpy[0], rpy[1], rpy[2])
    }

    pub fn compute_input_torque_from_disks(
        &self,
        thetas: [f64; 3],
        output_torque: Rotation3<f64>,
    ) -> [f64; 3] {
        let output_torque = output_torque.euler_angles();
        let output_torque = Vector3::new(output_torque.0, output_torque.1, output_torque.2);

        let rot = self.compute_forward_kinematics(thetas);
        let j_inv = self.jacobian_inverse(rot, thetas);

        self.compute_input_torque_from_j_inv(j_inv, output_torque)
            .into()
    }

    pub fn compute_output_torque_from_j_inv(
        &self,
        j_inv: Matrix3<f64>,
        input_torque: Vector3<f64>,
    ) -> Vector3<f64> {
        j_inv.transpose() * input_torque
    }

    pub fn compute_input_torque_from_j_inv(
        &self,
        j_inv: Matrix3<f64>,
        output_torque: Vector3<f64>,
    ) -> Vector3<f64> {
        let j = j_inv.pseudo_inverse(0.000001).unwrap();
        j.transpose() * output_torque
    }
}

#[cfg(test)]
mod tests {
    // use crate::{conversion::intrinsic_roll_pitch_yaw_to_matrix, Orbita3dKinematicsModel};

    // use rand::Rng;

    // const ROLL_RANGE: f64 = 30.0;
    // const PITCH_RANGE: f64 = 30.0;
    // const YAW_RANGE: f64 = 90.0;

    // fn random_rpy() -> [f64; 3] {
    //     let mut rng = rand::thread_rng();

    //     let roll = rng.gen_range(-ROLL_RANGE..ROLL_RANGE).to_radians();
    //     let pitch = rng.gen_range(-PITCH_RANGE..PITCH_RANGE).to_radians();
    //     let yaw = rng.gen_range(-YAW_RANGE..YAW_RANGE).to_radians();

    //     [roll, pitch, yaw]
    // }

    // #[test]
    // fn inverse_forward_torque() {
    //     let orb = Orbita3dKinematicsModel::default();

    //     let rpy = random_rpy();
    //     let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);

    //     let thetas = orb.compute_inverse_kinematics(rot).unwrap();

    //     let mut rng = rand::thread_rng();
    //     let input_torque = [
    //         rng.gen_range(-1.0..1.0),
    //         rng.gen_range(-1.0..1.0),
    //         rng.gen_range(-1.0..1.0),
    //     ];

    //     let output_torque = orb.compute_output_torque_from_disks(thetas, input_torque);
    //     let reconstructed = orb.compute_input_torque_from_disks(thetas, output_torque);

    //     assert!(
    //         (input_torque[0] - reconstructed[0]).abs() < 1e-2,
    //         "Fail for\n thetas: {:?}\n input torque: {:?}\n rec: {:?}\n",
    //         thetas,
    //         input_torque,
    //         reconstructed
    //     );
    //     assert!(
    //         (input_torque[1] - reconstructed[1]).abs() < 1e-2,
    //         "Fail for\n thetas: {:?}\n input torque: {:?}\n rec: {:?}\n",
    //         thetas,
    //         input_torque,
    //         reconstructed
    //     );
    //     assert!(
    //         (input_torque[2] - reconstructed[2]).abs() < 1e-2,
    //         "Fail for\n thetas: {:?}\n input torque: {:?}\n rec: {:?}\n",
    //         thetas,
    //         input_torque,
    //         reconstructed
    //     );
    // }
}

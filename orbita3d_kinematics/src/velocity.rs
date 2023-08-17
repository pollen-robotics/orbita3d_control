use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::Orbita3dKinematicsModel;

impl Orbita3dKinematicsModel {
    pub fn compute_output_velocity_from_disks(
        &self,
        thetas: [f64; 3],
        input_velocity: [f64; 3],
    ) -> Rotation3<f64> {
        let rot = self.compute_forward_kinematics(thetas);

        let j_inv = self.jacobian_inverse(rot, thetas);
        let res = self.compute_output_velocity_from_j_inv(j_inv, input_velocity.into());

        Rotation3::from_euler_angles(res[0], res[1], res[2])
    }

    pub fn compute_input_velocity_from_disks(
        &self,
        thetas: [f64; 3],
        output_velocity: Rotation3<f64>,
    ) -> [f64; 3] {
        let output_velocity = output_velocity.euler_angles();
        let output_velocity = Vector3::new(output_velocity.0, output_velocity.1, output_velocity.2);

        let rot = self.compute_forward_kinematics([thetas[0], thetas[1], thetas[2]]);
        let j_inv = self.jacobian_inverse(rot, thetas);
        self.compute_input_velocity_from_j_inv(j_inv, output_velocity)
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
    use crate::Orbita3dKinematicsModel;

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

    fn check_inverse_forward(thetas: [f64; 3], input_velocity: [f64; 3]) {
        let orb = Orbita3dKinematicsModel::default();

        let output_velocity = orb.compute_output_velocity_from_disks(thetas, input_velocity);
        let reconstructed = orb.compute_input_velocity_from_disks(thetas, output_velocity);

        for i in 0..3 {
            assert!(
                (input_velocity[i] - reconstructed[i]).abs() < 1e-2,
                "Fail for\n thetas: {:?}\n input velocity: {:?}\n rec: {:?}\n",
                thetas,
                input_velocity,
                reconstructed
            );
        }
    }

    // #[test]
    // fn inverse_forward_vel_value_1() {
    //     let thetas = [0.147376526054817, -0.0063153266133482155, 0.29099962984161976];
    //     let input_velocity = [0.6696758700667225, 0.1914613976070494, -0.3389136179061003];

    // }

    #[test]
    fn inverse_forward_vel() {
        // Using fixed value 1
        let thetas = [
            0.147376526054817,
            -0.0063153266133482155,
            0.29099962984161976,
        ];
        let input_velocity = [0.6696758700667225, 0.1914613976070494, -0.3389136179061003];
        check_inverse_forward(thetas, input_velocity);

        // // Using fixed value 2
        // let thetas = [-0.6799726966192987, -1.1128173034407476, -0.8489251256361031];
        // let input_velocity = [0.7810543324281887, -0.4502710350767902, 0.6821691832152244];
        // check_inverse_forward(thetas, input_velocity);

        // // Using random values
        // let orb = Orbita3dKinematicsModel::default();
        // let rpy: [f64; 3] = random_rpy();
        // let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
        // let thetas = orb.compute_inverse_kinematics(rot).unwrap();

        // let mut rng = rand::thread_rng();
        // let input_velocity = [
        //     rng.gen_range(-1.0..1.0),
        //     rng.gen_range(-1.0..1.0),
        //     rng.gen_range(-1.0..1.0),
        // ];
        // check_inverse_forward(thetas, input_velocity);
    }
}

mod forward;
mod inverse;

#[cfg(test)]
mod tests {
    use crate::{conversion::*, Orbita3dKinematicsModel};

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
    fn forward_inverse() {
        let mut orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        // As we don't have a nice random disks, we use the inverse to get one.
        let rpy = random_rpy();
        let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
        let disks = orb.compute_inverse_kinematics(rot).unwrap();

        let rot = orb.compute_forward_kinematics(disks);
        let reconstructed = orb.compute_inverse_kinematics(rot).unwrap();

        assert!(
            (disks[0] - reconstructed[0]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
        assert!(
            (disks[1] - reconstructed[1]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
        assert!(
            (disks[2] - reconstructed[2]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
    }

    #[test]
    fn inverse_forward() {
        // let mut orb = Orbita3DProblem::default();
        let mut orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        let rpy = random_rpy();
        let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
        let disks = orb.compute_inverse_kinematics(rot).unwrap();

        let rot = orb.compute_forward_kinematics(disks);
        let reconstructed = matrix_to_intrinsic_roll_pitch_yaw(rot);

        assert!(
            (rpy[0] - reconstructed[0]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
        assert!(
            (rpy[1] - reconstructed[1]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
        assert!(
            (rpy[2] - reconstructed[2]).abs() < 1e-2,
            "Fail for {:?}",
            rpy
        );
    }
}

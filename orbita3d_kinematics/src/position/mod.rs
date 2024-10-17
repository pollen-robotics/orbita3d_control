mod forward;
mod inverse;
pub use inverse::InverseSolutionErrorKind;

#[cfg(test)]
mod tests {
    use crate::{conversion::*, Orbita3dKinematicsModel};

    use rand::Rng;
    use std::f64::consts::PI;

    const ROLL_RANGE: f64 = 30.0;
    const PITCH_RANGE: f64 = 30.0;
    const YAW_RANGE: f64 = 180.0;
    const YAW_RANGE_MT: f64 = 900.0;

    fn random_rpy() -> [f64; 3] {
        let mut rng = rand::thread_rng();

        let roll = rng.gen_range(-ROLL_RANGE..ROLL_RANGE).to_radians();
        let pitch = rng.gen_range(-PITCH_RANGE..PITCH_RANGE).to_radians();
        let yaw = rng.gen_range(-YAW_RANGE..YAW_RANGE).to_radians();

        [roll, pitch, yaw]
    }

    fn random_rpy_mt() -> [f64; 3] {
        let mut rng = rand::thread_rng();

        let roll = rng.gen_range(-ROLL_RANGE..ROLL_RANGE).to_radians();
        let pitch = rng.gen_range(-PITCH_RANGE..PITCH_RANGE).to_radians();
        let yaw = rng.gen_range(-YAW_RANGE_MT..YAW_RANGE_MT).to_radians();

        [roll, pitch, yaw]
    }

    #[test]
    fn forward_inverse() {
        let orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        // As we don't have a nice random disks, we use the inverse to get one.
        // let rpy = random_rpy();

        for rpy in [
            [-0.1765150939300468, -0.1289717129142079, -1.05634041350347],
            [
                -0.0006058028930239779,
                0.5135135761900762,
                -1.5625687865275368,
            ],
            random_rpy(),
            random_rpy(),
            random_rpy(),
            random_rpy(),
            random_rpy(),
        ] {
            let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
            let disks = orb.compute_inverse_kinematics(rot).unwrap();

            let rot = orb.compute_forward_kinematics(disks);
            let reconstructed = orb.compute_inverse_kinematics(rot).unwrap();
            let out_rpy = matrix_to_intrinsic_roll_pitch_yaw(rot);

            assert!(
                (disks[0] - reconstructed[0]).abs() < 1e-2,
                "Fail for {:?} (out: {:?})",
                rpy,
                out_rpy
            );
            assert!(
                (disks[1] - reconstructed[1]).abs() < 1e-2,
                "Fail for {:?} (out: {:?})",
                rpy,
                out_rpy
            );
            assert!(
                (disks[2] - reconstructed[2]).abs() < 1e-2,
                "Fail for {:?} (out: {:?})",
                rpy,
                out_rpy
            );
        }
    }

    #[test]
    fn inverse_forward() {
        let orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        for rpy in [
            [-0.1765150939300468, -0.1289717129142079, -1.05634041350347],
            [
                -0.0006058028930239779,
                0.5135135761900762,
                -1.5625687865275368,
            ],
            random_rpy(),
            random_rpy(),
            random_rpy(),
            random_rpy(),
            random_rpy(),
        ] {
            let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
            let rot2 = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2] + 1.0 * PI);
            let disks = orb.compute_inverse_kinematics(rot).unwrap();

            let rot3 = orb.compute_forward_kinematics(disks);
            let reconstructed = matrix_to_intrinsic_roll_pitch_yaw(rot3);

            assert!(
                (rpy[0] - reconstructed[0]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[1] - reconstructed[1]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[2] - reconstructed[2]).abs() < 1e-2,
                "Fail for {:?} (out {:?}) yaw = {:?} {:?} \n{:?} \n{:?}",
                rpy,
                reconstructed,
                (rpy[2] - reconstructed[2]),
                (rpy[2] - reconstructed[2]) % (PI),
                rot,
                rot2
            );
        }
    }

    #[test]
    fn test_inverse_forward_multiturn_zero() {
        let orb = Orbita3dKinematicsModel {
            offset: 0.0_f64.to_radians(),
            ..Default::default()
        };

        for rpy in [
            [0.0, 0.0, 0.0],
            [
                -1.6380393168279918e-08,
                -6.7411586505787807e-09,
                -1.2207476544837933e-09,
            ],
            [
                -8.7242210721817127e-09,
                -3.5903508387229123e-09,
                -6.5017196483874694e-10,
            ],
            [
                -4.0890828050519669e-09,
                -1.6828140595413765e-09,
                -3.0473860986864965e-10,
            ],
            [
                -1.3678829496508414e-09,
                -5.6293618135299251e-10,
                -1.0194138108727733e-10,
            ],
            [
                -2.4477365943051214e-10,
                -1.0073372814871622e-10,
                -1.8241739831181564e-11,
            ],
            [
                -2.7481566284778824e-12,
                -1.1309716590980276e-12,
                -2.0480618171173342e-13,
            ],
        ] {
            let disks = orb.compute_inverse_kinematics_rpy_multiturn(rpy).unwrap();
            let reconstructed = orb.compute_forward_kinematics_rpy_multiturn(disks).unwrap();

            assert!(disks[0] < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);
            assert!(disks[1] < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);
            assert!(disks[2] < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);

            assert!(
                reconstructed[0] < 1e-6,
                "Fail for {:?} (reconstructed {:?})",
                rpy,
                reconstructed
            );

            assert!(
                reconstructed[1] < 1e-6,
                "Fail for {:?} (reconstructed {:?})",
                rpy,
                reconstructed
            );

            assert!(
                reconstructed[1] < 1e-6,
                "Fail for {:?} (reconstructed {:?})",
                rpy,
                reconstructed
            );

            assert!(
                (rpy[0] - reconstructed[0]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[1] - reconstructed[1]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[2] - reconstructed[2]).abs() < 1e-2,
                "Fail for {:?} (out {:?}) disks: {:?}",
                rpy,
                reconstructed,
                disks
            );
        }
    }
    #[test]
    fn test_inverse_forward_multiturn_pi() {
        let orb = Orbita3dKinematicsModel {
            offset: 0.0_f64.to_radians(),
            ..Default::default()
        };

        // test for -pi and pi
        for pi in [-PI, PI] {
            // test small values around pi yaw
            for rpy in [
                [0.0, 0.0, pi],
                [
                    -1.6380393168279918e-08,
                    -6.7411586505787807e-09,
                    pi+-1.2207476544837933e-09,
                ],
                [
                    -8.7242210721817127e-09,
                    -3.5903508387229123e-09,
                    pi+-6.5017196483874694e-10,
                ],
                [
                    -4.0890828050519669e-09,
                    -1.6828140595413765e-09,
                    pi+-3.0473860986864965e-10,
                ],
                [
                    -1.3678829496508414e-09,
                    -5.6293618135299251e-10,
                    pi+-1.0194138108727733e-10,
                ],
                [
                    -2.4477365943051214e-10,
                    -1.0073372814871622e-10,
                    pi+-1.8241739831181564e-11,
                ],
                [
                    -2.7481566284778824e-12,
                    -1.1309716590980276e-12,
                    pi+-2.0480618171173342e-13,
                ],
            ] {
                let disks = orb.compute_inverse_kinematics_rpy_multiturn(rpy).unwrap();
                let reconstructed = orb.compute_forward_kinematics_rpy_multiturn(disks).unwrap();

                assert!(disks[0] - pi < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);
                assert!(disks[1] - pi < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);
                assert!(disks[2] - pi < 1e-6, "Fail for {:?} (disks {:?})", rpy, disks);

                assert!(
                    reconstructed[0] < 1e-6,
                    "Fail for {:?} (reconstructed {:?})",
                    rpy,
                    reconstructed
                );

                assert!(
                    reconstructed[1]  < 1e-6,
                    "Fail for {:?} (reconstructed {:?})",
                    rpy,
                    reconstructed
                );

                assert!(
                    reconstructed[2]- pi  < 1e-6,
                    "Fail for {:?} (reconstructed {:?})",
                    rpy,
                    reconstructed
                );

                assert!(
                    (rpy[0] - reconstructed[0]).abs() < 1e-2,
                    "Fail for {:?} (out {:?})",
                    rpy,
                    reconstructed
                );
                assert!(
                    (rpy[1] - reconstructed[1]).abs() < 1e-2,
                    "Fail for {:?} (out {:?})",
                    rpy,
                    reconstructed
                );
                assert!(
                    (rpy[2] - reconstructed[2]).abs() < 1e-2,
                    "Fail for {:?} (out {:?}) disks: {:?}",
                    rpy,
                    reconstructed,
                    disks
                );
            }
        }
    }

    #[test]
    fn test_inverse_forward_multiturn() {
        let orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        for rpy in [
            [0.18745660173836912, 0.5219153111613383, -6.615005773238176],
            [-0.1765150939300468, -0.1289717129142079, -1.05634041350347],
            [
                -0.0006058028930239779,
                0.5135135761900762,
                -1.5625687865275368,
            ],
            [
                -0.33270742064984016,
                0.20383417343026636,
                -3.684412719450107,
            ],
            random_rpy_mt(),
            random_rpy_mt(),
            random_rpy_mt(),
            random_rpy_mt(),
            random_rpy_mt(),
        ] {
            let disks = orb.compute_inverse_kinematics_rpy_multiturn(rpy).unwrap();
            let reconstructed = orb.compute_forward_kinematics_rpy_multiturn(disks).unwrap();

            assert!(
                (rpy[0] - reconstructed[0]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[1] - reconstructed[1]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                reconstructed
            );
            assert!(
                (rpy[2] - reconstructed[2]).abs() < 1e-2,
                "Fail for {:?} (out {:?}) disks: {:?}",
                rpy,
                reconstructed,
                disks
            );
        }
    }

    #[test]
    fn inverse_forward_full_chain() {
        let orb = Orbita3dKinematicsModel {
            offset: 60.0_f64.to_radians(),
            ..Default::default()
        };

        for rpy in [[0.1, -0.1, PI / 2.0 - 0.1], [0.1, -0.1, PI / 2.0 - 0.1]] {
            let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
            let q = rotation_matrix_to_quaternion(rot);
            let rot2 = quaternion_to_rotation_matrix(q[0], q[1], q[2], q[3]);
            let disks = orb.compute_inverse_kinematics(rot2).unwrap();

            let rot3 = orb.compute_forward_kinematics(disks);
            let q2 = rotation_matrix_to_quaternion(rot3);
            let rpy2 = quaternion_to_roll_pitch_yaw(q2);

            assert!(
                (rpy[0] - rpy2[0]).abs() < 1e-2,
                "Fail for {:?} (out {:?})",
                rpy,
                rpy2
            );
        }
    }
}

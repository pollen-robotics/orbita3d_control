use nalgebra::{Matrix2, Matrix3, Rotation3, Vector2, Vector3};
use std::f64::consts::PI;


const TOLERANCE_ZERO_YAW: f64 = 1e-6; // Define a small tolerance for near-zero values


use crate::{conversion, Orbita3dKinematicsModel};

#[derive(Debug)]
/// Error that can occur when computing the inverse kinematics of the Orbita3d platform.
pub enum InverseSolutionErrorKind {
    /// No solution found.
    NoSolution(Rotation3<f64>),
    /// Invalid solution found.
    InvalidSolution(Rotation3<f64>, Vector3<f64>),
}
impl std::fmt::Display for InverseSolutionErrorKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InverseSolutionErrorKind::NoSolution(rot) => {
                write!(f, "No solution found for rotation matrix: {}", rot)
            }
            InverseSolutionErrorKind::InvalidSolution(rot, gammas) => write!(
                f,
                "Invalid solution found for rotation matrix: {} (gammas: {}) => disk angles are out of bounds.",
                rot, gammas
            ),
        }
    }
}
impl std::error::Error for InverseSolutionErrorKind {}

impl Orbita3dKinematicsModel {
    /// Compute the inverse kinematics of the Orbita3d platform.
    ///
    /// Compute the motor angles from the platform orientation.
    ///
    /// # Arguments
    /// * rot - The platform orientation as a rotation matrix.
    /// # Returns
    /// * The motor angles as a 3-element array, without the 120° offsets.
    pub fn compute_inverse_kinematics(
        &self,
        rot: Rotation3<f64>,
    ) -> Result<[f64; 3], InverseSolutionErrorKind> {
        let v = self.platform_unit_vectors_from_mat(rot).transpose();

        let thetas = self.find_thetas_from_v(v);
        for t in thetas.iter() {
            if t.is_nan() {
                return Err(InverseSolutionErrorKind::NoSolution(rot));
            }
        }

        // remove the 120° offsets and put in [-pi ; pi]
        let mut d1 = thetas[0];
        let mut d2 = thetas[1] - 120.0_f64.to_radians();
        let mut d3 = thetas[2] + 120.0_f64.to_radians();
        d1 = d1.sin().atan2(d1.cos());
        d2 = d2.sin().atan2(d2.cos());
        d3 = d3.sin().atan2(d3.cos());

        Ok([d1, d2, d3])
    }

    pub fn compute_inverse_kinematics_rpy_multiturn(
        &self,
        target_rpy: [f64; 3],
    ) -> Result<[f64; 3], InverseSolutionErrorKind> {
        let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(
            target_rpy[0],
            target_rpy[1],
            target_rpy[2],
        );
        let mut multiturn_offset: f64 = 0.0;

        let thetas = self.compute_inverse_kinematics(rot)?; // The ik returns a geometric solution with thetas in [-pi; pi] without the "natural" 120° offset (zero position is :[0, 0, 0])

        // let target_rpy_no_offet = [target_rpy[0], target_rpy[1], target_rpy[2] + self.offset]; //FIXME????
        let mut thetas: [f64; 3] = self.compute_valid_solution(target_rpy, thetas)?; //this does not seems to work correctly with offsets

        log::debug!("valid Thetas {:?}", thetas);
        // if yaw is more than Pi, we may have to deal with some edge cases
        let true_yaw = target_rpy[2] + self.offset; //FIXME????
        if true_yaw.abs() >= std::f64::consts::PI {
            // Compute the k*2*Pi offset if the yaw target is more than 1 full rotation

            // let nb_turns = (target_rpy[2] / std::f64::consts::TAU).trunc(); //number of full turn
            let nb_turns = (true_yaw / std::f64::consts::TAU).trunc(); //number of full turn
            if nb_turns.abs() >= 1.0 {
                multiturn_offset = std::f64::consts::TAU * (nb_turns);
            }
            // also, if yaw.abs().rem_euclid(2.0 * PI) > pi, we might want to consider the 2pi complement
            // if target_rpy[2].abs().rem_euclid(std::f64::consts::TAU) >= std::f64::consts::PI
            if true_yaw.abs().rem_euclid(std::f64::consts::TAU) >= std::f64::consts::PI
                && !(thetas[0].signum() == thetas[1].signum()
                    && thetas[1].signum() == thetas[2].signum())
            {
                multiturn_offset += target_rpy[2].signum() * std::f64::consts::TAU
            }

            log::debug!("Yaw more than Pi, nb full turns: {nb_turns}, yaw%2pi: {:?} offset: {multiturn_offset} theta before: {:?}",true_yaw.abs().rem_euclid(std::f64::consts::TAU),thetas);

            log::debug!("thetas {:?}", thetas);

            thetas.iter_mut().for_each(|x| *x += multiturn_offset);

            log::debug!("Thetas after offset: {:?}", thetas);
        }

        Ok(thetas)
    }

    pub fn check_gammas(&self, thetas: Vector3<f64>) -> Result<(), Box<dyn std::error::Error>> {
        let gammas = compute_gammas(thetas);
        // println!("CHECK GAMMAS: {:?}", gammas);
        for g in gammas.iter() {
            if !((*g > self.gamma_min) && (*g < self.gamma_max)) {
                let msg = format!(
                    "Gammas out of range: ! {:?} < {:?} < {:?} (thetas {:?})",
                    self.gamma_min, gammas, self.gamma_max, thetas
                );
                return Err((msg).into());
            }
        }
        Ok(())
    }

    pub fn compute_valid_solution(
        &self,
        target_rpy: [f64; 3],
        mut thetas: [f64; 3],
    ) -> Result<[f64; 3], InverseSolutionErrorKind> {
        // Select the "real world" solution from the geometric one: => disks should not cross each over
        // For each theta, there is 2 solutions (only one valid):
        // - The return angle in [-pi, pi]
        // - The 2pi complement
        // We should select the one that avoid crossing the other disks and that rotates in the correct yaw direction
        //
        // algo:
        // - generate all possible solutions
        // - check the validity of the solution (gammas)
        // - there should be maximum 2 valid set of solutions?
        // - select the right one (physically feasible) and if there are 2 solutions, select the one with the same yaw sign

        // generate solutions 2^3

        const NBSOLS: i32 = 8;
        let mut all_solutions = [[0.0_f64; 3]; NBSOLS as usize];
        // TODO: remove extra conversion?
        // let target = conversion::matrix_to_intrinsic_roll_pitch_yaw(rot);

        for i in 0..NBSOLS {
            for j in 0..3 {
                let val = NBSOLS * j + i;
                let ret = 1 & (val >> j);
                if ret != 0 {
                    all_solutions[i as usize][j as usize] = thetas[j as usize];
                } else {
                    all_solutions[i as usize][j as usize] =
                        thetas[j as usize] - thetas[j as usize].signum() * std::f64::consts::TAU;
                }
            }
        }
        let mut validvec = Vec::new();
        for sol in all_solutions {
            match self.check_gammas(sol.into()) {
                Ok(()) => validvec.push(sol),
                Err(_) => continue,
            }
        }
        log::debug!(
            "all solutions: {:?}\nvalid solutions: {:?}",
            all_solutions,
            validvec
        );
        // There is either one solution or 2 valid solutions
        if validvec.len() == 1 {
            thetas = validvec[0];
        } else {
            if validvec.is_empty() {
                log::debug!(
                    "NO VALID SOLUTION! target: {:?}\n thetas: {:?}\nall_solutions: {:?}",
                    target_rpy,
                    thetas,
                    all_solutions
                );
                let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(
                    target_rpy[0],
                    target_rpy[1],
                    target_rpy[2],
                );
                return Err(InverseSolutionErrorKind::InvalidSolution(
                    rot,
                    compute_gammas(thetas.into()),
                ));
            }
            
            // here we have the 2 solutions (both 2pi complement), we chose the one with the same yaw sign
            let mut yaw_sign = (target_rpy[2] + self.offset).signum();
            let mut theta_sign = validvec[0][0].signum();

            // If the yaw or thetas are very close to zero, treat them as effectively zero
            if (target_rpy[2] + self.offset).abs() < TOLERANCE_ZERO_YAW {
                yaw_sign = 0.0;
            }
            if validvec[0][0].abs() < TOLERANCE_ZERO_YAW {
                theta_sign = 0.0;
            }
            // Compare the yaw sign and theta sign
            // but now accounting for near-zero values
            if theta_sign == yaw_sign {
                thetas = validvec[0];
            } else {
                thetas = validvec[1];
            }
        }
        // log::debug!("valid Thetas {:?}", thetas);
        Ok(thetas)
    }

    fn find_thetas_from_v(&self, v: Matrix3<f64>) -> Vector3<f64> {
        let mut thetas = Vector3::zeros();
        for i in 0..3 {
            let a_i =
                -self.alpha.sin() * v.row(i)[0] - self.alpha.cos() * v.row(i)[2] - self.beta.cos();
            let b_i = self.alpha.sin() * v.row(i)[1];
            let c_i =
                self.alpha.sin() * v.row(i)[0] - self.alpha.cos() * v.row(i)[2] - self.beta.cos();

            let mut solutions_theta;

            // Unique solution
            if a_i.abs() <= 1.5 * f64::EPSILON {
                let unique_sol = -c_i / (2.0 * b_i);
                solutions_theta = [unique_sol.atan() * 2.0, PI];
            }
            // Polynome has 2 roots
            else {
                let d_i = b_i.powi(2) - a_i * c_i;
                if d_i < 0.0 {
                    thetas[i] = f64::NAN;
                    continue;
                }

                let dual_sol = [(-b_i + d_i.sqrt()) / a_i, (-b_i - d_i.sqrt()) / a_i];

                solutions_theta = dual_sol.map(|v| v.atan() * 2.0);
            }

            solutions_theta = solutions_theta.map(|v| v.rem_euclid(2.0 * std::f64::consts::PI));

            if solutions_theta[0].is_nan() && solutions_theta[1].is_nan() {
                thetas[i] = f64::NAN;
                continue;
            }

            let v_theta = Matrix2::from_columns(&[
                Vector2::from_row_slice(&[solutions_theta[0].cos(), solutions_theta[0].sin()]),
                Vector2::from_row_slice(&[solutions_theta[1].cos(), solutions_theta[1].sin()]),
            ]);

            let mut theta = 0.0;

            let v_i = Vector2::from_iterator(v.row(i).columns(0, 2).transpose().iter().cloned());

            if self.passiv_arms_direct {
                for (j, &sol) in solutions_theta.iter().enumerate() {
                    let v_theta_ij = Vector2::from_iterator(v_theta.columns(j, 1).iter().cloned());

                    if v_theta_ij.perp(&v_i) >= 0.0 {
                        theta = sol;
                    }
                }
            } else {
                for (j, &sol) in solutions_theta.iter().enumerate() {
                    let v_theta_ij = Vector2::from_iterator(v_theta.columns(j, 1).iter().cloned());
                    if v_theta_ij.perp(&v_i) < 0.0 {
                        theta = sol;
                    }
                }
            }
            thetas[i] = theta;
        }

        thetas
    }
}

fn compute_gammas(thetas: Vector3<f64>) -> Vector3<f64> {
    // Compute the angle difference between each 2 disks (disks without the 120° offset).

    Vector3::from_row_slice(&[
        120.0_f64.to_radians() + (thetas[1] - thetas[0]),
        120.0_f64.to_radians() + (thetas[2] - thetas[1]),
        120.0_f64.to_radians() + (thetas[0] - thetas[2]),
    ])
}

#[cfg(test)]
mod tests {
    use crate::conversion::intrinsic_roll_pitch_yaw_to_matrix;

    use super::*;

    #[test]
    fn ik_zero() {
        let orb = Orbita3dKinematicsModel::default();

        let rot = intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, 0.0);
        let thetas = orb.compute_inverse_kinematics(rot).unwrap();
        assert!(thetas[0].abs() < 1e-4);
        assert!(thetas[1].abs() < 1e-4);
        assert!(thetas[2].abs() < 1e-4);
    }

    #[test]
    fn pitch_only() {
        // In the basic condition:
        //    - disk 0 <--> X
        //    - so with only pitch (move within Y axis, meaning turn around X), disk 0 should not move

        let orb = Orbita3dKinematicsModel::default();
        let rot = intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.3, 0.0);
        let thetas = orb.compute_inverse_kinematics(rot).unwrap();
        assert!(thetas[0].abs() < 1e-4);
        assert!(thetas[1].abs() > 1e-4);
        assert!(thetas[2].abs() > 1e-4);
    }
    #[test]
    fn gammas() {
        let orb = Orbita3dKinematicsModel::default();

        let rot = intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, 0.0);
        let thetas = orb.compute_inverse_kinematics(rot).unwrap(); // thetas are without the 120° offset

        let gammas = compute_gammas(Vector3::from_row_slice(&[thetas[0], thetas[1], thetas[2]]));

        assert!((gammas[0] - 120.0_f64.to_radians()).abs() < 1e-4);
        assert!((gammas[1] - 120.0_f64.to_radians()).abs() < 1e-4);
        assert!((gammas[2] - 120.0_f64.to_radians()).abs() < 1e-4);
    }

    #[test]
    fn gammas_range() {
        let orb = Orbita3dKinematicsModel::default();

        let thetas: [f64; 3] = [
            0.0_f64.to_radians(),
            -120.0_f64.to_radians() + orb.gamma_min,
            120.0_f64.to_radians() + orb.gamma_max,
        ];

        // let gammas = compute_gammas(Vector3::from_row_slice(&[
        //     thetas[0],
        //     thetas[1] + 120.0_f64.to_radians(),
        //     thetas[2] - 120.0_f64.to_radians(),
        // ]));
        // println!(
        //     "GAMMAS: {:?} THETAS: {:?} gamma_min: {} gamma_max: {}",
        //     gammas, thetas, orb.gamma_min, orb.gamma_max
        // );

        // Thetas are at the extreme values, check should fail
        match orb.check_gammas(Vector3::from_row_slice(&[thetas[0], thetas[1], thetas[2]])) {
            Ok(()) => assert!(false),
            Err(_) => assert!(true),
        }
    }

    #[test]
    fn valid_close_to_zero() {
        let orb = Orbita3dKinematicsModel::default();

        let rpy = [-1.6380393168279918e-08, -6.7411586505787807e-09, -1.2207476544837933e-09];
        let rot = intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]);
        let thetas = orb.compute_inverse_kinematics(rot).unwrap();

        let valid_thetas = orb.compute_valid_solution(rpy, thetas).unwrap();

        assert!(valid_thetas[0].abs() < 1e-4);
        assert!(valid_thetas[1].abs() < 1e-4);
        assert!(valid_thetas[2].abs() < 1e-4);
    }
}

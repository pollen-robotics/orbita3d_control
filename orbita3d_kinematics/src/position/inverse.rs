use nalgebra::{Matrix2, Matrix3, Rotation3, Vector2, Vector3};
use std::f64::consts::PI;

use crate::Orbita3dKinematicsModel;

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
    /// * The motor angles as a 3-element array.
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

        // TODO: for now everything is working un modulo 2pi, but it prevents multiturn. A better way would be to work in the multiturn range and check gamma for all solutions +/- 2pi to find a feasible one?
        // // TODO: extract as a usable fonction
        // let gammas = compute_gammas(thetas);
        // for g in gammas.iter() {
        //     if !((*g > self.gamma_min) && (*g < self.gamma_max)) {
        //         return Err(InverseSolutionErrorKind::InvalidSolution(rot, gammas));
        //     }
        // }

        let _ = match self.check_gammas(thetas) {
            Ok(()) => Ok(thetas),
            Err(e) => {
                println!("{e}");
                Err(InverseSolutionErrorKind::InvalidSolution(
                    rot,
                    compute_gammas(thetas),
                ))
            }
        };

        let d1 = thetas[0];
        let d2 = thetas[1] - 120.0_f64.to_radians();
        let d3 = thetas[2] + 120.0_f64.to_radians();
        println!(
            "BEFORE ATAN2 d1: {}, d2: {}, d3: {} AFTER ATAN2 d1: {}, d2: {}, d3: {}",
            d1,
            d2,
            d3,
            d1.sin().atan2(d1.cos()),
            d2.sin().atan2(d2.cos()),
            d3.sin().atan2(d3.cos())
        );

        //TODO, check gammas after the atan2?

        Ok([
            d1.sin().atan2(d1.cos()),
            d2.sin().atan2(d2.cos()),
            d3.sin().atan2(d3.cos()),
            // d1, d2, d3,
        ])
    }

    pub fn check_gammas(&self, thetas: Vector3<f64>) -> Result<(), Box<dyn std::error::Error>> {
        let gammas = compute_gammas(thetas);
        for g in gammas.iter() {
            if !((*g > self.gamma_min) && (*g < self.gamma_max)) {
                let msg = format!(
                    "Gammas out of range: ! {:?} < {:?} < {:?}",
                    self.gamma_min, gammas, self.gamma_max
                );
                return Err((msg).into());
            }
        }
        Ok(())
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
    let mut th = thetas;

    // for t in th.iter_mut() {
    //     *t = t.rem_euclid(2.0 * PI);
    // }

    // Vector3::from_row_slice(&[
    //     (th[1] - th[0]).rem_euclid(2.0 * PI),
    //     (th[2] - th[1]).rem_euclid(2.0 * PI),
    //     (th[0] - th[2]).rem_euclid(2.0 * PI),
    // ])

    Vector3::from_row_slice(&[(th[1] - th[0]), (th[2] - th[1]), (th[0] - th[2])])
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
}

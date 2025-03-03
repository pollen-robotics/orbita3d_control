//! Kinematics model for Orbita3d
//!
//! ### Model definition
//! * _thetas_ disk angles
//! * _rot_ platform orientation
//!
//! All values are expressed in radians.
//!
//! See the [README.md](./README.md) for more information.

use nalgebra::{Matrix3, Rotation3, Vector3};

pub mod conversion;
mod jacobian;
mod position;
mod torque;
mod velocity;

pub use position::InverseSolutionErrorKind;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
/// Kinematics model for Orbita3d
pub struct Orbita3dKinematicsModel {
    pub alpha: f64,
    pub gamma_min: f64,
    pub offset: f64,
    pub beta: f64,
    pub gamma_max: f64,
    pub passiv_arms_direct: bool,
}

impl Default for Orbita3dKinematicsModel {
    /// Creates a new Orbita3dKinematicsModel with default values corresponding to Reachy's neck.
    fn default() -> Self {
        Orbita3dKinematicsModel {
            alpha: 54.0_f64.to_radians(),
            gamma_min: 0.0_f64.to_radians(),
            offset: 0.0_f64.to_radians(),
            beta: 90.0_f64.to_radians(),
            gamma_max: 180.0_f64.to_radians(),
            passiv_arms_direct: true,
        }
    }
}

impl Orbita3dKinematicsModel {
    fn platform_unit_vectors_from_mat(&self, rot: Rotation3<f64>) -> Matrix3<f64> {
        // Compute the unit vectors of the platform in the base frame, from an input rotation matrix
        let delta_v = 120.0_f64.to_radians();
        let mut beta = self.beta;

        if !self.passiv_arms_direct {
            beta *= -1.0;
        }
        let v_initial1 = Vector3::from_row_slice(&[beta.cos(), beta.sin(), 0.]);
        let v_initial2 =
            Vector3::from_row_slice(&[(beta + delta_v).cos(), (beta + delta_v).sin(), 0.]);
        let v_initial3 =
            Vector3::from_row_slice(&[(beta - delta_v).cos(), (beta - delta_v).sin(), 0.]);

        let roffset = Rotation3::from_euler_angles(0., 0., self.offset);

        let v_initial1 = roffset * v_initial1;
        let v_initial2 = roffset * v_initial2;
        let v_initial3 = roffset * v_initial3;

        let v_rotation = Matrix3::from_columns(&[v_initial1, v_initial2, v_initial3]);

        rot * v_rotation
    }
}

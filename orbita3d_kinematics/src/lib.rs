use nalgebra::{Matrix3, Rotation3, SVector, Vector3};

pub mod conversion;
mod jacobian;
mod position;
mod velocity;

#[derive(Copy, Clone)]
pub struct Orbita3dKinematicsModel {
    // holds current value of the n parameters
    p: SVector<f64, 6>,
    pub alpha: f64,
    pub gamma_min: f64,
    pub offset: f64,
    pub beta: f64,
    pub gamma_max: f64,
    pub passiv_arms_direct: bool,
    pub thetas: Vector3<f64>,
}

impl Default for Orbita3dKinematicsModel {
    fn default() -> Self {
        Orbita3dKinematicsModel {
            p: SVector::from_row_slice(&[0., 1., 0., 1., 0., 1.]),
            alpha: 54.0_f64.to_radians(),
            gamma_min: 0.0_f64.to_radians(),
            offset: 0.0_f64.to_radians(),
            beta: 90.0_f64.to_radians(),
            gamma_max: 180.0_f64.to_radians(),
            passiv_arms_direct: true,
            thetas: Vector3::from_row_slice(&[0., 0., 0.]),
        }
    }
}

impl Orbita3dKinematicsModel {
    fn platform_unit_vectors_from_mat(&self, rot: Rotation3<f64>) -> Matrix3<f64> {
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

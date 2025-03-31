use nalgebra::{Matrix3, Rotation3, RowVector3};

use crate::Orbita3dKinematicsModel;

impl Orbita3dKinematicsModel {
    /// Compute the inverse jacobian
    ///
    /// Compute the inverse jacobian from the platform orientation and the motor angles.
    ///
    /// # Arguments
    /// * rot - The platform orientation as a rotation matrix.
    /// * thetas - The motor angles as a 3-element array.
    /// # Returns
    /// * The inverse jacobian as a 3x3 matrix.
    pub fn jacobian_inverse(&self, rot: Rotation3<f64>, thetas: [f64; 3]) -> Matrix3<f64> {
        // has to be trasponsed here - not sure why (otherwise v.row(0) takes the frist column rather than row)
        // TODO verify furhter and find why this happens
        let v = self.platform_unit_vectors_from_mat(rot).transpose();

        let sa1 = self.alpha.sin();
        let ca1 = self.alpha.cos();
        let st1 = thetas[0].sin();
        let ct1 = thetas[0].cos();
        // these thetas need to have 120 degrees added or subtracted
        let st2 = (thetas[1] + 120.0_f64.to_radians()).sin();
        let ct2 = (thetas[1] + 120.0_f64.to_radians()).cos();
        let st3 = (thetas[2] - 120.0_f64.to_radians()).sin();
        let ct3 = (thetas[2] - 120.0_f64.to_radians()).cos();

        let row1_denom = -sa1 * st1 * v.row(0)[0] + sa1 * ct1 * v.row(0)[1];
        let row2_denom = -sa1 * st2 * v.row(1)[0] + sa1 * ct2 * v.row(1)[1];
        let row3_denom = -sa1 * st3 * v.row(2)[0] + sa1 * ct3 * v.row(2)[1];

        let row1 = RowVector3::new(
            (sa1 * st1 * v.row(0)[2] + ca1 * v.row(0)[1]) / row1_denom,
            (-sa1 * ct1 * v.row(0)[2] - ca1 * v.row(0)[0]) / row1_denom,
            1.0,
        );
        let row2 = RowVector3::new(
            (sa1 * st2 * v.row(1)[2] + ca1 * v.row(1)[1]) / row2_denom,
            (-sa1 * ct2 * v.row(1)[2] - ca1 * v.row(1)[0]) / row2_denom,
            1.0,
        );
        let row3 = RowVector3::new(
            (sa1 * st3 * v.row(2)[2] + ca1 * v.row(2)[1]) / row3_denom,
            (-sa1 * ct3 * v.row(2)[2] - ca1 * v.row(2)[0]) / row3_denom,
            1.0,
        );

        Matrix3::from_rows(&[row1, row2, row3])
    }

    /// Compute the jacobian
    ///
    /// The jacobian is computed by inverting the inverse jacobian.
    ///
    /// # Arguments
    /// * rot - The platform orientation as a rotation matrix.
    /// * thetas - The motor angles as a 3-element array.
    /// # Returns
    /// * The jacobian as a 3x3 matrix.
    pub fn jacobian(&self, rot: Rotation3<f64>, thetas: [f64; 3]) -> Matrix3<f64> {
        let j_inv = self.jacobian_inverse(rot, thetas);
        j_inv.try_inverse().unwrap()
    }
}

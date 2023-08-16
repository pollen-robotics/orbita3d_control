use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::{Matrix3, Owned, Rotation3, SMatrix, SVector, Vector3, U12, U6};
use ndarray_einsum_beta::einsum;
use nshare::{RefNdarray2, ToNalgebra};

use crate::Orbita3dKinematicsModel;

impl Orbita3dKinematicsModel {
    pub fn compute_forward_kinematics(&mut self, disk_position: [f64; 3]) -> Rotation3<f64> {
        let disk_position = Vector3::from_row_slice(&[
            disk_position[0],
            disk_position[1] + 120.0_f64.to_radians(),
            disk_position[2] - 120.0_f64.to_radians(),
        ]);

        // Forward of Orbita, takes the disk position (in radians) and return a rotation matrix
        // self.p = Vector6f64::from_row_slice(&[0., 1., 0., 1., 0., 1.]); //reset p?
        let st1 = disk_position[0].sin();
        let st2 = disk_position[1].sin();
        let st3 = disk_position[2].sin();

        let ct1 = disk_position[0].cos();
        let ct2 = disk_position[1].cos();
        let ct3 = disk_position[2].cos();

        let phis = self.compute_phis(disk_position).unwrap();
        let sp1_n = phis[0];
        let cp1_n = phis[1];

        let sp2_n = phis[2];
        let cp2_n = phis[3];

        let sp3_n = phis[4];
        let cp3_n = phis[5];

        // from the phis and the thetas we can compute the v_i vectors
        let mut v1sol = self.v_i(cp1_n, sp1_n, ct1, st1);
        let mut v2sol = self.v_i(cp2_n, sp2_n, ct2, st2);
        let mut v3sol = self.v_i(cp3_n, sp3_n, ct3, st3);

        let roffset = Rotation3::from_euler_angles(0., 0., self.beta);
        v1sol = roffset * v1sol;
        v2sol = roffset * v2sol;
        v3sol = roffset * v3sol;

        let v_mat = Matrix3::from_columns(&[v1sol, v2sol, v3sol]);

        let mut b1 =
            Vector3::from_row_slice(&[0.0_f64.to_radians().cos(), 0.0_f64.to_radians().sin(), 0.]);

        let mut b2 = Vector3::from_row_slice(&[
            120.0_f64.to_radians().cos(),
            120.0_f64.to_radians().sin(),
            0.,
        ]);

        let mut b3 = Vector3::from_row_slice(&[
            (-120.0_f64).to_radians().cos(),
            (-120.0_f64).to_radians().sin(),
            0.,
        ]);

        b1 = roffset * b1;
        b2 = roffset * b2;
        b3 = roffset * b3;

        let off = Rotation3::from_euler_angles(0.0, 0.0, self.offset);
        b1 = off * b1;
        b2 = off * b2;
        b3 = off * b3;

        let b_mat = Matrix3::from_columns(&[b1, b2, b3]);
        align_vectors(v_mat.transpose(), b_mat.transpose())
    }

    fn set_thetas(&mut self, disk_position: Vector3<f64>) {
        self.thetas.copy_from(&disk_position);
    }

    fn v_i(&self, cpi: f64, spi: f64, cti: f64, sti: f64) -> Vector3<f64> {
        let sa1 = self.alpha.sin();
        let sa2 = self.beta.sin();

        let ca1 = self.alpha.cos();
        let ca2 = self.beta.cos();

        Vector3::from_row_slice(&[
            -ca1 * cpi * sa2 * sti + ca2 * sa1 * sti + cti * sa2 * spi,
            ca1 * cpi * cti * sa2 - ca2 * cti * sa1 + sa2 * spi * sti,
            -ca1 * ca2 - cpi * sa1 * sa2,
        ])
    }
    fn w_i(&self, cti: f64, sti: f64) -> Vector3<f64> {
        let sa1 = self.alpha.sin();
        let ca1 = self.alpha.cos();

        Vector3::from_row_slice(&[sa1 * sti, -cti * sa1, -ca1])
    }

    fn phis_system_equations(&self) -> SVector<f64, 12> {
        // compute the system of equations
        let cp1 = self.p[0];
        let sp1 = self.p[1];
        let cp2 = self.p[2];
        let sp2 = self.p[3];
        let cp3 = self.p[4];
        let sp3 = self.p[5];
        let ca2 = self.beta.cos();

        let st1 = self.thetas[0].sin();
        let st2 = self.thetas[1].sin();
        let st3 = self.thetas[2].sin();
        let ct1 = self.thetas[0].cos();
        let ct2 = self.thetas[1].cos();
        let ct3 = self.thetas[2].cos();

        let v1n = self.v_i(cp1, sp1, ct1, st1);
        let v2n = self.v_i(cp2, sp2, ct2, st2);
        let v3n = self.v_i(cp3, sp3, ct3, st3);

        let w1n = self.w_i(ct1, st1);
        let w2n = self.w_i(ct2, st2);
        let w3n = self.w_i(ct3, st3);

        let eq1 = v1n.dot(&v2n) + 0.5;
        let eq2 = v2n.dot(&v3n) + 0.5;
        let eq3 = v3n.dot(&v1n) + 0.5;

        let eq4 = w1n.dot(&v1n) - ca2;
        let eq5 = w2n.dot(&v2n) - ca2;
        let eq6 = w3n.dot(&v3n) - ca2;

        let eq7 = v1n.dot(&v1n) - 1.;
        let eq8 = v2n.dot(&v2n) - 1.;
        let eq9 = v3n.dot(&v3n) - 1.;

        let eq10 = v1n[0] + v2n[0] + v3n[0];
        let eq11 = v1n[1] + v2n[1] + v3n[1];
        let eq12 = v1n[2] + v2n[2] + v3n[2];

        SVector::from_row_slice(&[
            eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12,
        ])
    }

    fn compute_phis(&mut self, disk_position: Vector3<f64>) -> Option<SVector<f64, 9>> {
        // Compute the phi angles with a least square minimization of the system of equations
        self.p = SVector::from_row_slice(&[0., 1., 0., 1., 0., 1.]);

        self.set_thetas(disk_position);

        let lm = LevenbergMarquardt::new().with_xtol(f64::EPSILON);
        let (result, report) = lm.minimize(*self);

        if !report.termination.was_successful() {
            return None;
        }

        let cp1_n = result.p[0];
        let sp1_n = result.p[1];
        let cp2_n = result.p[2];
        let sp2_n = result.p[3];
        let cp3_n = result.p[4];
        let sp3_n = result.p[5];

        let mut phi1 = sp1_n.atan2(cp1_n);
        let mut phi2 = sp2_n.atan2(cp2_n);
        let mut phi3 = sp3_n.atan2(cp3_n);

        if !self.passiv_arms_direct {
            phi1 += std::f64::consts::PI;
            phi2 += std::f64::consts::PI;
            phi3 += std::f64::consts::PI;
        }

        Some(SVector::from_row_slice(&[
            sp1_n, cp1_n, sp2_n, cp2_n, sp3_n, cp3_n, phi1, phi2, phi3,
        ]))
    }
}

fn align_vectors(a: Matrix3<f64>, b: Matrix3<f64>) -> Rotation3<f64> {
    // Find the rotation matrix to align two sets of vectors (based on scipy implementation)
    let na = a.ref_ndarray2().into_shape((3, 3)).unwrap();
    let nb = b.ref_ndarray2().into_shape((3, 3)).unwrap();

    let mat_b = einsum("ji,jk->ik", &[&na, &nb])
        .unwrap()
        .into_shape((3, 3))
        .unwrap();

    let matrix_b = mat_b.view().into_nalgebra();

    let mat_svd = matrix_b.svd(true, true);
    let mut u = mat_svd.u.unwrap();
    let vh = mat_svd.v_t.unwrap();

    let uv = u.clone() * vh.clone();

    if uv.determinant() < 0.0 {
        u.set_column(
            2,
            &Vector3::from_row_slice(&[-u.column(2)[0], -u.column(2)[1], -u.column(2)[2]]),
        );
    }

    let mat_c = u * vh;

    let m = Matrix3::from_row_slice(&[
        mat_c.row(0)[0],
        mat_c.row(0)[1],
        mat_c.row(0)[2],
        mat_c.row(1)[0],
        mat_c.row(1)[1],
        mat_c.row(1)[2],
        mat_c.row(2)[0],
        mat_c.row(2)[1],
        mat_c.row(2)[2],
    ]);

    Rotation3::from_matrix_unchecked(m)
}

// We implement a trait for every problem we want to solve
impl LeastSquaresProblem<f64, U12, U6> for Orbita3dKinematicsModel {
    type ParameterStorage = Owned<f64, U6>;
    type ResidualStorage = Owned<f64, U12>;
    type JacobianStorage = Owned<f64, U12, U6>;

    fn set_params(&mut self, p: &SVector<f64, 6>) {
        self.p.copy_from(p);
        // do common calculations for residuals and the Jacobian here
    }

    fn params(&self) -> SVector<f64, 6> {
        self.p
    }

    fn residuals(&self) -> Option<SVector<f64, 12>> {
        let r = self.phis_system_equations();
        Some(r)
    }

    fn jacobian(&self) -> Option<SMatrix<f64, 12, 6>> {
        // Compute the Jacobian of the optimization problem ie.
        // the matrix for which each rows are the derivatives of each equation by a parameter

        let sa1 = self.alpha.sin();
        let sa2 = self.beta.sin();

        let ca1 = self.alpha.cos();
        let ca2 = self.beta.cos();

        let cp1 = self.p[0];
        let sp1 = self.p[1];

        let cp2 = self.p[2];
        let sp2 = self.p[3];

        let cp3 = self.p[4];
        let sp3 = self.p[5];

        let st1 = self.thetas[0].sin();
        let ct1 = self.thetas[0].cos();

        let st2 = self.thetas[1].sin();
        let ct2 = self.thetas[1].cos();

        let st3 = self.thetas[2].sin();
        let ct3 = self.thetas[2].cos();

        // derivative of all the eq by cp1
        let col1 = SVector::from_row_slice(&[
            ca1.powi(2) * cp2 * ct1 * ct2 * sa2.powi(2)
                + ca1.powi(2) * cp2 * sa2.powi(2) * st1 * st2
                - ca1 * ca2 * ct1 * ct2 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st1 * st2
                + ca1 * ca2 * sa1 * sa2
                + ca1 * ct1 * sa2.powi(2) * sp2 * st2
                - ca1 * ct2 * sa2.powi(2) * sp2 * st1
                + cp2 * sa1.powi(2) * sa2.powi(2),
            0.0,
            ca1.powi(2) * cp3 * ct1 * ct3 * sa2.powi(2)
                + ca1.powi(2) * cp3 * sa2.powi(2) * st1 * st3
                - ca1 * ca2 * ct1 * ct3 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st1 * st3
                + ca1 * ca2 * sa1 * sa2
                + ca1 * ct1 * sa2.powi(2) * sp3 * st3
                - ca1 * ct3 * sa2.powi(2) * sp3 * st1
                + cp3 * sa1.powi(2) * sa2.powi(2),
            -ca1 * ct1.powi(2) * sa1 * sa2 - ca1 * sa1 * sa2 * st1.powi(2) + ca1 * sa1 * sa2,
            0.0,
            0.0,
            2.0 * ca1 * ct1 * sa2 * (ca1 * cp1 * ct1 * sa2 - ca2 * ct1 * sa1 + sa2 * sp1 * st1)
                - 2.0
                    * ca1
                    * sa2
                    * st1
                    * (-ca1 * cp1 * sa2 * st1 + ca2 * sa1 * st1 + ct1 * sa2 * sp1)
                - 2.0 * sa1 * sa2 * (-ca1 * ca2 - cp1 * sa1 * sa2),
            0.0,
            0.0,
            -ca1 * sa2 * st1,
            ca1 * ct1 * sa2,
            -sa1 * sa2,
        ]);

        // derivation by sp1
        let col2 = SVector::from_row_slice(&[
            -ca1 * cp2 * ct1 * sa2.powi(2) * st2
                + ca1 * cp2 * ct2 * sa2.powi(2) * st1
                + ca2 * ct1 * sa1 * sa2 * st2
                - ca2 * ct2 * sa1 * sa2 * st1
                + ct1 * ct2 * sa2.powi(2) * sp2
                + sa2.powi(2) * sp2 * st1 * st2,
            0.0,
            -ca1 * cp3 * ct1 * sa2.powi(2) * st3
                + ca1 * cp3 * ct3 * sa2.powi(2) * st1
                + ca2 * ct1 * sa1 * sa2 * st3
                - ca2 * ct3 * sa1 * sa2 * st1
                + ct1 * ct3 * sa2.powi(2) * sp3
                + sa2.powi(2) * sp3 * st1 * st3,
            0.0,
            0.0,
            0.0,
            2.0 * ct1 * sa2 * (-ca1 * cp1 * sa2 * st1 + ca2 * sa1 * st1 + ct1 * sa2 * sp1)
                + 2.0 * sa2 * st1 * (ca1 * cp1 * ct1 * sa2 - ca2 * ct1 * sa1 + sa2 * sp1 * st1),
            0.0,
            0.0,
            ct1 * sa2,
            sa2 * st1,
            0.0,
        ]);

        // derivation by cp2
        let col3 = SVector::from_row_slice(&[
            ca1.powi(2) * cp1 * ct1 * ct2 * sa2.powi(2)
                + ca1.powi(2) * cp1 * sa2.powi(2) * st1 * st2
                - ca1 * ca2 * ct1 * ct2 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st1 * st2
                + ca1 * ca2 * sa1 * sa2
                - ca1 * ct1 * sa2.powi(2) * sp1 * st2
                + ca1 * ct2 * sa2.powi(2) * sp1 * st1
                + cp1 * sa1.powi(2) * sa2.powi(2),
            ca1.powi(2) * cp3 * ct2 * ct3 * sa2.powi(2)
                + ca1.powi(2) * cp3 * sa2.powi(2) * st2 * st3
                - ca1 * ca2 * ct2 * ct3 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st2 * st3
                + ca1 * ca2 * sa1 * sa2
                + ca1 * ct2 * sa2.powi(2) * sp3 * st3
                - ca1 * ct3 * sa2.powi(2) * sp3 * st2
                + cp3 * sa1.powi(2) * sa2.powi(2),
            0.0,
            0.0,
            -ca1 * ct2.powi(2) * sa1 * sa2 - ca1 * sa1 * sa2 * st2.powi(2) + ca1 * sa1 * sa2,
            0.0,
            0.0,
            2.0 * ca1 * ct2 * sa2 * (ca1 * cp2 * ct2 * sa2 - ca2 * ct2 * sa1 + sa2 * sp2 * st2)
                - 2.0
                    * ca1
                    * sa2
                    * st2
                    * (-ca1 * cp2 * sa2 * st2 + ca2 * sa1 * st2 + ct2 * sa2 * sp2)
                - 2.0 * sa1 * sa2 * (-ca1 * ca2 - cp2 * sa1 * sa2),
            0.0,
            -ca1 * sa2 * st2,
            ca1 * ct2 * sa2,
            -sa1 * sa2,
        ]);

        // derivative by sp2
        let col4 = SVector::from_row_slice(&[
            ca1 * cp1 * ct1 * sa2.powi(2) * st2
                - ca1 * cp1 * ct2 * sa2.powi(2) * st1
                - ca2 * ct1 * sa1 * sa2 * st2
                + ca2 * ct2 * sa1 * sa2 * st1
                + ct1 * ct2 * sa2.powi(2) * sp1
                + sa2.powi(2) * sp1 * st1 * st2,
            -ca1 * cp3 * ct2 * sa2.powi(2) * st3
                + ca1 * cp3 * ct3 * sa2.powi(2) * st2
                + ca2 * ct2 * sa1 * sa2 * st3
                - ca2 * ct3 * sa1 * sa2 * st2
                + ct2 * ct3 * sa2.powi(2) * sp3
                + sa2.powi(2) * sp3 * st2 * st3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 * ct2 * sa2 * (-ca1 * cp2 * sa2 * st2 + ca2 * sa1 * st2 + ct2 * sa2 * sp2)
                + 2.0 * sa2 * st2 * (ca1 * cp2 * ct2 * sa2 - ca2 * ct2 * sa1 + sa2 * sp2 * st2),
            0.0,
            ct2 * sa2,
            sa2 * st2,
            0.0,
        ]);

        // derivation by cp3
        let col5 = SVector::from_row_slice(&[
            0.0,
            ca1.powi(2) * cp2 * ct2 * ct3 * sa2.powi(2)
                + ca1.powi(2) * cp2 * sa2.powi(2) * st2 * st3
                - ca1 * ca2 * ct2 * ct3 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st2 * st3
                + ca1 * ca2 * sa1 * sa2
                - ca1 * ct2 * sa2.powi(2) * sp2 * st3
                + ca1 * ct3 * sa2.powi(2) * sp2 * st2
                + cp2 * sa1.powi(2) * sa2.powi(2),
            ca1.powi(2) * cp1 * ct1 * ct3 * sa2.powi(2)
                + ca1.powi(2) * cp1 * sa2.powi(2) * st1 * st3
                - ca1 * ca2 * ct1 * ct3 * sa1 * sa2
                - ca1 * ca2 * sa1 * sa2 * st1 * st3
                + ca1 * ca2 * sa1 * sa2
                - ca1 * ct1 * sa2.powi(2) * sp1 * st3
                + ca1 * ct3 * sa2.powi(2) * sp1 * st1
                + cp1 * sa1.powi(2) * sa2.powi(2),
            0.0,
            0.0,
            -ca1 * ct3.powi(2) * sa1 * sa2 - ca1 * sa1 * sa2 * st3.powi(2) + ca1 * sa1 * sa2,
            0.0,
            0.0,
            2.0 * ca1 * ct3 * sa2 * (ca1 * cp3 * ct3 * sa2 - ca2 * ct3 * sa1 + sa2 * sp3 * st3)
                - 2.0
                    * ca1
                    * sa2
                    * st3
                    * (-ca1 * cp3 * sa2 * st3 + ca2 * sa1 * st3 + ct3 * sa2 * sp3)
                - 2.0 * sa1 * sa2 * (-ca1 * ca2 - cp3 * sa1 * sa2),
            -ca1 * sa2 * st3,
            ca1 * ct3 * sa2,
            -sa1 * sa2,
        ]);

        // derivative by sp3
        let col6 = SVector::from_row_slice(&[
            0.0,
            ca1 * cp2 * ct2 * sa2.powi(2) * st3
                - ca1 * cp2 * ct3 * sa2.powi(2) * st2
                - ca2 * ct2 * sa1 * sa2 * st3
                + ca2 * ct3 * sa1 * sa2 * st2
                + ct2 * ct3 * sa2.powi(2) * sp2
                + sa2.powi(2) * sp2 * st2 * st3,
            ca1 * cp1 * ct1 * sa2.powi(2) * st3
                - ca1 * cp1 * ct3 * sa2.powi(2) * st1
                - ca2 * ct1 * sa1 * sa2 * st3
                + ca2 * ct3 * sa1 * sa2 * st1
                + ct1 * ct3 * sa2.powi(2) * sp1
                + sa2.powi(2) * sp1 * st1 * st3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 * ct3 * sa2 * (-ca1 * cp3 * sa2 * st3 + ca2 * sa1 * st3 + ct3 * sa2 * sp3)
                + 2.0 * sa2 * st3 * (ca1 * cp3 * ct3 * sa2 - ca2 * ct3 * sa1 + sa2 * sp3 * st3),
            ct3 * sa2,
            sa2 * st3,
            0.0,
        ]);

        let j = SMatrix::from_columns(&[col1, col2, col3, col4, col5, col6]);

        Some(j)
    }
}

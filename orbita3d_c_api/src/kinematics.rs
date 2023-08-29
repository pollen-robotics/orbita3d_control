//! C API for the kinematics module of the Orbita3D library.

use orbita3d_kinematics::conversion;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
/// Kinematics model for Orbita3d
pub struct Orbita3dKinematicsModel {
    pub alpha: f64,
    pub gamma_min: f64,
    pub offset: f64,
    pub beta: f64,
    pub gamma_max: f64,
    pub passiv_arms_direct: bool,
}

impl Orbita3dKinematicsModel {
    #[no_mangle]
    /// Create a new Orbita3dKinematicsModel.
    pub extern "C" fn create_orbita3d_kinematics_model(
        alpha: f64,
        gamma_min: f64,
        offset: f64,
        beta: f64,
        gamma_max: f64,
        passiv_arms_direct: bool,
    ) -> Self {
        Orbita3dKinematicsModel {
            alpha,
            gamma_min,
            offset,
            beta,
            gamma_max,
            passiv_arms_direct,
        }
    }

    #[no_mangle]
    /// Compute the forward position kinematics.
    ///
    /// # Arguments
    /// * thetas - The motor angles as a 3-element array.
    /// * quat - Holder for the platform orientation as a quaternion result.
    /// # Returns
    /// * 0 if success, 1 if error.
    pub extern "C" fn orbita3d_kinematics_forward_position(
        &self,
        thetas: &[f64; 3],
        quat: &mut [f64; 4],
    ) -> i32 {
        *quat = conversion::rotation_matrix_to_quaternion(
            self.inner().compute_forward_kinematics(*thetas),
        );
        0
    }

    #[no_mangle]
    /// Compute the forward velocity.
    ///
    /// # Arguments
    /// * thetas - The motor angles as a 3-element array.
    /// * thetas_velocity - The motor velocities as a 3-element array.
    /// * output_velocity - Holder for the platform orientation velocity as a velocity pseudo vector.
    /// # Returns
    /// * 0 if success, 1 if error.
    pub extern "C" fn orbita3d_kinematics_forward_velocity(
        &self,
        thetas: &[f64; 3],
        thetas_velocity: &[f64; 3],
        output_velocity: &mut [f64; 3],
    ) -> i32 {
        let vel = self
            .inner()
            .compute_output_velocity(*thetas, *thetas_velocity);
        *output_velocity = vel.into();

        0
    }

    #[no_mangle]
    /// Compute the forward torque.
    ///
    /// # Arguments
    /// * thetas - The motor angles as a 3-element array.
    /// * thetas_torque - The motor torques as a 3-element array.
    /// * output_torque - Holder for the platform orientation torque as a pseudo vector result.
    /// # Returns
    /// * 0 if success, 1 if error.
    pub extern "C" fn orbita3d_kinematics_forward_torque(
        &self,
        thetas: &[f64; 3],
        thetas_torque: &[f64; 3],
        output_torque: &mut [f64; 3],
    ) -> i32 {
        let rot = self.inner().compute_output_torque(*thetas, *thetas_torque);
        *output_torque = rot.into();

        0
    }

    #[no_mangle]
    /// Compute the inverse position kinematics.
    ///
    /// # Arguments
    /// * quat - The platform orientation as a quaternion.
    /// * thetas - Holder for the motor angles as a 3-element array result.
    /// # Returns
    /// * 0 if success, 1 if no solution, 2 if invalid solution.
    pub extern "C" fn orbita3d_kinematics_inverse_position(
        &self,
        quat: &[f64; 4],
        thetas: &mut [f64; 3],
    ) -> i32 {
        match self
            .inner()
            .compute_inverse_kinematics(conversion::quaternion_to_rotation_matrix(
                quat[0], quat[1], quat[2], quat[3],
            )) {
            Ok(sol) => {
                *thetas = sol;
                0
            }
            Err(err) => match err {
                orbita3d_kinematics::InverseSolutionErrorKind::NoSolution(_) => 1,
                orbita3d_kinematics::InverseSolutionErrorKind::InvalidSolution(_, _) => 2,
            },
        }
    }

    #[no_mangle]
    /// Compute the inverse velocity.
    ///
    /// # Arguments
    /// * thetas - The motor angles as a 3-element array.
    /// * output_velocity - The platform orientation velocity as a velocity pseudo vector.
    /// * thetas_velocity - Holder for the motor velocities as a 3-element array result.
    /// # Returns
    /// * 0 if success, 1 if error.
    pub extern "C" fn orbita3d_kinematics_inverse_velocity(
        &self,
        thetas: &[f64; 3],
        output_velocity: &[f64; 3],
        thetas_velocity: &mut [f64; 3],
    ) -> i32 {
        *thetas_velocity = self
            .inner()
            .compute_input_velocity(*thetas, conversion::array_to_vector3(*output_velocity));

        0
    }

    #[no_mangle]
    /// Compute the inverse torque.
    ///
    /// # Arguments
    /// * thetas - The motor angles as a 3-element array.
    /// * output_torque - The platform orientation torque as a pseudo vector.
    /// * thetas_torque - Holder for the motor torques as a 3-element array result.
    /// # Returns
    /// * 0 if success, 1 if error.
    pub extern "C" fn orbita3d_kinematics_inverse_torque(
        &self,
        thetas: &[f64; 3],
        output_torque: &[f64; 3],
        thetas_torque: &mut [f64; 3],
    ) -> i32 {
        *thetas_torque = self
            .inner()
            .compute_input_torque(*thetas, conversion::array_to_vector3(*output_torque));

        0
    }

    fn inner(&self) -> orbita3d_kinematics::Orbita3dKinematicsModel {
        orbita3d_kinematics::Orbita3dKinematicsModel {
            alpha: self.alpha,
            gamma_min: self.gamma_min,
            offset: self.offset,
            beta: self.beta,
            gamma_max: self.gamma_max,
            passiv_arms_direct: self.passiv_arms_direct,
        }
    }
}

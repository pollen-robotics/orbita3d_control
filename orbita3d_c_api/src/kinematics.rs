use orbita3d_kinematics::conversion;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
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
    pub extern "C" fn orbita3d_kinematics_forward_velocity(
        &self,
        thetas: &[f64; 3],
        thetas_velocity: &[f64; 3],
        quat_velocity: &mut [f64; 4],
    ) -> i32 {
        let rot = self
            .inner()
            .compute_output_velocity_from_disks(*thetas, *thetas_velocity);
        *quat_velocity = conversion::rotation_matrix_to_quaternion(rot);

        0
    }

    #[no_mangle]
    pub extern "C" fn orbita3d_kinematics_forward_torque(
        &self,
        thetas: &[f64; 3],
        thetas_torque: &[f64; 3],
        quat_torque: &mut [f64; 4],
    ) -> i32 {
        let rot = self
            .inner()
            .compute_output_torque_from_disks(*thetas, *thetas_torque);
        *quat_torque = conversion::rotation_matrix_to_quaternion(rot);

        0
    }

    #[no_mangle]
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
            Err(_) => 1,
        }
    }

    #[no_mangle]
    pub extern "C" fn orbita3d_kinematics_inverse_velocity(
        &self,
        thetas: &[f64; 3],
        quat_velocity: &[f64; 4],
        thetas_velocity: &mut [f64; 3],
    ) -> i32 {
        let output_velocity = conversion::quaternion_to_rotation_matrix(
            quat_velocity[0],
            quat_velocity[1],
            quat_velocity[2],
            quat_velocity[3],
        );
        *thetas_velocity = self
            .inner()
            .compute_input_velocity_from_disks(*thetas, output_velocity);

        0
    }

    #[no_mangle]
    pub extern "C" fn orbita3d_kinematics_inverse_torque(
        &self,
        thetas: &[f64; 3],
        quat_torque: &[f64; 4],
        thetas_torque: &mut [f64; 3],
    ) -> i32 {
        let output_torque = conversion::quaternion_to_rotation_matrix(
            quat_torque[0],
            quat_torque[1],
            quat_torque[2],
            quat_torque[3],
        );
        *thetas_torque = self
            .inner()
            .compute_input_torque_from_disks(*thetas, output_torque);

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

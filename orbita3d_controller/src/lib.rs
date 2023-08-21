//! Orbita3dController
//!
//! This crate provides a controller for the Orbita3d actuator.
//!
//! # Overview
//!
//! ## Setup
//! - [x] Load configuration from a file
//! - [x] Support for different communication layers (Dynamixel like serial, Fake)
//!
//! ## Control
//! - [x] Enable/Disable torque
//! - [x] Read the current orientation as quaternion (position, velocity, torque)
//! - [x] Set the target orientation as quaternion (position)
//! - [x] Extra raw motors parameters (velocity limit, torque limit, pid gains)
//!
//! ## Communication
//! - [x] Fake motors
//! - [x] Dynamixel like serial
//! - [ ] EtherCAT communication
//!
//! # Examples
//! ```rust
//! use orbita3d_controller::Orbita3dController;
//!
//! let mut orbita3d = Orbita3dController::with_config("./config/fake.yaml").unwrap();
//!
//! let orientation = orbita3d.get_current_orientation().unwrap();
//! println!("Current orientation: {:?}", orientation);
//!
//! orbita3d.enable_torque(true).unwrap();
//!
//! let target = [0.0, 0.0, 0.7, 0.7];
//! orbita3d.set_target_orientation(target).unwrap();
//!
//! let orientation = orbita3d.get_current_orientation().unwrap();
//! println!("Current orientation: {:?}", orientation);
//! ```

pub mod io;
use io::{DynamixelSerialController, Orbita3dIOConfig};
use motor_toolbox_rs::{FakeMotorsController, MotorsController, Result, PID};
use orbita3d_kinematics::{conversion, Orbita3dKinematicsModel};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
/// Orbita3d Config
pub struct Orbita3dConfig {
    /// IO specific config
    pub io: Orbita3dIOConfig,
    /// Disks specific config
    pub disks: DisksConfig,
    /// Kinematics model config
    pub kinematics_model: Orbita3dKinematicsModel,
}

#[derive(Debug, Deserialize, Serialize)]
/// Disks Config
pub struct DisksConfig {
    /// Zeros of each disk (in rad), used as an offset
    pub zeros: [f64; 3],
    /// Reduction between the motor and the disk
    pub reduction: f64,
}

/// Orbita3d Controller
pub struct Orbita3dController {
    inner: Box<dyn MotorsController<3>>,
    kinematics: Orbita3dKinematicsModel,
}

impl Orbita3dController {
    /// Creates a new Orbita3dController using the given configuration file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        let f = std::fs::File::open(configfile)?;
        let config: Orbita3dConfig = serde_yaml::from_reader(f)?;

        let controller: Box<dyn MotorsController<3>> = match config.io {
            Orbita3dIOConfig::DynamixelSerial(dxl_config) => {
                let controller = DynamixelSerialController::new(
                    &dxl_config.serial_port,
                    dxl_config.id,
                    config.disks.zeros,
                    config.disks.reduction,
                )?;

                Box::new(controller)
            }
            Orbita3dIOConfig::FakeMotors(_) => {
                let controller = FakeMotorsController::<3>::new()
                    .with_offsets(config.disks.zeros.map(Some))
                    .with_reduction([Some(config.disks.reduction); 3]);

                Box::new(controller)
            }
        };

        Ok(Self {
            inner: controller,
            kinematics: config.kinematics_model,
        })
    }
}

impl Orbita3dController {
    /// Check if the torque is ON or OFF
    pub fn is_torque_on(&mut self) -> Result<bool> {
        let torques = self.inner.is_torque_on()?;
        assert!(torques.iter().all(|&t| t == torques[0]));
        Ok(torques[0])
    }
    /// Enable the torque
    ///
    /// # Arguments
    /// * reset_target: if true, the target position will be reset to the current position
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        if reset_target {
            let thetas = self.inner.get_current_position()?;
            self.inner.set_target_position(thetas)?;
        }
        self.inner.set_torque([true; 3])
    }
    /// Disable the torque
    pub fn disable_torque(&mut self) -> Result<()> {
        self.inner.set_torque([false; 3])
    }

    /// Get the current orientation (as quaternion (qx, qy, qz, qw))
    pub fn get_current_orientation(&mut self) -> Result<[f64; 4]> {
        let thetas = self.inner.get_current_position()?;
        let rot = self.kinematics.compute_forward_kinematics(thetas);
        Ok(conversion::rotation_matrix_to_quaternion(rot))
    }
    /// Get the current velocity (as quaternion (qx, qy, qz, qw))
    pub fn get_current_velocity(&mut self) -> Result<[f64; 4]> {
        let thetas = self.inner.get_current_position()?;
        let input_velocity = self.inner.get_current_velocity()?;

        let rot = self
            .kinematics
            .compute_output_velocity(thetas, input_velocity);
        Ok(conversion::rotation_matrix_to_quaternion(rot))
    }
    /// Get the current torque (as quaternion (qx, qy, qz, qw))
    pub fn get_current_torque(&mut self) -> Result<[f64; 4]> {
        let thetas = self.inner.get_current_position()?;
        let input_torque = self.inner.get_current_torque()?;

        let rot = self.kinematics.compute_output_torque(thetas, input_torque);
        Ok(conversion::rotation_matrix_to_quaternion(rot))
    }

    /// Get the target orientation (as quaternion (qx, qy, qz, qw))
    pub fn get_target_orientation(&mut self) -> Result<[f64; 4]> {
        let thetas = self.inner.get_target_position()?;
        let rot = self.kinematics.compute_forward_kinematics(thetas);
        Ok(conversion::rotation_matrix_to_quaternion(rot))
    }
    /// Set the target orientation (as quaternion (qx, qy, qz, qw))
    pub fn set_target_orientation(&mut self, target: [f64; 4]) -> Result<()> {
        let rot =
            conversion::quaternion_to_rotation_matrix(target[0], target[1], target[2], target[3]);
        let thetas = self.kinematics.compute_inverse_kinematics(rot)?;
        self.inner.set_target_position(thetas)
    }

    /// Get the velocity limit of each raw motor (in rad/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit orbita3d orientation!
    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<[f64; 3]> {
        self.inner.get_velocity_limit()
    }
    /// Set the velocity limit of each raw motor (in rad/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit orbita3d orientation!
    pub fn set_raw_motors_velocity_limit(&mut self, limit: [f64; 3]) -> Result<()> {
        self.inner.set_velocity_limit(limit)
    }
    /// Get the torque limit of each raw motor (in N.m)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit orbita3d orientation!
    pub fn get_raw_motors_torque_limit(&mut self) -> Result<[f64; 3]> {
        self.inner.get_torque_limit()
    }
    /// Set the torque limit of each raw motor (in N.m)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit orbita3d orientation!
    pub fn set_raw_motors_torque_limit(&mut self, limit: [f64; 3]) -> Result<()> {
        self.inner.set_torque_limit(limit)
    }
    /// Get the pid gains of each raw motor
    /// caution: this is the raw value used by the motors used inside the actuator, not on orbita3d orientation!
    pub fn get_raw_motors_pid_gains(&mut self) -> Result<[PID; 3]> {
        self.inner.get_pid_gains()
    }
    /// Set the pid gains of each raw motor
    /// caution: this is the raw value used by the motors used inside the actuator, not on orbita3d orientation!
    pub fn set_raw_motors_pid_gains(&mut self, gains: [PID; 3]) -> Result<()> {
        self.inner.set_pid_gains(gains)
    }
}

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
//! use orbita3d_controller::{FakeConfig, Orbita3dController};
//! use orbita3d_kinematics::Orbita3dKinematicsModel;
//!
//! let mut orbita3d = Orbita3dController::with_fake_motors(
//!     FakeConfig { kinematics_model: Orbita3dKinematicsModel::default() }
//! );
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

mod dynamixel_serial;
use dynamixel_serial::DynamixelSerialConfig;
mod fake;
pub use fake::FakeConfig;
use motor_toolbox_rs::{MultipleMotorsController, PID};
use orbita3d_kinematics::{conversion, Orbita3dKinematicsModel};
use serde::{Deserialize, Serialize};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Debug)]
pub struct Orbita3dController {
    inner: Box<dyn MultipleMotorsController<3>>,
    kinematics: Orbita3dKinematicsModel,
}

#[derive(Debug, Deserialize, Serialize)]
pub enum Orbita3dConfig {
    DynamixelSerial(DynamixelSerialConfig),
    FakeMotors(FakeConfig),
}

impl Orbita3dController {
    /// Creates a new Orbita3dController using the given configuration file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        let f = std::fs::File::open(configfile)?;
        let config: Orbita3dConfig = serde_yaml::from_reader(f)?;

        match config {
            Orbita3dConfig::DynamixelSerial(dxl_config) => Self::with_dynamixel_serial(dxl_config),
            Orbita3dConfig::FakeMotors(fake_config) => Ok(Self::with_fake_motors(fake_config)),
        }
    }
}

impl Orbita3dController {
    /// Check if the torque is ON or OFF
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.inner.is_torque_on()
    }
    /// Enable the torque
    ///
    /// # Arguments
    /// * reset_target: if true, the target position will be reset to the current position
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        self.inner.enable_torque(reset_target)
    }
    /// Disable the torque
    pub fn disable_torque(&mut self) -> Result<()> {
        self.inner.disable_torque()
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

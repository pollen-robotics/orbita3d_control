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
//! - [x] Poulpe serial dynamixel
//! - [x] EtherCAT communication
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
use io::{CachedDynamixelSerialController, DynamixelSerialController, Orbita3dIOConfig};
use motor_toolbox_rs::{FakeMotorsController, MotorsController, Result, PID};

use orbita3d_kinematics::{conversion, Orbita3dKinematicsModel};
use serde::{Deserialize, Serialize};
use std::{thread, time::Duration};

use crate::io::{
    CachedDynamixelPoulpeController, DynamixelPoulpeController, EthercatPoulpeController,
};

#[derive(Debug, Deserialize, Serialize)]
/// Orbita3d Config
pub struct Orbita3dConfig {
    /// IO specific config
    pub io: Orbita3dIOConfig,
    /// Disks specific config
    pub disks: DisksConfig,
    /// Kinematics model config
    pub kinematics_model: Orbita3dKinematicsModel,
    pub inverted_axes: [Option<bool>; 3],
}

#[derive(Debug, Deserialize, Serialize)]
/// Disks Config
pub struct DisksConfig {
    /// Zeros of each disk (in rad), used as an offset
    pub zeros: ZeroType,
    /// Reduction between the motor gearbox and the disk
    pub reduction: f64,
}

#[derive(Debug, Deserialize, Serialize)]
/// Zero type config
/// This is used to configure the zero of each disk
pub enum ZeroType {
    /// ApproximateHardwareZero config
    ApproximateHardwareZero(ApproximateHardwareZero),
    /// ZeroStartup config
    ZeroStartup(ZeroStartup),
    /// HallZero config
    HallZero(HallZero),
    /// FirmwareZero config
    FirmwareZero(FirmwareZero),
}

#[derive(Debug, Deserialize, Serialize)]
/// Firmwarezero config
pub struct FirmwareZero;

#[derive(Debug, Deserialize, Serialize)]
/// ApproximateHardwareZero config
pub struct ApproximateHardwareZero {
    /// Hardware zero of each disk (in rad)
    pub hardware_zero: [f64; 3],
}

#[derive(Debug, Deserialize, Serialize)]
/// HallZero config
pub struct HallZero {
    /// Hardware zero of each disk (in rad)
    pub hardware_zero: [f64; 3],

    /// Top/Middle/Bottom Hall active for the zero position
    pub hall_indice: [u8; 3],
}

#[derive(Debug, Deserialize, Serialize)]
/// ZeroStartup config
pub struct ZeroStartup;

/// Orbita3d Controller
pub struct Orbita3dController {
    inner: Box<dyn MotorsController<3> + Send>,
    pub kinematics: Orbita3dKinematicsModel,
}

#[derive(Debug, Deserialize, Serialize, Copy, Clone)]
/// Feedback struct
pub struct Orbita3dFeedback {
    pub orientation: [f64; 4],
}

impl Orbita3dController {
    /// Creates a new Orbita3dController using the given configuration file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        log::info!("Loading config file: {}", configfile);

        let f = match std::fs::File::open(configfile) {
            Ok(f) => f,
            Err(e) => {
                log::error!("Error opening config file: {}", e);
                return Err(e.into());
            }
        };
        let config: Orbita3dConfig = serde_yaml::from_reader(f)?;

        let controller: Box<dyn MotorsController<3> + Send> = match config.io {
            // This is a legacy mode, not maintained anymore
            Orbita3dIOConfig::DynamixelSerial(dxl_config) => match dxl_config.use_cache {
                true => {
                    let controller = CachedDynamixelSerialController::new(
                        &dxl_config.serial_port,
                        dxl_config.id,
                        config.disks.zeros,
                        config.disks.reduction,
                    )?;

                    log::info!("Using cached dynamixel controller {:?}", controller);

                    Box::new(controller)
                }
                false => {
                    let controller = DynamixelSerialController::new(
                        &dxl_config.serial_port,
                        dxl_config.id,
                        config.disks.zeros,
                        config.disks.reduction,
                    )?;

                    log::info!("Using dynamixel controller {:?}", controller);

                    Box::new(controller)
                }
            },
            // This is a legacy mode, not maintained anymore
            Orbita3dIOConfig::DynamixelPoulpe(dxl_config) => match dxl_config.use_cache {
                true => {
                    let controller = CachedDynamixelPoulpeController::new(
                        &dxl_config.serial_port,
                        dxl_config.id,
                        config.disks.zeros,
                        config.disks.reduction,
                    )?;

                    log::info!("Using cached poulpe dynamixel controller {:?}", controller);

                    Box::new(controller)
                }
                false => {
                    let controller = DynamixelPoulpeController::new(
                        &dxl_config.serial_port,
                        dxl_config.id,
                        config.disks.zeros,
                        config.disks.reduction,
                    )?;

                    log::info!("Using poulpe dynamixel controller {:?}", controller);

                    Box::new(controller)
                }
            },
            // The fake mode is heavily used for testing and debugging, it allows to run everything without any hardware
            Orbita3dIOConfig::FakeMotors(_) => {
                let mut controller = FakeMotorsController::<3>::new();

                let offsets = match config.disks.zeros {
                    ZeroType::ApproximateHardwareZero(zero) => zero.hardware_zero,
                    ZeroType::ZeroStartup(_) => controller.get_current_position()?,
                    ZeroType::HallZero(zero) => zero.hardware_zero,
                    ZeroType::FirmwareZero(_) => [0.0, 0.0, 0.0],
                };

                let controller = controller
                    .with_offsets(offsets.map(Some))
                    .with_reduction([Some(config.disks.reduction); 3])
                    .with_inverted_axes(config.inverted_axes);

                log::info!("Using fake motors controller {:?}", controller);

                Box::new(controller)
            }
            // The ethercat mode with the "Poulpe" electronics
            Orbita3dIOConfig::PoulpeEthercat(ethercat_config) => {
                let controller = EthercatPoulpeController::new(
                    &ethercat_config.url,
                    ethercat_config.id,
                    ethercat_config.name,
                    config.disks.zeros,
                    config.disks.reduction,
                    config.inverted_axes,
                )?;
                log::info!("Using poulpe ethercat controller {:?}", controller);

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
        if !self.is_torque_on()? && reset_target {
            let thetas = self.inner.get_current_position()?;
            thread::sleep(Duration::from_millis(1));
            self.inner.set_target_position(thetas)?;
            thread::sleep(Duration::from_millis(1));
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

    /// Get the current orientation (as intrinsic rpy with multiturn)
    pub fn get_current_rpy_orientation(&mut self) -> Result<[f64; 3]> {
        let thetas = self.inner.get_current_position()?;
        let inverted_axes = self.inner.output_inverted_axes();
        let mut rpy = self
            .kinematics
            .compute_forward_kinematics_rpy_multiturn(thetas)?;

        for i in 0..3 {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    rpy[i] = -rpy[i];
                }
            }
        }

        Ok(rpy)
    }

    /// Get the current velocity $\omega$ (as a velocity pseudo vector (wx, wy, wz) which defines the instantaneous axis of rotation and with the norm represents the velocity)
    pub fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        let thetas = self.inner.get_current_position()?;
        let input_velocity = self.inner.get_current_velocity()?;

        let rot = self
            .kinematics
            .compute_output_velocity(thetas, input_velocity);
        Ok(rot.into())
    }
    /// Get the current torque (as pseudo vector)
    pub fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        let thetas = self.inner.get_current_position()?;
        let input_torque = self.inner.get_current_torque()?;

        Ok(self
            .kinematics
            .compute_output_torque(thetas, input_torque)
            .into())
    }

    /// Get the target orientation (as quaternion (qx, qy, qz, qw))
    pub fn get_target_orientation(&mut self) -> Result<[f64; 4]> {
        let thetas = self.inner.get_target_position()?;
        let rot = self.kinematics.compute_forward_kinematics(thetas);
        Ok(conversion::rotation_matrix_to_quaternion(rot))
    }

    /// Get the target orientation (as intrinsic rpy with multiturn)
    pub fn get_target_rpy_orientation(&mut self) -> Result<[f64; 3]> {
        let thetas = self.inner.get_target_position()?;
        let rpy = self
            .kinematics
            .compute_forward_kinematics_rpy_multiturn(thetas)?;
        Ok(rpy)
    }

    /// Set the target orientation (as quaternion (qx, qy, qz, qw))
    pub fn set_target_orientation(&mut self, target: [f64; 4]) -> Result<()> {
        let rot =
            conversion::quaternion_to_rotation_matrix(target[0], target[1], target[2], target[3]);
        let thetas = self.kinematics.compute_inverse_kinematics(rot)?;
        self.inner.set_target_position(thetas)
    }

    /// Set the target orientation fro the roll pitch yaw intrinsic angles (taking multi turn into account)
    pub fn set_target_rpy_orientation(&mut self, target: [f64; 3]) -> Result<()> {
        let inverted_axes = self.inner.output_inverted_axes();
        let mut rpy_target = target;

        for i in 0..3 {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    rpy_target[i] = -rpy_target[i];
                }
            }
        }

        let thetas = self
            .kinematics
            .compute_inverse_kinematics_rpy_multiturn(rpy_target)?;

        self.inner.set_target_position(thetas)
    }

    /// Set the target orientation (as quaternion (qx, qy, qz, qw)) with feedback
    pub fn set_target_orientation_fb(&mut self, target: [f64; 4]) -> Result<Orbita3dFeedback> {
        let rot =
            conversion::quaternion_to_rotation_matrix(target[0], target[1], target[2], target[3]);

        let thetas = self.kinematics.compute_inverse_kinematics(rot)?;

        let fb: Result<[f64; 3]> = self.inner.set_target_position_fb(thetas);
        match fb {
            Ok(fb) => {
                let rot = self
                    .kinematics
                    .compute_forward_kinematics([fb[0], fb[1], fb[2]]); //Why the f*ck can't I use slice here?
                                                                        // let vel = self
                                                                        //     .kinematics
                                                                        //     .compute_output_velocity(thetas, [fb[3], fb[4], fb[5]]);
                                                                        // let torque = self
                                                                        //     .kinematics
                                                                        //     .compute_output_torque(thetas, [fb[6], fb[7], fb[8]]);

                Ok(Orbita3dFeedback {
                    orientation: conversion::rotation_matrix_to_quaternion(rot),
                    // velocity: [vel[0], vel[1], vel[2]],
                    // torque: [torque[0], torque[1], torque[2]],
                })
            }
            Err(e) => Err(e),
        }
    }

    /// Set the target orientation from roll pitch yaw (accepts yaw>180°) intrinsic angles with feedback => returns feedback rpy
    pub fn set_target_rpy_orientation_fb(&mut self, target: [f64; 3]) -> Result<[f64; 3]> {
        let inverted_axes = self.inner.output_inverted_axes();
        let mut rpy_target = target;

        for i in 0..3 {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    rpy_target[i] = -rpy_target[i];
                }
            }
        }

        let thetas = self
            .kinematics
            .compute_inverse_kinematics_rpy_multiturn(rpy_target)?;
        let fb: Result<[f64; 3]> = self.inner.set_target_position_fb(thetas);

        match fb {
            Ok(fb) => {
                let mut rpy = self
                    .kinematics
                    .compute_forward_kinematics_rpy_multiturn([fb[0], fb[1], fb[2]])?;
                for i in 0..3 {
                    if let Some(inverted) = inverted_axes[i] {
                        if inverted {
                            rpy[i] = -rpy[i];
                        }
                    }
                }
                Ok(rpy)
            }

            Err(e) => Err(e),
        }
    }

    /// Get the position of each raw motor (in rad)
    /// caution: this is the raw value used by the motors used inside the actuator, not the orbita3d orientation!
    pub fn get_raw_motors_positions(&mut self) -> Result<[f64; 3]> {
        let red = self.inner.reduction();
        match self.inner.get_current_position(){
            Ok(p) => {
                let mut pos = p;
                for i in 0..3 { pos[i] = pos[i] * red[i].unwrap();}
                Ok(pos)
            }
            Err(e) => Err(e)
        }
    }

    /// Get the velocity limit of each raw motor (in rad/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit to orbita3d orientation!
    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<[f64; 3]> {
        self.inner.get_velocity_limit()
    }
    /// Set the velocity limit of each raw motor (in rad/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit to orbita3d orientation!
    pub fn set_raw_motors_velocity_limit(&mut self, limit: [f64; 3]) -> Result<()> {
        self.inner.set_velocity_limit(limit)
    }
    /// Get the torque limit of each raw motor (in N.m)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit to orbita3d orientation!
    pub fn get_raw_motors_torque_limit(&mut self) -> Result<[f64; 3]> {
        self.inner.get_torque_limit()
    }
    /// Set the torque limit of each raw motor (in N.m)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit to orbita3d orientation!
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

    /// Get the raw motors velocity in rad/s (top, middle, bottom)
    pub fn get_raw_motors_velocity(&mut self) -> Result<[f64; 3]> {
        let red = self.inner.reduction();
        match self.inner.get_current_velocity(){
            Ok(v) => {
                let mut vel = v;
                for i in 0..3 { vel[i] = vel[i] * red[i].unwrap();}
                Ok(vel)
            }
            Err(e) => Err(e)
        }
    }
    /// Get the raw motors current in mA (top, middle, bottom)
    pub fn get_raw_motors_current(&mut self) -> Result<[f64; 3]> {
        let red = self.inner.reduction();
        match self.inner.get_current_torque(){
            Ok(t) => {
                let mut tor = t;
                for i in 0..3 { tor[i] = tor[i] / red[i].unwrap();} 
                Ok(tor)
            }
            Err(e) => Err(e)
        }
    }

    /// Get the axis sensors values (gearbox mounted absolute magnetic encoder)
    pub fn get_axis_sensors(&mut self) -> Result<[f64; 3]> {
        let red = self.inner.reduction();
        match self.inner.get_axis_sensors(){
            Ok(a) => {
                let mut ax = a;
                for i in 0..3 { ax[i] = ax[i] * red[i].unwrap();}
                Ok(ax)
            }
            Err(e) => Err(e)
        }
    }

    /// Get the axis sensor zeros values (random offset from factory)
    pub fn get_axis_sensor_zeros(&mut self) -> Result<[f64; 3]> {
        self.inner.get_axis_sensor_zeros()
    }

    /// Get the error codes from the motors
    pub fn get_error_codes(&mut self) -> Result<[i32; 3]> {
        self.inner.get_error_codes()
    }

    /// Get the temperature for each motor in °C from a NTC sensor in contact with the motor body (top, middle, bottom)
    pub fn get_motor_temperatures(&mut self) -> Result<[f64; 3]> {
        self.inner.get_motor_temperatures()
    }

    /// Get the temperature for each H-Gate in °C (top, middle, bottom)
    pub fn get_board_temperatures(&mut self) -> Result<[f64; 3]> {
        self.inner.get_board_temperatures()
    }

    /// Get the board state register
    pub fn get_board_state(&mut self) -> Result<u8> {
        self.inner.get_board_state()
    }
    /// Get the board state register
    pub fn set_board_state(&mut self, state: u8) -> Result<()> {
        self.inner.set_board_state(state)
    }

    /// Get the current mode of operation (cf. firmware_poulpe documentation)
    pub fn get_control_mode(&mut self) -> Result<[u8; 3]> {
        self.inner.get_control_mode()
    }

    /// Set the current mode of operation (cf. firmware_poulpe documentation). Currently valid: 0=NoMode, 1=ProfilePositionMode, 3=ProfileVelocityMode, 4=ProfileTorqueMode
    pub fn set_control_mode(&mut self, mode: [u8; 3]) -> Result<()> {
        self.inner.set_control_mode(mode)
    }

    /// Triggers and emergency stop
    pub fn emergency_stop(&mut self) {
        self.inner.emergency_stop()
    }
}

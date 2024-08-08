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
use io::{CachedDynamixelSerialController, DynamixelSerialController, Orbita3dIOConfig};
use motor_toolbox_rs::{FakeMotorsController, MotorsController, Result, PID};
use nalgebra::Vector3;
use orbita3d_kinematics::{conversion, Orbita3dKinematicsModel};
use serde::{Deserialize, Serialize};
use std::{thread, time::Duration};

use crate::io::{CachedDynamixelPoulpeController, DynamixelPoulpeController};

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
    kinematics: Orbita3dKinematicsModel,
}

#[derive(Debug, Deserialize, Serialize, Copy, Clone)]
/// Feedback struct
pub struct Orbita3dFeedback {
    pub orientation: [f64; 4],
    // pub velocity: [f64; 3],
    // pub torque: [f64; 3],
}

impl Orbita3dController {
    /// Creates a new Orbita3dController using the given configuration file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        log::info!("Loading config file: {}", configfile);

        let f = std::fs::File::open(configfile)?;
        let config: Orbita3dConfig = serde_yaml::from_reader(f)?;

        let controller: Box<dyn MotorsController<3> + Send> = match config.io {
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
                    .with_reduction([Some(config.disks.reduction); 3]);

                log::info!("Using fake motors controller {:?}", controller);

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
    /// Set the target orientation (as quaternion (qx, qy, qz, qw))
    pub fn set_target_orientation(&mut self, target: [f64; 4]) -> Result<()> {
        let rot =
            conversion::quaternion_to_rotation_matrix(target[0], target[1], target[2], target[3]);
        let thetas = self.kinematics.compute_inverse_kinematics(rot)?;
        self.inner.set_target_position(thetas)
    }

    /// Set the target orientation fro the roll pitch yaw intrinsic angles (taking ulti turn into account)
    pub fn set_target_rpy_orientation(&mut self, target: [f64; 3]) -> Result<()> {
        let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(target[0], target[1], target[2]);
        let mut thetas = self.kinematics.compute_inverse_kinematics(rot)?;

        // Check if |yaw|>=Pi => means that we have to deal with the [-Pi, Pi] range of the rotation matrix
        if target[2].abs() >= std::f64::consts::PI {
            let nb_turns = (target[2] / std::f64::consts::TAU).trunc(); //number of full turn
            let multiturn_offset = std::f64::consts::TAU * (target[2].signum() + nb_turns);
            thetas.iter_mut().for_each(|x| *x += multiturn_offset);
        }

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

    /// Set the target orientation from roll pitch yaw intrinsic angles with feedback => returns feedback rpy
    pub fn set_target_rpy_orientation_fb(&mut self, target: [f64; 3]) -> Result<[f64; 3]> {
        // Check if |yaw|>Pi => means that we have to deal with the [-Pi, Pi[ range of the rotation matrix
        // if target[2] > std::f64::consts::PI || target[2] <= -std::f64::consts::PI {
        let mut multiturn_offset: f64 = 0.0;
        let mut thetas: [f64; 3] = [0.0, 0.0, 0.0];
        if target[2].abs() >= std::f64::consts::PI {
            // remove the yaw component:
            // let rot = conversion::intrinsic_roll_pitch_yaw_to_matrix(target[0], target[1], 0.0);

            let rot =
                conversion::intrinsic_roll_pitch_yaw_to_matrix(target[0], target[1], target[2]);

            // let rpy_extrinsic = rot.euler_angles();
            // log::debug!("rpy extrinsic {:?}", rpy_extrinsic);

            thetas = self.kinematics.compute_inverse_kinematics(rot)?;
            // log::debug!("theta no yaw: {:?}", thetas);

            // debug
            // let rot_check = self
            //     .kinematics
            //     .compute_forward_kinematics([thetas[0], thetas[1], thetas[2]]); //Why the f*ck can't I use slice here?
            // let rpy_check = conversion::matrix_to_intrinsic_roll_pitch_yaw(rot_check);
            // log::debug!("rpy (check) {:?}", rpy_check);

            let nb_turns = (target[2] / std::f64::consts::TAU).trunc(); //number of full turn

            if nb_turns.abs() >= 1.0 {
                multiturn_offset = std::f64::consts::TAU * (nb_turns);
            }

            log::debug!("Yaw more than Pi, nb full turns: {nb_turns}, offset: {multiturn_offset} theta before: {:?}",thetas);

            // If the yaw is > Pi, then all the thetas should be of same sign?

            //First, put everything in the [0;2Pi] range than apply 2pi to give it a proper sign

            thetas.iter_mut().for_each(|x| {
                // *x = x.rem_euclid(std::f64::consts::TAU);
                log::warn!(
                    "TOTO {:?}",
                    *x - target[2].rem_euclid(std::f64::consts::TAU).abs()
                );
                // if (*x - target[2].rem_euclid(std::f64::consts::TAU)).abs() > std::f64::consts::PI {
                //     log::warn!("YO {}", x);
                //     // *x -= std::f64::consts::TAU;
                //     *x -= target[2].signum() * std::f64::consts::TAU;
                //     log::warn!("=> YO {}", x);
                // }
                if (x.signum() != target[2].signum())
                // && (*x - target[2].rem_euclid(std::f64::consts::TAU)).abs()
                //     > std::f64::consts::PI
                {
                    log::debug!(
                        "DEBUG {} {} {:?}",
                        x,
                        target[2],
                        *x - target[2].rem_euclid(std::f64::consts::TAU).abs()
                    );

                    *x += target[2].signum() * std::f64::consts::TAU;
                } else {
                    log::debug!("No offset");
                }
            });

            log::debug!("thetas in the [0, 2pi] range {:?}", thetas);
            // log::debug!("Put thetas in the correct sign {:?}", thetas);

            // if thetas[0].signum() != target[2].signum()
            //     || thetas[1].signum() != target[2].signum()
            //     || thetas[2].signum() != target[2].signum()
            // {
            //     thetas[0] += target[2].signum() * std::f64::consts::TAU;
            //     thetas[1] += target[2].signum() * std::f64::consts::TAU;
            //     thetas[2] += target[2].signum() * std::f64::consts::TAU;
            //     log::debug!("Put thetas in the correct sign {:?}", thetas);
            // }

            thetas.iter_mut().for_each(|x| {
                if nb_turns.abs() >= 1.0 || (x.signum() != multiturn_offset.signum())
                // || ((x.signum() == multiturn_offset.signum())
                //     && x.abs() < std::f64::consts::TAU)
                {
                    *x += multiturn_offset
                }
            });

            log::debug!("Thetas after offset: {:?}", thetas);
        } else {
            let rot =
                conversion::intrinsic_roll_pitch_yaw_to_matrix(target[0], target[1], target[2]);
            thetas = self.kinematics.compute_inverse_kinematics(rot)?;
        }

        // Last check of gammas before sending command. FIXME!! check_gamma is not ready to work outside [-pi,pi]
        // self.kinematics
        //     .check_gammas(Vector3::from_row_slice(&[thetas[0], thetas[1], thetas[2]]))?;

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
                let rpy_test = conversion::matrix_to_intrinsic_roll_pitch_yaw(rot);
                log::debug!("FB: {:?} raw rpy feedback {:?}", fb, rpy_test);

                // When do we know that the |yaw|>=180°? is min(disks)>=180°? check if forward/inverse is the same?
                let ik = self.kinematics.compute_inverse_kinematics(rot);
                let mut ik_disks: [f64; 3] = [0.0, 0.0, 0.0];
                match ik {
                    Ok(disks) => {
                        if (fb[0] - disks[0]).abs() >= 0.01_f64.to_radians()
                            || (fb[1] - disks[1]).abs() >= 0.01_f64.to_radians()
                            || (fb[2] - disks[2]).abs() >= 0.01_f64.to_radians()
                        {
                            log::debug!("IK/FK mismatch. Probable >180° rotation of disks");

                            //Extract the "yaw" component of the disks
                            let mut rpy = conversion::matrix_to_intrinsic_roll_pitch_yaw(rot);
                            log::debug!("=> rpy: {:?}", rpy);
                            let rot_noyaw =
                                conversion::intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], 0.0);
                            let rot_yawonly =
                                conversion::intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, rpy[2]);
                            let ik_noyaw = self.kinematics.compute_inverse_kinematics(rot_noyaw);
                            match ik_noyaw {
                                Ok(disks_noyaw) => {
                                    let disk_yaw_comp: [f64; 3] = [
                                        fb[0] - disks_noyaw[0],
                                        fb[1] - disks_noyaw[1],
                                        fb[2] - disks_noyaw[2],
                                    ];

                                    // let disk_yaw_comp =
                                    //     self.kinematics.compute_inverse_kinematics(rot_yawonly)?;

                                    // What is the sign of the disk angles? if the yaw >180° the sum is positive, if yaw<-180° the sum is negative
                                    let mut disk_yaw_avg: f64 = disk_yaw_comp.iter().sum();
                                    disk_yaw_avg /= 3.0;
                                    log::debug!(
                                        "AVERAGE YAW: {} YAW COMPONENT: {:?} NO_YAW: {:?}",
                                        disk_yaw_avg,
                                        disk_yaw_comp,
                                        disks_noyaw
                                    );

                                    if rpy[2].signum() != disk_yaw_avg.signum() {
                                        log::debug!("bad yaw sign");
                                        if rpy[2] < 0.0 {
                                            log::debug!("\t+TAU");
                                            rpy[2] += std::f64::consts::TAU;
                                        } else {
                                            log::debug!("\t-TAU");
                                            rpy[2] -= std::f64::consts::TAU;
                                        }
                                    }
                                    log::debug!("=> RPY with yaw sign {:?}", rpy);

                                    // From the average yaw of the disks, compute the real rpy
                                    // it can be 180<|yaw|<360 or |yaw|>360
                                    let nb_turns = (disk_yaw_avg / std::f64::consts::TAU).trunc(); //number of full turn
                                                                                                   // let nb_turns: f64 =
                                                                                                   //     (disk_yaw_avg / std::f64::consts::TAU).round();

                                    log::debug!("=> nb_turns {:?}", nb_turns);

                                    if (disk_yaw_avg.abs() >= std::f64::consts::PI)
                                        && (disk_yaw_avg.abs() < std::f64::consts::TAU)
                                    {
                                        // We are in 180<|yaw|<360
                                        if (nb_turns.abs() > 0.0 || nb_turns == -1.0)
                                        // && (disk_yaw_avg - rpy[2]).abs() > 0.1
                                        {
                                            log::debug!(
                                                "Adding offset {}",
                                                disk_yaw_avg.signum() * std::f64::consts::TAU
                                            );
                                            rpy[2] += disk_yaw_avg.signum() * std::f64::consts::TAU;
                                            //?? FIXME! Edge case here, sometimes, we should add the offset but we do not. No idea how to differentiate... Nb_turns from disks is generally less that goal nb_turn
                                        }

                                        log::debug!("180<|yaw|<360: {}", rpy[2]);
                                    } else {
                                        // We are in |yaw|>360 => how many turns?

                                        rpy[2] += nb_turns * std::f64::consts::TAU;
                                        log::debug!("|yaw|>360: nb_turns {nb_turns} {}", rpy[2]);
                                    }
                                    log::debug!("=> Out RPY {:?}", rpy);
                                    return Ok(rpy);
                                }
                                Err(e) => log::error!("IK error? {e}"),
                            }
                        }
                        ik_disks = disks; //??
                    }
                    Err(e) => log::error!("IK error? {e}"),
                }

                log::debug!("No extra yaw FB: {:?} thetas: {:?}", fb, ik_disks);
                let rpy = conversion::matrix_to_intrinsic_roll_pitch_yaw(rot);

                // If we did not detect any extra yaw
                Ok(rpy)
            }
            Err(e) => Err(e),
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

    /// Get the axis sensors values (gearbox mounted absolute magnetic encoder)
    pub fn get_axis_sensors(&mut self) -> Result<[f64; 3]> {
        self.inner.get_axis_sensors()
    }

    /// Get the board state register
    pub fn get_board_state(&mut self) -> Result<u8> {
        self.inner.get_board_state()
    }
    /// Get the board state register
    pub fn set_board_state(&mut self, state: u8) -> Result<()> {
        self.inner.set_board_state(state)
    }
}

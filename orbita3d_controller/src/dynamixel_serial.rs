use motor_toolbox_rs::{MultipleMotorsController, PID, MissingResisterErrror};
use orbita3d_kinematics::Orbita3dKinematicsModel;
use rustypot::{
    device::orbita_foc::{self, DiskValue},
    DynamixelSerialIO,
};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;
use std::time::Duration;

use crate::{Orbita3dController, Result};

#[derive(Debug, Deserialize, Serialize)]
pub struct DynamixelSerialConfig {
    kinematics_model: Orbita3dKinematicsModel,

    serial_port: String,
    id: u8,
}

pub struct DynamixelSerialController {
    serial_port: Box<dyn SerialPort>,
    io: DynamixelSerialIO,
    id: u8,
}

impl DynamixelSerialController {
    pub fn new(serial_port: &str, id: u8) -> Result<Self> {
        Ok(Self {
            serial_port: serialport::new(serial_port, 1_000_000)
                .timeout(Duration::from_millis(10))
                .open()?,
            io: DynamixelSerialIO::v1(),
            id,
        })
    }
}

impl Orbita3dController {
    pub fn with_dynamixel_serial(config: DynamixelSerialConfig) -> Result<Self> {
        Ok(Self {
            inner: Box::new(DynamixelSerialController::new(
                &config.serial_port,
                config.id,
            )?),
            kinematics: config.kinematics_model,
        })
    }
}

impl MultipleMotorsController<3> for DynamixelSerialController {
    fn name(&self) -> String {
        format!(
            "Dynamixel Serial Controller (port: {:?}, id: {})",
            self.serial_port.name(),
            self.id
        )
    }

    fn is_torque_on(&mut self) -> Result<bool> {
        orbita_foc::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
            .map(|torque| torque != 0)
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        orbita_foc::write_torque_enable(&self.io, self.serial_port.as_mut(), self.id, on as u8)
    }

    fn get_current_position(&mut self) -> Result<[f64; 3]> {
        let thetas =
            orbita_foc::read_present_position(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([
            thetas.top as f64,
            thetas.middle as f64,
            thetas.bottom as f64,
        ])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror("current_velocity".to_string())))
    }

    fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror("current_torque".to_string())))
    }

    fn get_target_position(&mut self) -> Result<[f64; 3]> {
        orbita_foc::read_goal_position(&self.io, self.serial_port.as_mut(), self.id).map(|thetas| {
            [
                thetas.top as f64,
                thetas.middle as f64,
                thetas.bottom as f64,
            ]
        })
    }

    fn set_target_position(&mut self, position: [f64; 3]) -> Result<()> {
        orbita_foc::write_goal_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            DiskValue {
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        )
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 3]> {
        orbita_foc::read_angle_velocity_limit(&self.io, self.serial_port.as_mut(), self.id)
            .map(|vel| [vel as f64, vel as f64, vel as f64])
    }

    fn set_velocity_limit(&mut self, velocity: [f64; 3]) -> Result<()> {
        assert!(velocity[0] == velocity[1] && velocity[1] == velocity[2]);

        orbita_foc::write_angle_velocity_limit(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            velocity[0] as f32,
        )
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror("torque_limit".to_string())))
    }

    fn set_torque_limit(&mut self, _torque: [f64; 3]) -> Result<()> {
        Err(Box::new(MissingResisterErrror("torque_limit".to_string())))
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        orbita_foc::read_angle_pid(&self.io, self.serial_port.as_mut(), self.id).map(|pid| {
            [
                PID {
                    p: pid.p as f64,
                    i: pid.i as f64,
                    d: pid.d as f64,
                },
                PID {
                    p: pid.p as f64,
                    i: pid.i as f64,
                    d: pid.d as f64,
                },
                PID {
                    p: pid.p as f64,
                    i: pid.i as f64,
                    d: pid.d as f64,
                },
            ]
        })
    }

    fn set_pid_gains(&mut self, pid: [PID; 3]) -> Result<()> {
        assert!(pid[0] == pid[1] && pid[1] == pid[2]);

        orbita_foc::write_angle_pid(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            orbita_foc::Pid {
                p: pid[0].p as f32,
                i: pid[0].i as f32,
                d: pid[0].d as f32,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::Orbita3dConfig;

    #[test]
    fn parse_config() {
        let config = r#"!DynamixelSerial
        kinematics_model:
          alpha: 0.0
          gamma_min: 0.0
          offset: 0.0
          beta: 0.0
          gamma_max: 0.0
          passiv_arms_direct: false
        serial_port: "/dev/ttyUSB0"
        id: 42
        "#;

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_str(config);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dConfig::DynamixelSerial(config) = config {
            assert_eq!(config.kinematics_model.alpha, 0.0);
            assert_eq!(config.kinematics_model.gamma_min, 0.0);
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, 0.0);
            assert_eq!(config.kinematics_model.gamma_max, 0.0);
            assert!(!config.kinematics_model.passiv_arms_direct);
            assert_eq!(config.serial_port, "/dev/ttyUSB0");
            assert_eq!(config.id, 42);
        } else {
            panic!("Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/dxl_serial.yaml").unwrap();

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dConfig::DynamixelSerial(config) = config {
            assert_eq!(config.kinematics_model.alpha, 50.0_f64.to_radians());
            assert_eq!(config.kinematics_model.gamma_min, 0.0);
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, PI / 2.0);
            assert_eq!(config.kinematics_model.gamma_max, PI);
            assert!(config.kinematics_model.passiv_arms_direct);
            assert_eq!(config.serial_port, "/dev/ttyUSB0");
            assert_eq!(config.id, 42);
        } else {
            panic!("Wrong config type");
        }
    }
}

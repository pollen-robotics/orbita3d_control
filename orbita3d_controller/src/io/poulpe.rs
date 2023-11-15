use motor_toolbox_rs::{Limit, MissingResisterErrror, MotorsController, RawMotorsIO, Result, PID};
use rustypot::{
    device::orbita3d_poulpe::{self, ValuePerMotor},
    DynamixelSerialIO,
};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;
use std::{f64::consts::PI, time::Duration};

use crate::ZeroType;

#[derive(Debug, Deserialize, Serialize)]
/// DynamixelPoulpe Config
pub struct DynamixelPoulpeConfig {
    /// Name of the serial port (eg. /dev/ttyUSB0)
    pub serial_port: String,
    /// Dynamixel ID
    pub id: u8,
    /// Use cache for the dynamixel poulpe serial controller
    /// Can drastically improve performance, but can hide some communication errors
    pub use_cache: bool,
}

#[derive(Debug)]
/// DynamixelPoulpeController - wrapper around the three disks motors
pub struct DynamixelPoulpeController {
    serial_port: Box<dyn SerialPort>,
    io: DynamixelSerialIO,
    id: u8,

    offsets: [Option<f64>; 3],
    reduction: [Option<f64>; 3],
    limits: [Option<Limit>; 3],
}

impl DynamixelPoulpeController {
    /// Creates a new DynamixelPoulpeController
    pub fn new(serial_port: &str, id: u8, zero: ZeroType, reductions: f64) -> Result<Self> {
        let mut controller = Self {
            serial_port: serialport::new(serial_port, 1_000_000)
                .timeout(Duration::from_millis(10))
                .open()?,
            io: DynamixelSerialIO::v1(),
            id,
            offsets: [None; 3],
            reduction: [Some(reductions); 3],
            limits: [None; 3],
        };

        match zero {
            ZeroType::ApproximateHardwareZero(zero) => {
                let current_pos = MotorsController::get_current_position(&mut controller)?;

                zero.hardware_zero
                    .iter()
                    .zip(current_pos.iter())
                    .enumerate()
                    .for_each(|(i, (&hardware_zero, &current_pos))| {
                        controller.offsets[i] = Some(find_closest_offset_to_zero(
                            current_pos,
                            hardware_zero,
                            reductions,
                        ));
                    });
            }
            ZeroType::ZeroStartup(_) => {
                let current_pos = MotorsController::get_current_position(&mut controller)?;

                current_pos
                    .iter()
                    .enumerate()
                    .for_each(|(i, &current_pos)| {
                        controller.offsets[i] = Some(current_pos);
                    });
            }
        }

        Ok(controller)
    }

    pub fn id(&self) -> u8 {
        self.id
    }
}

impl MotorsController<3> for DynamixelPoulpeController {
    fn io(&mut self) -> &mut dyn RawMotorsIO<3> {
        self
    }

    fn offsets(&self) -> [Option<f64>; 3] {
        self.offsets
    }

    fn reduction(&self) -> [Option<f64>; 3] {
        self.reduction
    }

    fn limits(&self) -> [Option<motor_toolbox_rs::Limit>; 3] {
        self.limits
    }
}

impl RawMotorsIO<3> for DynamixelPoulpeController {
    fn is_torque_on(&mut self) -> Result<[bool; 3]> {
        orbita3d_poulpe::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.top != 0, val.middle != 0, val.bottom != 0])
    }

    fn set_torque(&mut self, on: [bool; 3]) -> Result<()> {
        assert!(on.iter().all(|&t| t == on[0]));
        orbita3d_poulpe::write_torque_enable(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            ValuePerMotor {
                top: on[0] as u8,
                middle: on[1] as u8,
                bottom: on[2] as u8,
            },
        )
    }

    fn get_current_position(&mut self) -> Result<[f64; 3]> {
        let thetas =
            orbita3d_poulpe::read_present_position(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([
            thetas.top as f64,
            thetas.middle as f64,
            thetas.bottom as f64,
        ])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror(
            "current_velocity".to_string(),
        )))
    }

    fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror(
            "current_torque".to_string(),
        )))
    }

    fn get_target_position(&mut self) -> Result<[f64; 3]> {
        orbita3d_poulpe::read_goal_position(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
                [
                    thetas.top as f64,
                    thetas.middle as f64,
                    thetas.bottom as f64,
                ]
            },
        )
    }

    fn set_target_position(&mut self, position: [f64; 3]) -> Result<()> {
        orbita3d_poulpe::write_goal_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            ValuePerMotor {
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        )
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror(
            "velocity_limit".to_string(),
        )))
    }

    fn set_velocity_limit(&mut self, _velocity: [f64; 3]) -> Result<()> {
        Err(Box::new(MissingResisterErrror(
            "velocity_limit".to_string(),
        )))
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 3]> {
        Err(Box::new(MissingResisterErrror("torque_limit".to_string())))
    }

    fn set_torque_limit(&mut self, _torque: [f64; 3]) -> Result<()> {
        Err(Box::new(MissingResisterErrror("torque_limit".to_string())))
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        Err(Box::new(MissingResisterErrror("pid_gains".to_string())))
    }

    fn set_pid_gains(&mut self, _pid: [PID; 3]) -> Result<()> {
        Err(Box::new(MissingResisterErrror("pid_gains".to_string())))
    }
}

fn find_closest_offset_to_zero(current_position: f64, hardware_zero: f64, reduction: f64) -> f64 {
    //! Find the closest offset to zero
    //!
    //! The absolute encoder is placed before orbita reduction.
    //! Thus, we do not have the real orbita disk absolute position.
    //! But we only know where we are in a local dial of arc (2pi / reduction).
    //!
    //! To find the closest offset to our zero hardware, we assume that the current position is at maximum one dial from the hardware zero.
    log::info!(
        "find_closest_offset_to_zero: current_position: {}, hardware_zero: {}, reduction: {}",
        current_position,
        hardware_zero,
        reduction
    );

    let dial_arc = 2.0 * PI / reduction;

    let possibilities = [
        hardware_zero - dial_arc,
        hardware_zero,
        hardware_zero + dial_arc,
    ];
    log::debug!("possibilities: {:?}", possibilities);

    let best = possibilities
        .iter()
        .map(|&p| (p - current_position).abs())
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| possibilities[i])
        .unwrap();

    log::info!("best: {}", best);
    best
}

#[cfg(test)]
mod tests {
    use rand::Rng;
    use std::f64::consts::PI;

    use crate::{
        io::{poulpe::find_closest_offset_to_zero, Orbita3dIOConfig},
        Orbita3dConfig,
    };

    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/dxl_poulpe.yaml").unwrap();

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dIOConfig::DynamixelPoulpe(dxl_config) = config.io {
            assert_eq!(config.kinematics_model.alpha, 54.0_f64.to_radians());
            assert_eq!(config.kinematics_model.gamma_min, 40.0_f64.to_radians());
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, PI / 2.0);
            assert_eq!(config.kinematics_model.gamma_max, PI);
            assert!(config.kinematics_model.passiv_arms_direct);

            if let crate::ZeroType::ZeroStartup(_) = config.disks.zeros {
            } else {
                panic!("Wrong config type");
            }
            assert_eq!(config.disks.reduction, 4.2666667);

            assert_eq!(dxl_config.serial_port, "/dev/ttyACM0");
            assert_eq!(dxl_config.id, 42);
        } else {
            panic!("Wrong config type");
        }
    }

    #[test]
    fn test_approx() {
        let mut rng = rand::thread_rng();

        let pos = rng.gen_range(-PI..PI);
        let reduction = rng.gen_range(0.5..2.0);
        assert_eq!(find_closest_offset_to_zero(pos, pos, reduction), pos);

        let pos = 0.0;
        let hardware_zero = 0.25;
        let reduction = 1.0;

        assert_eq!(
            find_closest_offset_to_zero(pos, hardware_zero, reduction),
            0.25
        );

        let reduction = 100.0;
        assert_eq!(
            find_closest_offset_to_zero(pos, hardware_zero, reduction),
            0.25 - 2.0 * PI / reduction
        );
    }
}

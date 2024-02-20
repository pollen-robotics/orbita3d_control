use motor_toolbox_rs::{Limit, MissingResisterErrror, MotorsController, RawMotorsIO, Result, PID};
use rustypot::{
    device::orbita3d_poulpe::{self, MotorPositionSpeedLoad, MotorValue},
    DynamixelSerialIO,
};
use serde::{Deserialize, Serialize};
use serialport::{SerialPort, TTYPort};
use std::{error::Error, thread, time::Instant};
use std::{f64::consts::PI, f64::consts::TAU, time::Duration};

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
    // serial_port: Box<dyn SerialPort>,
    // serial_port: TTYPort,
    serial_port: Box<TTYPort>,
    io: DynamixelSerialIO,
    id: u8,

    offsets: [Option<f64>; 3],
    reduction: [Option<f64>; 3],
    // motor_reduction: [Option<f64>; 3],
    limits: [Option<Limit>; 3],
}

impl DynamixelPoulpeController {
    /// Creates a new DynamixelPoulpeController
    pub fn new(serial_port: &str, id: u8, zero: ZeroType, reductions: f64) -> Result<Self> {
        let mut controller = Self {
            serial_port: Box::new(
                serialport::new(serial_port, 2_000_000)
                    .timeout(Duration::from_millis(10))
                    .open_native()?,
            ),
            io: DynamixelSerialIO::v1(),
            id,
            offsets: [None; 3],
            reduction: [Some(reductions); 3],
            // motor_reduction: [Some(motor_reductions); 3],
            limits: [None; 3],
            // hall_indices: [None; 3],
        };

        controller.serial_port.set_exclusive(false)?;

        log::info!("Creacting controller");
        match zero {
            ZeroType::ApproximateHardwareZero(zero) => {
                log::info!("ApproximateHardwarezero");

                let current_pos = MotorsController::get_current_position(&mut controller)?;
                log::info!("Current position: {:?}", current_pos);

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
                log::info!("ZeroStartup");

                let current_pos = MotorsController::get_current_position(&mut controller)?;

                log::info!("Current position: {:?}", current_pos);

                current_pos
                    .iter()
                    .enumerate()
                    .for_each(|(i, &current_pos)| {
                        controller.offsets[i] = Some(current_pos);
                    });
            }
            ZeroType::HallZero(zero) => {
                log::info!("HallZero");

                let current_pos = MotorsController::get_current_position(&mut controller)?;
                let current_gearbox = MotorsController::get_axis_sensors(&mut controller)?;

                thread::sleep(Duration::from_millis(1));

                let curr_hall = orbita3d_poulpe::read_index_sensor(
                    &controller.io,
                    controller.serial_port.as_mut(),
                    controller.id,
                )?;
                let hall_idx: [u8; 3] = [curr_hall.top, curr_hall.middle, curr_hall.bottom];
                log::info!(
                    "Current position: {:?} axis: {:?} current hall: {:?}",
                    current_pos,
                    current_gearbox,
                    curr_hall
                );

                // Security, is there a mis-detection?
                if hall_idx.contains(&255)
                //255 is the value when no hall sensor is detected
                {
                    log::error!("HallZero: Hall sensor offsets not found! Check 'Donut' I2C connection or maybe configure another zeroing method?");
                    return Err(Box::new(MissingResisterErrror(
                        "Hall sensor not found".to_string(),
                    )));
                }

                // Security is there a duplicate?
                let mut vidx = hall_idx.to_vec();
                vidx.sort();
                vidx.dedup();
                if vidx.len() != 3 {
                    log::error!("HallZero: Duplicate in hall indices! Initialization failed...");

                    return Err(Box::new(MissingResisterErrror(
                        "Hall sensor not found".to_string(),
                    )));
                }

                thread::sleep(Duration::from_millis(1));

                log::debug!("HallZero: curr_pos: {:?} curr_hall_idx: {:?} hardware_zero: {:?} hall_zero: {:?}", current_pos, hall_idx,zero.hardware_zero,zero.hall_indice);

                //theoretical angle if we are at the center of the Hall sensor from the zero
                // Top zero is exactly in the middle of Hall 15 and Hall 0 (-11.25° from Hall 0)
                // Mid zero is exactly at -3.75° from Hall 5
                // Bot zero is exactly at 3.75° from Hall 10

                let mut zero_hall_offsets: [f64; 3] = [0.0, 0.0, 0.0];
                zero_hall_offsets[0] =
                    hall_diff(hall_idx[0], 0) * 22.5_f64.to_radians() + 11.25_f64.to_radians();
                zero_hall_offsets[1] =
                    hall_diff(hall_idx[1], 5) * 22.5_f64.to_radians() + 3.75_f64.to_radians();
                zero_hall_offsets[2] =
                    hall_diff(hall_idx[2], 10) * 22.5_f64.to_radians() - 3.75_f64.to_radians();

                let mut found_turn: [i16; 3] = [0; 3];

                zero.hardware_zero
                    .iter()
                    .zip(current_pos.iter())
                    .zip(hall_idx.iter())
                    .zip(zero_hall_offsets.iter())
                    .enumerate()
                    .for_each(
                        |(i, (((&hardware_zero, &current_pos), &hall_idx), &hall_zero))| {
                            let res = find_position_with_hall(
                                current_pos,
                                hardware_zero,
                                hall_zero,
                                hall_idx,
                                reductions,
                            );
                            controller.offsets[i] = Some(res.0);
                            found_turn[i] = res.1;
                        },
                    );
                log::debug!("Offsets: {:?}, turns: {:?}", controller.offsets, found_turn);

                // Security, did we found the same number of turn for each arm? (FIXME?)
                if !(found_turn[0] == found_turn[1] && found_turn[1] == found_turn[2]) {
                    log::error!("HallZero: Incoherent offsets!!");
                    controller.offsets[0] = None;
                    controller.offsets[1] = None;
                    controller.offsets[2] = None;
                    return Err(Box::new(MissingResisterErrror(
                        "Hall sensor not found".to_string(),
                    )));
                }
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
        let mut reduction = [None; 3];
        reduction.iter_mut().enumerate().for_each(|(i, r)| {
            *r = Some(self.reduction[i].unwrap());
        });
        reduction
    }

    fn limits(&self) -> [Option<motor_toolbox_rs::Limit>; 3] {
        self.limits
    }
}

impl RawMotorsIO<3> for DynamixelPoulpeController {
    fn is_torque_on(&mut self) -> Result<[bool; 3]> {
        orbita3d_poulpe::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.top, val.middle, val.bottom])
    }

    fn set_torque(&mut self, on: [bool; 3]) -> Result<()> {
        assert!(on.iter().all(|&t| t == on[0]));
        orbita3d_poulpe::write_torque_enable(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: on[0],
                middle: on[1],
                bottom: on[2],
            },
        )
    }

    fn get_current_position(&mut self) -> Result<[f64; 3]> {
        let thetas =
            orbita3d_poulpe::read_current_position(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([
            thetas.top as f64,
            thetas.middle as f64,
            thetas.bottom as f64,
        ])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        let vel =
            orbita3d_poulpe::read_current_velocity(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([vel.top as f64, vel.middle as f64, vel.bottom as f64])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        let torque =
            orbita3d_poulpe::read_current_torque(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([
            torque.top as f64,
            torque.middle as f64,
            torque.bottom as f64,
        ])
    }

    fn get_target_position(&mut self) -> Result<[f64; 3]> {
        orbita3d_poulpe::read_target_position(&self.io, self.serial_port.as_mut(), self.id).map(
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
        let fb = orbita3d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        );

        match fb {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn set_target_position_fb(&mut self, position: [f64; 3]) -> Result<[f64; 3]> {
        let fb = orbita3d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        );

        match fb {
            Ok(fb) => Ok([
                fb.position.top as f64,
                fb.position.middle as f64,
                fb.position.bottom as f64,
                // fb.speed.top as f64,
                // fb.speed.middle as f64,
                // fb.speed.bottom as f64,
                // fb.load.top as f64,
                // fb.load.middle as f64,
                // fb.load.bottom as f64,
            ]),
            Err(e) => Err(e),
        }
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 3]> {
        orbita3d_poulpe::read_velocity_limit(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
                [
                    thetas.top as f64,
                    thetas.middle as f64,
                    thetas.bottom as f64,
                ]
            },
        )
    }

    fn set_velocity_limit(&mut self, _velocity: [f64; 3]) -> Result<()> {
        orbita3d_poulpe::write_velocity_limit(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: _velocity[0] as f32,
                middle: _velocity[1] as f32,
                bottom: _velocity[2] as f32,
            },
        )
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 3]> {
        orbita3d_poulpe::read_torque_flux_limit(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
                [
                    thetas.top as f64,
                    thetas.middle as f64,
                    thetas.bottom as f64,
                ]
            },
        )
    }

    fn set_torque_limit(&mut self, _torque: [f64; 3]) -> Result<()> {
        orbita3d_poulpe::write_torque_flux_limit(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: _torque[0] as f32,
                middle: _torque[1] as f32,
                bottom: _torque[2] as f32,
            },
        )
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        orbita3d_poulpe::read_position_pid(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
                [
                    PID {
                        p: thetas.top.p as f64,
                        i: thetas.top.i as f64,
                        d: 0.0,
                    },
                    PID {
                        p: thetas.middle.p as f64,
                        i: thetas.middle.i as f64,
                        d: 0.0,
                    },
                    PID {
                        p: thetas.bottom.p as f64,
                        i: thetas.bottom.i as f64,
                        d: 0.0,
                    },
                ]
            },
        )
    }

    fn set_pid_gains(&mut self, _pid: [PID; 3]) -> Result<()> {
        orbita3d_poulpe::write_position_pid(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                top: orbita3d_poulpe::Pid {
                    p: _pid[0].p as i16,
                    i: _pid[0].i as i16,
                },
                middle: orbita3d_poulpe::Pid {
                    p: _pid[1].p as i16,
                    i: _pid[1].i as i16,
                },
                bottom: orbita3d_poulpe::Pid {
                    p: _pid[2].p as i16,
                    i: _pid[2].i as i16,
                },
            },
        )
    }
    fn get_axis_sensors(&mut self) -> Result<[f64; 3]> {
        let axis = orbita3d_poulpe::read_axis_sensor(&self.io, self.serial_port.as_mut(), self.id)?;
        Ok([axis.top as f64, axis.middle as f64, axis.bottom as f64])
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

fn find_position_with_hall(
    current_position: f64,
    hardware_zero: f64,
    hall_zero: f64,
    hall_index: u8,
    reduction: f64,
) -> (f64, i16) {
    //! Find the current position corrected by the hall sensor
    //! There is 16 Hall sensors on the disk output and a ratio 'reduction' between the disk and the motor gearbox output
    //! We knwow the 'hall_zero' index which correspond to the index of the Hall sensor closest to the disk zero position
    //! We also know the 'hall_index' which is the index of the Hall sensor closest to the current position
    //! Finally we know the 'hardware_zero' which is the position of the disk zero

    const MAX_TURN: usize = 3;
    let mut offset: [f64; MAX_TURN] = [0.0; MAX_TURN];
    let mut offset_search: [f64; MAX_TURN] = [0.0; MAX_TURN];
    let turn_offset = 2.0 * PI * reduction;
    let hall_offset = 2.0 * PI / 16.0 * reduction; //22.5° disk for each Hall sensor

    // let hall_diff = hall_diff(hall_index, hall_zero);

    let diff_gear = current_position * reduction - hardware_zero * reduction;
    let shortest_diff_gear = angle_diff(current_position * reduction, hardware_zero * reduction); //nul FIXME
    let shortest_to_zero = angle_diff(0.0, hardware_zero * reduction);

    let pos = (current_position * reduction) % TAU; //this should be the raw gearbox position
    let shortest_to_current = angle_diff(0.0, pos);
    let mut gearbox_turn = 0.0;

    log::debug!(
        "Diff: {:?} shortest diff: {:?} shortest_to_zero {:?} hall_zero_angle: {:?}",
        diff_gear,
        shortest_diff_gear,
        shortest_to_zero,
        hall_zero
    );

    for i in 0..offset.len() {
        // theoretical position of the gearbox starting from the zero and moving toward detected hall

        offset_search[i] = (hardware_zero * reduction) % TAU
            + (hall_zero * reduction) % TAU
            + ((i as f64 - (offset.len() / 2) as f64) * turn_offset) % TAU;
        offset_search[i] %= TAU;

        let residual = angle_diff(
            pos,
            (hardware_zero * reduction) % TAU + (hall_zero * reduction) % TAU,
        ) / reduction;

        // Offset to apply
        offset[i] = current_position
            - hall_zero
            - residual
            - (i as f64 - (offset.len() / 2) as f64)
                * (turn_offset / reduction - TAU * (reduction - reduction.floor()) / reduction);

        //in orbita ref
    }

    log::debug!(
        "Residual (gearbox) {:?} (orbita) {:?}",
        angle_diff(pos, (hall_zero * reduction) % TAU),
        angle_diff(pos, (hall_zero * reduction) % TAU) / reduction
    );
    log::debug!("possible offset (orbita domain): {:?}", offset);
    log::debug!("searching offset (gearbox domain): {:?}", offset_search);

    log::debug!(
        "current pos (gearbox): {:?} hardware_zero (gearbox): {:?} hall_idx: {:?} hall_zero: {:?} hall_offset: {:?} turn_offset: {:?}",
        pos,
        hardware_zero * reduction,
        hall_index as f64,
        hall_zero,
        hall_offset,
	turn_offset
    );

    let best = offset_search
        .iter()
        .map(|&p| {
            let d = angle_diff(p, pos).abs();
            log::debug!("Diff search: {:?}", d);
            d
        })
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| offset[i])
        .unwrap();

    let best_idx = offset.iter().position(|&x| x == best).unwrap();
    log::debug!(
        "best offset (orbita domain): {} gearbox domain: {:?}",
        best,
        offset_search[best_idx]
    );
    log::debug!(
        "It corresponds to {} turn (orbita domain)",
        best_idx as i16 - (offset.len() / 2) as i16
    );

    (best, best_idx as i16 - (offset.len() / 2) as i16)
}

pub fn angle_diff(angle_a: f64, angle_b: f64) -> f64 {
    let mut angle = angle_a - angle_b;
    angle = (angle + PI) % TAU - PI;
    if angle < -PI {
        angle + TAU
    } else {
        angle
    }
}

pub fn hall_diff(hall_a: u8, hall_b: u8) -> f64 {
    // shortest hall difference (16 discrete Hall)
    let d: f64 = hall_a as f64 - hall_b as f64;
    if d >= 0.0 {
        if d >= 8.0 {
            d - 16.0
        } else {
            d
        }
    } else if d >= -8.0 {
        d
    } else {
        d + 16.0
    }
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
            // assert_eq!(config.disks.reduction, 5.33333334); //TODO
            assert_eq!(config.disks.reduction, 4.2666667); //Old Orbita

            assert_eq!(dxl_config.serial_port, "/dev/ttyUSB0");
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

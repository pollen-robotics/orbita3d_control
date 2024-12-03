use motor_toolbox_rs::{Limit, MotorsController, RawMotorsIO, Result, PID};
use poulpe_ethercat_grpc::client::PoulpeRemoteClient;
use serde::{Deserialize, Serialize};
use std::{f64::consts::PI, f64::consts::TAU, time::Duration};

use log::{error, info};

use crate::ZeroType;

#[derive(Debug, Deserialize, Serialize)]
/// EthercatPoulpeController Config
pub struct PoulpeEthercatConfig {
    /// url of the ethercat master grpc serverS
    pub url: String,
    /// Actuator id
    pub id: u8,
}

#[derive(Debug)]
/// EthercatPoulpeController - wrapper around the three disks motors
pub struct EthercatPoulpeController {
    io: PoulpeRemoteClient,
    id: u16,

    offsets: [Option<f64>; 3],
    reduction: [Option<f64>; 3],
    // motor_reduction: [Option<f64>; 3],
    limits: [Option<Limit>; 3],
    inverted_axes: [Option<bool>; 3],
}

impl EthercatPoulpeController {
    /// Creates a new EthercatPoulpeController
    pub fn new(
        url: &str,
        id: u8,
        zero: ZeroType,
        reductions: f64,
        inverted_axes: [Option<bool>; 3],
    ) -> Result<Self> {
        let mut io = match PoulpeRemoteClient::connect(
            url.parse()?,
            vec![id as u16],
            Duration::from_secs_f32(0.002),
        ) {
            Ok(io) => io,
            Err(e) => {
                error!(
                    "Error while connecting to EthercatPoulpeController: {:?}",
                    e
                );
                return Err("Error while connecting to EthercatPoulpeController".into());
            }
        };

        // set the initial velocity and torque limit to 100%
        io.set_velocity_limit(id as u16, [1.0; 3].to_vec());
        io.set_torque_limit(id as u16, [1.0; 3].to_vec());

        let mut poulpe_controller = EthercatPoulpeController {
            io,
            id: id as u16,
            offsets: [None; 3],
            reduction: [Some(reductions); 3],
            limits: [None; 3],
            inverted_axes,
        };

        info!(
            "Orbita3d EthercatPoulpeController:\n\t - url: {:?}\n\t - id: {:?}",
            url, id
        );

        log::info!("Creacting controller");

        match zero {
            ZeroType::ApproximateHardwareZero(zero) => {
                log::info!("ApproximateHardwarezero");

                let current_pos = MotorsController::get_current_position(&mut poulpe_controller)?;
                log::info!("Current position: {:?}", current_pos);

                zero.hardware_zero
                    .iter()
                    .zip(current_pos.iter())
                    .enumerate()
                    .for_each(|(i, (&hardware_zero, &current_pos))| {
                        poulpe_controller.offsets[i] = Some(find_closest_offset_to_zero(
                            current_pos,
                            hardware_zero,
                            reductions,
                        ));
                    });
            }
            ZeroType::FirmwareZero(_) => {
                log::info!(
                    "FirmwareZero => zero has been done in firmware, no need to do it here."
                );
            }
            ZeroType::ZeroStartup(_) => {
                log::info!("ZeroStartup");

                let current_pos = MotorsController::get_current_position(&mut poulpe_controller)?;

                log::info!("Current position: {:?}", current_pos);

                current_pos
                    .iter()
                    .enumerate()
                    .for_each(|(i, &current_pos)| {
                        poulpe_controller.offsets[i] = Some(current_pos);
                    });
            }
            ZeroType::HallZero(_zero) => {
                log::error!("HallZero Not supported with Ethercat!");
            }
        }

        Ok(poulpe_controller)
    }

    pub fn id(&self) -> u8 {
        self.id as u8
    }
}

impl MotorsController<3> for EthercatPoulpeController {
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

    fn inverted_axes(&self) -> [Option<bool>; 3] {
        self.inverted_axes
    }
}

impl RawMotorsIO<3> for EthercatPoulpeController {
    fn name(&self) -> String {
        "EthercatPoulpeController".to_string()
    }

    fn is_torque_on(&mut self) -> Result<[bool; 3]> {
        match self.io.get_torque_state(self.id) {
            Ok(state) => Ok([state, state, state]),
            Err(_) => Err("Error while getting torque state".into()),
        }
    }

    fn set_torque(&mut self, on: [bool; 3]) -> Result<()> {
        assert!(on.iter().all(|&t| t == on[0]));
        if on.iter().any(|&x| x) {
            self.io.turn_on(self.id);
        } else {
            self.io.turn_off(self.id);
        }
        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 3]> {
        match self.io.get_position_actual_value(self.id) {
            Ok(position) => Ok([position[0] as f64, position[1] as f64, position[2] as f64]),
            Err(_) => Err("Error while getting position".into()),
        }
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        match self.io.get_velocity_actual_value(self.id) {
            Ok(velocity) => Ok([velocity[0] as f64, velocity[1] as f64, velocity[2] as f64]),
            Err(_) => Err("Error while getting velocity".into()),
        }
    }

    fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        match self.io.get_torque_actual_value(self.id) {
            Ok(torque) => Ok([torque[0] as f64, torque[1] as f64, torque[2] as f64]),
            Err(_) => Err("Error while getting torque".into()),
        }
    }

    fn get_target_position(&mut self) -> Result<[f64; 3]> {
        match self.io.get_target_position(self.id) {
            Ok(position) => Ok([position[0] as f64, position[1] as f64, position[2] as f64]),
            Err(_) => Err("Error while getting target position".into()),
        }
    }

    fn set_target_position(&mut self, position: [f64; 3]) -> Result<()> {
        let target_position = position.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_position(self.id, target_position);
        Ok(())
    }

    // fn get_target_velocity(&mut self) -> Result<[f64; 3]> {
    //     match self.io.get_target_velocity(self.id) {
    //         Ok(vel) => Ok([vel[0] as f64, vel[1] as f64, vel[2] as f64]),
    //         Err(_) => Err("Error while getting target velocity".into()),
    //     }
    // }

    fn set_target_velocity(&mut self, vel: [f64; 3]) -> Result<()> {
        let target_velocity = vel.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_velocity(self.id, target_velocity);
        Ok(())
    }

    // fn get_target_torque(&mut self) -> Result<[f64; 3]> {
    //     match self.io.get_target_torque(self.id) {
    //         Ok(vel) => Ok([vel[0] as f64, vel[1] as f64, vel[2] as f64]),
    //         Err(_) => Err("Error while getting target torque".into()),
    //     }
    // }

    fn set_target_torque(&mut self, vel: [f64; 3]) -> Result<()> {
        let target_torque = vel.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_torque(self.id, target_torque);
        Ok(())
    }

    fn set_target_position_fb(&mut self, position: [f64; 3]) -> Result<[f64; 3]> {
        let target_position = position.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_position(self.id, target_position);

        match self.io.get_position_actual_value(self.id) {
            Ok(position) => Ok([position[0] as f64, position[1] as f64, position[2] as f64]),
            Err(_) => Err("Error while getting position".into()),
        }
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 3]> {
        match self.io.get_velocity_limit(self.id) {
            Ok(limit) => Ok([limit[0] as f64, limit[1] as f64, limit[2] as f64]),
            Err(_) => Err("Error while getting velocity limit".into()),
        }
    }

    fn set_velocity_limit(&mut self, velocity: [f64; 3]) -> Result<()> {
        let velocity_limit = velocity.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_velocity_limit(self.id, velocity_limit);
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 3]> {
        match self.io.get_torque_limit(self.id) {
            Ok(limit) => Ok([limit[0] as f64, limit[1] as f64, limit[2] as f64]),
            Err(_) => Err("Error while getting torque limit".into()),
        }
    }

    fn set_torque_limit(&mut self, torque: [f64; 3]) -> Result<()> {
        let torque_limit = torque.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_torque_limit(self.id, torque_limit);
        Ok(())
    }

    //TODO
    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        Ok([PID {
            p: 0.0,
            i: 0.0,
            d: 0.0,
        }; 3])
    }

    fn set_pid_gains(&mut self, _pid: [PID; 3]) -> Result<()> {
        Ok(())
    }
    fn get_axis_sensors(&mut self) -> Result<[f64; 3]> {
        match self.io.get_axis_sensors(self.id) {
            Ok(sensor) => Ok([sensor[0] as f64, sensor[1] as f64, sensor[2] as f64]),
            Err(_) => Err("Error while getting axis sensors".into()),
        }
    }

    fn get_axis_sensor_zeros(&mut self) -> Result<[f64; 3]> {
        match self.io.get_axis_sensor_zeros(self.id) {
            Ok(sensor) => Ok([sensor[0] as f64, sensor[1] as f64, sensor[2] as f64]),
            Err(_) => Err("Error while getting axis sensor zeros".into()),
        }
    }

    fn get_board_state(&mut self) -> Result<u8> {
        match self.io.get_state(self.id) {
            Ok(state) => Ok(state as u8),
            Err(_) => Err("Error while getting board state".into()),
        }
    }

    fn get_error_codes(&mut self) -> Result<[i32; 3]> {
        match self.io.get_error_codes(self.id) {
            Ok(codes) => Ok([codes[0], codes[1], codes[2]]),
            Err(_) => Err("Error while getting error codes".into()),
        }
    }

    fn get_motor_temperatures(&mut self) -> Result<[f64; 3]> {
        match self.io.get_motor_temperatures(self.id) {
            Ok(temp) => Ok([temp[0] as f64, temp[1] as f64, temp[2] as f64]),
            Err(_) => Err("Error while getting motor temperatures".into()),
        }
    }

    fn get_board_temperatures(&mut self) -> Result<[f64; 3]> {
        match self.io.get_board_temperatures(self.id) {
            Ok(temp) => Ok([temp[0] as f64, temp[1] as f64, temp[2] as f64]),
            Err(_) => Err("Error while getting board temperatures".into()),
        }
    }

    fn set_board_state(&mut self, _state: u8) -> Result<()> {
        Ok(())
    }

    fn get_control_mode(&mut self) -> Result<[u8; 3]> {
        match self.io.get_mode_of_operation(self.id) {
            Ok(mode) => Ok([mode as u8, mode as u8, mode as u8]), //It is in fact the same for each axis (TODO make it board level?)
            Err(_) => Err("Error while getting mode of operation".into()),
        }
    }

    fn set_control_mode(&mut self, _mode: [u8; 3]) -> Result<()> {
        if !(_mode[0] == _mode[1] && _mode[1] == _mode[2]) {
            return Err("Error, invalid control mode".into());
        }
        self.io.set_mode_of_operation(self.id, _mode[0] as u32);
        Ok(())
    }

    fn emergency_stop(&mut self) {
        self.io.emergency_stop(self.id);
        error!("EMERCENCY STOP!");
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

#[allow(dead_code)]
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
    let _shortest_to_current = angle_diff(0.0, pos);
    let _gearbox_turn = 0.0;

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

#[allow(dead_code)]
pub fn angle_diff(angle_a: f64, angle_b: f64) -> f64 {
    let mut angle = angle_a - angle_b;
    angle = (angle + PI) % TAU - PI;
    if angle < -PI {
        angle + TAU
    } else {
        angle
    }
}

#[allow(dead_code)]
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
        io::{poulpe_ethercat::find_closest_offset_to_zero, Orbita3dIOConfig},
        Orbita3dConfig,
    };

    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/ethercat_poulpe.yaml").unwrap();

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dIOConfig::PoulpeEthercat(dxl_config) = config.io {
            assert_eq!(config.kinematics_model.alpha, 54.0_f64.to_radians());
            assert_eq!(config.kinematics_model.gamma_min, 40.0_f64.to_radians());
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, PI / 2.0);
            assert_eq!(config.kinematics_model.gamma_max, PI);
            assert!(config.kinematics_model.passiv_arms_direct);

            if let crate::ZeroType::FirmwareZero(_) = config.disks.zeros {
            } else {
                panic!("Wrong config type");
            }
            assert_eq!(config.disks.reduction, 5.333333333333333333);
            // assert_eq!(config.disks.reduction, 4.2666667); //Old Orbita

            assert_eq!(dxl_config.url, "http://127.0.0.1:50098");
            assert_eq!(dxl_config.id, 0);
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

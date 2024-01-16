use motor_toolbox_rs::{Limit, MissingResisterErrror, MotorsController, RawMotorsIO, Result, PID};
use rustypot::{
    device::orbita3d_poulpe::{self, MotorValue, MotorPositionSpeedLoad},
    DynamixelSerialIO,
};
use serde::{Deserialize, Serialize};
use serialport::{SerialPort, TTYPort};
use std::{f64::consts::PI, time::Duration};
use std::{error::Error, thread, time::Instant};

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
    hall_indices: [Option<u8>; 3],
    limits: [Option<Limit>; 3],
}

impl DynamixelPoulpeController {
    /// Creates a new DynamixelPoulpeController
    pub fn new(serial_port: &str, id: u8, zero: ZeroType, reductions: f64) -> Result<Self> {
        let mut controller = Self {
            serial_port: Box::new(serialport::new(serial_port, 2_000_000)
                .timeout(Duration::from_millis(10))
                .open_native()?),
            io: DynamixelSerialIO::v1(),
            id,
            offsets: [None; 3],
            reduction: [Some(reductions); 3],
            // motor_reduction: [Some(motor_reductions); 3],
            limits: [None; 3],
	    hall_indices: [None; 3],

        };

	// TTYPort::set_exclusive(&mut controller.serial_port, true)?;
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
		thread::sleep(Duration::from_millis(1));
                let curr_hall = orbita3d_poulpe::read_index_sensor(&controller.io, controller.serial_port.as_mut(), controller.id)?;
		let hall_idx:[u8;3]=[curr_hall.top,curr_hall.middle,curr_hall.bottom];
		log::info!("Current position: {:?} current hall: {:?}", current_pos, curr_hall);

		// let hall_idx:[u8;3]=[0,5,10]; //TEST
		if hall_idx.contains(&255) //255 is the value when no hall sensor is detected
		{
		    log::error!("HallZero: Hall sensor not found! Check 'Donut' I2C connection or maybe configure another zeroing method?");
		    return Err(Box::new(MissingResisterErrror("Hall sensor not found".to_string())));
		}

		thread::sleep(Duration::from_millis(1));

		log::debug!("HallZero: curr_pos: {:?} curr_hall_idx: {:?} hardware_zero: {:?} hall_zero: {:?}", current_pos, hall_idx,zero.hardware_zero,zero.hall_indice);

                zero.hardware_zero
                    .iter()
                    .zip(current_pos.iter())
                    .zip(hall_idx.iter())
                    .zip(zero.hall_indice.iter())
                    .enumerate()
                    .for_each(|(i, (((&hardware_zero, &current_pos), &hall_zero), &hall_idx))| {
                        controller.offsets[i] = Some(find_position_with_hall(
                            current_pos,
                            hardware_zero,
			    hall_zero,
			    hall_idx,
                            reductions,
                        ));
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
            MotorValue
	    {
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
        Ok([
            vel.top as f64,
            vel.middle as f64,
            vel.bottom as f64,
        ])
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
        let fb=orbita3d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue{
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        );

	match fb{

	    Ok(_)=>Ok(()),
	    Err(e)=>Err(e),
	}

    }


    fn set_target_position_fb(&mut self, position: [f64; 3]) -> Result<[f64;9]> {
        let fb=orbita3d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue{
                top: position[0] as f32,
                middle: position[1] as f32,
                bottom: position[2] as f32,
            },
        );

	match fb{
	    Ok(fb)=>Ok([fb.position.top as f64,fb.position.middle as f64,fb.position.bottom as f64,fb.speed.top as f64,fb.speed.middle as f64,fb.speed.bottom as f64,fb.load.top as f64,fb.load.middle as f64,fb.load.bottom as f64]),
	    Err(e)=>Err(e),
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
            MotorValue
	    {
                top: _velocity[0] as u32,
                middle: _velocity[1] as u32,
                bottom: _velocity[2] as u32,
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
            MotorValue
	    {
                top: _torque[0] as u16,
                middle: _torque[1] as u16,
                bottom: _torque[2] as u16,
            },
        )

    }

    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        orbita3d_poulpe::read_position_pid(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
		[
		    PID{
			p: thetas.top.p as f64,
			i: thetas.top.i as f64,
			d: 0.0,
		    },
		    PID{
			p: thetas.middle.p as f64,
			i: thetas.middle.i as f64,
			d: 0.0,
		    },
		    PID{
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
	    MotorValue
	    {
		top: orbita3d_poulpe::Pid{
		    p: _pid[0].p as i16,
		    i: _pid[0].i as i16,
		},
		middle: orbita3d_poulpe::Pid{
		    p: _pid[1].p as i16,
		    i: _pid[1].i as i16,
		},
		bottom: orbita3d_poulpe::Pid{
		    p: _pid[2].p as i16,
		    i: _pid[2].i as i16,
		},
	    },
	)
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

fn find_position_with_hall(current_position: f64, hardware_zero: f64, hall_zero: u8, hall_index:u8, reduction:f64) -> f64
{
    //! Find the current position corrected by the hall sensor
    //! There is 16 Hall sensors on the disk output and a ratio 'reduction' between the disk and the motor gearbox output
    //! We knwow the 'hall_zero' index which correspond to the index of the Hall sensor closest to the disk zero position
    //! We also know the 'hall_index' which is the index of the Hall sensor closest to the current position
    //! Finally we know the 'hardware_zero' which is the position of the disk zero

    let mut offset:[f64;95]=[0.0;95]; // 16 Hall +/- 2 full turns + 15 (=32+15=47) => 3 turns: we fall back on the same position...
    let hall_offset = 2.0 * PI / 16.0*reduction;
    for i in 0..95
    {
	offset[i]=hardware_zero-(-((i as f64)-47.0) * hall_offset);

    }

    log::debug!("possible offset: {:?}", offset);
    let pos=current_position-(hall_index as i16 - hall_zero as i16) as f64*hall_offset;
    log::debug!("current pos with hall: {:?}", pos);

    let best = offset
        .iter()
        .map(|&p| (p - pos).abs())
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| offset[i])
        .unwrap();

    log::debug!("best match: {}", best);
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
            // assert_eq!(config.disks.reduction, 5.33333334); //TODO
	    assert_eq!(config.disks.reduction,4.2666667); //Old Orbita

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

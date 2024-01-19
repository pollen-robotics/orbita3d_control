use cache_cache::Cache;
use motor_toolbox_rs::{MotorsController, RawMotorsIO, Result, PID};

use crate::ZeroType;

use super::DynamixelSerialController;

#[derive(Debug)]
/// CachedDynamixelSerialController - Cached version of the DynamixelSerialController
pub struct CachedDynamixelSerialController {
    inner: DynamixelSerialController,

    torque_on: Cache<u8, [bool; 3]>,
    target_position: Cache<u8, [f64; 3]>,
    velocity_limit: Cache<u8, [f64; 3]>,
    torque_limit: Cache<u8, [f64; 3]>,
    pid_gains: Cache<u8, [PID; 3]>,
}

impl CachedDynamixelSerialController {
    pub fn new(serial_port: &str, id: u8, zero: ZeroType, reductions: f64) -> Result<Self> {
        Ok(Self {
            inner: DynamixelSerialController::new(serial_port, id, zero, reductions)?,

            torque_on: Cache::keep_last(),
            target_position: Cache::keep_last(),
            velocity_limit: Cache::keep_last(),
            torque_limit: Cache::keep_last(),
            pid_gains: Cache::keep_last(),
        })
    }
}

impl MotorsController<3> for CachedDynamixelSerialController {
    fn io(&mut self) -> &mut dyn RawMotorsIO<3> {
        self
    }

    fn offsets(&self) -> [Option<f64>; 3] {
        self.inner.offsets()
    }

    fn reduction(&self) -> [Option<f64>; 3] {
        self.inner.reduction()
    }

    fn limits(&self) -> [Option<motor_toolbox_rs::Limit>; 3] {
        self.inner.limits()
    }
}

impl RawMotorsIO<3> for CachedDynamixelSerialController {
    fn is_torque_on(&mut self) -> Result<[bool; 3]> {
        self.torque_on
            .entry(self.inner.id())
            .or_try_insert_with(|_| RawMotorsIO::is_torque_on(&mut self.inner))
    }
    fn set_torque(&mut self, on: [bool; 3]) -> Result<()> {
        let current_on = RawMotorsIO::is_torque_on(self)?;

        if current_on != on {
            RawMotorsIO::set_torque(&mut self.inner, on)?;

            self.torque_on.insert(self.inner.id(), on);
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 3]> {
        RawMotorsIO::get_current_position(&mut self.inner)
    }
    fn get_current_velocity(&mut self) -> Result<[f64; 3]> {
        RawMotorsIO::get_current_velocity(&mut self.inner)
    }
    fn get_current_torque(&mut self) -> Result<[f64; 3]> {
        RawMotorsIO::get_current_torque(&mut self.inner)
    }

    fn get_target_position(&mut self) -> Result<[f64; 3]> {
        self.target_position
            .entry(self.inner.id())
            .or_try_insert_with(|_| RawMotorsIO::get_target_position(&mut self.inner))
    }
    fn set_target_position(&mut self, position: [f64; 3]) -> Result<()> {
        let current_position = RawMotorsIO::get_target_position(self)?;

        if current_position != position {
            RawMotorsIO::set_target_position(&mut self.inner, position)?;

            self.target_position.insert(self.inner.id(), position);
        }

        Ok(())
    }


    fn set_target_position_fb(&mut self, position: [f64; 3]) -> Result<[f64;9]> {
        let current_position = RawMotorsIO::get_target_position(self)?;

	let mut fb = [0.0;9];
        if current_position != position {
            fb=RawMotorsIO::set_target_position_fb(&mut self.inner, position)?;

            self.target_position.insert(self.inner.id(), position);
        }

        Ok(fb)
    }



    fn get_velocity_limit(&mut self) -> Result<[f64; 3]> {
        self.velocity_limit
            .entry(self.inner.id())
            .or_try_insert_with(|_| RawMotorsIO::get_velocity_limit(&mut self.inner))
    }
    fn set_velocity_limit(&mut self, velocity: [f64; 3]) -> Result<()> {
        let current_velocity = RawMotorsIO::get_velocity_limit(self)?;

        if current_velocity != velocity {
            RawMotorsIO::set_velocity_limit(&mut self.inner, velocity)?;

            self.velocity_limit.insert(self.inner.id(), velocity);
        }

        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 3]> {
        self.torque_limit
            .entry(self.inner.id())
            .or_try_insert_with(|_| RawMotorsIO::get_torque_limit(&mut self.inner))
    }
    fn set_torque_limit(&mut self, torque: [f64; 3]) -> Result<()> {
        let current_torque = RawMotorsIO::get_torque_limit(self)?;

        if current_torque != torque {
            RawMotorsIO::set_torque_limit(&mut self.inner, torque)?;

            self.torque_limit.insert(self.inner.id(), torque);
        }

        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 3]> {
        self.pid_gains
            .entry(self.inner.id())
            .or_try_insert_with(|_| RawMotorsIO::get_pid_gains(&mut self.inner))
    }
    fn set_pid_gains(&mut self, pid: [PID; 3]) -> Result<()> {
        let current_pid = RawMotorsIO::get_pid_gains(self)?;

        if current_pid != pid {
            RawMotorsIO::set_pid_gains(&mut self.inner, pid)?;

            self.pid_gains.insert(self.inner.id(), pid);
        }

        Ok(())
    }
    fn get_axis_sensors(&mut self) -> Result<[f64; 3]> {
		RawMotorsIO::get_axis_sensors(&mut self.inner)
	}

}

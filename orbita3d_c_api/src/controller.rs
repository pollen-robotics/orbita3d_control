//! C API for the controller module of the Orbita3D library.
use std::{ffi::CStr, sync::Mutex};

use motor_toolbox_rs::PID;
use once_cell::sync::Lazy;
use orbita3d_controller::Orbita3dController;

use crate::sync_map::SyncMap;
use env_logger;
// use log::debug;
static UID: Lazy<Mutex<u32>> = Lazy::new(|| Mutex::new(0));
static CONTROLLER: Lazy<SyncMap<u32, Orbita3dController>> = Lazy::new(SyncMap::new);

fn print_error(e: Box<dyn std::error::Error>) {
    // eprintln!("[ORBITA_3D] {:?}", e);
    log::debug!("[ORBITA_3D] Error: {:?}", e);
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
/// Create the Orbita3dController from a config file.
///
/// # Arguments
/// * configfile: *const libc::c_char - The path to the config file.
/// * uid: *mut u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the controller was created successfully, 1 otherwise.
pub extern "C" fn orbita3d_controller_from_config(
    configfile: *const libc::c_char,
    uid: &mut u32,
) -> i32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    let _ = env_logger::try_init();

    match Orbita3dController::with_config(configfile) {
        Ok(controller) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, controller);
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Check if controller is activated (are the motors on)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * is_on: *mut bool - The result of the check.
/// # Returns
/// * i32 - 0 if the controller is activated, 1 otherwise.
pub extern "C" fn orbita3d_is_torque_on(uid: u32, is_on: &mut bool) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().is_torque_on() {
        Ok(torque) => {
            *is_on = torque;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Enable the controller (turn on the motors)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * reset_target: bool - Reset the target orientation to the current orientation.
/// # Returns
/// * i32 - 0 if the controller was enabled successfully, 1 otherwise.
pub extern "C" fn orbita3d_enable_torque(uid: u32, reset_target: bool) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .enable_torque(reset_target)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Disable the controller (turn off the motors)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the controller was disabled successfully, 1 otherwise.
pub extern "C" fn orbita3d_disable_torque(uid: u32) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().disable_torque() {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *mut [f64; 4] - The current orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_current_orientation(uid: u32, orientation: &mut [f64; 4]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_orientation() {
        Ok(ori) => {
            *orientation = ori;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *mut [f64; 3] - The current orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_current_rpy_orientation(uid: u32, rpy: &mut [f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_current_rpy_orientation()
    {
        Ok(ori) => {
            *rpy = ori;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current velocity of the platform
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *mut [f64; 3] - The current velocity of the platform.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_current_velocity(uid: u32, velocity: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_velocity() {
        Ok(vel) => {
            *velocity = vel;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current torque applied by the actuator
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * torque: *mut [f64; 3] - The current torque applied to the platform.
/// # Returns
/// * i32 - 0 if the torque was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_current_torque(uid: u32, torque: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_torque() {
        Ok(tor) => {
            *torque = tor;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current target orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *mut [f64; 4] - The current target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_target_orientation(uid: u32, orientation: &mut [f64; 4]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_orientation() {
        Ok(ori) => {
            *orientation = ori;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current target orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *mut [f64; 3] - The current target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_target_rpy_orientation(uid: u32, rpy: &mut [f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_target_rpy_orientation()
    {
        Ok(ori) => {
            *rpy = ori;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the target orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *const [f64; 4] - The target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_target_orientation(uid: u32, orientation: &[f64; 4]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(*orientation)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the target orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *const [f64; 3] - The target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_target_rpy_orientation(uid: u32, rpy: &[f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_rpy_orientation(*rpy)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the tatget orientation and return the current orientation (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *const [f64; 4] - The target orientation of the platform.
/// * feedback: *mut [f64; 10] - The feedback of the controller.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_target_orientation_fb(
    uid: u32,
    orientation: &[f64; 4],
    // feedback: &mut [f64; 10],
    feedback: &mut [f64; 4],
) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation_fb(*orientation)
    {
        Ok(fb) => {
            feedback[0] = fb.orientation[0]; //FIXME: I don't know how to include the struct definition in the C bindings...
            feedback[1] = fb.orientation[1];
            feedback[2] = fb.orientation[2];
            feedback[3] = fb.orientation[3];
            // feedback[4] = fb.velocity[0];
            // feedback[5] = fb.velocity[1];
            // feedback[6] = fb.velocity[2];
            // feedback[7] = fb.torque[0];
            // feedback[8] = fb.torque[1];
            // feedback[9] = fb.torque[2];
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the tatget orientation and return the current orientation (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *const [f64; 3] - The target orientation of the platform.
/// * feedback: *mut [f64; 10] - The feedback of the controller.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_target_rpy_orientation_fb(
    uid: u32,
    rpy: &[f64; 3],
    // feedback: &mut [f64; 10],
    feedback: &mut [f64; 3],
) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_rpy_orientation_fb(*rpy)
    {
        Ok(fb) => {
            feedback[0] = fb[0];
            feedback[1] = fb[1];
            feedback[2] = fb[2];
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current velocity of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *mut [f64; 3] - The current velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_velocity_limit(uid: u32, limit: &mut [f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(lim) => {
            *limit = lim;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the current velocity of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *const [f64; 3] - The current velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_raw_motors_velocity_limit(uid: u32, limit: &[f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current torque limit of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * limit: *mut [f64; 3] - The current torque limit of the motors.
/// # Returns
/// * i32 - 0 if the torque limit was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_torque_limit(uid: u32, limit: &mut [f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(lim) => {
            *limit = lim;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the current torque limit of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * limit: *const [f64; 3] - The current torque limit of the motors.
/// # Returns
/// * i32 - 0 if the torque limit was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_raw_motors_torque_limit(uid: u32, limit: &[f64; 3]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the current PID gains of the motors
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * gains: *mut [[f64; 3]; 3] - The current PID gains of the motors.
/// # Returns
/// * i32 - 0 if the PID gains were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_pid_gains(uid: u32, gains: &mut [[f64; 3]; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_pid_gains() {
        Ok([pid_top, pid_middle, pid_bottom]) => {
            gains[0][0] = pid_top.p;
            gains[0][1] = pid_top.i;
            gains[0][2] = pid_top.d;
            gains[1][0] = pid_middle.p;
            gains[1][1] = pid_middle.i;
            gains[1][2] = pid_middle.d;
            gains[2][0] = pid_bottom.p;
            gains[2][1] = pid_bottom.i;
            gains[2][2] = pid_bottom.d;

            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the current PID gains of the motors
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * gains: *const [[f64; 3]; 3] - The current PID gains of the motors.
/// # Returns
/// * i32 - 0 if the PID gains were set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_raw_motors_pid_gains(uid: u32, gains: &[[f64; 3]; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_raw_motors_pid_gains([
        PID {
            p: gains[0][0],
            i: gains[0][1],
            d: gains[0][2],
        },
        PID {
            p: gains[1][0],
            i: gains[1][1],
            d: gains[1][2],
        },
        PID {
            p: gains[2][0],
            i: gains[2][1],
            d: gains[2][2],
        },
    ]) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw motor current of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * current: *mut [f64; 3] - The current of the motors.
/// # Returns
/// * i32 - 0 if the current was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_current(
    uid: u32,
    raw_motors_current: &mut [f64; 3],
) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_current() {
        Ok(c) => {
            *raw_motors_current = c;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw motor velocity of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * raw_motors_velocity: *mut [f64; 3] - The velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_velocity(
    uid: u32,
    raw_motors_velocity: &mut [f64; 3],
) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_velocity() {
        Ok(v) => {
            *raw_motors_velocity = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw motor position of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * raw_motors_position: *mut [f64; 3] - The position of the motors.
/// # Returns
/// * i32 - 0 if the position was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_raw_motors_position(
    uid: u32,
    raw_motors_position: &mut [f64; 3],
) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_positions() {
        Ok(p) => {
            *raw_motors_position = p;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the state of the board (BoardState - u8)
/// - See more info [here](../../poulpe_ethercat_controller/register/enum.BoardStatus.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * state: *mut u8 - The state of the board.
/// # Returns
/// * i32 - 0 if the state was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_board_state(uid: u32, state: &mut u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_board_state() {
        Ok(s) => {
            *state = s;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the state of the board (BoardState - u8)
/// - See more info [here](../../poulpe_ethercat_controller/register/enum.BoardStatus.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * state: *const u8 - The state of the board.
/// # Returns
/// * i32 - 0 if the state was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_board_state(uid: u32, state: &u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_board_state(*state) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw axis sensors values
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * axis: *mut [f64; 3] - The axis sensors values.
/// # Returns
/// * i32 - 0 if the axis sensors values were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_axis_sensors(uid: u32, axis: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_axis_sensors() {
        Ok(a) => {
            *axis = a;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw axis sensor zeros
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * axis: *mut [f64; 3] - The axis sensor zeros.
/// # Returns
/// * i32 - 0 if the axis sensor zeros were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_axis_sensor_zeros(uid: u32, axis: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_axis_sensor_zeros() {
        Ok(a) => {
            *axis = a;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw motor temperatures
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * temp: *mut [f64; 3] - The motor temperatures.
/// # Returns
/// * i32 - 0 if the motor temperatures were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_motor_temperatures(uid: u32, temp: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_motor_temperatures() {
        Ok(t) => {
            *temp = t;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the raw board temperatures
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * temp: *mut [f64; 3] - The board temperatures.
/// # Returns
/// * i32 - 0 if the board temperatures were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_board_temperatures(uid: u32, temp: &mut [f64; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_board_temperatures() {
        Ok(t) => {
            *temp = t;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the error codes
/// - see more info [here](../../poulpe_ethercat_controller/state_machine/struct.ErrorFlags.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * error: *mut [i32; 3] - The error codes.
/// # Returns
/// * i32 - 0 if the error codes were retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_error_codes(uid: u32, error: &mut [i32; 3]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_error_codes() {
        Ok(c) => {
            *error = c;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Get the control mode ( CiA402 control mode  - u8) : 1 - Profile Position Mode, 3 - Profile Velocity Mode, 4 - Profile Torque Mode
/// - See more info [here](../../poulpe_ethercat_controller/state_machine/enum.CiA402ModeOfOperation.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * mode: *mut u8 - The control mode.
/// # Returns
/// * i32 - 0 if the control mode was retrieved successfully, 1 otherwise.
pub extern "C" fn orbita3d_get_control_mode(uid: u32, mode: &mut u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_control_mode() {
        Ok(m) => {
            *mode = m[0];
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Set the control mode ( CiA402 control mode  - u8) : 1 - Profile Position Mode, 3 - Profile Velocity Mode, 4 - Profile Torque Mode
/// - See more info [here](../../poulpe_ethercat_controller/state_machine/enum.CiA402ModeOfOperation.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * mode: *const u8 - The control mode.
/// # Returns
/// * i32 - 0 if the control mode was set successfully, 1 otherwise.
pub extern "C" fn orbita3d_set_control_mode(uid: u32, mode: &u8) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_control_mode([*mode, *mode, *mode])
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
/// Send the emergency stop signal to the controller
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the emergency stop signal was sent successfully, 1 otherwise.
pub extern "C" fn orbita3d_emergency_stop(uid: u32) -> i32 {
    CONTROLLER.get_mut(&uid).unwrap().emergency_stop();
    0
}

/// Get the next available unique identifier
fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}

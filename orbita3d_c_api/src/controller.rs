use std::{ffi::CStr, sync::Mutex};

use motor_toolbox_rs::PID;
use once_cell::sync::Lazy;
use orbita3d_controller::Orbita3dController;

use crate::sync_map::SyncMap;

static UID: Lazy<Mutex<u32>> = Lazy::new(|| Mutex::new(0));
static CONTROLLER: Lazy<SyncMap<u32, Orbita3dController>> = Lazy::new(SyncMap::new);

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita3d_controller_from_config(
    configfile: *const libc::c_char,
    uid: &mut u32,
) -> i32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    match Orbita3dController::with_config(configfile) {
        Ok(controller) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, controller);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_is_torque_on(uid: u32, is_on: &mut bool) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().is_torque_on() {
        Ok(torque) => {
            *is_on = torque;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_enable_torque(uid: u32, reset_target: bool) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .enable_torque(reset_target)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_disable_torque(uid: u32) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().disable_torque() {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_current_orientation(uid: u32, orientation: &mut [f64; 4]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().get_current_orientation() {
        Ok(ori) => {
            *orientation = ori;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_current_velocity(uid: u32, velocity: &mut [f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().get_current_velocity() {
        Ok(vel) => {
            *velocity = vel;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_current_torque(uid: u32, torque: &mut [f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().get_current_torque() {
        Ok(tor) => {
            *torque = tor;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_target_orientation(uid: u32, orientation: &mut [f64; 4]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().get_target_orientation() {
        Ok(ori) => {
            *orientation = ori;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_target_orientation(uid: u32, orientation: &[f64; 4]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(*orientation)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_target_orientation_fb(
    uid: u32,
    orientation: &[f64; 4],
    // feedback: &mut [f64; 10],
    feedback: &mut [f64; 4],
) -> i32 {
    // thread::sleep(Duration::from_millis(1));
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
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_raw_motors_velocity_limit(uid: u32, limit: &mut [f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(lim) => {
            *limit = lim;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_raw_motors_velocity_limit(uid: u32, limit: &[f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_raw_motors_torque_limit(uid: u32, limit: &mut [f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(lim) => {
            *limit = lim;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_raw_motors_torque_limit(uid: u32, limit: &[f64; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_raw_motors_pid_gains(uid: u32, gains: &mut [[f64; 3]; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
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
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_raw_motors_pid_gains(uid: u32, gains: &[[f64; 3]; 3]) -> i32 {
    // thread::sleep(Duration::from_millis(1));
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
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_get_board_state(uid: u32, state: &mut u8) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().get_board_state() {
        Ok(s) => {
            *state = s;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_set_board_state(uid: u32, state: &u8) -> i32 {
    // thread::sleep(Duration::from_millis(1));
    match CONTROLLER.get_mut(&uid).unwrap().set_board_state(*state) {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}

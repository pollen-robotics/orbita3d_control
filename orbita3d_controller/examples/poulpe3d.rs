use orbita3d_controller::Orbita3dController;
use orbita3d_kinematics::conversion;
use std::f64::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

// use log::info;
// use log::Level;

use clap::Parser;

// #[derive(Parser, Debug)]
// #[command(author, version, about, long_about = None)]
// struct Args {
//     /// tty
//     #[arg(short, long, default_value = "/dev/ttyUSB0")]
//     serialport: String,
//     /// baud
//     #[arg(short, long, default_value_t = 2_000_000)]
//     baudrate: u32,

//     /// id
//     #[arg(short, long, default_value_t = 42)]
//     id: u8,

//     ///sinus amplitude (f64)
//     #[arg(short, long, default_value_t = 10.0)]
//     amplitude: f32,

//     ///sinus frequency (f64)
//     #[arg(short, long, default_value_t = 1.0)]
//     frequency: f32,
// }

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    // #[arg(default_value = "config/dxl_poulpe.yaml")]
    #[arg(short, long, default_value = "config/ethercat_poulpe.yaml")]
    configfile: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    log::info!("Config file: {}", args.configfile);

    let mut controller = Orbita3dController::with_config(&args.configfile)?;

    let t = controller.is_torque_on();
    match t {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));
    let cur = controller.get_current_orientation()?;
    log::info!("Current orientation: {:?}", cur);
    thread::sleep(Duration::from_millis(10));

    let curtarget = controller.get_target_orientation()?;
    log::info!("Current target: {:?}", curtarget);

    thread::sleep(Duration::from_millis(10));
    let vellimit = controller.get_raw_motors_velocity_limit()?;
    log::info!("Vel limit: {:?}", vellimit);

    thread::sleep(Duration::from_millis(10));
    let torquelimit = controller.get_raw_motors_torque_limit()?;
    log::info!("Torque limit: {:?}", torquelimit);

    thread::sleep(Duration::from_millis(10));
    let pid = controller.get_raw_motors_pid_gains()?;
    log::info!("Pid: {:?}", pid);

    let t = controller.disable_torque();
    match t {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));
    let curtarget = controller.get_target_orientation()?;
    log::info!("Current target: {:?}", curtarget);

    ////
    thread::sleep(Duration::from_millis(10));
    let t = controller.enable_torque(true);
    match t {
        Ok(_) => log::info!("Torque is on"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));
    ////
    let curtarget = controller.get_target_orientation()?;
    log::info!("Current target: {:?}", curtarget);

    let init_pos = controller.get_current_orientation()?;
    log::info!("Initial orientation: {:?}", init_pos);

    let _ = controller.set_target_orientation([0.0, 0.0, 0.0, 1.0]);
    thread::sleep(Duration::from_millis(2000));
    let init_pos = controller.get_current_orientation()?;
    log::info!("zero orientation: {:?}", init_pos);

    thread::sleep(Duration::from_millis(1000));

    // set velocity and torque limits
    let res = controller.set_raw_motors_velocity_limit([1.0, 1.0, 1.0]);
    match res {
        Ok(_) => log::info!("Velocity limit set"),
        Err(e) => log::error!("Error: {}", e),
    }
    let res = controller.set_raw_motors_torque_limit([1.0, 1.0, 1.0]);
    match res {
        Ok(_) => log::info!("Torque limit set"),
        Err(e) => log::error!("Error: {}", e),
    }

    //DEBUGGING
    // let r = controller.disable_torque();
    // match r {
    //     Ok(_) => log::info!("Torque is off"),
    //     Err(e) => log::error!("Error: {}", e),
    // }
    // thread::sleep(Duration::from_millis(10000));

    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();
    let freq: f64 = 0.25;
    let amplitude: f64 = PI / 8.0;

    loop {
        if t > 10.0 {
            break;
        }

        t = now.elapsed().unwrap().as_secs_f32();

        let s = amplitude * (2.0 * PI * freq * t as f64).sin();
        // let s = (t as f64) / 10.0 * std::f64::consts::TAU;

        let target_yaw_mat = conversion::intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, s);
        let target = conversion::rotation_matrix_to_quaternion(target_yaw_mat);

        let fb = controller.set_target_orientation_fb(target);
        let axis = controller.get_axis_sensors();
        match (fb, axis) {
            (Ok(fb), Ok(axis)) => {
                // log::info!("Feedback: {:?}", fb);
                let rpy = conversion::quaternion_to_roll_pitch_yaw(fb.orientation);
                log::info!("rpy: {:?}", rpy);
                println!(
                    "{:?} {:?} {:?} {:?} {:?}",
                    t, s, rpy[0], rpy[1], rpy[2], axis[0], axis[1], axis[2]
                );
            }
            Err(e) => log::error!("Error: {}", e),
        }

        thread::sleep(Duration::from_millis(1));
    }

    let t = controller.disable_torque();
    match t {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}

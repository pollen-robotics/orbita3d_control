use orbita3d_controller::Orbita3dController;
use orbita3d_kinematics::conversion;
use std::f64::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration, time::Instant};

use log::Level;
use log::{info, warn};

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long, default_value = "config/dxl_poulpe.yaml")]
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

    let r = controller.disable_torque();
    match r {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));
    println!("Set Orbita3D in the zero position! (5s)");
    thread::sleep(Duration::from_secs(5));
    let mut axis_sensors = controller.get_axis_sensors()?;

    for s in axis_sensors.iter_mut() {
        *s *= 12.0 / 64.0; //FIXME how to get the ratio?
    }

    println!("Zeros to write to the yaml config");
    println!();
    println!("disks:");
    println!("  zeros: !HallZero");
    println!("    hardware_zero: {:?}", axis_sensors);
    println!("    hall_indice: [0, 5, 11] (FIXME: This should be known from the hardware)");

    Ok(())
}

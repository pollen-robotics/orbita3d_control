use orbita3d_controller::Orbita3dController;

use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration, time::Instant};

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
    #[arg(short, long, default_value = "config/dxl_poulpe.yaml")]
    configfile: String,

}


fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    println!("Config file: {}", args.configfile);


    let mut controller = Orbita3dController::with_config(&args.configfile)?;

    let t=controller.is_torque_on();
    match t {
		Ok(t) => println!("Torque is {}", t),
		Err(e) => println!("Error: {}", e),
	}

    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop{

        if t > 10.0 {
            break;
        }

        t = now.elapsed().unwrap().as_secs_f32();

	let fb=controller.set_target_orientation_fb([0.0, 0.0, 0.0,1.0]);
	match fb {
		Ok(fb) => println!("Feedback: {:?}", fb),
		Err(e) => println!("Error: {}", e),
	}

	thread::sleep(Duration::from_millis(1));


    }


    Ok(())
}

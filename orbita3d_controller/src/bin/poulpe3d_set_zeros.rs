use clap::Parser;
use orbita3d_controller::Orbita3dController;
use poulpe_ethercat_grpc::server::launch_server;
use std::{error::Error, thread, time::Duration};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    // #[arg(short, long, default_value = "config/dxl_poulpe.yaml")]
    #[arg(short, long, default_value = "config/ethercat_poulpe.yaml")]
    configfile: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    log::info!("Starting the server");
    // run in a thread, do not block main thread
    thread::spawn(|| {
        tokio::runtime::Builder::new_multi_thread()
            .worker_threads(4)
            .enable_all()
            .build()
            .unwrap()
            .block_on(launch_server("config/ethercat.yaml"))
            .unwrap();
    });
    thread::sleep(Duration::from_secs(2));

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
    let r = controller.get_reduction();
    println!("Reduction: {:?}", r);
    for s in axis_sensors.iter_mut() {
        if let Some(reduction) = r[0] {
            *s *= 1.0 / reduction;
        } else {
            *s *= 12.0 / 64.0;
        }
    }

    println!("Zeros to write to the yaml config");
    println!();
    // println!("disks:");
    // println!("  zeros: !HallZero");
    println!("hardware_zero: {:?}", axis_sensors);
    // println!("    hall_indice: [0, 5, 11] (FIXME: This should be known from the hardware)");

    println!("\nOR\n");
    println!("Zeros to pass to cargo at the firmware compile time");
    println!();
    println!(
        "{:.32},{:.32},{:.32}",
        axis_sensors[0], axis_sensors[1], axis_sensors[2]
    );
    println!("Cargo command line for a release firmware:");
    println!(
        "DEFMT_LOG=off ZEROS={:.32},{:.32},{:.32} cargo run --release",
        axis_sensors[0], axis_sensors[1], axis_sensors[2]
    );

    Ok(())
}

use orbita3d_controller::Orbita3dController;

use serde::Deserialize;
use serde::Serialize;

use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};
// use log::info;
// use log::Level;

use clap::Parser;

use rerun;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value = "config/ethercat_poulpe.yaml")]
    configfile: String,

    #[arg(short, long, default_value = "input.csv")]
    input_csv: String,

    #[arg(short, long, default_value = "output.csv")]
    output_csv: String,

    #[arg(short, long)]
    viewer: bool,
}

#[derive(Debug, Deserialize)]
// #[serde(rename_all = "PascalCase")]
struct Input {
    timestamp: f64,
    torque_on: bool,
    target_roll: f64,
    target_pitch: f64,
    target_yaw: f64,
    velocity_limit_top: f64,
    velocity_limit_middle: f64,
    velocity_limit_bottom: f64,
    torque_limit_top: f64,
    torque_limit_middle: f64,
    torque_limit_bottom: f64,
}

#[derive(Debug, Serialize)]
struct Output {
    timestamp: f64,
    torque_on: bool,
    present_roll: f64,
    present_pitch: f64,
    present_yaw: f64,
    present_velocity_roll: f64,
    present_velocity_pitch: f64,
    present_velocity_yaw: f64,
    present_torque_roll: f64,
    present_torque_pitch: f64,
    present_torque_yaw: f64,
    axis_senror_roll: f64,
    axis_senror_pitch: f64,
    axis_senror_yaw: f64,
    board_state: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    let rec = if args.viewer {
        let _rec = rerun::RecordingStreamBuilder::new("Test Orbita3d").spawn()?;
        Some(_rec)
    } else {
        None
    };

    log::info!("Config file: {}", args.configfile);
    log::info!("Input csv file: {}", args.input_csv);

    let infile = match std::fs::File::open(&args.input_csv) {
        Ok(f) => f,
        Err(e) => {
            log::error!("Error opening input csv file: {}", e);
            return Err(e.into());
        }
    };

    let outfile = match std::fs::File::create_new(&args.output_csv) {
        Ok(f) => f,
        Err(e) => {
            log::error!("Error opening output csv file: {}", e);
            return Err(e.into());
        }
    };

    let mut input_csv = csv::Reader::from_reader(infile);
    let mut output_csv = csv::Writer::from_writer(outfile);

    let mut controller = Orbita3dController::with_config(&args.configfile)?;
    let t = controller.is_torque_on();
    match t {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    let t = controller.disable_torque(); //Start with torque_off
    match t {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }

    thread::sleep(Duration::from_millis(1000));

    let now = SystemTime::now();

    for in_csv in input_csv.deserialize() {
        let t = now.elapsed().unwrap().as_secs_f64();
        let input_csv_data: Input = in_csv?;
        log::debug!("INPUT: {:?}", input_csv_data);

        //Read feedback from Orbita
        let curr_rpy = controller.get_current_rpy_orientation()?;
        let torque = controller.is_torque_on()?;
        let curr_vel = controller.get_current_velocity()?;
        let curr_torque = controller.get_current_torque()?;
        let curr_axis = controller.get_axis_sensors()?;
        let curr_state = controller.get_board_state()?;
        output_csv.serialize(Output {
            timestamp: t,
            torque_on: torque,
            present_roll: curr_rpy[0],
            present_pitch: curr_rpy[1],
            present_yaw: curr_rpy[2],
            present_velocity_roll: curr_vel[0],
            present_velocity_pitch: curr_vel[1],
            present_velocity_yaw: curr_vel[2],
            present_torque_roll: curr_torque[0],
            present_torque_pitch: curr_torque[1],
            present_torque_yaw: curr_torque[2],
            axis_senror_roll: curr_axis[0],
            axis_senror_pitch: curr_axis[1],
            axis_senror_yaw: curr_axis[2],
            board_state: curr_state,
        })?;
        output_csv.flush()?;

        let tosleep = (input_csv_data.timestamp - t) * 1000.0;
        thread::sleep(Duration::from_millis(tosleep as u64));

        //Write commands to Orbita
        if input_csv_data.torque_on {
            controller.enable_torque(true)?;
        } else {
            controller.disable_torque()?;
        }
        controller.set_target_rpy_orientation([
            input_csv_data.target_roll,
            input_csv_data.target_pitch,
            input_csv_data.target_yaw,
        ])?;

        controller.set_raw_motors_velocity_limit([
            input_csv_data.velocity_limit_top,
            input_csv_data.velocity_limit_middle,
            input_csv_data.velocity_limit_bottom,
        ])?;

        controller.set_raw_motors_torque_limit([
            input_csv_data.torque_limit_top,
            input_csv_data.torque_limit_middle,
            input_csv_data.torque_limit_bottom,
        ])?;

        // Rerun
        if let Some(rec) = &rec {
            rec.set_time_seconds("timestamp", t);
            rec.log(
                "target/torque_on",
                &rerun::Scalar::new(if input_csv_data.torque_on { 1.0 } else { 0.0 }),
            )?;
            rec.log("target/board_state", &rerun::Scalar::new(curr_state as f64))?;

            rec.log(
                "position/target/roll",
                &rerun::Scalar::new(input_csv_data.target_roll),
            )?;
            rec.log(
                "position/target/pitch",
                &rerun::Scalar::new(input_csv_data.target_pitch),
            )?;
            rec.log(
                "position/target/yaw",
                &rerun::Scalar::new(input_csv_data.target_yaw),
            )?;

            rec.log("position/present/roll", &rerun::Scalar::new(curr_rpy[0]))?;
            rec.log("position/present/pitch", &rerun::Scalar::new(curr_rpy[1]))?;
            rec.log("position/present/yaw", &rerun::Scalar::new(curr_rpy[2]))?;

            rec.log("velocity/present/roll", &rerun::Scalar::new(curr_vel[0]))?;
            rec.log("velocity/present/pitch", &rerun::Scalar::new(curr_vel[1]))?;
            rec.log("velocity/present/yaw", &rerun::Scalar::new(curr_vel[2]))?;

            rec.log("torque/present/roll", &rerun::Scalar::new(curr_torque[0]))?;
            rec.log("torque/present/pitch", &rerun::Scalar::new(curr_torque[1]))?;
            rec.log("torque/present/yaw", &rerun::Scalar::new(curr_torque[2]))?;

            rec.log(
                "position/axis_sensor/roll",
                &rerun::Scalar::new(curr_axis[0]),
            )?;
            rec.log(
                "position/axis_sensor/pitch",
                &rerun::Scalar::new(curr_axis[1]),
            )?;
            rec.log(
                "position/axis_sensor/yaw",
                &rerun::Scalar::new(curr_axis[2]),
            )?;

            rec.log(
                "limits/velocity/top",
                &rerun::Scalar::new(input_csv_data.velocity_limit_top),
            )?;
            rec.log(
                "limits/velocity/middle",
                &rerun::Scalar::new(input_csv_data.velocity_limit_middle),
            )?;
            rec.log(
                "limits/velocity/bottom",
                &rerun::Scalar::new(input_csv_data.velocity_limit_bottom),
            )?;

            rec.log(
                "limits/torque/top",
                &rerun::Scalar::new(input_csv_data.torque_limit_top),
            )?;
            rec.log(
                "limits/torque/middle",
                &rerun::Scalar::new(input_csv_data.torque_limit_middle),
            )?;
            rec.log(
                "limits/torque/bottom",
                &rerun::Scalar::new(input_csv_data.torque_limit_bottom),
            )?;
        }
    }

    let torque = controller.disable_torque();
    match torque {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}

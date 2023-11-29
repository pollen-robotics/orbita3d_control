use std::time::Duration;

use clap::Parser;
use orbita3d_kinematics::conversion;

#[macro_use]
extern crate timeit;

/// Benchmark the communication with an Orbita3d
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[arg(short, long)]
    configfile: String,

    /// Number of iterations
    #[arg(short, long, default_value = "1000")]
    iterations: usize,
}

fn controller_hwi_read_iteration(
    orbita3d: &mut orbita3d_controller::Orbita3dController,
) -> Result<(), Box<dyn std::error::Error>> {
    let q = orbita3d.get_current_orientation()?;
    let _ = conversion::matrix_to_intrinsic_roll_pitch_yaw(
        conversion::quaternion_to_rotation_matrix(q[0], q[1], q[2], q[3]),
    );

    let _ = orbita3d.is_torque_on()?;

    let _ = orbita3d.get_raw_motors_pid_gains()?;

    Ok(())
}

fn controller_hwi_write_iteration(
    orbita3d: &mut orbita3d_controller::Orbita3dController,
) -> Result<(), Box<dyn std::error::Error>> {
    let rpy = [0.0, 0.0, 0.0];
    let q = conversion::rotation_matrix_to_quaternion(
        conversion::intrinsic_roll_pitch_yaw_to_matrix(rpy[0], rpy[1], rpy[2]),
    );

    orbita3d.set_target_orientation(q)?;
    orbita3d.enable_torque(false)?;

    Ok(())
}

fn controller_hwi_iteration(
    orbita3d: &mut orbita3d_controller::Orbita3dController,
) -> Result<(), Box<dyn std::error::Error>> {
    controller_hwi_read_iteration(orbita3d)?;
    controller_hwi_write_iteration(orbita3d)?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("Connecting to Orbita3d: {:?}", args.configfile);
    let mut orbita3d = orbita3d_controller::Orbita3dController::with_config(&args.configfile)?;

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        let _ = orbita3d.get_current_orientation()?;
    }));
    println!(
        "Reading current position: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_read_iteration(&mut orbita3d)?;
    }));
    println!(
        "Read hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_write_iteration(&mut orbita3d)?;
    }));
    println!(
        "Write hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_iteration(&mut orbita3d)?;
    }));
    println!(
        "Read/Write hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let kin = orbita3d_kinematics::Orbita3dKinematicsModel {
        alpha: 0.9424777960769379,
        gamma_min: 0.0,
        offset: 0.0,
        beta: 1.5707963267948966,
        gamma_max: 3.141592653589793,
        passiv_arms_direct: true,
    };

    let thetas = [0.20, -1.0, 0.320];
    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        kin.compute_forward_kinematics(thetas);
    }));
    println!(
        "Forward kinematics: {} µs",
        dur.as_micros() as f64 / args.iterations as f64
    );

    let rot = kin.compute_forward_kinematics(thetas);
    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        let _ = kin.compute_inverse_kinematics(rot);
    }));
    println!(
        "Inverse kinematics: {} µs",
        dur.as_micros() as f64 / args.iterations as f64
    );

    Ok(())
}

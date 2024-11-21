use orbita3d_controller::Orbita3dController;

use std::{error::Error, thread, time::Duration};

use clap::Parser;
use rand::Rng;

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
    #[arg(short, long, default_value = "config/fake.yaml")]
    configfile: String,
}

fn check(fb: Result<[f64; 3], Box<dyn Error>>, goalrpy: [f64; 3]) {
    match fb {
        Ok(fb) => {
            log::info!("Feedback: {:?}", fb);
            if !fb
                .iter()
                .zip(goalrpy.iter())
                .all(|(a, b)| (a - b).abs() < 0.001)
            {
                log::error!("FAIL! goal: {:?} fb: {:?}", goalrpy, fb);

                // return Ok(());
            } else {
                log::debug!("PASS goal: {:?} fb: {:?}", goalrpy, fb);
            }
        }
        Err(e) => log::error!("Error: {}", e),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    log::info!("Config file: {}", args.configfile);
    log::warn!("THIS TEST SHOULD BE RUN IN FAKE MODE!");
    thread::sleep(Duration::from_millis(2000));

    let mut controller = Orbita3dController::with_config(&args.configfile)?;

    let t = controller.is_torque_on();
    match t {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));

    let t = controller.enable_torque(true);
    match t {
        Ok(_) => log::info!("Torque is on"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));

    let pos = controller.get_current_orientation()?;
    log::debug!("Current pos: {:?}", pos);
    thread::sleep(Duration::from_millis(10));
    log::debug!("Setting pos to 0,0,0");
    let fb = controller.set_target_orientation_fb([0.0, 0.0, 0.0, 1.0]);
    thread::sleep(Duration::from_millis(20));

    match fb {
        Ok(fb) => log::info!("Feedback: {:?}", fb),
        Err(e) => log::error!("Error: {}", e),
    }

    let pos = controller.get_current_orientation()?;
    log::debug!("Current pos: {:?}", pos);

    thread::sleep(Duration::from_millis(10));

    log::warn!("Testing RPY orientations");

    let mut rng = rand::thread_rng();
    let mut nb_errors: i32 = 0;
    let mut nb_pass: i32 = 0;

    for i in 0..1000 {
        let goalrpy: [f64; 3] = [
            rng.gen_range(-30.0_f64.to_radians()..30.0_f64.to_radians()),
            rng.gen_range(-30.0_f64.to_radians()..30.0_f64.to_radians()),
            // 0.0,
            // 0.0,
            rng.gen_range(-900_f64.to_radians()..900.0_f64.to_radians()),
        ];
        log::warn!("{:?}: GOAL: {:?}", i, goalrpy);
        let fb = controller.set_target_rpy_orientation_fb(goalrpy);
        match fb {
            Ok(fb) => {
                log::info!("Feedback: {:?}", fb);
                if !fb
                    .iter()
                    .zip(goalrpy.iter())
                    .all(|(a, b)| (a - b).abs() < 0.001)
                {
                    log::error!("FAIL! goal: {:?} fb: {:?}", goalrpy, fb);
                    nb_errors += 1;
                    // return Ok(());
                } else {
                    nb_pass += 1;
                }
            }
            Err(e) => log::error!("Error: {}", e),
        }
    }

    log::error!("PASS {nb_pass} FAIL {nb_errors}\n\n");

    // Some particular cases

    let testgoal = [0.0, 0.0, 0.0];
    log::debug!("test0 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    let testgoal = [0.254094568674348, -0.4941453106759718, 12.773118573893246];
    log::debug!("test1 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [-0.522031955980035, 0.42673257898434913, -6.243858357924321];
    log::debug!("test2 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.32926029516436983, 0.49304842815495553, -3.021737893204481];
    log::debug!("test3 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [
        0.11075544973424115,
        -0.10496941816004264,
        -12.508312785250542,
    ];
    log::debug!("test4 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [-0.16291515774363804, 0.1089436806873755, 9.331672883679307];
    log::debug!("test5 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [
        -0.16291515774363804,
        0.1089436806873755,
        9.331672883679307 - std::f64::consts::TAU,
    ];
    log::debug!("test6 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [
        -0.10255885565548793,
        0.009602128594249165,
        -6.247874963070153,
    ];
    log::debug!("test7 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.4857022155993208, -0.4507141715595859, 5.768305673941647];
    log::debug!("test8 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.4857022155993208, -0.4507141715595859, -0.5148796332069132];
    log::debug!("test8bis {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.4857022155993208, -0.4507141715595859, 0.0];
    log::debug!("test8ter {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [
        -0.33270742064984016,
        0.20383417343026636,
        -3.684412719450107,
    ];
    log::debug!("test9 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.6800247253297892, 0.227258821252446, 12.32330519118726];
    log::debug!("test10 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [-0.47520725526211927, 0.6253306391024075, -7.277104606728697];
    log::debug!("test11 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [
        0.4612673206648755,
        -0.12651958314700162,
        -0.5558106095481818,
    ];
    log::debug!("test12 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.0, 0.0, 60.0_f64.to_radians()];
    log::debug!("test BASE OFFSET {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    let testgoal = [0.1973154999940666, 0.2937735806343913, -12.896980895625147];
    log::debug!("test13 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    log::debug!("\n\n");
    // let testgoal = [0.1483760233862842, 0.466902235138705, 3.1607743116155973];
    // let testgoal = [
    //     0.42718245141604405,
    //     0.029661080831071773,
    //     -3.510382902195902,
    // ];
    let testgoal = [-0.207195216319444, -0.1484046546447622, -6.835409619231218];
    log::debug!("test14 {:?}", testgoal);
    let fb = controller.set_target_rpy_orientation_fb(testgoal);
    check(fb, testgoal);

    let t = controller.disable_torque();
    match t {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));
    Ok(())
}

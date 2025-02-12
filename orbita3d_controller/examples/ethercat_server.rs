use poulpe_ethercat_grpc::server::launch_server;
use std::error::Error;
use tokio;

pub fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    log::info!("Starting server");
    // run in a thread std thread
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()
        .unwrap()
        .block_on(launch_server("config/ethercat.yaml"))?;

    return Ok(());
}

use serde::{Deserialize, Serialize};

mod cached_dynamixel_serial;
pub use cached_dynamixel_serial::CachedDynamixelSerialController;
mod dynamixel_serial;
pub use dynamixel_serial::DynamixelSerialController;
mod fake;

mod cached_poulpe;
pub use cached_poulpe::CachedDynamixelPoulpeController;
mod poulpe;
pub use poulpe::DynamixelPoulpeController;
mod poulpe_ethercat;
pub use poulpe_ethercat::EthercatPoulpeController;

#[derive(Debug, Deserialize, Serialize)]
/// IOConfig
pub enum Orbita3dIOConfig {
    /// DynamixelSerial Config
    DynamixelSerial(dynamixel_serial::DynamixelSerialConfig),
    /// DynamixelPoule Config
    DynamixelPoulpe(poulpe::DynamixelPoulpeConfig),
    /// FakeMotors Config
    FakeMotors(fake::FakeConfig),
    /// Ethercat DynamixelPoule Config
    PoulpeEthercat(poulpe_ethercat::PoulpeEthercatConfig),
}

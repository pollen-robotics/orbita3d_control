use serde::{Deserialize, Serialize};

mod fake;

#[cfg(feature = "build_dynamixel")]
mod cached_dynamixel_serial;
#[cfg(feature = "build_dynamixel")]
pub use cached_dynamixel_serial::CachedDynamixelSerialController;
#[cfg(feature = "build_dynamixel")]
mod dynamixel_serial;
#[cfg(feature = "build_dynamixel")]
pub use dynamixel_serial::DynamixelSerialController;
#[cfg(feature = "build_dynamixel")]
mod cached_poulpe;
#[cfg(feature = "build_dynamixel")]
pub use cached_poulpe::CachedDynamixelPoulpeController;
#[cfg(feature = "build_dynamixel")]
mod poulpe;
#[cfg(feature = "build_dynamixel")]
pub use poulpe::DynamixelPoulpeController;

#[cfg(feature = "build_ethercat")]
mod poulpe_ethercat;
#[cfg(feature = "build_ethercat")]
pub use poulpe_ethercat::EthercatPoulpeController;

#[derive(Debug, Deserialize, Serialize)]
/// IOConfig
pub enum Orbita3dIOConfig {
    #[cfg(feature = "build_dynamixel")]
    /// DynamixelSerial Config
    DynamixelSerial(dynamixel_serial::DynamixelSerialConfig),
    #[cfg(feature = "build_dynamixel")]
    /// DynamixelPoule Config
    DynamixelPoulpe(poulpe::DynamixelPoulpeConfig),
    /// FakeMotors Config
    FakeMotors(fake::FakeConfig),
    #[cfg(feature = "build_ethercat")]
    /// Ethercat Poulpe Config
    PoulpeEthercat(poulpe_ethercat::PoulpeEthercatConfig),
}

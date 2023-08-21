use serde::{Deserialize, Serialize};

mod dynamixel_serial;
pub use dynamixel_serial::DynamixelSerialController;
mod fake;

#[derive(Debug, Deserialize, Serialize)]
/// IOConfig
pub enum Orbita3dIOConfig {
    /// DynamixelSerial Config
    DynamixelSerial(dynamixel_serial::DynamixelSerialConfig),
    /// FakeMotors Config
    FakeMotors(fake::FakeConfig),
}

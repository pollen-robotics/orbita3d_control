use motor_toolbox_rs::FakeMotor;
use serde::{Deserialize, Serialize};

use crate::{Orbita3dController, Result};

#[derive(Debug, Deserialize, Serialize)]
pub struct FakeConfig {
    pub alpha: f64,
    pub gamma_min: f64,
    pub offset: f64,
    pub beta: f64,
    pub gamma_max: f64,
    pub passiv_arms_direct: bool,
}

impl Orbita3dController {
    pub fn with_fake_motors(
        alpha: f64,
        gamma_min: f64,
        offset: f64,
        beta: f64,
        gamma_max: f64,
        passiv_arms_direct: bool,
    ) -> Result<Self> {
        Ok(Self::new(
            [
                Box::<FakeMotor>::default(),
                Box::<FakeMotor>::default(),
                Box::<FakeMotor>::default(),
            ],
            alpha,
            gamma_min,
            offset,
            beta,
            gamma_max,
            passiv_arms_direct,
        ))
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::Orbita3dConfig;

    #[test]
    fn parse_config() {
        let config = r#"!FakeMotors
            alpha: 0.0
            gamma_min: 0.0
            offset: 0.0
            beta: 0.0
            gamma_max: 0.0
            passiv_arms_direct: false
        "#;

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_str(config);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dConfig::FakeMotors(config) = config {
            assert_eq!(config.alpha, 0.0);
            assert_eq!(config.gamma_min, 0.0);
            assert_eq!(config.offset, 0.0);
            assert_eq!(config.beta, 0.0);
            assert_eq!(config.gamma_max, 0.0);
            assert!(!config.passiv_arms_direct);
        } else {
            panic!("Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/fake.yaml").unwrap();

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dConfig::FakeMotors(config) = config {
            assert_eq!(config.alpha, 50.0_f64.to_radians());
            assert_eq!(config.gamma_min, 0.0);
            assert_eq!(config.offset, 0.0);
            assert_eq!(config.beta, PI / 2.0);
            assert_eq!(config.gamma_max, PI);
            assert!(config.passiv_arms_direct);
        } else {
            panic!("Wrong config type");
        }
    }
}

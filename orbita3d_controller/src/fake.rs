use motor_toolbox_rs::{FakeMotor, MultipleMotorsControllerWrapper};
use orbita3d_kinematics::Orbita3dKinematicsModel;
use serde::{Deserialize, Serialize};

use crate::Orbita3dController;

#[derive(Debug, Deserialize, Serialize)]
pub struct FakeConfig {
    pub kinematics_model: Orbita3dKinematicsModel,
}

impl Orbita3dController {
    pub fn with_fake_motors(config: FakeConfig) -> Self {
        Self {
            inner: Box::new(MultipleMotorsControllerWrapper::new([
                Box::<FakeMotor>::default(),
                Box::<FakeMotor>::default(),
                Box::<FakeMotor>::default(),
            ])),
            kinematics: config.kinematics_model,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::Orbita3dConfig;

    #[test]
    fn parse_config() {
        let config = r#"!FakeMotors
        kinematics_model:
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
            assert_eq!(config.kinematics_model.alpha, 0.0);
            assert_eq!(config.kinematics_model.gamma_min, 0.0);
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, 0.0);
            assert_eq!(config.kinematics_model.gamma_max, 0.0);
            assert!(!config.kinematics_model.passiv_arms_direct);
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
            assert_eq!(config.kinematics_model.alpha, 50.0_f64.to_radians());
            assert_eq!(config.kinematics_model.gamma_min, 0.0);
            assert_eq!(config.kinematics_model.offset, 0.0);
            assert_eq!(config.kinematics_model.beta, PI / 2.0);
            assert_eq!(config.kinematics_model.gamma_max, PI);
            assert!(config.kinematics_model.passiv_arms_direct);
        } else {
            panic!("Wrong config type");
        }
    }
}

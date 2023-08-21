use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
/// Fake Motor IO Config
pub struct FakeConfig {}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::{io::Orbita3dIOConfig, Orbita3dConfig};

    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/fake.yaml").unwrap();

        let config: Result<Orbita3dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita3dIOConfig::FakeMotors(_) = config.io {
            if let crate::ZeroType::ZeroStartup(_) = config.disks.zeros {
            } else {
                panic!("Wrong config type");
            }
            assert_eq!(config.disks.reduction, 1.0);

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

from tempfile import NamedTemporaryFile

import numpy as np
from orbita3d import Orbita3dController
from scipy.spatial.transform import Rotation


def get_fake_controller():
    with NamedTemporaryFile("w") as f:
        f.write(
            """
            io: !FakeMotors
            disks:
              zeros: !ZeroStartup
              reduction: 1.0
            kinematics_model:
              alpha: 0.8726646259971648 # 50 degrees
              gamma_min: 0.0
              offset: 0.0
              beta: 1.5707963267948966 # 90 degrees
              gamma_max: 3.141592653589793 # 180 degrees
              passiv_arms_direct: true
            """
        )
        f.seek(0)

        return Orbita3dController.from_config(f.name)


def random_quat():
    roll = np.random.uniform(-np.deg2rad(30), np.deg2rad(30))
    pitch = np.random.uniform(-np.deg2rad(30), np.deg2rad(30))
    yaw = np.random.uniform(-np.deg2rad(30), np.deg2rad(30))

    return Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()

import numpy as np
from scipy.spatial.transform import Rotation

from orbita3d import KinematicsModel


model = KinematicsModel(
    alpha=np.deg2rad(50.0),
    gamma_min=np.deg2rad(0.0),
    offset=np.deg2rad(0.0),
    beta=np.deg2rad(90.0),
    gamma_max=np.deg2rad(180.0),
    passiv_arms_direct=True,
)


def test_kinematics_position():
    rpy = np.random.uniform(-np.deg2rad(30), np.deg2rad(30), size=(3,))
    q = Rotation.from_euler("xyz", rpy).as_quat()

    disks = model.inverse_position(q)
    q_hat = model.forward_position(disks)

    assert np.allclose(q, q_hat)

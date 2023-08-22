import numpy as np

from . import get_fake_controller, random_quat


def test_controller():
    c = get_fake_controller()

    c.disable_torque()
    assert not c.is_torque_on()

    q = random_quat()
    c.set_target_orientation(q)
    assert np.allclose(c.get_target_orientation(), q)
    assert np.allclose(c.get_current_orientation(), (0, 0, 0, 1))

    c.enable_torque(reset_target=True)
    assert c.is_torque_on()

    q = random_quat()
    c.set_target_orientation(q)
    assert np.allclose(c.get_target_orientation(), q)
    assert np.allclose(c.get_current_orientation(), q)

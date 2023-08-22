from typing import Tuple

from ._orbita3d import ffi
from ._orbita3d import lib


class KinematicsModel:
    """Orbita3d kinematics model"""

    def __init__(
        self,
        alpha: float,
        gamma_min: float,
        offset: float,
        beta: float,
        gamma_max: float,
        passiv_arms_direct: bool,
    ) -> None:
        """Create a new Orbita3d kinematics model.

        See the [README.md](https://github.com/pollen-robotics/orbita3d_control/orbita3d_kinematics) for more information about the parameters.
        """
        self.model = ffi.new(
            "struct Orbita3dKinematicsModel *",
            lib.create_orbita3d_kinematics_model(
                alpha,
                gamma_min,
                offset,
                beta,
                gamma_max,
                passiv_arms_direct,
            ),
        )

    def forward_position(
        self, thetas: Tuple[float, float, float]
    ) -> Tuple[float, float, float, float]:
        """Compute the forward position kinematics of the Orbita3d.

        Args:
            thetas: The three motors angles (in radians).
        Returns:
            The quaternion representing the end-effector orientation (qx, qy, qz, qw).
        """
        thetas = ffi.new("double(*)[3]", tuple(thetas))
        q = ffi.new("double(*)[4]")

        check(lib.orbita3d_kinematics_forward_position(self.model, thetas, q))

        return tuple(q[0])

    def forward_velocity(
        self,
        thetas: Tuple[float, float, float],
        thetas_velocity: Tuple[float, float, float],
    ) -> Tuple[float, float, float, float]:
        """Compute the forward velocity kinematics of the Orbita3d.

        Args:
            thetas: The three motors angles (in radians).
            thetas_velocity: The three motors velocities (in radians per second).
        Returns:
            The quaternion representing the end-effector orientation velocity (qx, qy, qz, qw).
        """
        thetas = ffi.new("double(*)[3]", tuple(thetas))
        thetas_velocity = ffi.new("double(*)[3]", tuple(thetas_velocity))
        q_velocity = ffi.new("double(*)[4]")

        check(
            lib.orbita3d_kinematics_forward_velocity(
                self.model, thetas, thetas_velocity, q_velocity
            )
        )

        return tuple(q_velocity[0])

    def forward_torque(
        self,
        thetas: Tuple[float, float, float],
        thetas_torque: Tuple[float, float, float],
    ) -> Tuple[float, float, float, float]:
        """Compute the forward torque kinematics of the Orbita3d.

        Args:
            thetas: The three motors angles (in radians).
            thetas_torque: The three motors torques (in Newton meters).
        Returns:
            The quaternion representing the end-effector orientation torque (qx, qy, qz, qw).
        """
        thetas = ffi.new("double(*)[3]", tuple(thetas))
        thetas_torque = ffi.new("double(*)[3]", tuple(thetas_torque))
        q_torque = ffi.new("double(*)[4]")

        check(
            lib.orbita3d_kinematics_forward_torque(
                self.model, thetas, thetas_torque, q_torque
            )
        )

        return tuple(q_torque[0])

    def inverse_position(
        self, q: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float]:
        """Compute the inverse position kinematics of the Orbita3d.

        Args:
            q: The quaternion representing the end-effector orientation (qx, qy, qz, qw).
        Returns:
            The three motors angles (in radians).
        """
        q = ffi.new("double(*)[4]", tuple(q))
        thetas = ffi.new("double(*)[3]")

        check(lib.orbita3d_kinematics_inverse_position(self.model, q, thetas))

        return tuple(thetas[0])

    def inverse_velocity(
        self,
        q: Tuple[float, float, float, float],
        q_velocity: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        """Compute the inverse velocity kinematics of the Orbita3d.

        Args:
            q: The quaternion representing the end-effector orientation (qx, qy, qz, qw).
            q_velocity: The quaternion representing the end-effector orientation velocity (qx, qy, qz, qw).
        Returns:
            The three motors velocities (in radians per second).
        """
        q = ffi.new("double(*)[4]", tuple(q))
        q_velocity = ffi.new("double(*)[4]", tuple(q_velocity))
        thetas_velocity = ffi.new("double(*)[3]")

        check(
            lib.orbita3d_kinematics_inverse_velocity(
                self.model, q, q_velocity, thetas_velocity
            )
        )

        return tuple(thetas_velocity[0])

    def inverse_torque(
        self,
        q: Tuple[float, float, float, float],
        q_torque: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        """Compute the inverse torque kinematics of the Orbita3d.

        Args:
            q: The quaternion representing the end-effector orientation (qx, qy, qz, qw).
            q_torque: The quaternion representing the end-effector orientation torque (qx, qy, qz, qw).
        Returns:
            The three motors torques (in Newton meters).
        """
        q = ffi.new("double(*)[4]", tuple(q))
        q_torque = ffi.new("double(*)[4]", tuple(q_torque))
        thetas_torque = ffi.new("double(*)[3]")

        check(
            lib.orbita3d_kinematics_inverse_torque(
                self.model, q, q_torque, thetas_torque
            )
        )

        return tuple(thetas_torque[0])


class Orbita3dController:
    """Orbita3d controller."""

    def __init__(self, uid: int) -> None:
        """You should not call this constructor dierctly. Use from_config instead."""
        self.uid = uid

    @classmethod
    def from_config(cls, config: str) -> "Orbita3dController":
        """Create a new Orbita3d controller from a configuration file.

        Args:
            config: The configuration file path.
        Returns:
            A new Orbita3dController.
        """
        uid = ffi.new("uint32_t *")
        check(lib.create_orbita3d_controller_from_config(config.encode("utf-8"), uid))
        return cls(uid[0])

    def is_torque_on(self) -> bool:
        """Check if the torque is on.

        Returns:
            True if the torque is on, False otherwise.
        """
        torque_on = ffi.new("bool *")
        check(lib.orbita3d_is_torque_on(self.uid, torque_on))
        return torque_on[0]

    def enable_torque(self, reset_target: bool) -> None:
        """Enable the torque.

        Args:
            reset_target (bool): If True, the target position is reset to the current position.
        """
        check(lib.orbita3d_enable_torque(self.uid, reset_target))

    def disable_torque(self) -> None:
        """Disable the torque."""
        check(lib.orbita3d_disable_torque(self.uid))

    def get_current_orientation(self) -> Tuple[float, float, float, float]:
        """Get the current orientation of the end-effector.

        Returns:
            The quaternion representing the end-effector orientation (qx, qy, qz, qw).
        """
        q = ffi.new("double(*)[4]")
        check(lib.orbita3d_get_current_orientation(self.uid, q))
        return tuple(q[0])

    def get_current_velocity(self) -> Tuple[float, float, float, float]:
        """Get the current velocity of the end-effector.

        Returns:
            The quaternion representing the end-effector orientation velocity (qx, qy, qz, qw).
        """
        q_velocity = ffi.new("double(*)[4]")
        check(lib.orbita3d_get_current_velocity(self.uid, q_velocity))
        return tuple(q_velocity[0])

    def get_current_torque(self) -> Tuple[float, float, float, float]:
        """Get the current torque of the end-effector.

        Returns:
            The quaternion representing the end-effector orientation torque (qx, qy, qz, qw).
        """
        q_torque = ffi.new("double(*)[4]")
        check(lib.orbita3d_get_current_torque(self.uid, q_torque))
        return tuple(q_torque[0])

    def get_target_orientation(self) -> Tuple[float, float, float, float]:
        """Get the target orientation of the end-effector.

        Returns:
            The quaternion representing the end-effector orientation (qx, qy, qz, qw).
        """
        q = ffi.new("double(*)[4]")
        check(lib.orbita3d_get_target_orientation(self.uid, q))
        return tuple(q[0])

    def set_target_orientation(self, q: Tuple[float, float, float, float]) -> None:
        """Set the target orientation of the end-effector.

        Args:
            q: The quaternion representing the end-effector orientation (qx, qy, qz, qw).
        """
        q = ffi.new("double(*)[4]", tuple(q))
        check(lib.orbita3d_set_target_orientation(self.uid, q))

    def get_raw_motors_velocity_limit(self) -> Tuple[float, float, float]:
        """Get the raw motors velocity limit.

        Be carfeful, this is not the end-effector velocity limit. But this is the raw velocity limit of the disks!

        Returns:
            The raw motors velocity limit (in radians per second).
        """
        velocity_limit = ffi.new("double(*)[3]")
        check(lib.orbita3d_get_raw_motors_velocity_limit(self.uid, velocity_limit))
        return tuple(velocity_limit[0])

    def set_raw_motors_velocity_limit(
        self, velocity_limit: Tuple[float, float, float]
    ) -> None:
        """Set the raw motors velocity limit.

        Be carfeful, this is not the end-effector velocity limit. But this is the raw velocity limit of the disks!

        Args:
            velocity_limit: The raw motors velocity limit (in radians per second).
        """
        velocity_limit = ffi.new("double(*)[3]", tuple(velocity_limit))
        check(lib.orbita3d_set_raw_motors_velocity_limit(self.uid, velocity_limit))

    def get_raw_motors_torque_limit(self) -> Tuple[float, float, float]:
        """Get the raw motors torque limit.

        Be carfeful, this is not the end-effector torque limit. But this is the raw torque limit of the disks!

        Returns:
            The raw motors torque limit (in Newton meters).
        """
        torque_limit = ffi.new("double(*)[3]")
        check(lib.orbita3d_get_raw_motors_torque_limit(self.uid, torque_limit))
        return tuple(torque_limit[0])

    def set_raw_motors_torque_limit(
        self, torque_limit: Tuple[float, float, float]
    ) -> None:
        """Set the raw motors torque limit.

        Be carfeful, this is not the end-effector torque limit. But this is the raw torque limit of the disks!

        Args:
            torque_limit: The raw motors torque limit (in Newton meters).
        """
        torque_limit = ffi.new("double(*)[3]", tuple(torque_limit))
        check(lib.orbita3d_set_raw_motors_torque_limit(self.uid, torque_limit))

    def get_raw_motors_pid_gains(
        self,
    ) -> Tuple[
        Tuple[float, float, float],
        Tuple[float, float, float],
        Tuple[float, float, float],
    ]:
        """Get the raw motors PID gains.

        Be carfeful, this is not the end-effector PID gains. But this is the raw PID gains of the disks!

        Returns:
            The raw motors PID gains (kp, ki, kd).
        """
        pids = ffi.new("double(*)[3][3]")
        check(lib.orbita3d_get_raw_motors_pid_gains(self.uid, pids))
        return pids[0]

    def set_raw_motors_pid_gains(
        self,
        pids: Tuple[
            Tuple[float, float, float],
            Tuple[float, float, float],
            Tuple[float, float, float],
        ],
    ) -> None:
        """Set the raw motors PID gains.

        Be carfeful, this is not the end-effector PID gains. But this is the raw PID gains of the disks!

        Args:
            pids: The raw motors PID gains (kp, ki, kd).
        """
        pids = ffi.new("double(*)[3][3]", tuple(pids))
        check(lib.orbita3d_set_raw_motors_pid_gains(self.uid, pids))


def check(err):
    if err != 0:
        raise RuntimeError("Error code: {}".format(err))

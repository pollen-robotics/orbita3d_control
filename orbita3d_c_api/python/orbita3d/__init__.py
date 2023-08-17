from typing import Tuple

from ._orbita3d import ffi
from ._orbita3d import lib


class KinematicsModel:
    def __init__(
        self,
        alpha: float,
        gamma_min: float,
        offset: float,
        beta: float,
        gamma_max: float,
        passiv_arms_direct: bool,
    ) -> None:
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
        thetas = ffi.new("double(*)[3]", tuple(thetas))
        q = ffi.new("double(*)[4]")

        check(lib.orbita3d_kinematics_forward_position(self.model, thetas, q))

        return tuple(q[0])

    def forward_velocity(
        self,
        thetas: Tuple[float, float, float],
        thetas_velocity: Tuple[float, float, float],
    ) -> Tuple[float, float, float, float]:
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
        q = ffi.new("double(*)[4]", tuple(q))
        thetas = ffi.new("double(*)[3]")

        check(lib.orbita3d_kinematics_inverse_position(self.model, q, thetas))

        return tuple(thetas[0])

    def inverse_velocity(
        self,
        q: Tuple[float, float, float, float],
        q_velocity: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
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
        q = ffi.new("double(*)[4]", tuple(q))
        q_torque = ffi.new("double(*)[4]", tuple(q_torque))
        thetas_torque = ffi.new("double(*)[3]")

        check(
            lib.orbita3d_kinematics_inverse_torque(
                self.model, q, q_torque, thetas_torque
            )
        )

        return tuple(thetas_torque[0])


def check(err):
    if err != 0:
        raise RuntimeError("Error code: {}".format(err))

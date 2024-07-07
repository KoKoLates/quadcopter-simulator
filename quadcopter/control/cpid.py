import math
import numpy as np

from dataclasses import dataclass, field

from quadcopter import Quadcopter, wrap
from .controller import Controller


@dataclass
class PID(object):
    Kp: np.ndarray
    Ki: np.ndarray
    Kd: np.ndarray
    Ie: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def update(self, error: np.ndarray, derror: np.ndarray) -> np.ndarray:
        """update the PID controller output
        @param error: the error term
        @param derror: the derivative of the error term
        @return: control output
        """
        self.Ie += self.Ki * error
        return self.Kp * error + self.Ie + self.Kd * derror


@dataclass
class ControlConfig(object):
    position: PID
    attitude: PID

    yaw_limit: tuple[int, int] = (-900, 900)
    tilt_limit: tuple[int, int] = (-10, 10)
    motor_limit: tuple[int, int] = (4000, 9000)


class CPID(Controller):
    def __init__(self, config: ControlConfig, quad: Quadcopter) -> None:
        super(CPID, self).__init__(quad)
        self.position: PID = config.position
        self.attitude: PID = config.attitude

        ## initialize value clipper
        self._yaw_limit: tuple[int, int] = config.yaw_limit
        self.tilt_limit: tuple[int, int] = config.tilt_limit
        self.motor_limit: tuple[int, int] = config.motor_limit

        ## initialize the mixer matrix
        self._mixer_matrix: np.ndarray = np.array(
            [[1, 1, 0, 1], [1, 0, 1, -1], [1, -1, 0, 1], [1, 0, -1, -1]]
        )

    def _update(self) -> None:
        t_pos, t_yaw = self.target
        state: np.ndarray = self.quad.state
        position, velocity, attitude, angular_rate = (
            state[0:3],
            state[3:6],
            state[6:9],
            state[9:12],
        )

        ## position controller and clipping
        error_pos: np.ndarray = t_pos - position
        ux, uy, uz = self.position.update(error_pos, velocity)

        uz = np.clip(uz, self.motor_limit[0], self.motor_limit[1])

        ## attitude controller and clipping
        roll, pitch, yaw = attitude
        _, _, yaw_rate = angular_rate
        ax = ux * math.sin(yaw) - uy * math.cos(yaw)
        ay = ux * math.cos(yaw) + uy * math.sin(yaw)

        ax, ay = np.clip(ax, self.tilt_limit[0], self.tilt_limit[1]), np.clip(
            ay, self.tilt_limit[0], self.tilt_limit[1]
        )

        error_att: np.ndarray = np.array(
            [ax - roll, ay - pitch, 0.18 * (wrap(t_yaw - yaw)) - yaw_rate]
        )
        wx, wy, wz = self.attitude.update(error_att, angular_rate)
        wz = np.clip(wz, self._yaw_limit[0], self._yaw_limit[1])

        m = np.matmul(self._mixer_matrix, np.array([uz, wx, wy, wz]))
        m = np.clip(m, self.motor_limit[0], self.motor_limit[1])
        self._set_motors(m)

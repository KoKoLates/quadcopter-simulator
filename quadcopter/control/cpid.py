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
    Ie: np.ndarray = field(default=np.array([0, 0, 0]))

    def update(self, error: np.ndarray, derror: np.ndarray) -> np.ndarray:
        self.Ie += np.multiply(self.Ki, error)
        return np.multiply(self.Kp, error) + self.Ie + np.multiply(self.Kd, derror)


@dataclass
class ControlConfig(object):

    position: PID
    attitude: PID


class CPID(Controller):
    def __init__(self, config: ControlConfig, quad: Quadcopter) -> None:
        super(CPID, self).__init__(quad)
        self.position: PID = config.position
        self.attitude: PID = config.attitude

    def _update(self) -> None:
        t_pos, t_yaw = self.target
        states = np.array([1, 2, 3])
        vstate = np.array([1, 2, 3])
        ostate = np.array([1, 2, 3])

        error: np.ndarray = t_pos - states
        ux, uy, uz = self.position.update(error, vstate)

        np.clip(uz, 0, 100)

        roll, pitch, yaw = ostate
        yaw_dot = 1
        ax = 1 * (ux * math.sin(yaw) - uy * math.cos(yaw))
        ay = 1 * (ux * math.cos(yaw) + uy * math.sin(yaw))

        np.clip(ax, 0, 1)
        np.clip(ay, 0, 1)  # clip tilte

        error[0] = ax - pitch
        error[1] = ay - roll
        error[2] = 1 * wrap(t_yaw - yaw) - yaw_dot

        wx, wy, wz = self.attitude.update(error, ostate)

        np.clip(wz, 0, 1)

        i = np.array([[1, 1, 0, 1], [1, 0, 1, -1], [1, -1, 0, 1], [1, 0, -1, -1]])

        m = np.matmul(i, np.array([uz, wx, wy, wz]))
        self.motor(np.array(m))

import time
import math
import threading

import numpy as np
from scipy.integrate import ode

from quad import rotation_matrix, wrap
from quad import QuadConfig, MotorConfig


class Motors(object):
    def __init__(self, config: MotorConfig, n: int = 4) -> None:
        self.d: float = config.d
        self.p: float = config.pitch
        self.s: np.ndarray = np.full(n, config.speed)
        self.f: np.ndarray = np.full(n, config.thrust)

        self.c: float = (
            1.857e-11
            * math.pow(self.d)
            * math.sqrt(self.p)
        )

    def set_speed(self, speeds: np.ndarray) -> None:
        self.s = speeds * 60 / (2 * np.pi)
        self.f = self.c * np.square(self.s)


class Quadcopter(object):
    def __init__(self, config: QuadConfig) -> None:
        self.w: float = config.weight
        self.l: float = config.length
        self.r: float = config.radius

        Ix: float = (2 * self.w * self.r**2) / 5 + (2 * self.w * self.l**2)
        Iy: float = Ix
        Iz: float = (2 * self.w * self.r**2) / 5 + (4 * self.w * self.l**2)
        self.J: np.ndarray = np.array([[Ix, 0, 0], [0, Iy, 0], [0, 0, Iz]])
        self.J_inv: np.ndarray = np.linalg.inv(self.J)

        ## allocation matrix
        C: float = config.drag
        L: float = config.length
        self._allocation_matrix: np.ndarray = np.array(
            [[1, 1, 1, 1], [L, 0, -L, 0], [0, L, -L, 0], [C, -C, C, -C]]
        )

        ## intialize state
        self.state: np.ndarray = np.zeros(12)
        self.state[0:3] = np.array(config.initial_state[0])
        self.state[6:9] = np.array(config.initial_state[1])

        self.motors: Motors = Motors(config.motor, 4)
        self.solver: ode = ode(f=self._fetch_state).set_integrator("vode")

        # self._lock: threading.Lock = threading.Lock()
        self._time: float = time.time()
        self._execute: bool = True

    def start(self, dt: float, scale: float) -> None:
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        self._execute = False
        self._thread.join()

    def set_motor_speed(self, speeds: np.ndarray) -> None:
        self.motors.set_speed(speeds)

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self._time
        while self._execute:
            time.sleep(0)
            self._time = time.time()
            if (self._time - last) > rate:
                self._update(dt)
                last = self._time

    def _update(self, dt: float) -> None:
        self.solver.set_initial_value(self.state, 0).set_f_params(self.motors.f)
        self.solver.integrate(self.solver.t + dt)

        self.state = self.solver.y
        self.state[2] = max(0, self.state[2])
        self.state[6:9] = wrap(self.state[6:9])

    def _fetch_state(self, t, state, motors: np.ndarray) -> np.ndarray:
        """fetch the time rate of state for the integrator"""
        f: np.ndarray = self._allocation_matrix @ motors.T
        dstate: np.ndarray = np.zeros_like(self.state)

        dstate[0:3] = self.state[3:6]
        dstate[6:9] = (
            (rotation_matrix(self.state[6:9]) @ np.array([0, 0, f[0, 0]]).T).T / self.w
            + np.array([0, 0, -9.81])
        )
        dstate[6:9] = self.state[9:12]
        dstate[9:12] = self.J_inv @ (
            f[1:4] - np.cross(self.state[9:12], np.dot(self.J, self.state[9:12]))
        )
        return dstate


if __name__ == "__main__":
    config: QuadConfig = QuadConfig(
        weight=1.2,
        length=0.3,
        radius=0.1,
        initial_state=[[0, 0, 0], [0, 0, 0]],
        drag=0.0245,
        propeller=MotorConfig(10.0, 4.5),
    )

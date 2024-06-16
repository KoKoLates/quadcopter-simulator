import time
import threading

import numpy as np
from scipy.integrate import ode

from quad import rotation_matrix, wrap
from quad import QuadConfig, MotorConfig

from quad.quad import Motors
from quad.control import Controller


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
        L: float = config.length
        C: float = config.lift_const
        self._allocation_matrix: np.ndarray = np.array(
            [[1, 1, 1, 1], [L, 0, -L, 0], [0, L, -L, 0], [C, -C, C, -C]]
        )

        ## intialize state
        self.state: np.ndarray = np.zeros(12)
        self.state[0:3] = np.array(config.states[0])
        self.state[6:9] = np.array(config.states[1])

        self.motors: Motors = Motors(config.motors, 4)
        self.solver: ode = ode(f=self._fetch_state).set_integrator("vode")

        # self._lock: threading.Lock = threading.Lock()
        self._time: float = time.time()
        self._execute: bool = True

    def start(self, dt: float = 5e-2, scale: float = 1) -> None:
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        self._execute = False
        self._thread.join()

    def set_motor(self, speeds: np.ndarray) -> None:
        self.motors.set_speed(speeds)

    def get_time(self) -> float:
        return self._time

    def get_state(self) -> None:
        pass

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
        states=[[0, 0, 0], [0, 0, 0]],
        motors=MotorConfig(10.0, 4.5),
        lift_const=0.0245
    )

    

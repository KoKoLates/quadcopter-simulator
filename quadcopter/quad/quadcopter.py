import time
import threading
import numpy as np

from typing import Union
from scipy.integrate import ode

from quadcopter import QuadConfig
from quadcopter.quad import Motors

from quadcopter import rotation_matrix, wrap, np_arr_f64


class Quadcopter(object):
    def __init__(self, config: QuadConfig) -> None:
        self.w: float = config.weight
        self.l: float = config.length
        self.r: float = config.radius

        ## initialize momentum
        Ix: float = (2 * self.w * self.r**2) / 5 + (2 * self.w * self.l**2)
        Iy: float = Ix
        Iz: float = (2 * self.w * self.r**2) / 5 + (4 * self.w * self.l**2)
        self.J: np_arr_f64 = np.array([[Ix, 0, 0], [0, Iy, 0], [0, 0, Iz]])
        self.J_inv: np_arr_f64 = np.linalg.inv(self.J)

        ## initialize state
        self.state: np_arr_f64 = np.zeros(12)
        self.state[0:3] = np.array(config.states[0])
        self.state[6:9] = np.array(config.states[1])

        ## initialize motors
        self.motors: Motors = Motors(config.motors)

        ## initialize solver
        self.solver: ode = ode(f=self._fetch_state).set_integrator("vode")

        ## initialize allocation matrix
        L: float = config.length
        C: float = config.drag_coef
        self._allocation_matrix: np_arr_f64 = np.array(
            [[1, 1, 1, 1], [L, 0, -L, 0], [0, L, -L, 0], [C, -C, C, -C]]
        )

        self.time: float = time.time()
        self.execute: bool = True
        self.thread: Union[threading.Thread, None] = None

    def start(self, dt: float = 5e-2, scale: float = 1.0) -> None:
        self.thread = threading.Thread(target=self._threading, args=(dt, scale))
        self.thread.start()

    def stop(self) -> None:
        self.execute = False
        if self.thread is not None:
            self.thread.join()

    def set_motor_speeds(self, speeds: np_arr_f64) -> None:
        self.motors.set_speeds(speeds)

    def get_time(self) -> float:
        return self.time

    def get_state(self) -> np_arr_f64:
        return self.state

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self.time
        while self.execute:
            time.sleep(0)
            self.time = time.time()
            if self.time - last > rate:
                self._update(dt)
                last = self.time

    def _update(self, dt: float) -> None:
        self.solver.set_initial_value(self.state, 0).set_f_params(self.motors.thrust())
        self.solver.integrate(self.solver.t + dt)

        self.state = np.array(self.solver.y)
        self.state[2] = max(0, self.state[2])
        self.state[6:9] = wrap(self.state[6:9])

    def _fetch_state(self, t, state, thrust: np_arr_f64) -> np_arr_f64:
        f: np_arr_f64 = np.matmul(self._allocation_matrix, thrust)
        dstate: np_arr_f64 = np.zeros_like(self.state)

        dstate[0:3] = self.state[3:6]
        dstate[6:9] = np.matmul(
            rotation_matrix(self.state[6:9]), np.array([0, 0, f[0]])
        ) / self.w + np.array([0, 0, -9.81])
        dstate[6:9] = self.state[9:12]
        dstate[9:12] = np.matmul(
            self.J_inv,
            f[1:4] - np.cross(self.state[9:12], np.dot(self.J, self.state[9:12])),
        )
        return dstate

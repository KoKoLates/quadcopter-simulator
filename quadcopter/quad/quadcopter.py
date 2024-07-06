import time
import threading
import numpy as np

from typing import Union
from scipy.integrate import ode

from quadcopter import QuadConfig
from quadcopter.quad import Motors

from quadcopter import rotation_matrix, wrap


class Quadcopter(object):
    def __init__(self, config: QuadConfig) -> None:
        self.w: float = config.weight
        self.l: float = config.length
        self.r: float = config.radius

        ## initialize momentum of inertia
        Ix: float = (2 * self.w * self.r**2) / 5 + (2 * self.w * self.l**2)
        Iy: float = Ix
        Iz: float = (2 * self.w * self.r**2) / 5 + (4 * self.w * self.l**2)
        self.J: np.ndarray = np.array([[Ix, 0, 0], [0, Iy, 0], [0, 0, Iz]])
        self.J_inv: np.ndarray = np.linalg.inv(self.J)

        ## initialize state
        self._state: np.ndarray = np.zeros(12)
        self._state[0:3] = np.array(config.states[0])
        self._state[6:9] = np.array(config.states[1])

        ## initialize _motors
        self._motors: Motors = Motors(config.motors)

        ## initialize solver
        self.solver: ode = ode(f=self._fetch_state).set_integrator("vode")

        ## initialize allocation matrix
        L: float = config.length
        C: float = config.lift_const
        self._allocation_matrix: np.ndarray = np.array(
            [[1, 1, 1, 1], [L, 0, -L, 0], [0, L, -L, 0], [C, -C, C, -C]]
        )

        self._time: float = time.time()
        self._thread: Union[threading.Thread, None] = None
        self._execute: bool = True

    def start(self, dt: float = 5e-2, scale: float = 1.0) -> None:
        """start the quadcopter in a saperate threading
        @param dt: time step of simulation
        @param scale: scale factor for the simulation speed
        """
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        """stop the quadcopter simulation threading."""
        self._execute = False
        if self._thread is not None:
            self._thread.join()

    @property
    def time(self) -> float:
        return self._time

    @property
    def state(self) -> np.ndarray:
        return self._state
    
    @property
    def motors(self) -> Motors:
        return self._motors
    
    def set_motor_speeds(self, speeds: np.ndarray) -> None:
        self._motors.speeds = speeds

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self.time
        while self._execute:
            time.sleep(0)
            self._time = time.time()
            if self.time - last > rate:
                self._update(dt)
                last = self.time

    def _update(self, dt: float) -> None:
        self.solver.set_initial_value(self._state, 0).set_f_params(self._motors.thrust)
        self.solver.integrate(self.solver.t + dt)

        self._state = np.array(self.solver.y)
        self._state[2] = max(0, self._state[2])
        self._state[6:9] = wrap(self._state[6:9])

    def _fetch_state(self, t: float, state: np.ndarray, thrust: np.ndarray) -> np.ndarray:
        f: np.ndarray = self._allocation_matrix @ thrust
        dstate: np.ndarray = np.zeros_like(self._state)

        dstate[0:3] = self._state[3:6]
        dstate[3:6] = (
            rotation_matrix(self._state[6:9]) @ np.array([0, 0, f[0]]) / self.w
            + np.array([0, 0, -9.81])
        )
        dstate[6:9] = self._state[9:12]
        dstate[9:12] = (
            self.J_inv @ (f[1:4] - np.cross(self._state[9:12], self.J @ self._state[9:12]))
        )
        return dstate

import time
import threading
import numpy as np

from typing import Union
from scipy.integrate import ode
from dataclasses import dataclass

from quad import Motors, MotorConfig
from utils import wrap, rotation_matrix


@dataclass
class QuadConfig(object):
    weight: float
    length: float
    radius: float
    states: list[list[float]]
    motors: MotorConfig
    drag_c: float


class Quadcopter(object):
    def __init__(self, config: QuadConfig) -> None:
        self.w: float = config.weight
        self.l: float = config.length
        self.r: float = config.radius

        Ix: float = (2 * self.w * self.r**2) / 5 + (2 * self.w * self.l**2)
        Iy: float = Ix
        Iz: float = (2 * self.w * self.r**2) / 5 + (4 * self.w * self.l**2)
        self.J: np.ndarray = np.reshape([Ix, 0, 0, 0, Iy, 0, 0, 0, Iz], (3, 3))
        self.J_inv: np.ndarray = np.linalg.inv(self.J)

        self._states: np.ndarray = np.zeros(12)
        self._states[0:3] = np.array(config.states[0])
        self._states[6:9] = np.array(config.states[1])
        self._motors: Motors = Motors(config.motors)

        self._solver: ode = ode(f=self._fetch_state).set_integrator("vode")
        L: float = config.length
        C: float = config.drag_c
        self._alloc_mat: np.ndarray = np.array(
            [1, 1, 1, 1], [L, 0, -L, 0], [0, L, -L, 0], [C, -C, C, -C]
        ).reshape((4, 4))

        self._time: float = time.time()
        self._thread: Union[threading.Thread, None] = None
        self._execute: bool = False

    def start(self, dt: float = 5e-2, scale: float = 1.0) -> None:
        self._execute = True
        self._thread = threading.Thread()
        self._thread.start(target=self._threading, args=(dt, scale))

    def stop(self) -> None:
        self._execute = False
        if self._thread is not None:
            self._thread.join()

    @property
    def time(self) -> float:
        return self._time
    
    @property
    def states(self) -> np.ndarray:
        return self._states
    
    @property
    def motors(self) -> Motors:
        return self._motors
    
    def set_motor_speeds(self, speeds: np.ndarray) -> None:
        if np.any(speeds < 0) or len(speeds) != 4:
            raise ValueError("Input value not correspond to motor number.")
        
        self._motors.speeds = speeds

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self._time

        while self._execute:
            time.sleep(0)
            self._time = time.time()
            if self._time - last > rate:
                self._update(dt)
                last = self._time

    def _update(self, dt: float) -> None:
        self._solver.set_initial_value(self._states, 0).set_f_params(self._motors.thrust)
        self._solver.integrate(self._solver.t + dt)
        self._states = np.array(self._solver.y)
        self._states[2] = max(0, self.states[2])
        self._states[6:9] = wrap(self.states[6:9])

    def _fetch_state(
        self, t: float, state: np.ndarray, thrust: np.ndarray
    ) -> np.ndarray:
        f: np.ndarray = self._alloc_mat @ thrust
        d_states: np.ndarray = np.zeros_like(self._states)
        d_states[0:3] = self._states[3:6]
        d_states[3:6] = (
            rotation_matrix(self._states[6:9]) @ np.array([0, 0, f[0]]) / self.w + np.array([0, 0, -9.81])
        )
        d_states[6:9] = self._states[9:12]
        d_states[9:12] = (
            self.J_inv @ (f[1:4] - np.cross(self._states[9:12], self.J @ self._states[9:12]))
        )
        return d_states

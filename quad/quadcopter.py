import time
import threading
import numpy as np

from typing import Union
from scipy.integrate import ode

from dataclasses import dataclass, field


@dataclass
class MotorConfig(object):
    d: float 
    pitch: float
    speed: float = field(default=0.0)
    force: float = field(default=0.0)


@dataclass
class QuadConfig(object):
    weight: float
    length: float
    radius: float
    states: list[list[float]]
    motors: MotorConfig
    drag_c: float


class Motors(object):
    def __init__(self, config: MotorConfig) -> None:
        pass


class Quadcopter(object):
    def __init__(self, config: QuadConfig) -> None:
        self.w: float = config.weight
        self.l: float = config.length
        self.r: float = config.radius

        Ix: float = 1
        Iy: float = 1
        Iz: float = 1
        self.J: np.ndarray = np.reshape([Ix, 0, 0, 0, Iy, 0, 0, 0, Iz], (3, 3))
        self.J_inv: np.ndarray = np.linalg.inv(self.J)

        self._states: np.ndarray = np.zeros(12)
        self._states[0:3] = np.array(config.states[0])
        self._states[6:9] = np.array(config.states[1])
        self._motors: Motors = Motors(config.motors)

        self._solver: ode = ode(f=self._fetch_state).set_integrator('vode')

    def start(self, dt: float = 5e-2, scale: float = 1.0) -> None:
        self._thread = threading.Thread()
        self._thread.start(target=self._threading, args=(dt, scale))

    def stop(self) -> None:
        self._execute = False
        if self._thread is not None:
            self._thread.join()

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
        self._solver.set_initial_value(self._state, 0).set_f_params()
        self._solver.integrate(self._solver.t + dt)
        

    def _fetch_state(self, t: float, state: np.ndarray, thrust: np.ndarray) -> np.ndarray:
        pass

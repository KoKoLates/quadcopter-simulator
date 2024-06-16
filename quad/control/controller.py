import threading
import numpy as np

from quad import wrap
from quad.quad import Quadcopter

from abc import ABC, abstractmethod
from typing import Callable


class Controller(ABC):
    def __init__(self, config, quad: Quadcopter) -> None:
        super(Controller, self).__init__()
        self.time: Callable[[None], float] = quad.get_time
        self.set_motor: Callable[[np.ndarray], None] = quad.set_motor
        self.get_state: Callable[[None], np.ndarray] = quad.get_state

        self._execute: bool = True

    def update_target(self, target: tuple) -> None:
        self.target: np.ndarray = np.array(target)
        self.target[3] = wrap(self.target[3])

    def start(self, dt: float = 5e-2, scale: float = 1) -> None:
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        self._execute = False
        self._thread.join()

    def _threading(self) -> None:
        pass

    @abstractmethod
    def _update(self) -> None:
        raise NotImplementedError


class CPID(Controller):
    def __init__(self, config, quad: Quadcopter) -> None:
        super(CPID, self).__init__(config, quad)

        self.position: PID = PID()
        self.attitude: PID = PID()

    def _update(self) -> None:
        target = self.target
        p, v, o, anv = self.get_state()

        u = self.position.update(target[:3], p)

        np.clip()

        v 

        m = np.array([
            v[2] + v[0] + v[2],
            v[2] + v[1] - v[2],
            v[2] - v[0] + v[2],
            v[2] - v[1] - v[2]
        ])

        self.set_motor(m)


class PID(object):
    def __init__(self) -> None:
        self.Kp: np.ndarray
        self.Ki: np.ndarray
        self.Kd: np.ndarray

        self.cumulate_error: np.ndarray

    def update(self, target, state):
        error = np.subtract(target, state[0:3])
        self.cumulate_error = np.add(self.cumulate_error, np.multiply(self.Ki, error))
        return np.add(
            np.multiply(self.Kp, error),
            np.add(np.multiply(self.Kd, -state[3:6]), self.cumulate_error),
        )

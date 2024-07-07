import math
import numpy as np

from quadcopter import MotorConfig


class Motors(object):
    def __init__(self, config: MotorConfig, n: int = 4) -> None:
        self._d: float = config.d
        self._p: float = config.pitch
        self._s: np.ndarray = np.full(n, config.speed)
        self._f: np.ndarray = np.full(n, config.force)

        self._n: int = n
        self._c: float = (  # constant
            1.857e-11 * math.pow(self._d, 2) * math.sqrt(self._p)
        )

    @property
    def speeds(self) -> np.ndarray:
        return self._s

    @property
    def thrust(self) -> np.ndarray:
        return self._f

    @speeds.setter
    def speeds(self, speeds: np.ndarray) -> None:
        if len(speeds) != self._n:
            raise ValueError("Wrong number of input speeds for motors")

        if np.any(speeds < 0):
            raise ValueError("Input speeds should be positive or zero")

        self._s = speeds
        self._f = self._c * np.square(self._s * 60 / (2 * np.pi))  

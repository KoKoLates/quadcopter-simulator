import math
import numpy as np

from dataclasses import dataclass, field

@dataclass
class MotorConfig(object):
    d: float
    pitch: float
    speed: float = field(default=0.0)
    force: float = field(default=0.0)


class Motors(object):
    def __init__(self, config: MotorConfig, n: int = 4) -> None:
        self._d: float = config.d
        self._p: float = config.pitch
        self._s: np.ndarray = np.full(n, config.speed)
        self._f: np.ndarray = np.full(n, config.force)

        self._n: int = n
        self._c: float = 1.857e-11 * math.pow(self._d, 2) * math.sqrt(self._p)

    @property
    def speeds(self) -> np.ndarray:
        return self._s
    
    @property
    def thrust(self) -> np.ndarray:
        return self._f
    
    @speeds.setter
    def speeds(self, speeds: np.ndarray) -> None:
        if len(speeds) != self._n:
            raise ValueError("Wrong input number of speeds for motors.")
        
        if np.any(speeds < 0):
            raise ValueError("Input speeds should be positive or zero.")
        
        self._s = speeds
        self._f = self._c * np.square(self._s * 60 / (np.pi * 2))

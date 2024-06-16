import math
import numpy as np

from quad import MotorConfig

class Motors(object):
    def __init__(self, config: MotorConfig, n: int = 4) -> None:
        self.d: float = config.d
        self.p: float = config.pitch
        self.s: np.ndarray = np.full(n, config.speed)
        self.f: np.ndarray = np.full(n, config.force)

        self.c: float = ( # constant
            1.857e-11
            * math.pow(self.d, 2)
            * math.sqrt(self.p)
        )

    def set_speed(self, speeds: np.ndarray) -> None:
        self.s = speeds
        rpm: np.ndarray = self.s * 60 / (2 * np.pi)
        self.f = self.c * np.square(rpm)

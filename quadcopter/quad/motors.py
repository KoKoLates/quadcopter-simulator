import math
import numpy as np

from quadcopter import MotorConfig, np_arr_f64


class Motors(object):
    def __init__(self, config: MotorConfig, n: int = 4) -> None:
        self._d: float = config.d
        self._p: float = config.pitch
        self._s: np_arr_f64 = np.full(n, config.speed)
        self._f: np_arr_f64 = np.full(n, config.force)

        self._c: float = ( # constant
            1.857e-11
            * math.pow(self._d, 2)
            * math.sqrt(self._p)
        )

    def set_speeds(self, speeds: np_arr_f64) -> None:    
        self._s = speeds
        rpm: np_arr_f64 = self._s * 60 / (2 * np.pi)
        self._f = self._c * np.square(rpm)

    def speeds(self) -> np_arr_f64:
        return self._s
    
    def thrust(self) -> np_arr_f64:
        return self._f

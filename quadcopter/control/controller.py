import time
import threading
import numpy as np

from typing import Callable
from abc import ABC, abstractmethod

from quadcopter import wrap, np_arr_f64
from quadcopter import Quadcopter


class Controller(ABC):
    def __init__(self, quad: Quadcopter) -> None:
        super(Controller, self).__init__()
        self.quad: Quadcopter = quad

        self.times: Callable[[], float] = self.quad.get_time
        self.state: Callable[[], None] = self.quad.get_state
        self.motor: Callable[[np_arr_f64], None] = self.quad.set_motor_speeds

        self.execute: bool = True

    def start(self, dt: float = 5e-3, scale: float = 1.0) -> None:
        self.thread = threading.Thread(target=self._threading, args=(dt, scale))
        self.thread.start()

    def stop(self) -> None:
        self.execute = False
        if self.thread is not None:
            self.thread.join()

    def update_target(self, target: tuple) -> None:
        self.target: tuple[np_arr_f64, np_arr_f64] = (
            np.array(target[:3]),
            wrap(target[3]),
        )

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self.quad.get_time()
        while self.execute:
            time.sleep(0)
            self.time = self.quad.get_time()
            if last - self.time > rate:
                self._update()
                last = self.time

    @abstractmethod
    def _update(self) -> None:
        return NotImplemented

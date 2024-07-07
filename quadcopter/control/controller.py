import time
import threading
import numpy as np

from typing import Callable, Union
from abc import ABC, abstractmethod

from quadcopter import wrap
from quadcopter import Quadcopter


class Controller(ABC):
    def __init__(self, quad: Quadcopter) -> None:
        super(Controller, self).__init__()
        self.quad: Quadcopter = quad
        self._set_motors: Callable[[np.ndarray], None] = self.quad.set_motor_speeds

        self._thread: Union[threading.Thread, None] = None
        self._execute: threading.Event = threading.Event()

    def start(self, dt: float = 5e-3, scale: float = 1.0) -> None:
        self._execute.set()
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        self._execute.clear()
        if self._thread is not None:
            self._thread.join()

    def update_target(self, target: tuple[float, float, float, float]) -> None:
        if len(target) != 4:
            raise ValueError(
                "Input target must be a tuple with exactly four elements: (x, y, z, yaw)"
            )

        self.target: tuple[np.ndarray, np.ndarray] = (
            np.array(target[:3], dtype=float),
            wrap(np.array([target[3]], dtype=float)),
        )

    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self.quad.time
        while self._execute.is_set():
            time.sleep(0)
            current_time: float = self.quad.time
            if last - current_time > rate:
                self._update()
                last = current_time

    @abstractmethod
    def _update(self) -> None:
        pass

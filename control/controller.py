import numpy as np

from quad import wrap
from abc import ABC, abstractmethod


class Controller(ABC):
    def __init__(self) -> None:
        super(Controller, self).__init__()

    def update_target(self, target: tuple) -> None:
        """update target waypoint (x, y, z, yaw)"""
        self.target: np.ndarray = np.ndarray(
            list(map(lambda i: wrap(i[1]) if i[0] == 3 else i[1], enumerate(target)))
        )

    @abstractmethod
    def update(self) -> None:
        pass

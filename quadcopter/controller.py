from abc import ABC, abstractmethod

import numpy as np

class Controller(ABC):
    def __init__(self) -> None:
        super().__init__()


    @abstractmethod
    def update(self) -> None:
        pass

    @staticmethod
    def wrap_angle(value: float | np.ndarray) -> float | np.ndarray:
        return (value + np.pi) % (np.pi * 2) - np.pi
    

class PositionController(Controller):
    def __init__(self) -> None:
        super().__init__()

    def update(self) -> None:
        return super().update()
    

class VelocityController(Controller):
    def __init__(self) -> None:
        super().__init__()

    def update(self) -> None:
        return super().update()
import numpy as np
from .controller import Controller


class CPID(Controller):
    """cascade PID controller"""
    def __init__(self) -> None:
        super(CPID, self).__init__()

        self.position = PID()
        self.attitude = PID()

    def update(self, state) -> np.ndarray:
        """
        state is from quadcopter with `(x, y, z, θ, φ, ψ)`
        and return will be `(T, τₓ, τᵧ, τₓ)`
        """

        error = np.subtract(self.target[:3], state.p)
        self.position.update(error)

        

class PID(object):
    def __init__(self) -> None:
        self.KP: np.ndarray = np.array()
        self.KI: np.ndarray = np.array()
        self.KD: np.ndarray = np.array()
        
        self.errors: np.ndarray = np.array()

    def update(self, error: np.ndarray) -> np.ndarray:
        pass

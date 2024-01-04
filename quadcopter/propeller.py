from dataclasses import dataclass
import math

@dataclass
class Propeller(object):
    diameter: float
    pitch: float
    speed: float | int = 0.0
    thrust: float = 0.0

    def set_speed(self, speed: float) -> None:
        self.speed = speed
        self.thrust = 4.392e-8 * self.speed * \
            math.pow(self.diameter, 3.5) / (math.sqrt(self.pitch))
        self.thrust *= (4.23e-4 * self.speed * self.pitch)

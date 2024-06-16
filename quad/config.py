from dataclasses import dataclass, field


@dataclass
class MotorConfig(object):
    d: float
    pitch: float
    speed: float = field(default=0.0)
    thrust: float = field(default=0.0)


@dataclass
class QuadConfig(object):
    weight: float
    length: float
    radius: float
    initial_state: list[list, list]

    drag: float
    motor: MotorConfig

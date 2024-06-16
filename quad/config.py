from abc import ABC
from dataclasses import dataclass, field


@dataclass
class MotorConfig(object):
    d: float
    pitch: float
    speed: float = field(default=0.0)
    force: float = field(default=0.0)


@dataclass
class QuadConfig(object):
    weight: float
    length: float
    radius: float
    states: list[list, list]

    motors: MotorConfig
    lift_const: float


@dataclass
class ControlConfig(object):
    position_k: list
    attitude_k: list

    


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
    states: list[list[float]]

    motors: MotorConfig
    drag_coef: float


def load_config(file_path: str) -> QuadConfig:
    raise NotImplementedError

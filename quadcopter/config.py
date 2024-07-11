import json
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
    lift_const: float


def load_config(file_path: str) -> QuadConfig:
    with open(file_path, "r") as file:
        data = json.load(file)

    return QuadConfig(
        weight=data["weight"],
        length=data["length"],
        radius=data["radius"],
        states=[
            data["initial_states"]["position"],
            data["initial_states"]["attitude"]
        ],
        motors=MotorConfig(
            d=data["motors"]["diameter"],
            pitch=data["motors"]["pitch"]
        ),
        lift_const=data["lift_const"]
    )

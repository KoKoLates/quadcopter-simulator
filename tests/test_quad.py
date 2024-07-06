import time
import pytest
import numpy as np

from quadcopter import Quadcopter
from quadcopter import QuadConfig, MotorConfig


@pytest.fixture
def quadcopter() -> Quadcopter:
    config: QuadConfig = QuadConfig(
        weight=1.0,
        length=0.5,
        radius=0.2,
        states=[[0, 0, 0], [0, 0, 0]],
        motors=MotorConfig(0.1, 0.2),
        lift_const=0.1,
    )
    return Quadcopter(config)


def test_quad_init(quadcopter: Quadcopter) -> None:
    assert quadcopter.w == 1.0
    assert quadcopter.l == 0.5
    assert quadcopter.r == 0.2
    assert quadcopter.time != 0.0
    assert np.array_equal(quadcopter.state, np.zeros(12))


def test_quad_getter_setter(quadcopter: Quadcopter) -> None:
    speeds: np.ndarray = np.array([1200.0, 1100.0, 1050.0, 1000.0])
    quadcopter.set_motor_speeds(speeds)
    assert np.array_equal(quadcopter.motors.speeds, speeds)

    rpm: np.ndarray = speeds * 60 / (2 * np.pi)
    expect: np.ndarray = (
        1.857e-11
        * quadcopter.motors._d**2
        * np.sqrt(quadcopter.motors._p)
        * np.square(rpm)
    )
    result: np.ndarray = quadcopter.motors.thrust
    assert np.allclose(expect, result, atol=1e-6)


def test_quad_solver(quadcopter: Quadcopter) -> None:
    state: np.ndarray = quadcopter.state
    quadcopter.start(dt=0.1)

    current_time = 0.0
    while current_time < 5.0:
        if current_time % 1.0 == 0:
            new_speeds = np.random.uniform(low=0.0, high=100.0, size=4)
            quadcopter.set_motor_speeds(new_speeds)

        time.sleep(0.1)
        current_time += 0.1

    quadcopter.stop()
    result: np.ndarray = quadcopter.state

    assert not np.allclose(state, result)


if __name__ == "__main__":
    pytest.main()

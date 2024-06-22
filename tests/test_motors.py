import pytest
import numpy as np

from quadcopter import Motors, MotorConfig


def test_motor_init() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    assert motors._d == 10
    assert motors._p == 2
    assert np.array_equal(motors.speeds(), np.full(4, 0))
    assert np.array_equal(motors.thrust(), np.full(4, 0))
    assert motors._c == pytest.approx(2.626e-9)


def test_motor_set_speeds() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    assert np.array_equal(motors.speeds(), np.full(4, 0))
    speeds: np.ndarray = np.full(4, 1000)
    motors.set_speeds(speeds=speeds)

    assert np.array_equal(motors.speeds(), speeds)
    assert motors.thrust() == pytest.approx(np.full(4, 0.2394802))

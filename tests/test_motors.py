import pytest
import numpy as np

from quadcopter import Motors, MotorConfig


def test_motor_init() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    assert motors._d == 10
    assert motors._p == 2
    assert np.array_equal(motors._s, np.array([0, 0, 0, 0]))
    assert np.array_equal(motors._f, np.array([0, 0, 0, 0]))
    assert motors._c == pytest.approx(2.626e-9)


def test_motor_set_speeds() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    speeds: np.ndarray = np.array([1000, 1000, 1000, 1000])
    motors.set_speeds(speeds=speeds)

    assert np.array_equal(motors._s, speeds)
    assert motors._f == pytest.approx(np.array([0.2394802, 0.2394802, 0.2394802, 0.2394802]))

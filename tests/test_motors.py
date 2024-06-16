import pytest
import numpy as np

from quad import Motors, MotorConfig


def test_motor_init() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    assert motors.d == 10
    assert motors.p == 2
    assert np.array_equal(motors.s, np.array([0, 0, 0, 0]))
    assert np.array_equal(motors.f, np.array([0, 0, 0, 0]))
    assert motors.c == pytest.approx(2.626e-9)


def test_motor_set_speed() -> None:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    motors: Motors = Motors(config)

    speeds: np.ndarray = np.array([1000, 1000, 1000, 1000])
    motors.set_speed(speeds=speeds)

    assert np.array_equal(motors.s, speeds)
    assert motors.f == pytest.approx(np.array([0.2394802, 0.2394802, 0.2394802, 0.2394802]))

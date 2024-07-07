import pytest
import numpy as np

from quadcopter import Motors, MotorConfig


@pytest.fixture
def motors() -> Motors:
    config: MotorConfig = MotorConfig(d=10, pitch=2)
    return Motors(config)


def test_motor_init(motors: Motors) -> None:
    assert motors._d == 10
    assert motors._p == 2
    assert np.array_equal(motors.speeds, np.zeros(4))
    assert np.array_equal(motors.thrust, np.zeros(4))
    assert motors._c == pytest.approx(2.626e-9)


def test_motor_set_speeds(motors: Motors) -> None:
    assert np.array_equal(motors.speeds, np.zeros(4))
    speeds: np.ndarray = np.full(4, 1000)
    motors.speeds = speeds
    assert np.array_equal(motors.speeds, speeds)
    assert np.allclose(motors.thrust, np.full(4, 0.2394802), atol=1e-7)


def test_motor_set_speeds_varied(motors: Motors) -> None:
    speeds: np.ndarray = np.array([1000, 1500, 2000, 2500])
    motors.speeds = speeds
    expect = motors._c * np.square(speeds * 60 / (2 * np.pi))
    assert np.array_equal(motors.speeds, speeds)
    assert np.allclose(motors.thrust, expect, atol=1e-7)


def test_motor_invalid_speeds(motors: Motors) -> None:
    with pytest.raises(ValueError):
        motors.speeds = np.array([1000, -1500, 2000, 2500])

    with pytest.raises(ValueError):
        motors.speeds = np.array([1000, 1500, 2000])


if __name__ == "__main__":
    pytest.main()

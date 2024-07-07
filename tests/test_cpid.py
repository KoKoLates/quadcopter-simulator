import pytest
import numpy as np

from quadcopter import wrap
from quadcopter import MotorConfig
from quadcopter import Quadcopter, QuadConfig
from quadcopter.control import CPID, PID, ControlConfig


@pytest.fixture
def quad() -> Quadcopter:
    config: QuadConfig = QuadConfig(
        weight=1.0,
        length=0.5,
        radius=0.2,
        states=[[0, 0, 0], [0, 0, 0]],
        motors=MotorConfig(0.1, 0.2),
        lift_const=0.1,
    )
    return Quadcopter(config)


@pytest.fixture
def ctrl(quad: Quadcopter) -> CPID:
    config: ControlConfig = ControlConfig(
        position=PID(
            Kp=np.array([1.0, 1.0, 1.0]),
            Ki=np.array([0.1, 0.1, 0.1]),
            Kd=np.array([0.01, 0.01, 0.01]),
        ),
        attitude=PID(
            Kp=np.array([1.0, 1.0, 1.0]),
            Ki=np.array([0.1, 0.1, 0.1]),
            Kd=np.array([0.01, 0.01, 0]),
        ),
    )
    return CPID(config, quad)


def test_ctrl_init(ctrl: CPID, quad: Quadcopter) -> None:
    assert ctrl.quad is quad
    assert isinstance(ctrl.position, PID)
    assert isinstance(ctrl.attitude, PID)

    target: tuple = (1.0, 1.0, 1.0, 0.5)
    ctrl.update_target(target)
    assert np.allclose(ctrl.target[0], np.array(target[:3]))
    assert np.allclose(ctrl.target[1], wrap(np.array(target[3])))


def test_ctrl_thread(ctrl: CPID) -> None:
    ctrl.start()
    assert ctrl._execute.is_set()
    ctrl.stop()
    assert not ctrl._execute.is_set()


def test_ctrl_update() -> None:
    pid = PID(
        Kp=np.array([1.0, 1.0, 1.0]),
        Ki=np.array([0.1, 0.1, 0.1]),
        Kd=np.array([0.01, 0.01, 0.01]),
    )
    error = np.array([0.5, 0.5, 0.5])
    derror = np.array([0.1, 0.1, 0.1])
    output = pid.update(error, derror)
    expect = np.array([0.551, 0.551, 0.551])
    assert np.allclose(output, expect)

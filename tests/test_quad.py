import pytest
import numpy as np

from quadcopter import wrap, rotation_matrix
from quadcopter import QuadConfig, Quadcopter


def test_quad_init() -> None:
    pass


def test_wrap_angle() -> None:
    
    angles: np.ndarray = np.array(np.pi)
    angles = wrap(angles)


def test_rotation_matrix() -> None:
    pass

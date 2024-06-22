import numpy as np
from quadcopter import wrap, rotation_matrix


def test_wrap_angle() -> None:
    assert wrap(np.array(np.pi)) == -np.pi
    assert wrap(np.array(np.pi * 3)) == -np.pi
    assert wrap(np.array(np.pi * -2)) == 0
    assert wrap(np.array(np.pi * 5 / 4)) == -np.pi * 3 / 4
    assert wrap(np.array(np.pi * -3 / 2)) == np.pi / 2


def test_rotation_matrix() -> None:
    angles = (0, 0, 0)
    assert np.allclose(rotation_matrix(np.array(angles)), np.identity(3))

    angles = (90, 0, 0)
    expect = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    result = rotation_matrix(np.array(angles))
    assert np.allclose(result, expect), f"expect: {expect}, get {result}"

    angles = (0, 90, 0)
    expect = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    result = rotation_matrix(np.array(angles))
    assert np.allclose(result, expect), f"expect: {expect}, get {result}"

    angles = (0, 0, 90)
    expect = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    result = rotation_matrix(np.array(angles))
    assert np.allclose(result, expect), f"expect: {expect}, get {result}"

    angles = (-45, -30, -60)
    cr, cp, cy = np.cos(np.radians([-45, -30, -60]))
    sr, sp, sy = np.sin(np.radians([-45, -30, -60]))
    RX = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    RY = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    RZ = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    expect = np.dot(RZ, np.dot(RY, RX))
    result = rotation_matrix(np.array(angles))
    assert np.allclose(result, expect), f"expect: {expect}, get {result}"

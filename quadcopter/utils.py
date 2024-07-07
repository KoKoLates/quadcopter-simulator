import numpy as np
import numpy.typing as npt


def wrap(angles: np.ndarray) -> np.ndarray:
    """Wrap the input angles into the range between [−π,π)
    @param angles: input angles to be wrapped
    @return: wrapped angles
    """
    return (angles + np.pi) % (np.pi * 2) - np.pi


def rotation_matrix(angles: np.ndarray) -> np.ndarray:
    """Input roll, pitch and yaw angle and make them into a 
    rotation matrix from body frame to inertia frame
    @param angles: `(roll, pitch, yaw)`
    @return: rotation matrix from body frame to inertia frame
    """
    cp, cr, cy = np.cos(np.radians(angles))
    sp, sr, sy = np.sin(np.radians(angles))
    RP = np.array([[1,  0,  0],
                   [0, cp,-sp],
                   [0, sp, cp]])
    RR = np.array([[cr, 0, sr],
                   [0,  1,  0],
                   [-sr,0, cr]])
    RY = np.array([[cy,-sy, 0],
                   [sy, cy, 0],
                   [0,  0,  1]])
    return np.dot(RY, np.dot(RR, RP))

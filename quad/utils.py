import numpy as np

def wrap(angles: np.ndarray) -> np.ndarray:
    """wrap the input angles into the range of [−π,π)
    @param angle: input angles to be wrapped
    @return: wrapped angles
    """
    return (angles + np.pi) % (np.pi * 2) - np.pi


def rotation_matrix(angles: np.ndarray) -> np.ndarray:
    """input the roll, pitch and yaw and make it into a rotation 
    matrix for body frame to inertia frame conversion
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
    return RY @ RR @ RP

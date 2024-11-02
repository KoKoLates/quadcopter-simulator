import numpy as np

def rotation_matrix(angles: np.ndarray) -> np.ndarray:
    """ Form the rotation matrix based on the input angles `(roll, pitch, yaw)`"""
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

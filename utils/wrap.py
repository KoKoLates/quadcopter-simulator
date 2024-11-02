import numpy as np

def wrap(angles: np.ndarray) -> np.ndarray:
    """ Wrap input angles into range between `[−π,π)` """
    return (angles + np.pi) % (np.pi * 2) - np.pi

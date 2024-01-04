import datetime
import numpy as np

from . import propeller

QUADCOPTER = {
    'pos': [0, 0, 0],
    'rot': [0, 0, 0],
    'l': 0.4, 'r': 0.1,
    'weight': 1.2,
    'propeller': [10, 4.5]
}

class Quadcopter(object):
    def __init__(self, parameter: dict) -> None:
        self.quadcopter: dict = {}
        self.time: datetime = datetime.datetime.now()

        ## initialize the state
        self.quadcopter.update({'state': np.zeros(12)}) 
        self.quadcopter.get('state')[0: 3] = parameter.get('pos')
        self.quadcopter.get('state')[6, 9] = parameter.get('rot')

        self.quadcopter.update({'motor': {}})
        for idx in range(4):
            self.quadcopter.get('motor').update(
                {idx + 1: propeller.Propeller(parameter.get('propeller')[0], 
                                              parameter.get('propeller')[1])})
            
        
        self.execute: bool = True


    def set_motor_speed(self, speeds: list) -> None:
        for idx, value in enumerate(speeds):
            self.quadcopter.get('motor').get(idx + 1).set_speed(value)


    @staticmethod
    def rotation_matrix(angles: list) -> np.ndarray:
        radians = np.radians(angles)
        cp, cr, cy = np.cos(radians)
        sp, sr, sy = np.sin(radians)
        R_P = np.array([[1,  0,  0], 
                        [0, cp,-sp],
                        [0, sp, cp]])
        R_R = np.array([[cr, 0, sr],
                        [0,  1,  0],
                        [-sr,0, sp]])
        R_Y = np.array([[cy,-sy, 0],
                        [sy,cy,  0],
                        [0,  0,  1]])
        R = np.dot(R_Y, np.dot(R_R, R_P))
        return R
    
    
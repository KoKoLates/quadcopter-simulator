import time
import threading
import numpy as np

from typing import Union
from scipy.integrate import ode

from dataclasses import dataclass



@dataclass
class QuadConfig(object):
    weight: float
    length: float
    radius: float
    states: tuple
    lift_const: float
     

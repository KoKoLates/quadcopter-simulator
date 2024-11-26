import math
import time
import threading
import numpy as np

from typing import Callable, Union

from quad import Quadcopter
from utils import wrap


class PID(object):
    def __init__(self, config: tuple) -> None:
        self.Kp = config[0]
        self.Ki = config[1]
        self.Kd = config[2]
        self.Ie = np.zeros(3)

    def update(self, err: np.ndarray, rate: np.ndarray) -> np.ndarray:
        self.Ie += self.Ki * err
        return self.Kp * err + self.Ie + self.Kd * rate


class Controller(object):
    def __init__(self, quad: Quadcopter) -> None:
        self._quad: Quadcopter = quad
        self._set_motor: Callable[[np.ndarray], None] = self._quad.set_motor_speeds

        self._thread: Union[threading.Thread, None] = None
        self._execute: threading.Event = threading.Event

        self.position: PID = PID()
        self.attitude: PID = PID()
        self._clip_yaw = (-900, 900)
        self._clip_ang = (-10, 10)
        self._clip_mot = (4000, 9000)

        self._mixer_mat: np.ndarray = np.array(
            [1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, 1, 1, 0, -1, -1]
        ).reshape((4, 4))

    def start(self, dt: float = 5e-3, scale: float = 1.0) -> None:
        self._execute.set()
        self._thread = threading.Thread(target=self._threading, args=(dt, scale))
        self._thread.start()

    def stop(self) -> None:
        self._execute.clear()
        if self._thread is not None:
            self._thread.join()

    def update_target(self, target: tuple) -> None:
        if len(target) != 4:
            raise ValueError("Target Input should be the tuple of (x, y, z, yaw)")

        pose, yaw = target[:3], target[3]
        self.target = (np.array(pose).dtype(float), wrap(np.array(yaw).dtype(float)))    
    
    def _threading(self, dt: float, scale: float) -> None:
        rate: float = scale * dt
        last: float = self._quad.time
        while self._execute.is_set():
            time.sleep(0)
            current: float = self._quad.time
            if last - current > rate:
                self._update()
                last = current

    def _update(self) -> None:
        pose_t, yaw_t = self.target
        states: np.ndarray = self._quad.states
        p, v, a, w = states[0:3], states[3:6], states[6:9], states[9:12]

        # position control
        err: np.ndarray = pose_t - p
        ux, uy, uz = self.position.update(err, v)
        uz = np.clip(uz, self._clip_mot[0], self._clip_mot[1])

        # attitude control
        roll, pitch, yaw = a
        _, _, yaw_rate = w
        ax = ux * math.sin(yaw) - uy * math.cos(yaw)
        ay = ux * math.cos(yaw) + uy * math.sin(yaw)
        ax = np.clip(ax, self._clip_ang[0], self._clip_ang[1])
        ay = np.clip(ay, self._clip_ang[0], self._clip_ang[1])

        err = np.array([ax - roll, ay - pitch, 0.18 * (wrap(yaw_t - yaw) - yaw_rate)])
        wx, wy, wz = self.attitude.update(err, a)
        wz = np.clip(wz, self._clip_yaw[0], self._clip_yaw[1])

        m = np.matmul(self._mixer_mat, np.array([uz, wx, wy, wz]))
        m = np.clip(m, self._clip_mot[0], self._clip_mot[1])
        self._set_motor(m)

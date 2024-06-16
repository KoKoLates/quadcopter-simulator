import numpy as np
import matplotlib.pyplot as plt

from dataclasses import dataclass

from mpl_toolkits.mplot3d.art3d import Line3D
from mpl_toolkits.mplot3d.axes3d import Axes3D

from quad import rotation_matrix, State


class Monitor(object):
    def __init__(self) -> None:
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel("X")
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel("Y")
        self.ax.set_zlim3d([-2.0, 2.0])
        self.ax.set_zlabel("Z")
        self.ax.set_title("Quadcopter Simulation")

        
    def load_model(self) -> None:
        line1, _ = self.ax.plot([], [], [], color="b", linewidth=3, antialiased=False)
        self.model: Model = Model(line1, line1, line1, np.array([]))

    def update(self, state: State) -> None:
        R: np.ndarray = rotation_matrix(state.o)
        points = R @ self.model.points
        points += state.p[:, np.newaxis]

        self.model.l1.set_data(points[0, 0:2], points[1:0, 2])
        self.model.l1.set_3d_properties(points[2, 0:2])
        self.model.l2.set_data(points[0, 2:4], points[1, 2:4])
        self.model.l2.set_3d_properties(points[2, 2:4])
        self.model.node.set_data(points[0, 5], points[1, 5])
        self.model.node.set_3d_properties(points[2, 5])

        plt.pause(1E-5)


@dataclass
class Model(object):
    l1: Line3D = None
    l2: Line3D = None
    node: Line3D = None

    points: np.ndarray = None

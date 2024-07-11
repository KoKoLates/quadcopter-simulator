import numpy as np
import matplotlib.pyplot as plt

from dataclasses import dataclass

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D

from quadcopter import rotation_matrix
from quadcopter.quad import Quadcopter


@dataclass
class Model(object):
    l1: Line3D
    l2: Line3D
    node: Line3D
    points: np.ndarray


class Monitor(object):
    def __init__(self, quad: Quadcopter) -> None:
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel("X")
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel("Y")
        self.ax.set_zlim3d([-2.0, 2.0])
        self.ax.set_zlabel("Z")
        self.ax.set_title("Quadcopter Simulation")

        self.quad: Quadcopter = quad
        self.load_model()

    def load_model(self) -> None:
        line1 = Line3D([], [], [], color="b", linewidth=3, antialiased=False)
        line2 = Line3D([], [], [], color="r", linewidth=3, antialiased=False)
        line3 = Line3D([], [], [], color="r", linewidth=3, antialiased=False)
        
        L: float = self.quad.l
        point: np.ndarray = np.array([
            [-L, 0, 0], [ L, 0, 0], [ 0,-L, 0], 
            [ 0, L, 0], [ 0, 0, 0], [ 0, 0, 0]
        ]).T
        
        self.model: Model = Model(line1, line2, line3, point)

    def update(self) -> None:
        position, _, attitude, _ = self.quad.state
        R: np.ndarray = rotation_matrix(attitude)
        points = R @ self.model.points
        points += position[:, np.newaxis]

        self.model.l1.set_data(points[0, 0:2], points[1:0, 2])
        self.model.l1.set_3d_properties(points[2, 0:2])
        self.model.l2.set_data(points[0, 2:4], points[1, 2:4])
        self.model.l2.set_3d_properties(points[2, 2:4])
        self.model.node.set_data(points[0, 5], points[1, 5])
        self.model.node.set_3d_properties(points[2, 5])

        plt.pause(1e-5)

import sys
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

## finish the matplotlib for drone (and maybe for
## the attitude first. Not think about the Qt first)

class ViewPanel(object):
    def __init__(self, quadcopter: dict) -> None:
        self.fig = plt.figure()
        self.quadcopter: dict = quadcopter

        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-2., 2.])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-2., 2.])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([-2., 2.])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')

        self.initialize()
        self.fig.canvas.mpl_connect(
            'key_press_event', self.keypress_process)

    def initialize(self) -> None:
        self.quadcopter['l1'], _ = self.ax.plot(
            [], [], [], color='b', linewidth=3, antialiased=False)
        self.quadcopter['l2'], _ = self.ax.plot(
            [], [], [], color='r', linewidth=3, antialiased=False)
        self.quadcopter['node'], _ = self.ax.plot(
            [], [], [], marker='o', color='k', 
            markersize=3, antialiased=False)

    def update(self) -> None:
        R = ViewPanel.rotation_matrix(self.quadcopter['orientation'])
        L = self.quadcopter['L']
        points: np.ndarray = np.array([
            [-L, 0, 0], [ L, 0, 0], [ 0,-L, 0], 
            [ 0, L, 0], [ 0, 0, 0], [ 0, 0, 0]]).T
        points = np.dot(R, points)
        points[0, :] += self.quadcopter['position'][0]
        points[1, :] += self.quadcopter['position'][1]
        points[2, :] += self.quadcopter['position'][2]

        self.quadcopter['l1'].set_data(points[0, 0:2],points[1, 0:2])
        self.quadcopter['l1'].set_3d_properties(points[2, 0:2])
        self.quadcopter['l2'].set_data(points[0, 2:4],points[1, 2:4])
        self.quadcopter['l2'].set_3d_properties(points[2, 2:4])
        self.quadcopter['node'].set_data(points[0, 5],points[1, 5])
        self.quadcopter['node'].set_3d_properties(points[2, 5])

        plt.pause(1e-5)


    def keypress_process(self, event) -> None:
        key_mapping: dict = {
            'x': (self.ax.get_ylim3d, self.ax.set_ylim3d, 0.2),
            'w': (self.ax.get_ylim3d, self.ax.set_ylim3d,-0.2),
            'd': (self.ax.get_xlim3d, self.ax.set_xlim3d, 0.2),
            's': (self.ax.get_xlim3d, self.ax.set_xlim3d,-0.2)}
        sys.stdout.flush()
        if event.key not in key_mapping:
            return 
        
        get_lim, set_lim, delta = key_mapping[event.key]
        set_lim([limit + delta for limit in list(get_lim())])

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
import numpy as np

from quad import *
from simulator import Simulator


class WaypointLoader(object):
    def __init__(self, waypoints: list[tuple]) -> None:
        ## can add some check here, like:
        # 1. tuple format with (x, y, z, yaw)
        # 2. yaw and z limit, z most be non-negative
        self.waypoints: list[tuple] = waypoints
        self.index: int = 0


def main() -> None:
    waypoints: list[tuple] = [
        (1, 1, 2, 0),
        (1, -1, 4, 0),
        (-1, -1, 2, 0),
        (-1, 1, 4, 0),
    ]

    quad_config = QuadConfig(
        weight=1.2,
        length=0.3,
        radius=0.1,
        states=[[0, 0, 0], [0, 0, 0]],
        motors=MotorConfig(10, 4.5, 0, 0),
        drag_c=0.1,
    )
    quad: Quadcopter = Quadcopter(quad_config)
    app: Simulator = Simulator(quad_config)

    quad = Quadcopter(quad_config)
    ctrl = Controller()

    quad.start()
    ctrl.start()

    try:
        while True:
            for target in waypoints:
                ctrl.update_target(target)
                # keep update and wait reach
                app.update(quad.states)

    except KeyboardInterrupt:
        quad.stop()
        ctrl.stop()
        print("Simulation Stopped.")


if __name__ == "__main__":
    main()

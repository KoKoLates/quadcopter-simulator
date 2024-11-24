import numpy as np

from quad import *
from simulator import Simulator


def main() -> None:
    quad_config = QuadConfig(
        10, 10, 1, [0, 0, 0], 
        MotorConfig(10, 10, 0, 0), 10
    )
    ctrl_config = ControlConfig(

    )

    quad = Quadcopter(quad_config)
    ctrl = Controller(ctrl_config)

    waypoints = (1, 1, 1, 0)
    quad.start()
    ctrl.start() 

    try:
        while True:
            ctrl.update_target(waypoints)

    except KeyboardInterrupt:
        quad.stop()
        ctrl.stop()
        print("Simulation Stopped.")

if __name__ == "__main__":
    main()

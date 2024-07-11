import argparse
import numpy as np

from quadcopter import MotorConfig, QuadConfig, load_config
from quadcopter.quad import Quadcopter
from quadcopter.control import PID, CPID, ControlConfig


def main() -> None:
    quad_config: QuadConfig = load_config("./cfg/quad.json")
    quad: Quadcopter = Quadcopter(quad_config)

    ctrl_config: ControlConfig = ControlConfig(
        position=PID(
            Kp=[300,300,7000],
            Ki=[0.04,0.04,4.5],
            Kd=[450,450,5000]
        ),
        attitude=PID(
            Kp=[22000,22000,1500],
            Ki=[0,0,1.2],
            Kd=[12000,12000,0]
        )
    )
    ctrl: CPID = CPID(ctrl_config, quad)

    target: tuple = (1, 1, 1, 0)

    quad.start()
    ctrl.start()

    try:
        while True:
            ctrl.update_target(target)

    except KeyboardInterrupt:
        quad.stop()
        ctrl.stop()
        print("Simulation Stopped")


if __name__ == '__main__':
    main()

import argparse
import numpy as np

from quadcopter import MotorConfig, QuadConfig
from quadcopter.quad import Quadcopter
from quadcopter.control import PID, CPID, ControlConfig


def main() -> None:
    quad_config: QuadConfig = QuadConfig(
        weight=1.0,
        length=0.5,
        radius=0.2,
        states=[[0, 0, 0], [0, 0, 0]],
        motors=MotorConfig(0.1, 0.2),
        lift_const=0.1,
    )
    quad: Quadcopter = Quadcopter(quad_config)

    ctrl_config: ControlConfig = ControlConfig(
        position=PID(
            Kp=np.array([300,300,7000]),
            Ki=np.array([0.04,0.04,4.5]),
            Kd=np.array([450,450,5000])
        ),
        attitude=PID(
            Kp=np.array([22000,22000,1500]),
            Ki=np.array([0,0,1.2]),
            Kd=np.array([12000,12000,0])
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

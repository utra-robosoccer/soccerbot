#!/usr/bin/env python3

from math import pi

import numpy as np
from visual_kinematics.RobotSerial import *


def main():
    np.set_printoptions(precision=3, suppress=True)

    dh_params = np.array(
        [
            [0, 0.0, 0, 0.0],
            [0.16, 0.16, np.deg2rad(-90), np.deg2rad(-90)],
            [0.0, 0.60, np.deg2rad(90), 0],
        ]
    )  # mod

    dh_params = np.array(
        [
            [0.16, 0.16, np.deg2rad(90), 0],
            [0.0, 0.60, np.deg2rad(90), np.deg2rad(90)],
            [0, 0.0, 0, 0.0],
        ]
    )

    # dh_params = np.array([[0, 0., 0, 0.],
    #                       [0, 0, 0.5 * pi,0],
    #
    #                       ]) # mod
    # dh_params = np.array([
    #                       [0, 0, 0.5 * pi, 0],
    #                       [0, 0., 0, 0.],
    #                       ])
    robot = RobotSerial(dh_params)  # ,dh_type="modified")
    # robot = RobotSerial(dh_params ,dh_type="modified")
    # =====================================
    # forward
    # =====================================
    # theta = np.array([np.deg2rad(10), np.deg2rad(20)])

    theta = np.array([np.deg2rad(10), np.deg2rad(20), np.deg2rad(30)])
    f = robot.forward(theta)

    print("-------forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    print("end frame xyz:")
    print(
        f.t_3_1.reshape(
            [
                3,
            ]
        )
    )
    print("end frame abc:")
    print(f.euler_3)
    print("end frame rotational matrix:")
    print(f.r_3_3)
    print("end frame quaternion:")
    print(f.q_4)
    print("end frame angle-axis:")
    print(f.r_3)

    robot.show()


if __name__ == "__main__":
    main()

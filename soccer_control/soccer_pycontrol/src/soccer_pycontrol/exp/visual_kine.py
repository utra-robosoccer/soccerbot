#!/usr/bin/env python3

from math import pi

import numpy as np
from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotSerial import RobotSerial


def main():
    np.set_printoptions(precision=3, suppress=True)

    # dh_params_mod = np.array(
    #     [
    #         [0, 0.0,0, 0.0],
    #         [0., 0.16, np.deg2rad(-90), np.deg2rad(-90)],
    #         [0.0, 0.60, np.deg2rad(90), 0],
    #     ]
    # )  # mod
    #
    # dh_params = np.array(
    #     [
    #         [0., -0.16, np.deg2rad(90), np.deg2rad(180)],
    #         [0., 0.60, np.deg2rad(90), np.deg2rad(90)],
    #         [0, 0.0, 0, 0.0],
    #     ]
    # )

    dh_params_mod = np.array(
        [
            [0, 0.0, 0, 0.0],
            [0.0, 0.0, np.deg2rad(90), np.deg2rad(90)],
            [0.0, 0.0, np.deg2rad(90), 0],
            [0, 0.93, 0, 0.0],
            [0, 0.93, 0, 0.0],
            [0, 0.0, np.deg2rad(-90), 0.0],
        ]
    )

    dh_params = np.array(
        [
            [0.0, 0.0, np.deg2rad(90), 0],
            [0.0, 0.0, np.deg2rad(90), np.deg2rad(90)],
            [0, 0.89, 0, 0.0],
            [0, 0.827, 0, 0.0],
            [0, 0.0, np.deg2rad(90), 0.0],
            [0, 0.0, 0, 0],
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
    robot = RobotSerial(dh_params)
    # robot_mod = RobotSerial(dh_params_mod, dh_type="modified")
    # =====================================
    # forward
    # =====================================
    # theta = np.array([np.deg2rad(50), np.deg2rad(20), np.deg2rad(0)])
    # theta = np.array([np.deg2rad(0), np.deg2rad(0)])
    # theta = np.array([np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)])

    theta = np.array([np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)])
    theta = np.array([np.deg2rad(10), np.deg2rad(10), np.deg2rad(10), np.deg2rad(10), np.deg2rad(10), np.deg2rad(10)])
    theta = np.array([-2.220446049250313e-16, 0.6309126945505448, 0.15353047774239625, -0.3188605651802334, 0.1653300874378374, -0.6309126945505449])
    f = robot.forward(theta)

    xyz = np.array([[0], [-1], [1.369]])
    abc = np.array([pi / 2, -pi / 2, 0])
    end = Frame.from_euler_3(abc, xyz)
    ff = robot.inverse(end)
    print("inverse is successful: {0}".format(robot.is_reachable_inverse))
    print("axis values: \n{0}".format(robot.axis_values))
    print("-------forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    #
    # f2 = robot_mod.forward(theta)
    #
    # print("-------forward-2------")
    # print("end frame t_4_4:")
    # print(f2.t_4_4)

    robot.show()
    # robot_mod.show()


if __name__ == "__main__":
    main()

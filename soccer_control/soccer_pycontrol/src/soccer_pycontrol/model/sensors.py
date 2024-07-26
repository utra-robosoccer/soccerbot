from typing import List

import numpy as np
import pybullet as pb
from soccer_pycontrol.old.links import Links

from soccer_common import Transformation


class Sensors:
    """
    Interfaces with pybullet to extract sensor data.
    """

    def __init__(self, body: pb.loadURDF):
        # TODO does this need to be a class?
        self.body = body
        self.imu_link = pb.getNumJoints(self.body) - 1

    def get_pose(self) -> Transformation:
        """
        Get the 3D pose of the robot

        :return: The 3D pose of the robot
        """
        [position, quaternion] = pb.getLinkState(self.body, linkIndex=self.imu_link)[4:6]  # TODO double check
        return Transformation(position=position, quaternion=quaternion)

    def get_imu(self) -> Transformation:
        """
        Simulates the IMU at the IMU link location.
        TODO: Add noise model, make the refresh rate vary (currently in sync with the PyBullet time steps)
        TODO maybe change name
        :return: concatenated 3-axes values for linear acceleration and angular velocity
        """
        # TODO add unit test with rotation

        # 6:8 for linear and angular velocity this gets the imu link position and orientation
        [pos, orientation] = pb.getLinkState(self.body, linkIndex=self.imu_link, computeLinkVelocity=1)[4:6]  # TODO double check

        return Transformation(pos, orientation)

    def get_euler_angles(self) -> np.ndarray:
        imu = self.get_imu()

        if imu is None:
            return np.array([0, 0, 0])  # TODO should have a better error checking

        return imu.orientation_euler
        # TODO maybe default return?

    def get_foot_pressure_sensors(self, floor: pb.loadURDF) -> List[bool]:
        """
        Checks if 4 corners of the each feet are in contact with ground #
        TODO fix docstring
        # TODO add unit test with matplot for imu
        | Indices for looking from above on the feet plates
        |   Left         Right
        | 4-------5    0-------1
        | |   ^   |    |   ^   |      ^
        | |   |   |    |   |   |      | forward direction
        | |       |    |       |
        | 6-------7    2-------3

        :param floor: PyBullet body id of the plane the robot is walking on.
        :return: boolean array of 8 contact points on both feet, True: that point is touching the ground False: otherwise
        """
        locations = [False] * 8

        right_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.RIGHT_LEG_6)
        left_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.LEFT_LEG_6)

        right_center = np.array(pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[4])
        left_center = np.array(pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[4])

        right_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[5]).rotation_matrix
        left_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[5]).rotation_matrix

        # TODO verify calculations
        for point in right_pts:
            index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
            locations[index[1] + index[0] * 2] = True
        for point in left_pts:
            index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
            locations[index[1] + (index[0] * 2) + 4] = True
        return locations

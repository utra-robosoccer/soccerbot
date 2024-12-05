from typing import List

import cv2
import numpy as np
import pybullet as pb

from soccer_common import Transformation


class Sensors:
    """
    Interfaces with pybullet to extract sensor data.
    """

    def __init__(self, body: pb.loadURDF, ball: pb.loadURDF):
        # TODO does this need to be a class?
        self.body = body
        self.ball = ball
        # TODO should get based on name
        self.imu_link = pb.getNumJoints(self.body) - 1
        self.imu_ready = False
        self.get_imu()  # to init

    def get_pose(self, link=None) -> Transformation:
        """
        Get the 3D pose of the robot

        :return: The 3D pose of the robot
        """
        [position, quaternion] = pb.getLinkState(self.body, linkIndex=self.imu_link if not link else link)[4:6]  # TODO double check
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
        self.imu_ready = True
        return Transformation(pos, orientation).orientation_euler

    def get_ball(self):
        [position, quaternion] = pb.getBasePositionAndOrientation(self.body)
        [ball_position, ball_quaternion] = pb.getBasePositionAndOrientation(self.ball)
        trans = (np.array(ball_position) - np.array(position)).tolist()
        return Transformation(position=trans, quaternion=ball_quaternion)

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
        # TODO fix the links
        #
        # right_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.RIGHT_LEG_6)
        # left_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.LEFT_LEG_6)
        #
        # right_center = np.array(pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[4])
        # left_center = np.array(pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[4])
        #
        # right_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[5]).rotation_matrix
        # left_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[5]).rotation_matrix
        #
        # # TODO verify calculations
        # for point in right_pts:
        #     index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
        #     locations[index[1] + index[0] * 2] = True
        # for point in left_pts:
        #     index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
        #     locations[index[1] + (index[0] * 2) + 4] = True
        return locations

    def get_camera_image(self):
        """
        Captures the image from the camera mounted on the robot
        """
        # Add more offsets later
        camera_position = self.get_pose(link=1).position
        camera_target = [camera_position[0] + 1, camera_position[1], camera_position[2]]

        camera_up = [0, 0, 1]

        view_matrix = pb.computeViewMatrix(camera_position, camera_target, camera_up)

        width, height = 640, 480
        fov = 78
        aspect = width / height
        near = 0.2
        far = 100

        projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)

        images = pb.getCameraImage(width, height, view_matrix, projection_matrix, shadow=False, renderer=pb.ER_BULLET_HARDWARE_OPENGL)

        # NOTE: the ordering of height and width change based on the conversion
        img = np.reshape(images[2], (height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        # rgb_opengl = np.reshape(images[2], (height, width, 4))[:, :, :3] * 1. / 255.
        # depth_buffer_opengl = np.reshape(images[3], [width, height])
        # depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
        # seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.

        return img

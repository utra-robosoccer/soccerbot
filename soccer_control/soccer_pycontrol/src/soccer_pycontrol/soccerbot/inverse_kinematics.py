from typing import List

import numpy as np
import pybullet as pb
import scipy
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.motor_control import MotorControl

from soccer_common import Transformation


class InverseKinematics:
    def __init__(self, body: pb.loadURDF, walking_torso_height: float):
        # TODO might change these dependcies later
        self.body = body

        #: Ready Pose angle for arm 1
        self.arm_0_center = -0.45  # rospy.get_param("arm_0_center", -0.45)

        #: Ready Pose angle for arm 2
        self.arm_1_center = np.pi * 0.8  # rospy.get_param("arm_0_center", np.pi * 0.8)

        # TODO lots of duplicates with foot step planner need to fix it
        pitch_correction = Transformation([0, 0, 0], euler=[0, 0, 0])
        # rospy.get_param("torso_offset_pitch", 0.0), 0])
        self.torso_offset = Transformation([0, 0, 0]) @ pitch_correction
        # [rospy.get_param("torso_offset_x", 0), 0, 0]) @ pitch_correction

        #: Dimensions of the foot collision box #TODO get it from URDF also what do they mean
        self.foot_box = [0.09, 0.07, 0.01474]
        #  rospy.get_param("foot_box", [0.09, 0.07, 0.01474])

        # : Transformations from the right foots joint position to the center of the collision box of the foot
        # https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        self.right_foot_joint_center_to_collision_box_center = [0.00385, 0.00401, -0.00737]
        # rospy.get_param("right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737])

        # TODO what does this do exactly and why do we need it
        self.foot_center_to_floor = -self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2]  # 0.0221

        # Calculate Constants
        H34 = self.get_link_transformation(Links.RIGHT_LEG_4, Links.RIGHT_LEG_3)
        H45 = self.get_link_transformation(Links.RIGHT_LEG_5, Links.RIGHT_LEG_4)

        # TODO verify calculations
        self.DH = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [H34[2, 3], 0, 0, 0],
                [H45[2, 3], 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )
        # TODO could get walking height from urdf but now need to know why it was chosen
        self.torso_to_right_hip = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_1)
        self.right_hip_to_left_hip = self.get_link_transformation(Links.LEFT_LEG_1, Links.RIGHT_LEG_1)

        pitch_correction = Transformation([0, 0, 0], euler=[0, 0, 0])
        # rospy.get_param("torso_offset_pitch_ready", 0.0), 0])
        self.right_foot_init_position = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_6)
        print(self.right_foot_init_position.position)
        # TODO what does this do also dont like that it needs to happen as first step, clean up interface
        self.right_foot_init_position[2, 3] = -walking_torso_height + self.foot_center_to_floor
        self.right_foot_init_position[0, 3] -= 0.0  # rospy.get_param("torso_offset_x_ready", 0.0)
        print(self.right_foot_init_position.position)
        # self.right_foot_init_position = pitch_correction @ self.right_foot_init_position

        self.left_foot_init_position = self.get_link_transformation(Links.TORSO, Links.LEFT_LEG_6)
        self.left_foot_init_position[2, 3] = -walking_torso_height + self.foot_center_to_floor

        self.left_foot_init_position[0, 3] -= 0.0  # rospy.get_param("torso_offset_x_ready", 0.0)
        # self.left_foot_init_position = pitch_correction @ self.left_foot_init_position

    def get_link_transformation(self, link1, link2):
        """
        Gives the H-trasnform between two links
        :param link1: Starting link
        :param link2: Ending link
        :return: H-transform from starting link to the ending link
        """
        # TODO compare with calc inverse kinematics
        if link1 == Links.TORSO:
            link1world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link1world = pb.getLinkState(self.body, link1)[4:6]

        if link2 == Links.TORSO:
            link2world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link2world = pb.getLinkState(self.body, link2)[4:6]

        link1worldrev = pb.invertTransform(link1world[0], link1world[1])

        final_transformation = pb.multiplyTransforms(link2world[0], link2world[1], link1worldrev[0], link1worldrev[1])
        return Transformation(np.round(list(final_transformation[0]), 5), np.round(list(final_transformation[1]), 5))

    def inverseKinematicsRightFoot(self, transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        # TODO add plots and verify calculations
        # TODO all IK is done from 0,0,0 assumption
        # TODO make the library independent from pybullet
        transformation[0:3, 3] = transformation[0:3, 3] - self.torso_to_right_hip[0:3, 3]
        invconf = scipy.linalg.inv(transformation)
        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4)

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = (((((tmp1**2) + (tmp2**2)) - ((d3**2) + (d4**2))) / 2) / d3) / d4
        tmp3 = np.arctan2(D, -np.sqrt(1 - (D**2)))

        tmpX = tmp3 - (np.pi / 2)
        if tmpX < 0:
            tmpX = tmpX + (2 * np.pi)
        theta4 = -(np.unwrap([tmpX])[0])

        assert theta4 < 4.6

        alp = np.arctan2(tmp1, tmp2)
        beta = np.arctan2(-d3 * np.cos(tmp3), d4 + (d3 * np.sin(tmp3)))
        theta5 = np.pi / 2 - (alp - beta)

        H34 = Transformation(dh=[self.DH[3, 0], self.DH[3, 1], self.DH[3, 2], theta4])
        H45 = Transformation(dh=[self.DH[4, 0], self.DH[4, 1], self.DH[4, 2], theta5])
        H56 = Transformation(dh=[self.DH[5, 0], self.DH[5, 1], self.DH[5, 2], theta6])
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(H36))
        assert np.linalg.norm(H03[0:3, 3]) - d3 < 0.03

        angles = Transformation(rotation_matrix=scipy.linalg.inv(H03[0:3, 0:3])).orientation_euler
        theta3 = np.pi / 2 - angles[0]
        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    # TODO make the IK independent of pybullet

    def inverseKinematicsLeftFoot(self, transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: Motor angles for the left foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] + self.right_hip_to_left_hip[0:3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.inverseKinematicsRightFoot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def ready(self) -> List[float]:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """

        # hands
        configuration = [0.0] * len(Joints)
        configuration[Joints.RIGHT_ARM_1] = self.arm_0_center
        configuration[Joints.LEFT_ARM_1] = self.arm_0_center
        configuration[Joints.RIGHT_ARM_2] = self.arm_1_center
        configuration[Joints.LEFT_ARM_2] = self.arm_1_center

        # right leg
        thetas = self.inverseKinematicsRightFoot(np.copy(self.right_foot_init_position))

        # thetas = self.inverseKinematicsRightFoot(Transformation(position=[ -0.085, -0.035, -0.29289]))
        configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        # left leg
        thetas = self.inverseKinematicsLeftFoot(np.copy(self.left_foot_init_position))
        configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]

        return configuration

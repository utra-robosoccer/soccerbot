from os.path import expanduser

import numpy as np
import pybullet as pb
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.motor_control import MotorControl
from soccer_pycontrol.sensors import Sensors

from soccer_common import Transformation
from soccer_common.utils import wrapToPi


class PybulletModel:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, pose: Transformation = Transformation(), robot_model: str = "bez1"):
        """
        Class for interacting with pybullet model

        """
        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = 0.315  # rospy.get_param("walking_torso_height", 0.315)

        # TODO read from yaml?

        # self.pose = self.set_walking_torso_height(pose)

        home = expanduser("~")
        self.body = pb.loadURDF(
            home + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/{robot_model}.urdf",
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | 0,
            basePosition=pose.position,
            baseOrientation=pose.quaternion,
        )
        # self.set_pose(pose)

        self.motor_control = MotorControl(self.body)
        self.sensors = Sensors(self.body)

    # Pose
    def set_walking_torso_height(self, pose: Transformation) -> Transformation:
        """
        Takes a 2D pose and sets the height of the pose to the height of the torso
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        """
        # if pose.position[2] < self.walking_torso_height:
        pose.position = (pose.position[0], pose.position[1], self.walking_torso_height)

        return pose

    def set_pose(self, pose: Transformation = Transformation()) -> None:
        """
        Teleports the robot to the desired pose

        :param pose: 3D position in pybullet
        """
        self.pose = self.set_walking_torso_height(pose)

        # Remove the roll and yaw from the pose TODO why also your code and comment is wrong
        [r, p, y] = pose.orientation_euler
        self.pose.orientation_euler = [r, 0, 0]
        if pb.isConnected():
            pb.resetBasePositionAndOrientation(self.body, self.pose.position, self.pose.quaternion)

from os.path import expanduser

import numpy as np
import pybullet as pb
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.motor_control import MotorControl
from soccer_pycontrol.sensors import Sensors
from soccer_pycontrol.soccerbot.inverse_kinematics_pybullet import (
    InverseKinematicsPybullet,
)

from soccer_common import Transformation
from soccer_common.utils import wrapToPi


class PybulletModel:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, pose: Transformation = Transformation(), robot_model: str = "bez1", fixed_base: bool = False):
        """
        Class for interacting with pybullet model

        """
        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = 0.315  # rospy.get_param("walking_torso_height", 0.315)

        # TODO read from yaml?

        home = expanduser("~")
        self.body = pb.loadURDF(
            home + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/{robot_model}.urdf",
            useFixedBase=fixed_base,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | 0,
            basePosition=pose.position,
            baseOrientation=pose.quaternion,
        )
        self.motor_control = MotorControl(self.body)
        self.sensors = Sensors(self.body)
        self.ik = InverseKinematicsPybullet(self.body, self.walking_torso_height)

        if not fixed_base:
            self.set_pose(pose)

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

    def ready(self) -> None:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """

        configuration = self.ik.ready()

        # head
        configuration[Joints.HEAD_1] = self.motor_control.configuration[Joints.HEAD_1]
        configuration[Joints.HEAD_2] = self.motor_control.configuration[Joints.HEAD_2]

        # Slowly ease into the ready position
        previous_configuration = self.motor_control.configuration

        # TODO clean up math
        for r in np.arange(0, 1.00, 0.040):
            self.motor_control.configuration[0:18] = (
                np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
            ).tolist()
            self.motor_control.set_motor()
            pb.stepSimulation()
            import time

            time.sleep(1 / 100)

        # TODO do i need this
        # self.motor_control.configuration_offset = [0] * len(Joints)

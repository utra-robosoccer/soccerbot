import math
import os
from typing import Optional

import numpy as np
import rospy
import scipy
import tf
import tf2_py
import torch
from geometry_msgs.msg import Pose2D, PoseStamped
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.inverse_kinematics.ik_actions import IKActions
from soccer_pycontrol.model.model_ros.kinematic_data_ros import KinematicDataROS
from soccer_pycontrol.model.model_ros.motor_control_ros import MotorControlROS
from soccer_pycontrol.model.model_ros.sensors_ros import SensorsROS

# from soccer_pycontrol.old.joints import Joints
from std_msgs.msg import Empty

from soccer_common import Transformation
from soccer_msgs.msg import RobotState


class BezROS(Bez):
    def __init__(self):

        self.data = KinematicDataROS()

        self.motor_control = MotorControlROS(self.data.motor_names)
        self.sensors = SensorsROS()

        self.ik_actions = IKActions(self.data)

    # TODO dont like this placement
    def set_walking_torso_height(self, pose: Transformation) -> Transformation:
        """
        Takes a 2D pose and sets the height of the pose to the height of the torso
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        """
        # if pose.position[2] < self.walking_torso_height:
        # pose.position = (pose.position[0], pose.position[1], self.data.walking_torso_height)
        p = pose
        position = p.position
        position[2] = self.data.walking_torso_height
        p.position = position
        return p

    def ready(self) -> None:
        super(BezROS, self).ready()

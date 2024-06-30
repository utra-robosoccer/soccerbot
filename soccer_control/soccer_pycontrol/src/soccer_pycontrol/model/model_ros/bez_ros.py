import os

import rospy
import tf
from nav_msgs.msg import Odometry
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.inverse_kinematics.ik_actions import IKActions
from soccer_pycontrol.model.inverse_kinematics.kinematic_data import KinematicData
from soccer_pycontrol.model.model_ros.kinematic_data_ros import KinematicDataROS
from soccer_pycontrol.model.model_ros.motor_control_ros import MotorControlROS
from soccer_pycontrol.model.model_ros.sensors_ros import SensorsROS

from soccer_common import Transformation


class BezROS(Bez):
    def __init__(self):

        self.data = KinematicDataROS()

        self.motor_control = MotorControlROS(self.data.motor_names)
        self.sensors = SensorsROS()

        self.ik_actions = IKActions(self.data)

        self.pose = Transformation()  # TODO need a better way

import numpy as np
import rospy
from soccer_pycontrol.model.inverse_kinematics.kinematic_data import KinematicData

from soccer_common import Transformation


class KinematicDataROS(KinematicData):
    def __init__(self):  # TODO needs a better way to organize
        robot_model = rospy.get_param("robot_model", "bez1")
        if rospy.get_param("/use_sim_time", True):
            sim = "_sim"
        else:
            sim = ""

        super(KinematicDataROS, self).__init__(robot_model=robot_model, sim=sim)
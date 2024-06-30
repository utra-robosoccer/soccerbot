import numpy as np
import rospy
from soccer_pycontrol.model.inverse_kinematics.kinematic_data import KinematicData

from soccer_common import Transformation


class KinematicDataROS(KinematicData):
    def __init__(self):
        robot_model = rospy.get_param("robot_model", "bez1")
        arm_0_center = rospy.get_param("arm_0_center", -0.45)
        arm_1_center = rospy.get_param("arm_0_center", np.pi * 0.8)
        walking_torso_height = rospy.get_param("walking_torso_height", 0.315)
        torso_offset_x_ready = rospy.get_param("torso_offset_x_ready", 0.0)
        ready_pitch_correction = Transformation([0, 0, 0], euler=[0, rospy.get_param("torso_offset_pitch_ready", 0.0), 0])
        foot_box = rospy.get_param("foot_box", [0.09, 0.07, 0.01474])
        right_foot_joint_center_to_collision_box_center = rospy.get_param(
            "right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737]
        )
        cleats_offset = rospy.get_param(
            "cleats_offset", -0.01634
        )  #: Additional height added by cleats and grass, consists of 1cm grass and 0.5cm cleats

        super(KinematicDataROS, self).__init__(
            robot_model=robot_model,
            arm_0_center=arm_0_center,
            arm_1_center=arm_1_center,
            walking_torso_height=walking_torso_height,
            torso_offset_x_ready=torso_offset_x_ready,
            ready_pitch_correction=ready_pitch_correction,
            foot_box=foot_box,
            right_foot_joint_center_to_collision_box_center=right_foot_joint_center_to_collision_box_center,
            cleats_offset=cleats_offset,
        )

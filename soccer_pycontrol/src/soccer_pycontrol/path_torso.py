import math

import numpy as np
import rospy
import scipy

from soccer_common.transformation import Transformation
from soccer_pycontrol.path_foot import PathFoot


class PathTorso(PathFoot):

    #: How much the torso bounces up and down while following the torso trajectory (m)
    torso_zdiff_sway = rospy.get_param("torso_zdiff_sway", 0.00)

    #: How much the torso sways left and right while following the torso trajectory  (m)
    torso_sidediff_sway = rospy.get_param("torso_sidediff_sway", -0.025)

    #: How much the torso rotates while following the torso trajectory (yaw, pitch, roll)
    torso_thetadiff_sway = rospy.get_param("torso_thetadiff_sway", [0.0, 0.0, 0.0])

    def __init__(self, start_transform, end_transform, foot_center_to_floor):
        super().__init__(start_transform, end_transform, foot_center_to_floor)

        # Calculate the foot for the first step (based on destination)
        axang_angle, axang_vector = Transformation.get_axis_angle_from_quaternion(self.start_transform.quaternion)
        theta1 = axang_angle
        diff_transform = np.matmul(end_transform, scipy.linalg.inv(start_transform))
        theta2 = np.arctan2(diff_transform[1, 3], diff_transform[0, 3])
        if (theta2 - theta1) % (2 * np.pi) > np.pi:
            self.first_step_left = 0
        else:
            self.first_step_left = 1

    def torsoPosition(self, t: float) -> Transformation:
        """
        Retrieves the torso position at a given time of the path

        :param t: Time since the beginning of the robot path
        :return: The transformation of the torso position on the path
        """

        [step_num, right_foot_ratio, left_foot_ratio] = self.leftRightFootStepRatio(t, 1)
        [right_foot_action, _] = self.whatIsTheFootDoing(step_num)
        if len(right_foot_action) == 2:
            ratio = right_foot_ratio
        else:
            ratio = left_foot_ratio

        # Base position for the torso
        if step_num == 0:
            _from = self.getTorsoStepPose(0)
            _to = self.getTorsoStepPose(1)
            body_movement_ratio = ratio / 2
        elif step_num == self.num_steps() - 1:
            _from = self.getTorsoStepPose(step_num - 1)
            _to = self.getTorsoStepPose(step_num)
            body_movement_ratio = (ratio / 2) + (1 / 2)
        else:
            if ratio < 0.5:
                _from = self.getTorsoStepPose(step_num - 1)
                _to = self.getTorsoStepPose(step_num)
                body_movement_ratio = ratio + 0.5
            else:
                _from = self.getTorsoStepPose(step_num)
                _to = self.getTorsoStepPose(step_num + 1)
                body_movement_ratio = ratio - 0.5

        position = self.parabolicPath(_from, _to, 0.0, 0.0, 0.0, body_movement_ratio)

        # Vertical Sway (sinusoidal wave)
        [_, right_foot_ratio, _] = self.leftRightFootStepRatio(t, 3)
        if len(right_foot_action) == 2:  # Left foot moving, lean right
            ratio = right_foot_ratio
        else:
            ratio = left_foot_ratio

        if t < self.half_step_time():
            zdiff = self.torso_zdiff_sway * (1 - np.cos(ratio * np.pi))
        elif t > self.duration() - self.half_step_time():
            zdiff = self.torso_zdiff_sway * (1 - np.cos((ratio * np.pi) + np.pi))
        else:
            zdiff = self.torso_zdiff_sway * (1 - np.cos((ratio * 2 * np.pi) + np.pi))

        # Horizontal Sway (exponential decay)
        [_, right_foot_ratio, left_foot_ratio] = self.leftRightFootStepRatio(t, 3)
        if len(right_foot_action) == 2:
            ratio = right_foot_ratio
            is_right_foot = -1
        else:
            ratio = left_foot_ratio
            is_right_foot = 1

        r = -4 * (ratio**2) + 4 * ratio
        ydiff = r * self.torso_sidediff_sway * is_right_foot
        thetadiff = ydiff / self.torso_sidediff_sway * np.array(self.torso_thetadiff_sway)

        H = Transformation(euler=thetadiff)  # H = eul2tform(thetadiff)
        H.position = [-0.005, ydiff, zdiff]

        position = position @ H

        return position

    def show(self, fig=None):
        """
        Draws the torso position

        :param fig: Figure Handle
        """
        i = 0
        iterator = np.linspace(0, self.duration(), num=math.ceil(self.duration() / self.step_precision) + 1)
        tfInterp = np.zeros((4, 4, len(iterator)))
        for t in iterator:
            tfInterp[:, :, i] = self.torsoPosition(t)
            i = i + 1
        self.show_tf(fig, tfInterp, len(iterator))

import functools
from copy import deepcopy

import numpy as np
import rospy

from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi
from soccer_pycontrol.path_section import PathSection


class PathSectionShort(PathSection):
    """
    A path section made up an initial rotation, followed by a linear path and a final rotatio
    """

    def __init__(self, start_transform: Transformation, end_transform: Transformation):
        self.start_transform: Transformation = start_transform
        self.end_transform: Transformation = end_transform

        self.steps_per_second_default = rospy.get_param("steps_per_second_default", 2.5)  # try 6 motors P = 09.25

        #: How much to incrase the rotation angle when turning (for calibration purposes)
        self.scale_yaw = rospy.get_param("scale_yaw", 1.0)  # Increase the rotation by angle

        if self.isWalkingBackwards():
            torso_step_length = rospy.get_param("torso_step_length_short_backwards", 0.025)
        else:
            torso_step_length = rospy.get_param("torso_step_length_short_forwards", 0.035)

        self.angular_torso_step_length = 0.25  # radians Radians per angular step
        self.angular_speed = self.steps_per_second_default * self.angular_torso_step_length  # Rotational speed in radians per second
        super().__init__(start_transform, end_transform, torso_step_length)

        if self.angle_distance != 0 and self.distance == 0:
            self.angular_torso_step_length = self.angle_distance / np.ceil(self.angle_distance / self.angular_torso_step_length)
            if self.torsoStepCount() <= 1:
                self.angular_speed = self.steps_per_second_default * self.angular_torso_step_length

    def poseAtRatio(self, r):
        diff_position = self.end_transform.position[0:2] - self.start_transform.position[0:2]
        start_angle = self.start_transform.orientation_euler[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        intermediate_angle = wrapToPi(intermediate_angle - start_angle) * self.scale_yaw + start_angle
        if self.isWalkingBackwards():
            intermediate_angle = wrapToPi(intermediate_angle + np.pi)
        if diff_position[0] == 0 and diff_position[1] == 0:  # If the movement is only a rotation
            intermediate_angle = start_angle
        final_angle = self.end_transform.orientation_euler[0]

        step_1_duration = abs(wrapToPi(intermediate_angle - start_angle)) / self.angular_speed
        step_2_duration = np.linalg.norm(diff_position) / self.speed
        step_3_duration = abs(wrapToPi(intermediate_angle - final_angle)) / self.angular_speed

        total_duration = step_1_duration + step_2_duration + step_3_duration
        t = r * total_duration

        if t == 0:
            pose = deepcopy(self.start_transform)
            return pose
        elif t < step_1_duration != 0:
            # First turn
            pose = deepcopy(self.start_transform)
            percentage = t / step_1_duration
            angle = start_angle + wrapToPi(intermediate_angle - start_angle) * percentage
            pose.orientation_euler = [angle, 0, 0]
            return pose
        elif step_1_duration < t <= step_1_duration + step_2_duration != 0:
            # Then go straight
            pose = deepcopy(self.start_transform)
            percentage = (t - step_1_duration) / step_2_duration
            position = diff_position * percentage + self.start_transform.position[0:2]
            pose.position = np.concatenate((position, [pose.position[2]]))
            pose.orientation_euler = [intermediate_angle, 0, 0]
            return pose
        elif step_1_duration + step_2_duration < t <= step_1_duration + step_2_duration + step_3_duration != 0:
            # Then turn
            pose = deepcopy(self.end_transform)
            percentage = (t - step_1_duration - step_2_duration) / step_3_duration
            angle = intermediate_angle + wrapToPi(final_angle - intermediate_angle) * percentage
            pose.orientation_euler = [angle, 0, 0]
            return pose
        else:
            pose = deepcopy(self.end_transform)
            return pose

    def getRatioFromStep(self, step_num):
        diff_position = self.end_transform.position[0:2] - self.start_transform.position[0:2]
        start_angle = self.start_transform.orientation_euler[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        if self.isWalkingBackwards():
            intermediate_angle = wrapToPi(intermediate_angle + np.pi)
        final_angle = self.end_transform.orientation_euler[0]

        step_1_angular_distance = abs(wrapToPi(intermediate_angle - start_angle))
        step_2_distance = np.linalg.norm(diff_position)
        step_3_angular_distance = abs(wrapToPi(intermediate_angle - final_angle))

        step_1_steps = step_1_angular_distance / self.angular_torso_step_length
        step_2_steps = step_2_distance / self.torso_step_length
        step_3_steps = step_3_angular_distance / self.angular_torso_step_length
        if step_1_steps + step_2_steps + step_3_steps == 0:
            ratio = 0
        else:
            ratio = step_num / (step_1_steps + step_2_steps + step_3_steps)
        return ratio

    def duration(self):
        return self.distance / self.speed + self.angle_distance / self.angular_speed

    @functools.lru_cache
    def isWalkingBackwards(self):
        # Hacky attribute obtained from the calibration
        if hasattr(self.end_transform, "is_walking_backwards"):
            return self.end_transform.is_walking_backwards

        diff_position = self.end_transform.position[0:2] - self.start_transform.position[0:2]
        start_angle = self.start_transform.orientation_euler[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        return abs(wrapToPi(intermediate_angle - start_angle)) > np.pi / 2

    def torsoStepCount(self):
        return self.linearStepCount() + self.angularStepCount()

    def angularStepCount(self):
        return self.angle_distance / self.angular_torso_step_length

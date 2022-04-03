import functools

from soccer_common.transformation import Transformation
import numpy as np
from soccer_pycontrol.path_section import PathSection
from soccer_pycontrol.utils import wrapToPi
from copy import deepcopy

class PathSectionShort(PathSection):
    steps_per_second_default = 2.5 # try 6 motors P = 09.25
    scale_yaw = 1.0 # Increase the rotation by angle

    def __init__(self, start_transform: Transformation, end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform

        if self.isWalkingBackwards():
            bodystep_size = 0.025
        else:
            bodystep_size = 0.035

        self.angular_bodystep_size = 0.25  # radians Radians per angular step
        self.angular_speed = self.steps_per_second_default * self.angular_bodystep_size  # Rotational speed in radians per second
        super().__init__(start_transform, end_transform, bodystep_size)

    def poseAtRatio(self, r):
        diff_position = self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]
        start_angle = self.start_transform.get_orientation_euler()[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        intermediate_angle = wrapToPi(intermediate_angle - start_angle) * self.scale_yaw + start_angle
        if self.isWalkingBackwards():
            intermediate_angle = wrapToPi(intermediate_angle + np.pi)
        final_angle = self.end_transform.get_orientation_euler()[0]

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
            pose.set_orientation(Transformation.get_quaternion_from_euler([angle, 0, 0]))
            return pose
        elif step_1_duration < t <= step_1_duration + step_2_duration != 0:
            # Then go straight
            pose = deepcopy(self.start_transform)
            percentage = (t - step_1_duration) / step_2_duration
            position = diff_position * percentage + self.start_transform.get_position()[0:2]
            pose.set_position(np.concatenate((position, [pose.get_position()[2]])))
            pose.set_orientation(Transformation.get_quaternion_from_euler([intermediate_angle, 0, 0]))
            return pose
        elif step_1_duration + step_2_duration < t <= step_1_duration + step_2_duration + step_3_duration != 0:
            # Then turn
            pose = deepcopy(self.end_transform)
            percentage = (t - step_1_duration - step_2_duration) / step_3_duration
            angle = intermediate_angle + wrapToPi(final_angle - intermediate_angle) * percentage
            pose.set_orientation(Transformation.get_quaternion_from_euler([angle, 0, 0]))
            return pose
        else:
            pose = deepcopy(self.end_transform)
            return pose

    def getRatioFromStep(self, step_num):
        diff_position = self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]
        start_angle = self.start_transform.get_orientation_euler()[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        if self.isWalkingBackwards():
            intermediate_angle = wrapToPi(intermediate_angle + np.pi)
        final_angle = self.end_transform.get_orientation_euler()[0]

        step_1_angular_distance = abs(wrapToPi(intermediate_angle - start_angle))
        step_2_distance = np.linalg.norm(diff_position)
        step_3_angular_distance = abs(wrapToPi(intermediate_angle - final_angle))

        step_1_steps = step_1_angular_distance / self.angular_bodystep_size
        step_2_steps = step_2_distance / self.bodystep_size
        step_3_steps = step_3_angular_distance / self.angular_bodystep_size
        if step_1_steps + step_2_steps + step_3_steps == 0:
            ratio = 0
        else:
            ratio = step_num / (step_1_steps + step_2_steps + step_3_steps)
        return ratio

    def duration(self):
        return self.distance / self.speed + self.angle_distance / self.angular_speed

    @functools.lru_cache
    def isWalkingBackwards(self):
        diff_position = self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]
        start_angle = self.start_transform.get_orientation_euler()[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        return abs(wrapToPi(intermediate_angle - start_angle)) > np.pi / 2

    def bodyStepCount(self):
        return self.linearStepCount() + self.angularStepCount()

    def angularStepCount(self):
        return self.angle_distance / self.angular_bodystep_size

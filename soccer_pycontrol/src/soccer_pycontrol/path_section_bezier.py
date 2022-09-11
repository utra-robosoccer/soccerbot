import functools

import numpy as np
import rospy
from scipy.special import comb

from soccer_common.transformation import Transformation
from soccer_pycontrol.path_section import PathSection


class PathSectionBezier(PathSection):
    """
    A path section made up of bezier curves
    """

    #: The amount of torso steps it takes to make the starting and final turn
    turn_duration = rospy.get_param("turn_duration", 3)

    def __init__(self, start_transform: Transformation, end_transform: Transformation):
        self.start_transform: Transformation = start_transform
        self.end_transform: Transformation = end_transform
        if self.isWalkingBackwards():
            torso_step_length = self.torso_step_length_default * self.backwards_torso_step_length_ratio
        else:
            torso_step_length = self.torso_step_length_default

        super().__init__(start_transform, end_transform, torso_step_length)

    def poseAtRatio(self, r):
        pose = self.bezierPositionAtRatio(self.start_transform, self.end_transform, r)
        pose_del = self.bezierPositionAtRatio(self.start_transform, self.end_transform, r + 0.001)

        if self.isWalkingBackwards():
            del_pose = pose.position - pose_del.position
        else:
            del_pose = pose_del.position - pose.position

        # If walking backwards
        del_theta = np.arctan2(del_pose[1], del_pose[0])
        del_psi = np.arctan2(del_pose[2], np.linalg.norm(del_pose[0:2]))

        pose.orientation_euler = [del_theta, -del_psi, 0.0]
        return pose

    def bezierPositionAtRatio(self, start_transform, end_transform, r):
        # If the distance is small, use turn in place strategy, otherwise cubic bezier

        p1 = start_transform
        if self.isWalkingBackwards():
            p2 = np.matmul(start_transform, Transformation([-self.speed * PathSectionBezier.turn_duration, 0.0, 0.0]))
            p3 = np.matmul(end_transform, Transformation([self.speed * PathSectionBezier.turn_duration, 0.0, 0.0]))
        else:
            p2 = np.matmul(start_transform, Transformation([self.speed * PathSectionBezier.turn_duration, 0.0, 0.0]))
            p3 = np.matmul(end_transform, Transformation([-self.speed * PathSectionBezier.turn_duration, 0.0, 0.0]))
        p4 = end_transform

        p1_pos = p1.position
        p2_pos = p2.position
        p3_pos = p3.position
        p4_pos = p4.position

        position = np.array([0.0, 0.0, 0.0])
        for d in range(0, 3):
            bez_param = [p1_pos[d], p2_pos[d], p3_pos[d], p4_pos[d]]

            # Cubic bezier
            for i in range(0, 4):
                position[d] = position[d] + bez_param[i] * comb(3, i, exact=True, repetition=False) * ((1 - r) ** (3 - i)) * (r**i)
        return Transformation(position)

    def getRatioFromStep(self, step_num):
        idx = np.argmin(np.abs((step_num * self.torso_step_length) - self.distanceMap[:, 1]))
        return self.distanceMap[idx, 0]

    def duration(self):
        return self.distance / self.speed

    @functools.lru_cache
    def isWalkingBackwards(self):
        start_angle = self.start_transform.orientation_euler[0]
        del_pose = self.end_transform.position - self.start_transform.position
        if np.dot([np.cos(start_angle), np.sin(start_angle)], del_pose[0:2]) < 0:
            return True
        return False

    def torsoStepCount(self):
        return self.linearStepCount()

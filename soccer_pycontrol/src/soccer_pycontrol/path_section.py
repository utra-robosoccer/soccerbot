import abc
import functools
import time
from abc import ABC

import numpy as np
import rospy

from soccer_common.transformation import Transformation
from soccer_pycontrol.utils import wrapTo2Pi, wrapToPi


class PathSection(ABC):
    backwards_bodystep_size_ratio = rospy.get_param("backwards_bodystep_size_ratio", 0.5)  # How much smaller the body step is for backwards movement
    bodystep_size_default = rospy.get_param("bodystep_size_default", 0.15)
    steps_per_second_default = rospy.get_param("steps_per_second_default", 0.15)


    def __init__(self, start_transform: Transformation, end_transform: Transformation, bodystep_size=bodystep_size_default):
        self.start_transform = start_transform
        self.end_transform = end_transform

        self.bodystep_size = bodystep_size
        self.steps_per_second = PathSection.steps_per_second_default
        self.speed = self.steps_per_second * self.bodystep_size
        self.precision = 0.05 * self.bodystep_size

        # Calculate distance and angular distance
        precisions = np.linspace(self.precision, 1.0, num=(int(1.0 / self.precision) + 1))
        self.distance = 0
        self.distance_original = 0
        self.angle_distance = 0
        self.distanceMap = np.zeros((len(precisions) + 1, 2))
        self.distanceMap[0, 0:2] = [0.0, 0.0]

        j = 1
        precisions = np.linspace(self.precision, 1.0, num=(int(1.0 / self.precision) + 1))
        prev_pose = self.poseAtRatio(0)
        for i in precisions:
            new_pose = self.poseAtRatio(i)
            self.distance = self.distance + Transformation.get_distance(prev_pose, new_pose)
            self.angle_distance = self.angle_distance + abs(wrapToPi(new_pose.get_orientation_euler()[0] - prev_pose.get_orientation_euler()[0]))
            prev_pose = new_pose

            self.distanceMap[j, 0:2] = [i, self.distance]
            j = j + 1

    @abc.abstractmethod
    def poseAtRatio(self, r):
        pass

    def linearStepCount(self):
        return self.distance / self.bodystep_size

    def angularStepCount(self):
        return 0

    @abc.abstractmethod
    def getRatioFromStep(self, step_num):
        pass

    def getBodyStepPose(self, step_num):
        ratio = self.getRatioFromStep(step_num)
        return self.poseAtRatio(ratio)

    @abc.abstractmethod
    def duration(self):
        pass

    @abc.abstractmethod
    @functools.lru_cache
    def isWalkingBackwards(self):
        pass

    @abc.abstractmethod
    def bodyStepCount(self):
        pass

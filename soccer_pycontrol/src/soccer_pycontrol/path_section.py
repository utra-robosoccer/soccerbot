import abc
import functools
from soccer_pycontrol.transformation import Transformation
import numpy as np
from path import Path
from utils import wrapTo2Pi, wrapToPi
from abc import ABC

class PathSection(ABC):
    precision = 0.05 * Path.bodystep_size

    def __init__(self, start_transform: Transformation,  end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform

        # Calculate distance and angular distance
        precisions = np.linspace(self.precision, 1.0, num=(int(1.0 / self.precision) + 1))
        self.distance = 0
        self.angle_distance = 0
        self.distanceMap = np.zeros((len(precisions) + 1, 2))
        self.distanceMap[0, 0:2] = [0., 0.]

        j = 1
        precisions = np.linspace(self.precision, 1.0, num=(int(1.0 / self.precision) + 1))
        prev_pose = self.poseAtRatio(0)
        for i in precisions:
            new_pose = self.poseAtRatio(i)
            self.distance = self.distance + Transformation.get_distance(prev_pose, new_pose)
            self.angle_distance = self.angle_distance + abs(
                wrapToPi(new_pose.get_orientation_euler()[0] - prev_pose.get_orientation_euler()[0]))
            prev_pose = new_pose

            self.distanceMap[j, 0:2] = [i, self.distance]
            j = j + 1
        pass

    @abc.abstractmethod
    def poseAtRatio(self, r):
        pass

    def linearStepCount(self):
        return int(np.floor(self.distance / Path.bodystep_size))

    def angularStepCount(self):
        return int(np.floor(self.angle_distance / Path.angular_bodystep_size))

    def adjustBodyStepSizes(self):
        # Round to nearest step
        s_count = self.linearStepCount()
        if self.distance != 0:
            if self.distance % self.bodystep_size < (self.bodystep_size / 2):
                self.bodystep_size = self.distance / s_count
            else:
                self.bodystep_size = self.distance / (s_count + 1)

        s_count = self.angularStepCount()
        if self.angle_distance != 0 and s_count != 0:
            if self.angle_distance != 0 and self.angle_distance % self.angular_bodystep_size < (
                    self.angular_bodystep_size / 2):
                self.angular_bodystep_size = self.angle_distance / s_count
            else:
                self.angular_bodystep_size = self.angle_distance / (s_count + 1)

    @abc.abstractmethod
    def getBodyStepPose(self, step_num):
        pass

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
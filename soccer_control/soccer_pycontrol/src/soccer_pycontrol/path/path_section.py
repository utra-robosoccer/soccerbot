import abc
import functools
import math
from abc import ABC

import numpy as np
import rospy

from soccer_common.transformation import Transformation
from soccer_common.utils import wrapTo2Pi, wrapToPi


class PathSection(ABC):
    """
    Base class for a Path Section
    """

    def __init__(
        self, start_transform: Transformation, end_transform: Transformation, torso_step_length: float = 0.04, steps_per_second_default: float = 2.5
    ):
        """
        Initializes the Path Section

        :param start_transform: Start Transform
        :param end_transform: End Transform
        :param torso_step_length: Length of a torso step
        """

        #: How much distance is a torso step (equivalent to a half step)
        self.torso_step_length = torso_step_length  # rospy.get_param("torso_step_length", 0.04)

        #: How many torso steps per second, approximately equivalent to foot steps per second
        self.steps_per_second_default = steps_per_second_default  # rospy.get_param("steps_per_second_default", 2.5)

        self.start_transform: Transformation = start_transform
        self.end_transform: Transformation = end_transform

        self.steps_per_second = self.steps_per_second_default
        self.speed = self.steps_per_second * self.torso_step_length
        self.precision = 0.05 * self.torso_step_length

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
            self.distance = self.distance + Transformation.distance(prev_pose, new_pose)
            self.angle_distance = self.angle_distance + abs(wrapToPi(new_pose.orientation_euler[0] - prev_pose.orientation_euler[0]))
            prev_pose = new_pose

            self.distanceMap[j, 0:2] = [i, self.distance]
            j = j + 1

        # Adjusting body step size to account for extra distance
        if self.distance != 0:
            self.torso_step_length = self.distance / math.ceil(self.distance / self.torso_step_length)
            if self.torsoStepCount() <= 1:
                self.speed = self.steps_per_second * self.torso_step_length

    @abc.abstractmethod
    def poseAtRatio(self, r: float) -> Transformation:
        """
        Get the pose of the torso between the ratio

        :param r: float between [0, 1]
        :return: pose of the torso between the ratio
        """
        pass

    def linearStepCount(self) -> int:
        """
        How many steps in the forward and backwards direction

        :return: Number of steps in float, indicates half steps
        """
        return self.distance / self.torso_step_length

    def angularStepCount(self):
        """
        How many steps are made for rotating on spot

        :return: Number of steps in float, indicates half steps
        """
        return 0

    @abc.abstractmethod
    def getRatioFromStep(self, step_num: int):
        """
        Get the ratio of the Path Section
        :param step_num:
        :return: float ratio
        """
        pass

    def getBodyStepPose(self, step_num):
        ratio = self.getRatioFromStep(step_num)
        return self.poseAtRatio(ratio)

    @abc.abstractmethod
    def duration(self):
        """
        The total time of the path section

        :return: Duration in seconds
        """
        pass

    @abc.abstractmethod
    @functools.lru_cache
    def isWalkingBackwards(self):
        """
        Whether of not the robot is walking backwards

        :return: True if backwards, else False
        """

        pass

    @abc.abstractmethod
    def torsoStepCount(self):
        """
        Total number of torso steps

        :return: Torso steps
        """

        pass

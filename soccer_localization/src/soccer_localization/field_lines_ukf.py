import functools
import math
import os
from math import atan2, cos, nan, sin, sqrt, tan
from typing import Optional

import cv2
import numpy as np
import rospy
from field import *
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance
from nav_msgs.msg import OccupancyGrid
from scipy.linalg import block_diag

from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi


# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
class FieldLinesUKF:
    def __init__(self):
        points_fn = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2, kappa=0, subtract=self.residual_x)
        self.ukf = UKF(
            dim_x=3,
            dim_z=3 + 8,
            fx=self.move,
            hx=self.Hx,
            dt=0.01,
            points=points_fn,
            x_mean_fn=self.state_mean,
            z_mean_fn=self.z_mean,
            residual_x=self.residual_x,
            residual_z=self.residual_h,
        )

        self.ukf.x = np.array([-4, -3.15, 1.57])  # Initial state
        self.ukf.P = np.diag([0.0004, 0.0004, 0.002])  # Initial covariance (2cm, 2cm, 3 degrees)

        self.R_walking = np.diag([4, 2, 0.1])
        self.R_localizing = np.diag([0.9, 0.9, 0.1])
        self.R_ready = np.diag([0.1, 0.1, 0.1])

        self.R_goal_posts_localizing = np.diag([1e-4, 1e-4] * 4)  # Range, bearing, range, bearing, ...
        self.R_goal_posts_not_localizing = np.diag([1e4, 1e4] * 4)  # Range, bearing, range, bearing, ...

        self.ukf.R = block_diag(
            self.R_walking, self.R_goal_posts_not_localizing
        )  # Noise from measurement updates (x, y, theta), trust the y more than the x

        self.Q_walking = np.diag([9e-5, 9e-5, 5e-4])
        self.Q_localizing = np.diag([9e-5, 9e-5, 5e-4])
        self.Q_ready = np.diag([9e-5, 9e-5, 5e-4])
        self.ukf.Q = self.Q_walking  # Noise from navigation movements (2cm 2cm)

    def map_update(self, map: OccupancyGrid):
        self.map = map

    def move(self, x: [float], dt: float, u: [float]) -> [float]:
        pos = np.array([[cos(x[2]), -sin(x[2])], [sin(x[2]), cos(x[2])]]) @ np.array([u[0] * dt, u[1] * dt]) + np.array(x[0:2])
        return [pos[0], pos[1], x[2] + u[2] * dt]

    def residual_h(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        y[4] = wrapToPi(y[4])
        y[6] = wrapToPi(y[6])
        y[8] = wrapToPi(y[8])
        y[10] = wrapToPi(y[10])
        return y

    def residual_x(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    @functools.cached_property
    def goal_post_locations(self):
        A = 9
        D = 2.6
        goal_post_locations = [(A / 2, D / 2), (A / 2, -D / 2), (-A / 2, D / 2), (-A / 2, -D / 2)]
        return goal_post_locations

    def Hx(self, x):
        """
        takes a state variable and returns the measurement
        that would correspond to that state

        :param x: offset transform of robot from field
        :return: state of the robot, bearing and range of all the goal posts [x, y, theta, goal_post_1 dist, goal_post_1 angle, ...]

        # Dimensions given here https://cdn.robocup.org/hl/wp/2021/06/V-HL21_Rules_v4.pdf
        """

        hx = []
        for goal_post in self.goal_post_locations:
            dist = sqrt((goal_post[0] - x[0]) ** 2 + (goal_post[1] - x[1]) ** 2)
            angle = atan2(goal_post[1] - x[1], goal_post[0] - x[0])
            hx.extend([dist, wrapToPi(angle - x[2])])

        return np.concatenate((x, hx))

    def state_mean(self, sigmas, Wm):
        x = np.zeros(3)

        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
        x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
        x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
        x[2] = atan2(sum_sin, sum_cos)
        return x

    def z_mean(self, sigmas, Wm):
        z_count = sigmas.shape[1]
        x = np.zeros(z_count)
        x[0] = np.mean(sigmas[:, 0])
        x[1] = np.mean(sigmas[:, 1])
        x[2] = np.arctan2(np.sum(np.sin(sigmas[:, 2])) / len(sigmas), np.sum(np.cos(sigmas[:, 2])) / len(sigmas))

        x[3] = np.mean(sigmas[:, 3], axis=0)
        x[4] = np.arctan2(np.sum(np.sin(sigmas[:, 4])) / len(sigmas), np.sum(np.cos(sigmas[:, 4])) / len(sigmas))
        x[5] = np.mean(sigmas[:, 5], axis=0)
        x[6] = np.arctan2(np.sum(np.sin(sigmas[:, 6])) / len(sigmas), np.sum(np.cos(sigmas[:, 6])) / len(sigmas))
        x[7] = np.mean(sigmas[:, 7], axis=0)
        x[8] = np.arctan2(np.sum(np.sin(sigmas[:, 8])) / len(sigmas), np.sum(np.cos(sigmas[:, 8])) / len(sigmas))
        x[9] = np.mean(sigmas[:, 9], axis=0)
        x[10] = np.arctan2(np.sum(np.sin(sigmas[:, 10])) / len(sigmas), np.sum(np.cos(sigmas[:, 10])) / len(sigmas))
        return x

    def predict(self, u, dt):
        assert dt >= 0
        self.ukf.predict(dt=dt, u=u)
        assert not math.isnan(self.ukf.x[0])
        assert not np.any(np.diagonal(self.ukf.P) <= 0)

    def update(self, z, transform_confidence):
        assert not any((math.isnan(z[i]) for i in range(0, 3)))

        R = np.copy(self.ukf.R)
        R[0, 0] = R[0, 0] / max(0.001, transform_confidence[0] ** 2)
        R[1, 1] = R[1, 1] / max(0.001, transform_confidence[1] ** 2)
        R[2, 2] = R[2, 2] / max(0.001, transform_confidence[2] ** 2)

        R[3, 3] = R[4, 4] = R[5, 5] = R[6, 6] = R[7, 7] = R[8, 8] = R[9, 9] = R[10, 10] = 1e9
        z = np.concatenate((z, np.zeros(8)))
        # TODO fix this
        self.ukf.update(z, R)
        assert not math.isnan(self.ukf.x[0])
        assert not np.any(np.diagonal(self.ukf.P) <= 0)

    def update_goal_posts(self, z):
        R = np.copy(self.ukf.R)
        R[0, 0] = R[1, 1] = R[2, 2] = 1e9
        self.ukf.update(z, R)
        assert not math.isnan(self.ukf.x[0])
        assert not np.any(np.diagonal(self.ukf.P) <= 0)

    def draw_covariance(self):
        plot_covariance((self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=1, facecolor="k", alpha=0.1)

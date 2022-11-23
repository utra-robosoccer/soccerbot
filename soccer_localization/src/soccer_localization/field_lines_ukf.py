import math
import os
from math import atan2, cos, nan, sin, sqrt, tan
from typing import Optional

import cv2
import numpy as np
import rospy
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance
from nav_msgs.msg import OccupancyGrid

from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi


# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
class FieldLinesUKF:
    def __init__(self):
        points_fn = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2, kappa=0, subtract=self.residual_x)
        self.ukf = UKF(
            dim_x=3,
            dim_z=3,
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

        self.R_walking = np.diag([1, 1, 4e4])
        self.R_localizing = np.diag([1.5, 1.5, 0.25])
        self.ukf.R = self.R_walking  # Noise from measurement updates (x, y, theta), trust the y more than the x

        self.Q_walking = np.diag([1e-4, 1e-4, 1e-4])
        self.Q_localizing = np.diag([1e-18, 1e-18, 4e-4])
        self.Q_do_nothing = np.diag([1e-18, 1e-18, 1e-18])
        self.ukf.Q = self.Q_walking  # Noise from navigation movements (2cm 2cm)

    def map_update(self, map: OccupancyGrid):
        self.map = map

    def move(self, x: [float], dt: float, u: [float]) -> [float]:
        pos = np.array([[cos(x[2]), -sin(x[2])], [sin(x[2]), cos(x[2])]]) @ np.array([u[0] * dt, u[1] * dt]) + np.array(x[0:2])
        return [pos[0], pos[1], x[2] + u[2] * dt]

    def residual_h(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    def residual_x(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    def Hx(self, x):
        """
        takes a state variable and returns the measurement
        that would correspond to that state

        :param x: offset transform of robot from field
        :return: an array of distance and bearings in relation to the robot
        """

        return x

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
        return x

    def predict(self, u, dt):
        assert dt >= 0
        self.ukf.predict(dt=dt, u=u)
        assert not math.isnan(self.ukf.x[0])

    def update(self, z):
        assert not any((math.isnan(z[i]) for i in range(0, 3)))
        self.ukf.update(z)
        assert not math.isnan(self.ukf.x[0])

    def draw_covariance(self):
        plot_covariance((self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=1, facecolor="k", alpha=0.1)

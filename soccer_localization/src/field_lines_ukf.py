import os
from math import atan2, cos, sin, sqrt, tan

import cv2
import numpy as np
import rospy
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance
from nav_msgs.msg import OccupancyGrid

from soccer_common.utils import wrapToPi

dt = 0.5
wheelbase = 0.1
sigma_range = 0.05
sigma_bearing = 0.05


class FieldLinesUKF:
    def __init__(self, debug=True):
        self.debug = debug

        self.i = 0  # The current step of the UKF

        points = MerweScaledSigmaPoints(n=3, alpha=0.00001, beta=2, kappa=0, subtract=self.residual_x)
        self.ukf = UKF(
            dim_x=3,
            # dim_z=2 * len(landmarks),
            fx=self.move,
            hx=self.Hx,
            dt=dt,
            points=points,
            x_mean_fn=self.state_mean,
            z_mean_fn=self.z_mean,
            residual_x=self.residual_x,
            residual_z=self.residual_h,
        )

        self.ukf.x = np.array([-4, -3.15, 1.57])
        self.ukf.P = np.diag([0.05, 0.05, 0.05])
        # self.ukf.R = np.diag([sigma_range**2, sigma_bearing**2] * len(landmarks))
        self.ukf.Q = np.eye(3) * 0.0001

        # ROS information
        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.map_update)
        self.map: OccupancyGrid = None

    def map_update(self, map: OccupancyGrid):
        self.map = map

    def sample_landmarks(self):
        # TODO better landmark sampling

        src_path = os.path.dirname(os.path.realpath(__file__))
        map_path = src_path + "/../test/robocup.png"

        img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

        lm = np.column_stack(np.where(img == 0))

        lm = ((lm - np.array(img.shape) / 2) / np.array(img.shape)) * np.array([6 + 2, 9 + 2])
        return np.fliplr(lm[::10])

    def move(self, x: [float], dt: float, u: [float], wheelbase: float = 0.0001) -> [float]:
        # x = [x, y, theta]
        # u = [v, alpha] (alpha is the steering angle
        # Returns the next position
        theta = x[2]
        vel = u[0]
        steering_angle = u[1]
        dist = vel * dt

        if abs(steering_angle) > 0.001:  # is robot turning?
            beta = (dist / wheelbase) * tan(steering_angle)
            r = wheelbase / tan(steering_angle)  # radius

            sinh, sinhb = sin(theta), sin(theta + beta)
            cosh, coshb = cos(theta), cos(theta + beta)
            return x + np.array([-r * sinh + r * sinhb, r * cosh - r * coshb, beta])
        else:  # moving in straight line
            return x + np.array([dist * cos(theta), dist * sin(theta), 0])

    def residual_h(self, a, b):
        y = a - b
        # data in format [dist_1, bearing_1, dist_2, bearing_2,...]
        for i in range(0, len(y), 2):
            y[i + 1] = wrapToPi(y[i + 1])
        return y

    def residual_x(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    def Hx(self, x, landmarks):
        # x - robot location
        # landmarks - px and py fixed landmark coordinates
        # Returns an array of distance and bearings in relation to the robot

        """takes a state variable and returns the measurement
        that would correspond to that state."""
        hx = []
        for lmark in landmarks:
            px, py = lmark
            dist = sqrt((px - x[0]) ** 2 + (py - x[1]) ** 2)
            angle = atan2(py - x[1], px - x[0])
            hx.extend([dist, wrapToPi(angle - x[2])])
        return np.array(hx)

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

        for z in range(0, z_count, 2):
            sum_sin = np.sum(np.dot(np.sin(sigmas[:, z + 1]), Wm))
            sum_cos = np.sum(np.dot(np.cos(sigmas[:, z + 1]), Wm))

            x[z] = np.sum(np.dot(sigmas[:, z], Wm))
            x[z + 1] = atan2(sum_sin, sum_cos)
        return x

    def predict_and_update(self, u, z, landmarks):
        draw_covariance_interval = 10

        # Prediction and drop covariance
        self.ukf.predict(u=u, wheelbase=wheelbase)

        if self.debug and self.i % draw_covariance_interval == 0:
            plot_covariance((self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=3, facecolor="k", alpha=0.3)

        # Update and draw covariance
        self.ukf.update(z, landmarks=landmarks)

        if self.debug and self.i % draw_covariance_interval == 0:
            plot_covariance((self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=3, facecolor="g", alpha=0.8)

        self.i = self.i + 1

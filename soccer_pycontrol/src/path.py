from transformation import Transformation
import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt
import math

class Path:
    bodystep_size = 0.025 # Not absolutely fixed, will be modified slightly when
    speed = 3.0 * 0.025
    turn_duration = 4 # Number of body steps to turn
    step_size = 0.02 # Time for a single step

    pre_footstep_ratio = 0.15 # Ratio of fullstep duration to keep foot on ground on prefootstep
    post_footstep_ratio = 0.25 # Ratio of fullstep duration to keep foot on ground on postfootstep

    def __init__(self, start_transform=Transformation(), end_transform=Transformation()):
        self.start_transform = start_transform
        self.end_transform = end_transform

        # Compute approximate distance and distance map
        precision = 0.05 * self.bodystep_size
        precisions = np.linspace(precision, 1.0 , num=(int(1.0/precision) + 1)) #precision:precision: 1
        self.distance = 0.
        prev_pose = self.positionAtRatio(0)
        self.distanceMap = np.zeros((len(precisions) + 1, 2))
        self.distanceMap[0, 0:2] = [0., 0.]

        j = 1
        for i in precisions:
            new_pose = self.positionAtRatio(i)
            self.distance = self.distance + Transformation.get_distance(prev_pose, new_pose)
            prev_pose = new_pose

            self.distanceMap[j, 0:2] = [i, self.distance]
            j = j + 1

        # Round to nearest step
        s_count = self.bodyStepCount()
        if self.distance % self.bodystep_size < (self.bodystep_size / 2):
            self.bodystep_size = self.distance / s_count
        else:
            self.bodystep_size = self.distance / (s_count + 1)

    def getBodyStep(self, n):
        idx = np.argmin(np.abs((n * self.bodystep_size) - self.distanceMap[:, 1]))
        return self.positionAtRatio(self.distanceMap[idx, 0])

    def bodyStepCount(self):
        return int(np.floor(self.distance / self.bodystep_size))

    def duration(self):
        return self.distance / self.speed

    def bodyStepTime(self):
        return self.duration() / self.bodyStepCount()

    def positionAtRatio(self, t):
        pose = self.poseAtRatio(t)
        pose_del = self.poseAtRatio(t + 0.001)
        del_pose = pose_del.get_position() - pose.get_position()

        del_theta = np.arctan2(del_pose[1], del_pose[0])
        del_psi = np.arctan2(del_pose[2], np.linalg.norm(del_pose[0:2]))

        orientation = Transformation.get_quaternion_from_euler([del_theta, -del_psi, 0.])
        pose.set_orientation(orientation)
        return pose

    def poseAtRatio(self, t):
        p1 = self.start_transform
        p2 = np.matmul(self.start_transform, Transformation([self.speed * self.turn_duration, 0., 0.]))
        p3 = np.matmul(self.end_transform, Transformation([-self.speed * self.turn_duration, 0., 0.]))
        p4 = self.end_transform

        p1_pos = p1.get_position()
        p2_pos = p2.get_position()
        p3_pos = p3.get_position()
        p4_pos = p4.get_position()

        position = np.array([0.0, 0.0, 0.0])
        for d in range(0, 3):
            bez_param = [p1_pos[d], p2_pos[d], p3_pos[d], p4_pos[d]]

            # Cubic bezier
            for i in range(0,4):
                position[d] = position[d] + bez_param[i] * comb(3, i, exact=True, repetition=False) * ((1 - t) ** (3 - i)) * (t ** i)
        return Transformation(position)

    def show(self, fig=None):
        position = np.zeros((self.bodyStepCount() + 1, 3))
        orientation = np.zeros((self.bodyStepCount() + 1, 3))
        for i in range(0,self.bodyStepCount()+1,1):                   # i = 0:1: obj.bodyStepCount
            step = self.getBodyStep(i)
            position[i, 0:3] = step.get_position()
            orientation[i, 0:3] = np.matmul(step[0:3, 0:3], np.reshape(np.array([0.005, 0., 0.]), (3, 1)))[:,0]
        if fig is None:
            fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_autoscale_on(True)
        ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1], orientation[:, 2])#, AutoScaleFactor=0.1)

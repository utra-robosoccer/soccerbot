import functools

from transformation import Transformation
import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt
from copy import deepcopy

class Path:
    bodystep_size = 0.040 # m Not absolutely fixed, will be modified slightly when
    angular_bodystep_size = 0.4 # radians Radians per angular step
    steps_per_second = 2.4
    speed = steps_per_second * bodystep_size # m/s
    angular_speed = steps_per_second * angular_bodystep_size # Rotational speed in radians per second
    turn_duration = 8 # Number of body steps to turn
    step_size = 0.02 # Time for a single time step

    pre_footstep_ratio = 0.15 # Ratio of fullstep duration to keep foot on ground on prefootstep
    post_footstep_ratio = 0.25 # Ratio of fullstep duration to keep foot on ground on postfootstep

    def __init__(self, start_transform=Transformation(), end_transform=Transformation()):
        self.start_transform = start_transform
        self.end_transform = end_transform

        # Compute approximate distance and distance map
        precision = 0.05 * self.bodystep_size
        precisions = np.linspace(precision, 1.0 , num=(int(1.0/precision) + 1))
        self.distance = 0
        self.angle_distance = 0
        prev_pose = self.poseAtRatio(0)
        self.distanceMap = np.zeros((len(precisions) + 1, 2))
        self.distanceMap[0, 0:2] = [0., 0.]

        j = 1
        for i in precisions:
            new_pose = self.poseAtRatio(i)
            self.distance = self.distance + Transformation.get_distance(prev_pose, new_pose)
            self.angle_distance = self.angle_distance + abs(new_pose.get_orientation_euler()[0] - prev_pose.get_orientation_euler()[0])
            prev_pose = new_pose

            self.distanceMap[j, 0:2] = [i, self.distance]
            j = j + 1

        # Round to nearest step
        s_count = self.linearStepCount()
        if self.distance != 0:
            if self.distance % self.bodystep_size < (self.bodystep_size / 2):
                self.bodystep_size = self.distance / s_count
            else:
                self.bodystep_size = self.distance / (s_count + 1)

        s_count = self.angularStepCount()
        if self.angle_distance != 0:
            if self.angle_distance != 0 and self.angle_distance % self.angular_bodystep_size < (self.angular_bodystep_size / 2):
                self.angular_bodystep_size = self.angle_distance / s_count
            else:
                self.angular_bodystep_size = self.angle_distance / (s_count + 1)

    def getBodyStep(self, n):
        if self.isRotateInPlace():
            diff_position = self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]
            start_angle = self.start_transform.get_orientation_euler()[0]
            intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
            final_angle = self.end_transform.get_orientation_euler()[0]

            # TODO make sure the rotate in place is correct
            step_1_angular_distance = abs(intermediate_angle - start_angle)
            step_2_distance = np.linalg.norm(diff_position)
            step_3_angular_distance = abs(intermediate_angle - final_angle)

            step_1_steps = step_1_angular_distance / self.angular_bodystep_size
            step_2_steps = step_2_distance / self.bodystep_size
            step_3_steps = step_3_angular_distance / self.angular_bodystep_size
            if step_1_steps + step_2_steps + step_3_steps == 0:
                ratio = 0
            else:
                ratio = n / (step_1_steps + step_2_steps + step_3_steps)
            return self.poseAtRatio(ratio)
        else:
            idx = np.argmin(np.abs((n * self.bodystep_size) - self.distanceMap[:, 1]))
            return self.poseAtRatio(self.distanceMap[idx, 0])

    def linearStepCount(self):
        return int(np.floor(self.distance / self.bodystep_size))

    def angularStepCount(self):
        return int(np.floor(self.angle_distance / self.angular_bodystep_size))

    def bodyStepCount(self):
        return self.linearStepCount() + self.angularStepCount()

    def duration(self):
        if self.isRotateInPlace():
            return self.distance / self.speed + self.angle_distance / self.angular_speed
        return self.distance / self.speed

    def bodyStepTime(self):
        return self.duration() / self.bodyStepCount()

    @functools.lru_cache
    def isWalkingBackwards(self):
        start_angle = self.start_transform.get_orientation_euler()[0]
        del_pose = self.end_transform.get_position() - self.start_transform.get_position()
        if np.dot([np.cos(start_angle), np.sin(start_angle)], del_pose[0:2]) < 0:
            return True
        return False

    # If the path is short, rotate in place, go straight and then rotate in place instead
    @functools.lru_cache
    def isRotateInPlace(self):
        return np.linalg.norm(self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]) < self.bodystep_size * self.turn_duration * 2

    def poseAtRatio(self, r):

        if self.isRotateInPlace():
            return self.poseAtRatioRotateInPlace(r)
        else:
            pose = self.bezierPositionAtRatio(r)
            pose_del = self.bezierPositionAtRatio(r + 0.001)

            if self.isWalkingBackwards():
                del_pose = pose.get_position() - pose_del.get_position()
            else:
                del_pose = pose_del.get_position() - pose.get_position()

            # If walking backwards
            del_theta = np.arctan2(del_pose[1], del_pose[0])
            del_psi = np.arctan2(del_pose[2], np.linalg.norm(del_pose[0:2]))

            orientation = Transformation.get_quaternion_from_euler([del_theta, -del_psi, 0.])
            pose.set_orientation(orientation)
        return pose

    def poseAtRatioRotateInPlace(self, r):
        diff_position = self.end_transform.get_position()[0:2] - self.start_transform.get_position()[0:2]
        start_angle = self.start_transform.get_orientation_euler()[0]
        intermediate_angle = np.arctan2(diff_position[1], diff_position[0])
        final_angle = self.end_transform.get_orientation_euler()[0]

        # TODO make sure the rotate in place is correct
        step_1_duration = abs(intermediate_angle - start_angle) / self.angular_speed
        step_2_duration = np.linalg.norm(diff_position) / self.speed
        step_3_duration = abs(intermediate_angle - final_angle) / self.angular_speed

        total_duration = step_1_duration + step_2_duration + step_3_duration
        t = r * total_duration

        if t == 0:
            pose = deepcopy(self.start_transform)
            return pose
        elif t < step_1_duration != 0:
            pose = deepcopy(self.start_transform)
            percentage = t / step_1_duration
            angle = (intermediate_angle - start_angle) * percentage
            pose.set_orientation(Transformation.get_quaternion_from_euler([angle, 0, 0]))
            return pose
        elif step_1_duration < t <= step_1_duration + step_2_duration != 0:
            pose = deepcopy(self.start_transform)
            percentage = (t - step_1_duration) / step_2_duration
            position = diff_position * percentage + self.start_transform.get_position()[0:2]
            pose.set_position(np.concatenate((position, [pose.get_position()[2]])))
            pose.set_orientation(Transformation.get_quaternion_from_euler([intermediate_angle, 0, 0]))
            return pose
        elif step_1_duration + step_2_duration < t <= step_1_duration + step_2_duration + step_3_duration != 0:
            pose = deepcopy(self.end_transform)
            percentage = (t - step_1_duration - step_2_duration) / step_3_duration
            angle = (final_angle - intermediate_angle) * percentage
            pose.set_orientation(Transformation.get_quaternion_from_euler([angle, 0, 0]))
            return pose
        else:
            pose = deepcopy(self.end_transform)
            return pose


    def bezierPositionAtRatio(self, r):
        # If the distance is small, use turn in place strategy, otherwise cubic bezier

        p1 = self.start_transform
        if self.isWalkingBackwards():
            p2 = np.matmul(self.start_transform, Transformation([- self.speed * self.turn_duration, 0., 0.]))
            p3 = np.matmul(self.end_transform, Transformation([self.speed * self.turn_duration, 0., 0.]))
        else:
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
                position[d] = position[d] + bez_param[i] * comb(3, i, exact=True, repetition=False) * ((1 - r) ** (3 - i)) * (r ** i)
        return Transformation(position)

    def show(self):
        position = np.zeros((self.bodyStepCount() + 1, 3))
        orientation = np.zeros((self.bodyStepCount() + 1, 3))
        for i in range(0,self.bodyStepCount()+1,1):                   # i = 0:1: obj.bodyStepCount
            step = self.getBodyStep(i)
            position[i, 0:3] = step.get_position()
            orientation[i, 0:3] = np.matmul(step[0:3, 0:3], np.reshape(np.array([0.015, 0., 0.]), (3, 1)))[:,0]

        ax = plt.gca(projection='3d')
        ax.set_autoscale_on(True)
        ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1], orientation[:, 2])
        # ax.set_xlim(-0.2,1)
        # ax.set_ylim(-0.3,0.3)
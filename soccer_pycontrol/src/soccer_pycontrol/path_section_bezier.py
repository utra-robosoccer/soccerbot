import functools
from soccer_common.transformation import Transformation
import numpy as np
from scipy.special import comb
from soccer_pycontrol.path_section import PathSection

class PathSectionBezier(PathSection):
    turn_duration = 5  # Number of body steps to turn

    def poseAtRatio(self, r):
        pose = self.bezierPositionAtRatio(self.start_transform, self.end_transform, r)
        pose_del = self.bezierPositionAtRatio(self.start_transform, self.end_transform, r + 0.001)

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

    def bezierPositionAtRatio(self, start_transform, end_transform, r):
        # If the distance is small, use turn in place strategy, otherwise cubic bezier

        p1 = start_transform
        if self.isWalkingBackwards():
            p2 = np.matmul(start_transform, Transformation([- PathSection.speed * PathSectionBezier.turn_duration, 0., 0.]))
            p3 = np.matmul(end_transform, Transformation([PathSection.speed * PathSectionBezier.turn_duration, 0., 0.]))
        else:
            p2 = np.matmul(start_transform, Transformation([PathSection.speed * PathSectionBezier.turn_duration, 0., 0.]))
            p3 = np.matmul(end_transform, Transformation([-PathSection.speed * PathSectionBezier.turn_duration, 0., 0.]))
        p4 = end_transform

        p1_pos = p1.get_position()
        p2_pos = p2.get_position()
        p3_pos = p3.get_position()
        p4_pos = p4.get_position()

        position = np.array([0.0, 0.0, 0.0])
        for d in range(0, 3):
            bez_param = [p1_pos[d], p2_pos[d], p3_pos[d], p4_pos[d]]

            # Cubic bezier
            for i in range(0, 4):
                position[d] = position[d] + bez_param[i] * comb(3, i, exact=True, repetition=False) * (
                            (1 - r) ** (3 - i)) * (r ** i)
        return Transformation(position)

    def getRatioFromStep(self, step_num):
        idx = np.argmin(np.abs((step_num * self.bodystep_size) - self.distanceMap[:, 1]))
        return self.distanceMap[idx, 0]

    def duration(self):
        return self.distance / PathSection.speed

    @functools.lru_cache
    def isWalkingBackwards(self):
        start_angle = self.start_transform.get_orientation_euler()[0]
        del_pose = self.end_transform.get_position() - self.start_transform.get_position()
        if np.dot([np.cos(start_angle), np.sin(start_angle)], del_pose[0:2]) < 0:
            return True
        return False

    def bodyStepCount(self):
        return self.linearStepCount()

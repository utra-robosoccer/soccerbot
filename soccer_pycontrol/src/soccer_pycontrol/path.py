from soccer_pycontrol.transformation import Transformation
import numpy as np
import matplotlib.pyplot as plt
from path_section import PathSection
from path_section_bezier import PathSectionBezier
from path_section_short import PathSectionShort

class Path:
    bodystep_size = 0.04 #try 0.05  # m Not absolutely fixed, will be modified slightly when
    angular_bodystep_size = 0.4  # radians Radians per angular step
    steps_per_second = 2.4 # try 6 motors P = 09.25
    speed = steps_per_second * bodystep_size  # m/s
    angular_speed = steps_per_second * angular_bodystep_size  # Rotational speed in radians per second
    turn_duration = 4  # Number of body steps to turn
    step_size = 0.02  # Time for a single time step

    pre_footstep_ratio = 0.15  # Ratio of fullstep duration to keep foot on ground on prefootstep
    post_footstep_ratio = 0.25  # Ratio of fullstep duration to keep foot on ground on postfootstep
    precision = 0.05 * bodystep_size

    def __init__(self, start_transform: Transformation,  end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform

        self.path_sections = []

        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

    def createPathSection(self, start_transform: Transformation, end_transform: Transformation):
        is_short_distance = np.linalg.norm(start_transform[0:2] - end_transform[0:2]) < Path.bodystep_size * Path.turn_duration * 3
        if is_short_distance:
            return PathSectionShort(start_transform, end_transform)
        else:
            return PathSectionBezier(start_transform, end_transform)

    def linearStepCount(self):
        linearStepCount = 0
        for path_section in self.path_sections:
            linearStepCount += path_section.angularStepCount()
        return linearStepCount

    def angularStepCount(self):
        angularStepCount = 0
        for path_section in self.path_sections:
            angularStepCount += path_section.angularStepCount()
        return angularStepCount

    def bodyStepCount(self):
        bodyStepCount = 0
        for path_section in self.path_sections:
            bodyStepCount += path_section.bodyStepCount()
        return bodyStepCount

    def duration(self):
        duration = 0
        for path_section in self.path_sections:
            duration += path_section.duration()
        return duration

    # Do not use in the walking engine
    def estimatedPositionAtTime(self, t):
        estimated_ratio = t / self.duration()
        return self.poseAtRatio(estimated_ratio)

    def isFinished(self, t):
        return t >= self.duration()

    def bodyStepTime(self):
        return self.duration() / self.bodyStepCount()

    # Return the subpath and the corresponding ratio
    def getSubPathSectionAndRatio(self, r: float):
        total_duration = self.duration()

        cumulative_ratio = 0
        for path_section in self.path_sections:
            ratio = path_section.duration() / total_duration
            next_cumulative_ratio = cumulative_ratio + ratio
            if next_cumulative_ratio > r:
                return (r - cumulative_ratio)/ratio, path_section
            cumulative_ratio = next_cumulative_ratio

        raise Exception("Invalid ratio " + str(r))

    def poseAtRatio(self, r: float):
        ratio, path_section = self.getSubPathSectionAndRatio(r)
        return path_section.poseAtRatio(ratio)

    # Get estimated path ratio from the current time plus one more step, and then find the distance of that ratio and set the distance
    def terminateWalk(self, t):
        estimated_ratio = (t + 2 / self.steps_per_second) / self.duration()
        estimated_sub_ratio, selected_path_section = self.getSubPathSectionAndRatio(estimated_ratio)

        for i in range(len(selected_path_section.distanceMap)):
            if selected_path_section.distanceMap[i, 0] > estimated_sub_ratio:
                selected_path_section.distance = min(selected_path_section.distance, selected_path_section.distanceMap[i, 1])
                break

        # Remove any future subsections
        reached = False
        for path_section in self.path_sections:
            if path_section == selected_path_section:
                reached = True
                continue
            if reached:
                self.path_sections.remove(path_section)


    def show(self):
        position = np.zeros((self.bodyStepCount() + len(self.path_sections), 3))
        orientation = np.zeros((self.bodyStepCount() + len(self.path_sections), 3))
        for path_section in self.path_sections:
            for i in range(0, path_section.bodyStepCount() + 1, 1):  # i = 0:1: obj.bodyStepCount
                step = path_section.getBodyStepPose(i)
                position[i, 0:3] = step.get_position()
                orientation[i, 0:3] = np.matmul(step[0:3, 0:3], np.reshape(np.array([0.015, 0., 0.]), (3, 1)))[:, 0]

        ax = plt.gca(projection='3d')
        ax.set_autoscale_on(True)
        ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1],
                  orientation[:, 2])
        # ax.set_xlim(-0.2,1)
        # ax.set_ylim(-0.3,0.3)

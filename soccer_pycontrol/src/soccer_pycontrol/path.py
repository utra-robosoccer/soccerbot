from math import floor

from soccer_pycontrol.transformation import Transformation
import numpy as np
import matplotlib.pyplot as plt
from path_section import PathSection
from path_section_bezier import PathSectionBezier
from path_section_short import PathSectionShort

class Path:
    steps_per_second = PathSectionBezier.steps_per_second
    step_size = 0.02  # Time for a single time step

    pre_footstep_ratio = 0.15  # Ratio of fullstep duration to keep foot on ground on prefootstep
    post_footstep_ratio = 0.25  # Ratio of fullstep duration to keep foot on ground on postfootstep

    def __init__(self, start_transform: Transformation,  end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform

        self.path_sections = []

        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

    def createPathSection(self, start_transform: Transformation, end_transform: Transformation):
        is_short_distance = np.linalg.norm(start_transform[0:2] - end_transform[0:2]) < PathSection.bodystep_size * PathSectionBezier.turn_duration * 3
        if is_short_distance:
            print("Creating Short Path")
            print("Start Transform")
            print(start_transform)
            print("End Transform")
            print(end_transform)
            return PathSectionShort(start_transform, end_transform)
        else:
            print("Creating Bezier Path")
            print("Start Transform")
            print(start_transform)
            print("End Transform")
            print(end_transform)
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
        return floor(bodyStepCount)

    def getBodyStepPose(self, step_num):
        count = step_num
        for path_section in self.path_sections:
            if count <= path_section.bodyStepCount():
                return path_section.getBodyStepPose(count)
            count = count - path_section.bodyStepCount()

        # Error print stuff
        for path_section in self.path_sections:
            print(path_section.bodyStepCount())
        raise Exception("Invalid body step calculation " + str(count))

    def duration(self):
        duration = 0
        for path_section in self.path_sections:
            duration += path_section.duration()
        return duration

    # Do not use in the walking engine
    def estimatedPositionAtTime(self, t):
        estimated_ratio = min(t / self.duration(), 1)
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
            if next_cumulative_ratio >= r:
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

    def dynamicallyUpdateGoalPosition(self, t, end_transform):
        start_transform = self.estimatedPositionAtTime(t + 2 / self.steps_per_second)
        self.terminateWalk(t)
        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

    def show(self):
        position = np.zeros((self.bodyStepCount(), 3))
        orientation = np.zeros((self.bodyStepCount(), 3))
        for i in range(0, self.bodyStepCount(), 1):  # i = 0:1: obj.bodyStepCount
            step = self.getBodyStepPose(i)
            position[i, 0:3] = step.get_position()
            orientation[i, 0:3] = np.matmul(step[0:3, 0:3], np.reshape(np.array([0.015, 0., 0.]), (3, 1)))[:, 0]

        ax = plt.gca(projection='3d')
        ax.set_autoscale_on(True)
        ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1],
                  orientation[:, 2])
        ax.set_zlim(0, 0.4)
        print("Done")
        # Show subpaths
        # position = np.zeros((self.bodyStepCount() + len(self.path_sections), 3))
        # orientation = np.zeros((self.bodyStepCount() + len(self.path_sections), 3))
        # colors = np.zeros((self.bodyStepCount() + len(self.path_sections), 3))
        # j = 0
        # for path_section in self.path_sections:
        #     color = np.random.rand(3)
        #     for i in range(0, path_section.bodyStepCount() + 1, 1):  # i = 0:1: obj.bodyStepCount
        #         step = path_section.getBodyStepPose(i)
        #         position[j, 0:3] = step.get_position()
        #         orientation[j, 0:3] = np.matmul(step[0:3, 0:3], np.reshape(np.array([0.015, 0., 0.]), (3, 1)))[:, 0]
        #         colors[j] = color
        #         j = j + 1
        #
        # ax = plt.gca(projection='3d')
        # ax.set_autoscale_on(True)
        # ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1],
        #           orientation[:, 2], colors=colors)
        # print("hi")
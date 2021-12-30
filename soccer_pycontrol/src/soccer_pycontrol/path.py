from math import floor, ceil

from soccer_geometry import Transformation
import numpy as np
import matplotlib.pyplot as plt
from path_section import PathSection
from path_section_bezier import PathSectionBezier
from path_section_short import PathSectionShort
import time

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
        # is_short_distance = np.linalg.norm(start_transform[0:2] - end_transform[0:2]) < PathSection.bodystep_size * PathSectionBezier.turn_duration * 3
        is_short_distance = False
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

    def getTimePathOfNextStepOld(self, t):
        def getBodyStepTime(step_num):
            count = step_num
            time = 0
            for path_section in self.path_sections:
                if count <= path_section.bodyStepCount():
                    ratio = path_section.getRatioFromStep(count)
                    time = time + ratio * path_section.duration()
                    distance = path_section.bodystep_size * count
                    return time, ratio, distance, path_section
                count = count - path_section.bodyStepCount()
                time = time + path_section.duration()
            raise Exception("Invalid body step calculation " + str(count))

        for i in range(0, self.bodyStepCount(), 1):
            time, ratio, path_distance, path_section = getBodyStepTime(i)
            if t < time:
                return time, ratio, path_distance, path_section, i

    def getTimePathOfNextStep(self, t):
        ratio = t / self.duration()
        ratio, path_section = self.getSubPathSectionAndRatio(ratio)

        # Get next bodystep
        bodystep_count = 0
        for path in self.path_sections:
            if path != path_section:
                bodystep_count = bodystep_count + path_section.bodyStepCount()
            else:
                break
        subpath_count = (path_section.distance * ratio) / path_section.bodystep_size
        bodystep_count += subpath_count
        bodystep_count = ceil(bodystep_count)

        # Get information about this body step
        def getBodyStepTime(step_num):
            count = step_num
            time = 0
            for path_section in self.path_sections:
                if count <= path_section.bodyStepCount():
                    ratio = path_section.getRatioFromStep(count)
                    time = time + ratio * path_section.duration()
                    distance = path_section.bodystep_size * count
                    return time, ratio, distance, path_section
                count = count - path_section.bodyStepCount()
                time = time + path_section.duration()
            raise Exception("Invalid body step calculation " + str(count))

        time, ratio, distance, path_section = getBodyStepTime(bodystep_count)
        return time, ratio, distance, path_section, bodystep_count

    # Get estimated path ratio from the current time plus one more step, and then find the distance of that ratio and set the distance
    def terminateWalk(self, t):
        time, ratio, path_distance, path_section, step = self.getTimePathOfNextStep(t)
        path_section.distance = path_distance

        # Remove any future subsections
        reached = False
        for p in self.path_sections:
            if p == path_section:
                reached = True
                continue
            if reached:
                self.path_sections.remove(p)

    def dynamicallyUpdateGoalPosition(self, t, end_transform):

        t_new, ratio, path_distance, path_section, step = self.getTimePathOfNextStep(t + 2)
        start_transform = self.getBodyStepPose(step)
        self.terminateWalk(t_new)
        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

        return t_new

    def show(self):
        position = np.zeros((self.bodyStepCount(), 3))
        orientation = np.zeros((self.bodyStepCount(), 3))
        colors = np.zeros((self.bodyStepCount(), 4))
        colors_arrow_ends = np.zeros((self.bodyStepCount()*2, 4))

        section_color_map = {}
        for i in range(0, len(self.path_sections)):
            section_color_map[i] = np.append(np.random.rand(3), 1)


        for i in range(0, self.bodyStepCount(), 1):  # i = 0:1: obj.bodyStepCount
            step = self.getBodyStepPose(i)
            position[i, 0:3] = step.get_position()
            orientation[i, 0:3] = (step[0:3, 0:3] @ np.reshape(np.array([0.015, 0., 0.]), (3, 1)))[:, 0]

            count = i
            section = 0
            for path_section in self.path_sections:
                if count <= path_section.bodyStepCount():
                    break
                count = count - path_section.bodyStepCount()
                section = section + 1
            colors[i] = section_color_map[section]
            colors_arrow_ends[2*i] = section_color_map[section]
            colors_arrow_ends[2*i+1] = section_color_map[section]

        ax = plt.gca(projection='3d')
        ax.set_autoscale_on(True)
        colors = np.concatenate((colors, colors_arrow_ends))
        ax.quiver(position[:, 0], position[:, 1], position[:, 2], orientation[:, 0], orientation[:, 1],
                  orientation[:, 2], colors=colors)
        ax.set_zlim(0, 0.4)
        return ax
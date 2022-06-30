import math
from math import ceil, floor

import matplotlib.pyplot as plt
import numpy as np
import rospy

from soccer_common import Transformation
from soccer_pycontrol.path_section import PathSection
from soccer_pycontrol.path_section_bezier import PathSectionBezier
from soccer_pycontrol.path_section_short import PathSectionShort
from soccer_pycontrol.utils import wrapToPi


class Path:
    step_precision = rospy.get_param("step_precision", 0.02)  # Time for a single time step

    pre_footstep_ratio = rospy.get_param("pre_footstep_ratio", 0.15)  # Ratio of fullstep duration to keep foot on ground on prefootstep
    post_footstep_ratio = rospy.get_param("post_footstep_ratio", 0.25)  # Ratio of fullstep duration to keep foot on ground on postfootstep

    def __init__(self, start_transform: Transformation, end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform

        self.path_sections = []

        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

    def isShortPath(self, start_transform: Transformation, end_transform: Transformation):
        theta_diff = wrapToPi(end_transform.get_orientation_euler()[0] - start_transform.get_orientation_euler()[0])
        pos_theta_diff = math.atan2(
            end_transform.get_position()[1] - start_transform.get_position()[1],
            end_transform.get_position()[0] - start_transform.get_position()[0],
        )
        if abs(wrapToPi(pos_theta_diff - theta_diff)) > math.pi / 4:
            return True

        return (
            np.linalg.norm(end_transform.get_position()[0:2] - start_transform.get_position()[0:2])
            < PathSection.bodystep_size_default * PathSectionBezier.turn_duration * 4
        )

    def createPathSection(self, start_transform: Transformation, end_transform: Transformation):
        is_short_distance = self.isShortPath(start_transform, end_transform)
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
            if total_duration == 0:
                return 0, path_section
            ratio = path_section.duration() / total_duration
            next_cumulative_ratio = cumulative_ratio + ratio
            if next_cumulative_ratio >= r:
                return (r - cumulative_ratio) / ratio, path_section
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
                bodystep_count = bodystep_count + path.bodyStepCount()
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
        t_change = t + 1
        if len(self.path_sections) >= 1 and self.path_sections[-1] is PathSectionShort:
            raise Exception("If the last path is a Short paths, it cannot be further modified")
        if self.duration() - t_change < 1:
            raise Exception("There is not enough time to update the position, current time { t } , duration of current path { self.duration()}")

        t_new, ratio, path_distance, path_section, step = self.getTimePathOfNextStep(t_change)
        start_transform = self.getBodyStepPose(step)

        # TODO allow bezier paths to go to short paths and vice versa
        if self.isShortPath(start_transform, end_transform):
            raise Exception("Cannot append a short path to a bezier path")

        self.terminateWalk(t_new)
        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

        return t_new

    def show(self):
        position = np.zeros((self.bodyStepCount(), 3))
        orientation = np.zeros((self.bodyStepCount(), 3))
        colors = np.zeros((self.bodyStepCount(), 4))
        colors_arrow_ends = np.zeros((self.bodyStepCount() * 2, 4))

        section_color_map = {}
        for i in range(0, len(self.path_sections)):
            section_color_map[i] = np.append(np.random.rand(3), 1)

        for i in range(0, self.bodyStepCount(), 1):  # i = 0:1: obj.bodyStepCount
            step = self.getBodyStepPose(i)
            position[i, 0:3] = step.get_position()
            orientation[i, 0:3] = (step[0:3, 0:3] @ np.reshape(np.array([0.015, 0.0, 0.0]), (3, 1)))[:, 0]

            count = i
            section = 0
            for path_section in self.path_sections:
                if count <= path_section.bodyStepCount():
                    break
                count = count - path_section.bodyStepCount()
                section = section + 1
            colors[i] = section_color_map[section]
            colors_arrow_ends[2 * i] = section_color_map[section]
            colors_arrow_ends[2 * i + 1] = section_color_map[section]

        ax = plt.gca(projection="3d")
        ax.set_autoscale_on(True)
        colors = np.concatenate((colors, colors_arrow_ends))
        ax.quiver(
            position[:, 0],
            position[:, 1],
            position[:, 2],
            orientation[:, 0],
            orientation[:, 1],
            orientation[:, 2],
            colors=colors,
        )
        ax.set_zlim(0, 0.4)

        # Equalize X and Y axes
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        xlen = xlim[1] - xlim[0]
        ylen = ylim[1] - ylim[0]
        if xlen > ylen:
            ymid = 0.5 * (ylim[0] + ylim[1])
            ylim_new = [ymid - xlen / 2, ymid + xlen / 2]
            ax.set_ylim(ylim_new)
        else:
            xmid = 0.5 * (xlim[0] + xlim[1])
            xlim_new = [xmid - ylen / 2, xmid + ylen / 2]
            ax.set_xlim(xlim_new)

        ax.view_init(90, 0)
        return ax

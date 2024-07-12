import functools
import math
from math import ceil, floor

import matplotlib.pyplot as plt
import numpy as np
import scipy
from soccer_pycontrol.path.path_section import PathSection
from soccer_pycontrol.path.path_section_bezier import PathSectionBezier
from soccer_pycontrol.path.path_section_short import PathSectionShort

from soccer_common import Transformation
from soccer_common.utils import wrapToPi


class Path:
    """
    Path of the robot, imagine a line underneath the torso on the ground between the two feet
    Consists of a list of bezier or short path sections
    """

    # with open(root_dir + "/config/topics.yaml", "r") as file:
    #     configuration = yaml.safe_load(file)
    #     topics = list(configuration["topics"].values())
    #     command = f"rosbag record -O {name} "
    #     command += " ".join(topics)

    def __init__(self, start_transform: Transformation, end_transform: Transformation, step_precision: float = 0.02):
        """
        Initialization function for Path, creates a single path section, other path sections are only added when the route needs
        to change

        :param start_transform: Starting Robot Position
        :param end_transform: Ending Robot Position
        """

        #: How precise the curves are calculated. The amount of movement per given step_precision is calculated (s)
        self.step_precision = step_precision  # rospy.get_param("step_precision", 0.02)
        # TODO only used for plotting
        self.start_transform: Transformation = start_transform
        self.end_transform: Transformation = end_transform

        self.path_sections = []  #: A list of Path Sections used to define the path

        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

    @functools.cached_property
    def start_transformed_inv(self):  # TODO is this needed
        return scipy.linalg.inv(self.start_transform)

    def isShortPath(self, start_transform: Transformation, end_transform: Transformation):
        """
        Determine whether the movement can be done with a short path of another type of path (bezier). Bezier is used only
        on simple forward paths, the rest use ShortPath

        :param start_transform: The start transform
        :param end_transform: The end transform
        :return:
        """
        # TODO why were these conditions choosen
        # If there is too much final rotation
        theta_diff = wrapToPi(end_transform.orientation_euler[0] - start_transform.orientation_euler[0])
        pos_theta_diff = math.atan2(
            end_transform.position[1] - start_transform.position[1],
            end_transform.position[0] - start_transform.position[0],
        )
        if abs(wrapToPi(pos_theta_diff - theta_diff)) > math.pi / 4:
            return True

        # If there is too much initial rotation
        start_end_diff = end_transform @ scipy.linalg.inv(start_transform)
        start_end_angle = math.atan2(start_end_diff[1, 3], start_end_diff[0, 3])
        if abs(start_end_angle) > math.pi / 4:
            return True

        # If the distance is too close
        return np.linalg.norm(end_transform.position[0:2] - start_transform.position[0:2]) < 1

    def createPathSection(self, start_transform: Transformation, end_transform: Transformation) -> PathSection:
        """
        Create a path section between the two transforms

        :param start_transform: Starting robot position
        :param end_transform: Ending robot position
        :return: Depending on whether its a short path, a bezier path or a short path
        """
        is_short_distance = self.isShortPath(start_transform, end_transform)
        if is_short_distance:
            return PathSectionShort(start_transform, end_transform)
        else:
            return PathSectionBezier(start_transform, end_transform)

    @functools.lru_cache
    def linearStepCount(self) -> float:
        """
        How many footsteps used for moving forward and backwards the robot takes to run this path

        :return: The number of steps
        """

        linearStepCount = 0
        for path_section in self.path_sections:
            linearStepCount += path_section.linearStepCount()
        return linearStepCount

    @functools.lru_cache
    def angularStepCount(self):
        """
        How many footsteps used for turning the robot takes to run this path

        :return: The number of steps
        """
        angularStepCount = 0
        for path_section in self.path_sections:
            angularStepCount += path_section.angularStepCount()
        return angularStepCount

    @functools.lru_cache
    def torsoStepCount(self) -> int:
        """
        Number of torso steps, a torso step is the amount of steps the body moves (similar to foot steps, but doesn't include
        turning in place.

        :return:
        """
        torsoStepCount = 0
        for path_section in self.path_sections:
            torsoStepCount += path_section.torsoStepCount()
        return floor(torsoStepCount)

    @functools.lru_cache
    def getTorsoStepPose(self, step_num) -> Transformation:
        """
        Get the torso position given a certain torso step

        :param step_num: The body step of the path
        :return: A position of the robot's torso at the spot
        """
        # TODO why is this dynamic and not built into an array of all steps
        count = step_num
        for path_section in self.path_sections:
            if count <= path_section.torsoStepCount():
                return path_section.getBodyStepPose(count)
            count = count - path_section.torsoStepCount()
        raise Exception(f"Invalid body step calculation {count}/{self.torsoStepCount()}")

    @functools.lru_cache
    def duration(self) -> float:
        """
        Get the duration in seconds for a given path

        :return: Duration of the paths in seconds
        """
        duration = 0
        for path_section in self.path_sections:
            duration += path_section.duration()
        return duration

    @functools.lru_cache
    def estimatedPositionAtTime(self, t) -> Transformation:
        """
        Get an estimated position of the robot given a certain time. Used by strategy simulation.
        Do not use in the walking engine

        :param t: The relative time of the path
        :return: Position of the torso at the given time
        """

        estimated_ratio = min(t / self.duration(), 1)
        return self.poseAtRatio(estimated_ratio)

    def isFinished(self, t):
        """
        Whether the path is complete

        :param t The relative time of the path
        :return: True if the path has been completed walking
        """

        return t >= self.duration()

    @functools.lru_cache
    def torsoStepTime(self):
        """
        The average time it takes for the robot's torso to make a single step

        :return: time in seconds
        """
        return self.duration() / self.torsoStepCount()

    # Return the subpath and the corresponding ratio
    @functools.lru_cache
    def getSubPathSectionAndRatio(self, r: float) -> (PathSection, float):
        """
        Returns the PathSection and ratio of the pathsection given a ratio of the entire Path

        :param r: ratio of the entire Path
        :return: Path section and the ratio of the Path Section
        """

        total_duration = self.duration()
        # TODO could probably sum up a lot of these at the init and make them access through the path section
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

    def poseAtRatio(self, r: float) -> Transformation:
        """
        Get the position given the ratio of the entire path

        :param r: ratio of the entire Path
        :return: Position of the torso at that time
        """
        # TODO all this ratio stuff seems messy and overly complex
        ratio, path_section = self.getSubPathSectionAndRatio(r)
        return path_section.poseAtRatio(ratio)

    def getTimePathOfNextStep(self, t) -> (float, float, float, PathSection, float):
        """
        Get a series of information about the path at a certain time

        :param t: The time relative to the entire path's time
        :return: a list of relavant information
        """
        # TODO why not use a data class also this is barely used
        ratio = t / self.duration()
        ratio, path_section = self.getSubPathSectionAndRatio(ratio)

        # Get next torsostep
        torsostep_count = 0
        for path in self.path_sections:
            if path != path_section:
                torsostep_count = torsostep_count + path.torsoStepCount()
            else:
                break
        subpath_count = (path_section.distance * ratio) / path_section.torso_step_length
        torsostep_count += subpath_count
        torsostep_count = ceil(torsostep_count)

        # Get information about this body step
        def getBodyStepTime(step_num):
            count = step_num
            time = 0
            for path_section in self.path_sections:
                if count <= path_section.torsoStepCount():
                    ratio = path_section.getRatioFromStep(count)
                    time = time + ratio * path_section.duration()
                    distance = path_section.torso_step_length * count
                    return time, ratio, distance, path_section
                count = count - path_section.torsoStepCount()
                time = time + path_section.duration()
            raise Exception("Invalid body step calculation " + str(count))

        time, ratio, distance, path_section = getBodyStepTime(torsostep_count)
        return time, ratio, distance, path_section, torsostep_count

    def terminateWalk(self, t):
        """
        Get estimated path ratio from the current time plus one more step, and then find the distance of that ratio and set the distance.
        Stops the walking at a given time, ends the current PathSection's trajectory

        :param t: Time relative to the Path
        """

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
        """
        Updates the goal position dynamically (not working)

        :param t: Current time
        :param end_transform: New goal position
        """

        t_change = t + 1
        if len(self.path_sections) >= 1 and self.path_sections[-1] is PathSectionShort:
            raise Exception("If the last path is a Short paths, it cannot be further modified")
        if self.duration() - t_change < 1:
            raise Exception("There is not enough time to update the position, current time { t } , duration of current path { self.duration()}")

        t_new, ratio, path_distance, path_section, step = self.getTimePathOfNextStep(t_change)
        start_transform = self.getTorsoStepPose(step)

        # TODO allow bezier paths to go to short paths and vice versa
        if self.isShortPath(start_transform, end_transform):
            raise Exception("Cannot append a short path to a bezier path")

        self.terminateWalk(t_new)
        p = self.createPathSection(start_transform, end_transform)
        self.path_sections.append(p)

        return t_new

    def show(self):
        """
        Show some plots about the path
        """

        position = np.zeros((self.torsoStepCount(), 3))
        orientation = np.zeros((self.torsoStepCount(), 3))
        colors = np.zeros((self.torsoStepCount(), 4))
        colors_arrow_ends = np.zeros((self.torsoStepCount() * 2, 4))

        section_color_map = {}
        for i in range(0, len(self.path_sections)):
            section_color_map[i] = np.append(np.random.rand(3), 1)

        for i in range(0, self.torsoStepCount(), 1):  # i = 0:1: obj.torsoStepCount
            step = self.getTorsoStepPose(i)
            position[i, 0:3] = step.position
            orientation[i, 0:3] = (step[0:3, 0:3] @ np.reshape(np.array([0.015, 0.0, 0.0]), (3, 1)))[:, 0]

            count = i
            section = 0
            for path_section in self.path_sections:
                if count <= path_section.torsoStepCount():
                    break
                count = count - path_section.torsoStepCount()
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


if __name__ == "__main__":
    p = Path(Transformation(), Transformation([0.5, 0, 0], [0, 0, 0, 1]))
    ax = p.show()
    plt.show()

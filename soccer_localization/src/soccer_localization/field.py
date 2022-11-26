import os
import time
from collections import namedtuple
from functools import cached_property
from typing import Dict, List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np
import scipy
from matplotlib.collections import PathCollection

from soccer_common import Transformation
from soccer_common.utils import wrapToPi

Point = namedtuple("Point", "x y")
Transform2D = namedtuple("Transform2D", "dx dy dtheta")
Line = namedtuple("Line", "p1 p2")
Circle = namedtuple("Circle", "center radius")

A = 9
B = 6
C = 0.6
D = 2.6
E = 1
F = 3
G = 1.5
H = 1.5
I = 1
J = 2
K = 5
lw = 0.05


class Field:
    def __init__(self):
        # Dimensions given here https://cdn.robocup.org/hl/wp/2021/06/V-HL21_Rules_v4.pdf

        self.distance_point_threshold = 2.2
        self.max_detected_line_parallel_offset_error = 0.1
        self.max_detected_line_perpendicular_offset_error = 0.3

        self.path_plots: Dict[str, PathCollection] = {}
        self.path_points: Dict[str, list] = {}

    @cached_property
    def lines(self) -> List[Union[Line, Circle]]:
        lines: List[Union[Line, Circle]] = []

        # Edge lines
        lines.append(Line(Point(x=-A / 2, y=-B / 2), Point(x=A / 2, y=-B / 2)))
        lines.append(Line(Point(x=-A / 2, y=B / 2), Point(x=A / 2, y=B / 2)))
        lines.append(Line(Point(x=-A / 2, y=-B / 2), Point(x=-A / 2, y=B / 2)))
        lines.append(Line(Point(x=A / 2, y=-B / 2), Point(x=A / 2, y=B / 2)))
        lines.append(Line(Point(x=0, y=-B / 2), Point(x=0, y=B / 2)))

        # Penalty area lines
        lines.append(Line(Point(x=-A / 2, y=-K / 2), Point(x=-(A - 2 * J) / 2, y=-K / 2)))
        lines.append(Line(Point(x=-A / 2, y=K / 2), Point(x=-(A - 2 * J) / 2, y=K / 2)))
        lines.append(Line(Point(x=(A - 2 * J) / 2, y=-K / 2), Point(x=A / 2, y=-K / 2)))
        lines.append(Line(Point(x=(A - 2 * J) / 2, y=K / 2), Point(x=A / 2, y=K / 2)))
        lines.append(Line(Point(x=-(A - 2 * J) / 2, y=-K / 2), Point(x=-(A - 2 * J) / 2, y=K / 2)))
        lines.append(Line(Point(x=(A - 2 * J) / 2, y=-K / 2), Point(x=(A - 2 * J) / 2, y=K / 2)))

        # Goal area lines
        lines.append(Line(Point(x=-A / 2, y=-F / 2), Point(x=-(A - 2 * E) / 2, y=-F / 2)))
        lines.append(Line(Point(x=-A / 2, y=F / 2), Point(x=-(A - 2 * E) / 2, y=F / 2)))
        lines.append(Line(Point(x=(A - 2 * E) / 2, y=-F / 2), Point(x=A / 2, y=-F / 2)))
        lines.append(Line(Point(x=(A - 2 * E) / 2, y=F / 2), Point(x=A / 2, y=F / 2)))
        lines.append(Line(Point(x=-(A - 2 * E) / 2, y=-F / 2), Point(x=-(A - 2 * E) / 2, y=F / 2)))
        lines.append(Line(Point(x=(A - 2 * E) / 2, y=-F / 2), Point(x=(A - 2 * E) / 2, y=F / 2)))

        # Crosses
        def create_cross(pos_x):
            lines.append(Line(Point(pos_x - 0.1, y=0), Point(pos_x + 0.1, y=0)))
            lines.append(Line(Point(pos_x, y=-0.1), Point(pos_x, y=0.1)))

        create_cross(A / 2 - G)
        create_cross(-A / 2 + G)
        # create_cross(0)

        # Circle
        lines.append(Circle(center=Point(x=0, y=0), radius=H))

        return lines

    def draw(self):

        # Draw field
        plt.axes().set_facecolor("limegreen")

        # Circle
        for line in self.lines:
            if type(line) is Circle:
                plt.gca().add_patch(plt.Circle(line.center, line.radius + lw / 2, color="white"))
                plt.gca().add_patch(plt.Circle(line.center, line.radius - lw / 2, color="limegreen"))
                break

        # Lines
        for line in self.lines:
            if type(line) is Line:
                if abs(line.p2.x - line.p1.x) > abs(line.p2.y - line.p1.y):
                    plt.fill_between(x=[line.p1.x - lw / 2, line.p2.x + lw / 2], y1=line.p1.y - lw / 2, y2=line.p1.y + lw / 2, color="white")
                else:
                    plt.fill_betweenx(y=[line.p1.y - lw / 2, line.p2.y + lw / 2], x1=line.p1.x - lw / 2, x2=line.p1.x + lw / 2, color="white")

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("UKF Robot localization")
        if "DISPLAY" in os.environ:
            plt.show(block=False)

    def filterWorldFramePoints(self, current_transform: Transformation, point_cloud_array: np.array) -> Optional[np.array]:
        # Filter points by distance from current transform
        points_distance_from_current_sq = point_cloud_array[:, 0] ** 2 + point_cloud_array[:, 1] ** 2
        point_cloud_meets_dist_threshold_index = np.where(points_distance_from_current_sq < self.distance_point_threshold**2)
        point_cloud_array = point_cloud_array[point_cloud_meets_dist_threshold_index]
        if len(point_cloud_array) == 0:
            return None

        rotation_matrix = current_transform.matrix[0:2, 0:2]
        world_frame_points = np.stack([current_transform.position[0:2]] * len(point_cloud_array), axis=1) + (
            rotation_matrix @ point_cloud_array[:, 0:2].T
        )

        # TODO filter density
        return world_frame_points

    def matchPointsWithMap(self, current_transform: Transformation, point_cloud_array: np.array) -> Union[Transformation, None]:

        start = time.time()
        # Filter points by distance from current transform
        world_frame_points = self.filterWorldFramePoints(current_transform, point_cloud_array)
        if world_frame_points is None:
            return
        # Get closest line to each point
        distance_matrix = np.zeros((len(self.lines), world_frame_points.shape[1]))
        diff_x = np.zeros((len(self.lines), world_frame_points.shape[1]))  # real points - line
        diff_y = np.zeros((len(self.lines), world_frame_points.shape[1]))  # real points - line

        for line_id, line in enumerate(self.lines):
            if type(line) is Line:
                if line.p1.x != line.p2.x:
                    assert line.p1.x < line.p2.x
                    line_horizontal = True
                else:
                    assert line.p1.y < line.p2.y
                    line_horizontal = False

                if line_horizontal:
                    x_left = line.p1.x - lw / 2 - self.max_detected_line_parallel_offset_error
                    x_right = line.p2.x + lw / 2 + self.max_detected_line_parallel_offset_error
                    y = line.p1.y

                    y_diff = world_frame_points[1, :] - y
                    distance_matrix[line_id, :] = np.where(
                        (world_frame_points[0, :] >= x_left) & (world_frame_points[0, :] <= x_right), y_diff**2, float("inf")
                    )
                    diff_y[line_id, :] = y_diff
                else:
                    y_bottom = line.p1.y - lw / 2 - self.max_detected_line_parallel_offset_error
                    y_top = line.p2.y + lw / 2 + self.max_detected_line_parallel_offset_error
                    x = line.p1.x

                    x_diff = world_frame_points[0, :] - x
                    distance_matrix[line_id, :] = np.where(
                        (world_frame_points[1, :] >= y_bottom) & (world_frame_points[1, :] <= y_top), x_diff**2, float("inf")
                    )
                    diff_x[line_id, :] = x_diff
                pass
            else:
                distance = world_frame_points[0, :] ** 2 + world_frame_points[1, :] ** 2 - line.radius**2
                distance_matrix[line_id, :] = np.where(distance > lw / 2 + 0.5, float("inf"), distance)

                # TODO test this and optimize the speed
                yx_ratio = np.arctan2(world_frame_points[1, :], world_frame_points[0, :])
                diff_y[line_id, :] = distance * np.sin(yx_ratio)
                diff_x[line_id, :] = distance * np.cos(yx_ratio)

        closest_line = np.argmin(distance_matrix, axis=0)
        closest_dist = np.min(distance_matrix, axis=0)
        index_meets_dist_threshold = np.where(closest_dist < self.max_detected_line_perpendicular_offset_error**2)

        if len(index_meets_dist_threshold[0]) == 0:
            return None

        index_array = np.arange(len(closest_line))

        closest_line_diff_x = diff_x[closest_line, index_array]
        closest_line_diff_y = diff_y[closest_line, index_array]
        points_meet_dist_threshold = world_frame_points.T[index_meets_dist_threshold].T
        closest_line_diff_x = closest_line_diff_x[index_meets_dist_threshold]
        closest_line_diff_y = closest_line_diff_y[index_meets_dist_threshold]

        closest_line_diff_x_valid_count = np.count_nonzero(closest_line_diff_x)
        closest_line_diff_y_valid_count = np.count_nonzero(closest_line_diff_y)
        if closest_line_diff_x_valid_count > 0:
            diff_x_avg = np.sum(closest_line_diff_x) / closest_line_diff_x_valid_count
        else:
            diff_x_avg = 0
        if closest_line_diff_y_valid_count > 0:
            diff_y_avg = np.sum(closest_line_diff_y) / closest_line_diff_y_valid_count
        else:
            diff_y_avg = 0
        assert not np.isnan(diff_x_avg)
        assert not np.isnan(diff_y_avg)

        center_of_all_points = np.average(points_meet_dist_threshold, axis=1)
        points_meet_dist_threshold_delta = np.subtract(points_meet_dist_threshold, np.expand_dims(center_of_all_points, axis=1))
        points_meet_dist_threshold_shift = points_meet_dist_threshold_delta - np.concatenate([[closest_line_diff_x], [closest_line_diff_y]])

        angle_original = np.arctan2(points_meet_dist_threshold_delta[1, :], points_meet_dist_threshold_delta[0, :])
        angle_new = np.arctan2(points_meet_dist_threshold_shift[1, :], points_meet_dist_threshold_shift[0, :])
        angle_diff = angle_new - angle_original
        angle_diff = wrapToPi(angle_diff)
        angle_diff_avg = np.average(angle_diff)

        transform_delta_center_to_robot = scipy.linalg.inv(current_transform) @ Transformation(
            pos_theta=[center_of_all_points[0], center_of_all_points[1], 0]
        )

        offset_transform = (
            transform_delta_center_to_robot
            @ Transformation(pos_theta=[-diff_x_avg, -diff_y_avg, angle_diff_avg])
            @ scipy.linalg.inv(transform_delta_center_to_robot)
        )

        end = time.time()
        rospy.loginfo_throttle(60, f"Match Points with Map rate (s) :  {(end - start)}")
        return offset_transform

    def drawPointsOnMap(self, current_transform: Transformation, point_cloud_array: np.array, label: str, color: str):
        world_frame_points = self.filterWorldFramePoints(current_transform, point_cloud_array)

        if label in self.path_plots:
            self.path_plots[label].remove()

        self.path_plots[label] = plt.scatter(world_frame_points[0, :], world_frame_points[1, :], marker=".", s=1, label=label, color=color)

    def drawPathOnMap(self, current_transform: Transformation, label: str, color: str):
        if label not in self.path_points:
            self.path_points[label] = []

        self.path_points[label].append(current_transform.pos_theta)

        if label in self.path_plots:
            self.path_plots[label].remove()

        a = np.array(self.path_points[label])
        self.path_plots[label] = plt.scatter(a[:, 0], a[:, 1], marker=".", s=1, label=label, color=color)

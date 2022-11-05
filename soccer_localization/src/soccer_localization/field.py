import time
from collections import namedtuple
from functools import cached_property
from typing import List, Optional, Union

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PathCollection

from soccer_common import Transformation

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

        self.distance_point_threshold = 2
        self.max_detected_line_parallel_offset_error = 0.1
        self.max_detected_line_perpendicular_offset_error = 0.3

        self.points_scatter: Optional[PathCollection] = None
        self.path_ground_truth: Optional[PathCollection] = None
        self.points_ground_truth: [] = []
        self.path_odom: Optional[PathCollection] = None
        self.points_odom: [] = []

        self.path_odom_uncorrected: Optional[PathCollection] = None
        self.points_odom_uncorrected: [] = []

        self.path_vo: Optional[PathCollection] = None
        self.points_vo: [] = []

        pass

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

        # Circle
        lines.append(Circle(center=Point(x=0, y=0), radius=H))

        return lines

    def draw(self):

        # Draw field
        plt.axes().set_facecolor("limegreen")

        # Circle
        plt.gca().add_patch(plt.Circle((0, 0), H / 2 + lw / 2, color="white"))
        plt.gca().add_patch(plt.Circle((0, 0), H / 2 - lw / 2, color="limegreen"))

        # Outer rectangle
        plt.fill_between(x=[-A / 2 - lw / 2, A / 2 + lw / 2], y1=B / 2 - lw / 2, y2=B / 2 + lw / 2, color="white")
        plt.fill_between(x=[-A / 2 - lw / 2, A / 2 + lw / 2], y1=-B / 2 - lw / 2, y2=-B / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=-A / 2 - lw / 2, x2=-A / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=A / 2 - lw / 2, x2=A / 2 + lw / 2, color="white")
        plt.fill_betweenx(y=[-B / 2 - lw / 2, B / 2 + lw / 2], x1=-lw / 2, x2=lw / 2, color="white")

        # Penalty Area (large box)
        def draw_double_boxes(h, v):
            plt.fill_between(x=[-A / 2, -A / 2 + h], y1=v / 2 - lw / 2, y2=v / 2 + lw / 2, color="white")
            plt.fill_between(x=[-A / 2, -A / 2 + h], y1=-v / 2 - lw / 2, y2=-v / 2 + lw / 2, color="white")
            plt.fill_between(x=[A / 2 - h, A / 2], y1=v / 2 - lw / 2, y2=v / 2 + lw / 2, color="white")
            plt.fill_between(x=[A / 2 - h, A / 2], y1=-v / 2 - lw / 2, y2=-v / 2 + lw / 2, color="white")
            plt.fill_betweenx(y=[-v / 2 - lw / 2, v / 2 + lw / 2], x1=-A / 2 + h - lw / 2, x2=-A / 2 + h + lw / 2, color="white")
            plt.fill_betweenx(y=[-v / 2 - lw / 2, v / 2 + lw / 2], x1=A / 2 - h - lw / 2, x2=A / 2 - h + lw / 2, color="white")

        draw_double_boxes(J, K)
        draw_double_boxes(E, F)
        draw_double_boxes(-C, D)

        # Crosses
        def draw_cross(pos_x):
            plt.fill_between(x=[pos_x - 0.1, pos_x + 0.1], y1=-lw / 2, y2=lw / 2, color="white")
            plt.fill_betweenx(y=[-0.1, 0.1], x1=pos_x - lw / 2, x2=pos_x + lw / 2, color="white")

        draw_cross(A / 2 - G)
        draw_cross(-A / 2 + G)
        draw_cross(0)

        plt.xlabel("Y (m)")
        plt.ylabel("X (m)")
        plt.title("UKF Robot localization")
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
        return world_frame_points

    def matchPointsWithMap(self, current_transform: Transformation, point_cloud_array: np.array) -> Union[Transformation, None]:
        start = time.time()
        # Filter points by distance from current transform
        world_frame_points = self.filterWorldFramePoints(current_transform, point_cloud_array)
        if world_frame_points is None:
            return
        # Get closest line to each point
        distance_matrix = np.zeros((len(self.lines), world_frame_points.shape[1]))
        diff_x = np.zeros((len(self.lines), world_frame_points.shape[1]))
        diff_y = np.zeros((len(self.lines), world_frame_points.shape[1]))

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

                    x_diff = world_frame_points[1, :] - x
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

        diff_x_avg = np.average(closest_line_diff_x)
        diff_y_avg = np.average(closest_line_diff_y)

        center_of_all_points = np.average(points_meet_dist_threshold, axis=1)
        points_meet_dist_threshold_delta = np.subtract(points_meet_dist_threshold, np.expand_dims(center_of_all_points, axis=1))
        points_meet_dist_threshold_shift = points_meet_dist_threshold_delta + np.concatenate([[closest_line_diff_x], [closest_line_diff_y]])

        angle_original = np.arctan2(points_meet_dist_threshold_delta[1, :], points_meet_dist_threshold_delta[0, :])
        angle_new = np.arctan2(points_meet_dist_threshold_shift[1, :], points_meet_dist_threshold_shift[0, :])
        angle_diff = angle_new - angle_original
        angle_diff_avg = np.average(angle_diff)

        offset_transform = Transformation(pos_theta=[diff_x_avg, diff_y_avg, angle_diff_avg])

        end = time.time()
        print(f"Offset Transform: {offset_transform.pos_theta}. Time took {(end - start)}")
        return offset_transform

    def drawPointsOnMap(self, current_transform: Transformation, point_cloud_array: np.array):
        world_frame_points = self.filterWorldFramePoints(current_transform, point_cloud_array)

        if self.points_scatter is not None:
            self.points_scatter.remove()

        if self.path_odom is not None:
            self.path_odom.remove()

        self.points_odom.append(current_transform.pos_theta)
        a = np.array(self.points_odom)
        self.path_odom = plt.scatter(a[:, 0], a[:, 1], marker=".", s=1, label="Robot Odom", color="orange")
        self.points_scatter = plt.scatter(world_frame_points[0, :], world_frame_points[1, :], marker=".", s=1, label="Points", color="red")

    def drawGroundTruthOnMap(self, current_transform: Transformation):
        if self.path_ground_truth is not None:
            self.path_ground_truth.remove()

        self.points_ground_truth.append(current_transform.pos_theta)
        a = np.array(self.points_ground_truth)
        self.path_ground_truth = plt.scatter(a[:, 0], a[:, 1], marker=".", s=1, label="Robot Ground Truth", color="yellow")

    def drawUncorrectedOdom(self, current_transform: Transformation):
        if self.path_odom_uncorrected is not None:
            self.path_odom_uncorrected.remove()

        self.points_odom_uncorrected.append(current_transform.pos_theta)
        a = np.array(self.points_odom_uncorrected)
        self.path_odom_uncorrected = plt.scatter(a[:, 0], a[:, 1], marker=".", s=1, label="Robot Odom Uncorrected", color="blue")

    def drawVoOnMap(self, current_transform: Transformation):
        if self.path_vo is not None:
            self.path_vo.remove()

        self.points_vo.append(current_transform.pos_theta)
        a = np.array(self.points_vo)
        self.path_vo = plt.scatter(a[:, 0], a[:, 1], marker=".", s=1, label="Robot Visual Odometry", color="red")

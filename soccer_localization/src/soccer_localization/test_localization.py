import os
from functools import cached_property
from pathlib import Path
from typing import List, Union

import gdown
import matplotlib.pyplot as plt
import numpy as np
import pytest
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pcl2

from soccer_common.transformation import Transformation
from soccer_localization.field import Circle, Field, Line, Point
from soccer_localization.field_lines_ukf import FieldLinesUKF
from soccer_localization.field_lines_ukf_ros import FieldLinesUKFROS
from soccer_msgs.msg import RobotState


def retrieve_bag(url="https://drive.google.com/uc?id=1T_oyM1rZwWgUy6A6KlsJ7Oqn8J3vpGDo", bag_name="localization"):
    src_path = os.path.dirname(os.path.realpath(__file__))
    folder_path = Path(src_path + "/../../data/")
    if not folder_path.is_dir():
        os.makedirs(folder_path)

    test_path = src_path + f"/../../data/{bag_name}.bag"
    bag_path = Path(test_path)
    if not bag_path.is_file():
        print(f"Bag not found at {test_path}. Downloading ...")
        zip_path = test_path.replace("bag", "zip")
        gdown.download(url, zip_path, quiet=False)
        import zipfile

        with zipfile.ZipFile(zip_path, "r") as zip_ref:
            zip_ref.extractall(folder_path)
        os.remove(zip_path)
    return test_path


@pytest.mark.parametrize("t_start", [(60)])
def test_points_correction(t_start):
    rospy.init_node("test")

    plt.figure("Localization")

    map = Field()
    map.draw()

    bag = rosbag.Bag(retrieve_bag())

    transform_gt = None
    transform_gt_offset = Transformation(pos_theta=[0.05, 0.05, 0.05])
    # transform_gt_offset = Transformation()

    for topic, msg, t in bag.read_messages(topics=["/robot1/odom_combined", "/tf", "/robot1/field_point_cloud"]):

        if topic == "/tf":
            # Process ground truth information
            for transform in msg.transforms:
                if transform.child_frame_id == "robot1/base_footprint_gt":
                    transform_gt = Transformation(geometry_msgs_transform=transform.transform)

        elif topic == "/robot1/field_point_cloud":
            if t.secs < t_start:
                continue

            map.drawPathOnMap(transform_gt, label="Ground Truth", color="brown")

            point_cloud = pcl2.read_points_list(msg)
            point_cloud_array = np.array(point_cloud)
            current_transform = transform_gt_offset @ Transformation(pos_theta=transform_gt.pos_theta)
            offset_transform = map.matchPointsWithMap(current_transform, point_cloud_array)
            if offset_transform is not None:
                # pp = offset_transform.pos_theta
                # pp[2] = 0
                # offset_transform.pos_theta = pp
                vo_transform = offset_transform @ current_transform
                map.drawPathOnMap(vo_transform, label="VO Odometry", color="blue")
                map.drawPointsOnMap(current_transform, point_cloud_array, label="Odom Points", color="brown")
                map.drawPointsOnMap(vo_transform, point_cloud_array, label="Odom Points Adjusted", color="red")
                plt.title(f"UKF Robot localization (t = {round(t.secs + t.nsecs * 1e-9)})")
                plt.xlim((-5, -3))
                plt.ylim((-3.5, 2))
                plt.legend()

                if "DISPLAY" in os.environ:
                    plt.draw()
                    plt.waitforbuttonpress(timeout=1.01)

    plt.close()


def display_rosbag_map(bag, map, debug=False, pos_theta_start=[-4, -3.15, np.pi / 2]):

    f = FieldLinesUKFROS(map=map)
    f.robot_state.status = RobotState.STATUS_READY
    initial_pose = Transformation(pos_theta=pos_theta_start)
    f.ukf.x = initial_pose.pos_theta

    path_ukf = []
    path_ukf_t = []
    path_odom = []
    path_odom_t = []
    path_gt = []
    path_gt_t = []
    path_vo = []  # Path Point Cloud
    path_vo_t = []
    path_covariance = []

    predict_it = 0
    for topic, msg, t in bag.read_messages(topics=["/robot1/odom_combined", "/tf", "/robot1/field_point_cloud"]):
        if topic == "/robot1/field_point_cloud":
            current_transform = Transformation(pos_theta=f.ukf.x)
            point_cloud_array, vo_transform, vo_pos_theta = f.field_point_cloud_callback(msg)
            if vo_pos_theta is not None:
                path_vo.append(vo_pos_theta)
                path_vo_t.append(t.to_sec())

            if debug:
                map.drawPathOnMap(Transformation(pos_theta=f.ukf.x), label="VO Odometry", color="orange")

                if vo_pos_theta is not None:
                    map.drawPathOnMap(vo_transform, label="VO Odometry", color="red")
                    map.drawPointsOnMap(vo_transform, point_cloud_array, label="Odom Points Corrected", color="red")
                if point_cloud_array is not None:
                    # if transform_gt is not None:
                    #     map.drawPointsOnMap(transform_gt, point_cloud_array, label="Odom Points Ground Truth", color="white")
                    map.drawPointsOnMap(current_transform, point_cloud_array, label="Odom Points", color="black")

                plt.title(f"UKF Robot localization (t = {round(t.secs + t.nsecs * 1e-9)})")
                # plt.xlim((-5, -3))
                # plt.ylim((-3.5, 1))
                plt.legend()
                plt.draw()
                plt.waitforbuttonpress()

        elif topic == "/tf":
            # Process ground truth information
            for transform in msg.transforms:
                if transform.child_frame_id in ["robot1/base_footprint_gt", "body"]:
                    transform_gt = Transformation(geometry_msgs_transform=transform.transform)
                    path_gt.append(transform_gt.pos_theta)
                    path_gt_t.append(t.to_sec())
                    if debug:
                        map.drawPathOnMap(transform_gt, label="Ground Truth", color="black")

        elif topic == "/robot1/odom_combined":
            odom_t = f.odom_callback(msg)
            if odom_t is None:
                continue

            # Draw covariance
            predict_it += 1
            if predict_it % 40 == 0:
                f.draw_covariance()

            # Add path UKF
            path_ukf.append(f.ukf.x)
            path_covariance.append(np.sqrt(np.diag(f.ukf.P)))
            path_ukf_t.append(t.to_sec())

            # Draw uncorrect odom
            odom_uncorrected = initial_pose @ odom_t
            path_odom.append(odom_uncorrected.pos_theta)
            path_odom_t.append(t.to_sec())
            if debug:
                map.drawPathOnMap(odom_uncorrected, label="Uncorrected Odom", color="blue")

    path_ukf = np.array(path_ukf)
    path_odom = np.array(path_odom)
    path_gt = np.array(path_gt)
    path_vo = np.array(path_vo)
    path_covariance = np.array(path_covariance)

    path_ukf_t = np.array(path_ukf_t)
    path_odom_t = np.array(path_odom_t)
    path_gt_t = np.array(path_gt_t)
    path_vo_t = np.array(path_vo_t)

    # Add the patch to the Axes
    if len(path_odom) != 0:
        plt.plot(path_odom[:, 0], path_odom[:, 1], color="yellow", linewidth=0.5, label="Odom Path")
    if len(path_ukf) != 0:
        plt.plot(path_ukf[:, 0], path_ukf[:, 1], color="white", linewidth=0.5, label="UKF Path")
    if len(path_gt) != 0:
        plt.plot(path_gt[:, 0], path_gt[:, 1], color="orange", linewidth=0.5, label="Ground Truth Path")
    if len(path_vo) != 0:
        plt.plot(path_vo[:, 0], path_vo[:, 1], color="red", linewidth=0.5, label="VO Points")
    plt.xlim((-5, 5))
    plt.ylim((-4, 4))
    plt.tight_layout()
    plt.legend()
    plt.waitforbuttonpress(timeout=10)

    def plt_dim_error(dim=0, label="X"):
        plt.figure(f"{label} Error")
        plt.title(f"{label} Dimension Odom, UKF estimate, Ground Truth and Visual Odometry")
        plt.fill_between(
            path_ukf_t,
            path_ukf[:, dim] + path_covariance[:, dim],
            path_ukf[:, dim] - path_covariance[:, dim],
            color="yellow",
            label="Variance for odom path",
        )
        plt.plot(path_odom_t, path_odom[:, dim], color="blue", linewidth=0.5, label="Odom Path")
        plt.plot(path_ukf_t, path_ukf[:, dim], color="green", linewidth=0.5, label="UKF Path")
        plt.plot(path_gt_t, path_gt[:, dim], color="orange", linewidth=0.5, label="Ground Truth Path")
        plt.scatter(path_vo_t, path_vo[:, dim], color="red", marker=".", s=1, label="VO Points")
        plt.grid(visible=True)
        plt.legend()
        plt.xlabel("t (s)")
        plt.ylabel(label)

    # Plot X Error
    plt_dim_error(0, "X")
    plt_dim_error(1, "Y")
    plt_dim_error(2, "Theta")

    if "DISPLAY" in os.environ:
        plt.show(block=True)
        plt.waitforbuttonpress()
        plt.close("all")


def test_walk_forward():
    rospy.init_node("test")

    plt.figure("Localization")

    map = Field()
    map.draw()

    bag = rosbag.Bag(retrieve_bag())
    display_rosbag_map(bag=bag, map=map, debug=False)


def test_walk_forward_center_map():

    rospy.init_node("test")

    plt.figure("Localization")

    map = Field()
    map.draw()
    bag = rosbag.Bag(retrieve_bag(url="https://drive.google.com/uc?id=1VNHkAu10cfFJzcpTvc0zR8Jm-tI_6xQ8", bag_name="localization_2"))
    display_rosbag_map(bag=bag, map=map, debug=False, pos_theta_start=[-1, -3.15, np.pi / 2])


class FreehicleField(Field):
    @cached_property
    def lines(self) -> List[Union[Line, Circle]]:
        lines: List[Union[Line, Circle]] = []

        linegap = 8 / 9
        centergap = ((350 - 55 * 2) / 350 * 2) * 2

        for i in [-4, 4]:
            lines.append(Line(Point(x=0, y=i), Point(x=4.5, y=i)))
            for xgap in range(6):
                lines.append(Line(Point(x=xgap * linegap, y=-centergap + i), Point(x=xgap * linegap, y=centergap + i)))

        return lines


def test_freehicle_movement():
    rospy.init_node("test")
    rospy.set_param("distance_point_threshold", 2)
    plt.figure("Localization")

    map = FreehicleField()
    map.draw()

    bag = rosbag.Bag(retrieve_bag(url="https://drive.google.com/uc?id=1o2HGsaSvf4mQhZG_iXMhgR5Z1lXJNOGF", bag_name="freehicle_out"))
    display_rosbag_map(bag=bag, map=map, debug=False, pos_theta_start=[0, 0, 0])


def test_show_ukf_stuff():
    import soccer_localization.utils.ukf_internal as ukf_internal

    f = FieldLinesUKF()
    f.draw_covariance()
    ukf_internal.plot_sigmas(f.ukf.points_fn, x=f.ukf.x, cov=f.ukf.P)

    if "DISPLAY" in os.environ:
        plt.show(block=False)
        plt.waitforbuttonpress(timeout=10)
        plt.close("all")

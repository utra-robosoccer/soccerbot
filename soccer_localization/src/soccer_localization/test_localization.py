import os
from pathlib import Path

import gdown
import matplotlib.pyplot as plt
import numpy as np
import pytest
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pcl2

from soccer_common.transformation import Transformation
from soccer_localization.field import Field
from soccer_localization.field_lines_ukf import FieldLinesUKF
from soccer_localization.field_lines_ukf_ros import FieldLinesUKFROS
from soccer_msgs.msg import RobotState


def retrieve_bag():
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + "/test/localization.bag"
    bag_path = Path(test_path)
    if not bag_path.is_file():
        print(f"Bag not found at {test_path}. Downloading ...")
        url = "https://drive.google.com/uc?id=1HpBAFg1FFYhiCaEN5K81b2sQ3rq9i3Nx"
        gdown.download(url, test_path, quiet=False)
    return test_path


@pytest.mark.parametrize("t_start", [(80)])
def test_points_correction(t_start):
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
                    plt.waitforbuttonpress(timeout=0.01)

    plt.close()


def test_walk_forward():
    rospy.init_node("test")

    plt.figure("Localization")
    debug = False

    map = Field()
    map.draw()

    bag = rosbag.Bag(retrieve_bag())

    f = FieldLinesUKFROS()
    f.robot_state.status = RobotState.STATUS_READY
    initial_pose = Transformation(pos_theta=[-4, -3.15, np.pi / 2])
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
                f.update(vo_pos_theta)
                if debug:
                    map.drawPathOnMap(vo_transform, label="VO Odometry", color="red")
                    map.drawPathOnMap(Transformation(pos_theta=f.ukf.x), label="VO Odometry", color="orange")
                    map.drawPointsOnMap(current_transform, point_cloud_array, label="Odom Points", color="brown")
                    map.drawPointsOnMap(vo_transform, point_cloud_array, label="Odom Points Corrected", color="orange")

                    plt.title(f"UKF Robot localization (t = {round(t.secs + t.nsecs * 1e-9)})")
                    plt.xlim((-5, -3))
                    plt.ylim((-3.5, 1))
                    plt.legend()
                    plt.draw()
                    plt.waitforbuttonpress()

        elif topic == "/tf":
            # Process ground truth information
            for transform in msg.transforms:
                if transform.child_frame_id == "robot1/base_footprint_gt":
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
    plt.plot(path_odom[:, 0], path_odom[:, 1], color="yellow", linewidth=0.5, label="Odom Path")
    plt.plot(path_ukf[:, 0], path_ukf[:, 1], color="white", linewidth=0.5, label="UKF Path")
    plt.plot(path_gt[:, 0], path_gt[:, 1], color="orange", linewidth=0.5, label="Ground Truth Path")
    plt.plot(path_vo[:, 0], path_vo[:, 1], color="red", linewidth=0.5, label="VO Points")
    plt.xlim((-5, 5))
    plt.ylim((-4, 4))
    plt.tight_layout()
    plt.legend()

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
        plt.show(block=False)
        plt.waitforbuttonpress(timeout=10)
        plt.close("all")


def test_show_ukf_stuff():
    import soccer_localization.utils.ukf_internal as ukf_internal

    f = FieldLinesUKF()
    f.draw_covariance()
    ukf_internal.plot_sigmas(f.ukf.points_fn, x=f.ukf.x, cov=f.ukf.P)

    if "DISPLAY" in os.environ:
        plt.show(block=False)
        plt.waitforbuttonpress(timeout=10)
        plt.close("all")

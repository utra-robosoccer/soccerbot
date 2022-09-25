import copy
import os
from pathlib import Path

import gdown
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pcl2

from soccer_common.transformation import Transformation
from soccer_localization.field_lines_ukf import FieldLinesUKF


def test_simple():
    plt.figure("Localization")

    # Draw field
    plt.axes().set_facecolor("limegreen")

    # Dimensions given here https://cdn.robocup.org/hl/wp/2021/06/V-HL21_Rules_v4.pdf
    lw = 0.05
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

    plt.axis("equal")
    plt.xlabel("Y (m)")
    plt.ylabel("X (m)")
    plt.title("UKF Robot localization")

    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + "/test/localization.bag"
    bag_path = Path(test_path)

    if not bag_path.is_file():
        print(f"Bag not found at {test_path}. Downloading ...")
        url = "https://drive.google.com/uc?id=1HpBAFg1FFYhiCaEN5K81b2sQ3rq9i3Nx"
        gdown.download(url, test_path, quiet=False)

    bag = rosbag.Bag(test_path)

    f = FieldLinesUKF()
    initial_pose = Transformation(pos_theta=[-4, -3.15, np.pi / 2])
    f.ukf.x = initial_pose.pos_theta

    path_ukf = []
    path_odom = []
    path_gt = []

    odom_t_previous = None
    predict_it = 0
    for topic, msg, t in bag.read_messages(topics=["/robot1/odom_combined", "/tf", "/robot1/field_point_cloud"]):
        if topic == "/robot1/field_point_cloud":
            point_cloud = pcl2.read_points_list(msg)
            point_cloud_array = np.array(point_cloud)
            current_transform = Transformation(pos_theta=f.ukf.x)
            rotation_matrix = current_transform.matrix[0:2, 0:2]
            # world_frame_points = (rotation_matrix @ point_cloud_array[:, 0:2].T)
            world_frame_points = np.stack([current_transform.position[0:2]] * len(point_cloud_array), axis=1) + (
                rotation_matrix @ point_cloud_array[:, 0:2].T
            )
            if t.secs == 10 and t.nsecs == 544000000:
                plt.scatter(world_frame_points[0, :], world_frame_points[1, :], marker=".", s=1, label="Points", color="yellow")
            pass
        if topic == "/tf":
            # Process ground truth information
            for transform in msg.transforms:
                if transform.child_frame_id == "robot1/base_footprint_gt":
                    transform_gt = Transformation(geometry_msgs_transform=transform.transform)
                    path_gt.append(transform_gt.pos_theta)
                    pass
                pass
        elif topic == "/robot1/odom_combined":
            if odom_t_previous is None:
                odom_t_previous = Transformation(pose=msg.pose.pose, timestamp=t)
                continue
            odom_t = Transformation(pose=msg.pose.pose, timestamp=t)

            diff_transformation: Transformation = np.linalg.inv(odom_t_previous) @ odom_t
            dt = odom_t.timestamp - odom_t_previous.timestamp
            dt_secs = dt.secs + dt.nsecs * 1e-9
            if dt_secs == 0:
                continue

            f.predict(diff_transformation.pos_theta / dt_secs, dt_secs)
            predict_it += 1
            if predict_it % 40 == 0:
                f.draw_covariance()

            path_ukf.append(f.ukf.x)
            path_odom.append((initial_pose @ odom_t).pos_theta)
            odom_t_previous = odom_t

    path_ukf = np.array(path_ukf)
    path_odom = np.array(path_odom)
    path_gt = np.array(path_gt)

    # Add the patch to the Axes
    plt.plot(path_odom[:, 0], path_odom[:, 1], color="yellow", linewidth=0.5, label="Odom Path")
    plt.plot(path_ukf[:, 0], path_ukf[:, 1], color="white", linewidth=0.5, label="UKF Path")
    plt.plot(path_gt[:, 0], path_gt[:, 1], color="orange", linewidth=0.5, label="Ground Truth Path")
    plt.legend()

    plt.show(block=True)

    plt.figure("Error X, Y, Z")

    pass


def test_show_ukf_stuff():
    import soccer_localization.utils.ukf_internal as ukf_internal

    f = FieldLinesUKF()
    f.draw_covariance()
    ukf_internal.plot_sigmas(f.ukf.points_fn, x=f.ukf.x, cov=f.ukf.P)
    plt.show()

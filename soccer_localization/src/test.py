import copy
import os
from pathlib import Path

import gdown
import matplotlib.pyplot as plt
import numpy as np
import rosbag

from soccer_common.transformation import Transformation
from soccer_localization.src.field_lines_ukf import FieldLinesUKF


def test_simple():
    plt.figure("Localization")
    plt.axes().set_facecolor("limegreen")
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + "/../test/localization.bag"
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

    odom_t_previous = None
    for topic, msg, t in bag.read_messages(topics=["/robot1/odom_combined"]):
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

        path_ukf.append(f.ukf.x)
        path_odom.append((initial_pose @ odom_t).pos_theta)

        odom_t_previous = odom_t

    path_ukf = np.array(path_ukf)
    path_odom = np.array(path_odom)

    # Add the patch to the Axes
    plt.plot(path_odom[:, 0], path_odom[:, 1], color="k", linewidth=0.5)
    plt.plot(path_ukf[:, 0], path_ukf[:, 1], color="red", linewidth=0.5)

    xs = [-4.5, 4.5, 4.5, -4.5, -4.5, 0, 0]
    ys = [-3, -3, 3, 3, -3, -3, 3]
    plt.plot(xs, ys, color="white")
    plt.axis("equal")
    plt.xlabel("Y (m)")
    plt.ylabel("X (m)")
    plt.title("UKF Robot localization")
    plt.show()

    pass

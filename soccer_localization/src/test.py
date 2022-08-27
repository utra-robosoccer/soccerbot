import os

import bagpy
import matplotlib as mpl
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
from bagpy import bagreader
from geometry_msgs.msg import PoseArray, PoseStamped

from soccer_common.transformation import Transformation
from soccer_localization.src.field_lines_ukf import FieldLinesUKF


def test_simple():
    f = FieldLinesUKF()
    plt.figure()
    plt.axes().set_facecolor("limegreen")
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + "/../test/localization.bag"
    bag = rosbag.Bag(test_path)
    last_t = 0.0
    last_transformation = Transformation()
    track = []
    count = 0
    for topic, msg, t in bag.read_messages(topics=["/robot1/odom_combined"]):
        p = Transformation(pose=msg.pose.pose)
        diff_transformation: Transformation = p @ np.linalg.inv(last_transformation)
        dt = float((t.to_sec())) - last_t
        if dt == 0:
            continue
        u = diff_transformation.pos_theta / dt
        f.predict(u, dt)
        print(p.pos_theta)
        last_t = float((t.to_sec()))
        last_transformation = p
        track.append(diff_transformation.pos_theta)
        count += 1
        # if count > 1000:
        #     break

    track = np.array(track)
    # Add the patch to the Axes

    plt.plot(track[:, 0], track[:, 1], color="k", lw=2)
    xs = [-4.5, 4.5, 4.5, -4.5, -4.5, 0, 0]
    ys = [-3, -3, 3, 3, -3, -3, 3]
    plt.plot(xs, ys, color="white")
    plt.axis("equal")

    plt.title("UKF Robot localization")
    plt.show()

    pass

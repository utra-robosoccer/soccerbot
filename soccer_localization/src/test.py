import os

import rosbag

from soccer_localization.src.field_lines_ukf import FieldLinesUKF


def test_simple():
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + "/../test/startup_walk.bag"
    bag = rosbag.Bag(test_path)
    for topic, msg, t in bag.read_messages():
        print(msg)

    f = FieldLinesUKF()

    pass

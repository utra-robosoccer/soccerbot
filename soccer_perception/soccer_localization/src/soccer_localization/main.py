#!/usr/bin/env python3
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rclpy
from soccer_localization.field_lines_ukf_ros import FieldLinesUKFROS

if __name__ == "__main__":
    self.init_node("soccer_localization")
    self.get_logger().info("Initializing Soccer Localization")

    f = FieldLinesUKFROS()
    self.spin()

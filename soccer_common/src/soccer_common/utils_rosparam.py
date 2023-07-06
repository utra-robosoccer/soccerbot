from os import path
from unittest.mock import MagicMock

import rosparam
import rospy
import yaml


def set_rosparam_from_yaml_file(param_path: str = "", delete_params=True, convert_logs_to_prints=True):
    if rospy.get_node_uri() is None:
        rospy.init_node("test")

    if convert_logs_to_prints:
        rospy.set_param("/use_sim_time", False)

        rospy.loginfo_throttle = lambda a, b: None
        rospy.loginfo = lambda a: print(a)
        rospy.logwarn = lambda a: print(a)
        rospy.logerr = lambda a: print(a)

    if delete_params:
        rosparam.delete_param("/")
    if path.exists(param_path):
        with open(param_path) as f:
            param_info = yaml.safe_load(f)
            rosparam.upload_params(rospy.get_namespace(), param_info)

from os import path

import rospy
import yaml


def set_rosparam_from_yaml_file(params_path: str):
    rospy.get_param = rospy.get_param_cached

    def post_to_rospy(param_name, param_value, parent_name):
        if type(param_value) is dict:
            for sub_name, sub_value in param_value.items():
                post_to_rospy(sub_name, sub_value, f"{parent_name}/{sub_name}")
        rospy.set_param(param_name, param_value)

    hivebot_info_path = f"{params_path}.yaml"
    if path.exists(hivebot_info_path):
        with open(hivebot_info_path) as f:
            hivebot_info = yaml.safe_load(f)
            for name, value in hivebot_info.items():
                post_to_rospy(name, value, f"{name}")


def mock_ros():
    from unittest.mock import MagicMock

    rospy.set_param("/use_sim_time", False)
    rospy.get_namespace = MagicMock(return_value="/robot1/")
    rospy.loginfo_throttle = lambda a, b: None
    rospy.loginfo = lambda a: print(a)
    rospy.logwarn = lambda a: print(a)
    rospy.logerr = lambda a: print(a)

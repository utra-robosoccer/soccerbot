from os import path
from unittest.mock import MagicMock


import rclpy
import yaml


def set_rosparam_from_yaml_file(param_path: str = "", delete_params=True, convert_logs_to_prints=True):
    if self.get_node_uri() is None:
        self.init_node("test")

    if convert_logs_to_prints:
        self.set_param("/use_sim_time", False)

        self.get_logger().error = lambda a, b: None
        self.loginfo = lambda a: print(a)
        self.logwarn = lambda a: print(a)
        self.logerr = lambda a: print(a)

    if delete_params:
        rosparam.delete_param("/")
    if path.exists(param_path):
        with open(param_path) as f:
            param_info = yaml.safe_load(f)
            rosparam.upload_params(self.get_namespace(), param_info)

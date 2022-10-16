from typing import Dict, List

param_paths: Dict[str, List[str]] = {}


def mock_ros(robot_model="bez1", real_robot=False, param_path=None):
    from os.path import exists
    from unittest.mock import MagicMock

    import yaml

    if not real_robot:
        import rospy

        rospy.Time = MagicMock()
        joint_state = MagicMock()
        joint_state.position = [0.0] * 18
        rospy.wait_for_message = MagicMock(return_value=joint_state)
        rospy.init_node("test")
        rospy.set_param("/use_sim_time", False)
        rospy.Time.now = MagicMock(return_value=0)
        rospy.get_namespace = MagicMock(return_value="/robot1/")
        rospy.Duration = lambda a: a
        rospy.loginfo_throttle = lambda a, b: None
        rospy.loginfo = lambda a: print(a)
        rospy.logwarn = lambda a: print(a)
        rospy.logerr = lambda a: print(a)
    else:
        import rospy

    if param_path is not None:
        if not exists(param_path):
            print(f"Config Path {param_path} does not exist")
        else:
            if robot_model not in param_paths:
                param_paths[robot_model] = []
            if param_path not in param_paths[robot_model]:
                param_paths[robot_model].append(param_path)

    def get_param(a, b=None):
        a = a.lstrip("~")
        if a == "robot_model":
            return robot_model

        if robot_model not in param_paths:
            return b
        for param_path in param_paths[robot_model]:
            with open(param_path, "r") as g:
                y = yaml.safe_load(g)
                found = True
                for c in a.split("/"):
                    if y is None or c not in y:
                        found = False
                        break
                    y = y[c]
                if found:
                    return y

        return b

    rospy.get_param = get_param

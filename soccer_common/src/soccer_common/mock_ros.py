def mock_ros(robot_model, real_robot, config_path):
    import sys
    from os.path import exists
    from unittest.mock import MagicMock

    import yaml

    if not real_robot:
        sys.modules["rospy"] = MagicMock()
        sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
        import rospy

        rospy.Time = MagicMock()
        joint_state = MagicMock()
        joint_state.position = [0.0] * 18
        rospy.wait_for_message = MagicMock(return_value=joint_state)
        rospy.loginfo_throttle = lambda a, b: None
    else:
        import rospy

    def get_param(a, b=None):
        a = a.lstrip("~")
        if a == "robot_model":
            return robot_model

        if not exists(config_path):
            print(f"{config_path} does not exist")
            return b

        with open(config_path, "r") as g:
            y = yaml.safe_load(g)
            for c in a.split("/"):
                if y is None or c not in y:
                    return b
                y = y[c]
            return y

    rospy.get_param = get_param
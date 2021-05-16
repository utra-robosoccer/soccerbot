import os
from soccer_pycontrol.src.transformation import Transformation

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import matplotlib as plt
plt.use('tkagg')

USE_ROS = True

if __name__ == '__main__':

    if USE_ROS:
        import rospy
        import soccerbot_controller_ros
        rospy.init_node("soccer_control")
        walker = soccerbot_controller_ros.SoccerbotControllerRos()
        walker.run()
    else:
        import soccerbot_controller
        walker = soccerbot_controller.SoccerbotController()
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([1, 0, 0]), show=False)
        walker.soccerbot.robot_path.show()
        walker.soccerbot.calculate_angles(show=True)
        walker.run()

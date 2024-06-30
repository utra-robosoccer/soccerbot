import os

import rospy
import tf
from nav_msgs.msg import Odometry
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.stabilize import Stabilize
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine
from soccer_pycontrol.walk_engine.walk_engine_ros.foot_step_planner_ros import (
    FootStepPlannerROS,
)


class WalkEngineROS(WalkEngine):
    def __init__(self, bez: BezROS):
        self.bez = bez
        self.step_planner = FootStepPlannerROS(
            walking_torso_height=self.bez.data.walking_torso_height, foot_center_to_floor=self.bez.data.foot_center_to_floor
        )  # TODO should this be past or should it get through rosparam

        self.pid = Stabilize()

        self.terminate_walk = False
        self.prepare_walk_time = rospy.get_param("prepare_walk_time", 2)

        self.t = 0

        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()

    def publishOdometry(self, time: rospy.Time):
        """
        Send the odometry of the robot to be used in localization by ROS
        """

        o = Odometry()
        o.header.stamp = time
        o.header.frame_id = os.environ["ROS_NAMESPACE"][1:] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"][1:] + "/base_link"
        o.pose.pose = self.step_planner.odom_pose.pose
        o.pose.pose.position.z = 0

        # fmt: off
        o.pose.covariance = [1e-4, 0, 0, 0, 0, 0,
                             0, 1e-4, 0, 0, 0, 0,
                             0, 0, 1e-18, 0, 0, 0,
                             0, 0, 0, 1e-18, 0, 0,
                             0, 0, 0, 0, 1e-18, 0,
                             0, 0, 0, 0, 0, 1e-2]
        # fmt: on
        self.odom_publisher.publish(o)

        # TODO this seems to be static  self.pose.position[2] = 0.34
        # Publish the height of the center of the torso of the robot (used for camera vision calculations and odometry)
        height = 0.34 + self.bez.data.foot_box[2] / 2 + self.bez.data.cleats_offset
        self.br.sendTransform(
            (0, 0, height),
            (0, 0, 0, 1),
            time,
            os.environ["ROS_NAMESPACE"] + "/torso",
            os.environ["ROS_NAMESPACE"] + "/base_footprint",
        )
        pass

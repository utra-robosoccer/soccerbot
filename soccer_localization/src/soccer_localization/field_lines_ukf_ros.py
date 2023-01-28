import os
from typing import Optional

import numpy as np
import rospy
import scipy
import sensor_msgs.point_cloud2 as pcl2
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2

from soccer_common import Transformation
from soccer_localization.field import Field
from soccer_localization.field_lines_ukf import FieldLinesUKF

# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
from soccer_msgs.msg import RobotState


class FieldLinesUKFROS(FieldLinesUKF):
    def __init__(self, map=Field()):
        super().__init__()

        self.initial_pose_initiated = False
        self.odom_subscriber = rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, self.odom_callback, queue_size=1)
        self.field_point_cloud_subscriber = rospy.Subscriber("field_point_cloud", PointCloud2, self.field_point_cloud_callback, queue_size=1)
        self.field_point_cloud_transformed_publisher = rospy.Publisher("field_point_cloud_transformed", PointCloud2, queue_size=1)
        self.initial_pose_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback, queue_size=1)
        self.amcl_pose_publisher = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, queue_size=1)
        self.map = map

        self.initial_pose = Transformation(pos_theta=[-4, -3.15, np.pi / 2])  # TODO get this
        self.ukf.x = self.initial_pose.pos_theta

        self.odom_t_previous = None

        self.br = tf.TransformBroadcaster()
        self.timestamp_last = rospy.Time(0)

        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.robot_state_callback)
        self.robot_state = RobotState()

        rospy.loginfo("Soccer Localization UKF initiated")

    def robot_state_callback(self, robot_state: RobotState):
        self.robot_state = robot_state

    def odom_callback(self, pose_msg: PoseWithCovarianceStamped):
        if self.robot_state.status not in [
            RobotState.STATUS_LOCALIZING,
            RobotState.STATUS_READY,
            RobotState.STATUS_DETERMINING_SIDE,
            RobotState.STATUS_WALKING,
        ]:
            return

        if self.odom_t_previous is None:
            self.odom_t_previous = Transformation(pose_with_covariance_stamped=pose_msg)
            self.odom_t_previous.orientation_euler = [self.odom_t_previous.orientation_euler[0], 0, 0]  # Needed to remove non yaw values
            return
        odom_t = Transformation(pose_with_covariance_stamped=pose_msg)
        odom_t.orientation_euler = [odom_t.orientation_euler[0], 0, 0]  # Needed to remove non yaw values

        diff_transformation: Transformation = scipy.linalg.inv(self.odom_t_previous) @ odom_t
        dt = odom_t.timestamp - self.odom_t_previous.timestamp
        dt_secs = dt.secs + dt.nsecs * 1e-9
        if dt_secs == 0:
            return

        if np.all(diff_transformation.pos_theta < 0.000001):
            self.ukf.Q = self.Q_do_nothing
            self.ukf.R = self.R_localizing
        elif self.robot_state.status in [RobotState.STATUS_LOCALIZING, RobotState.STATUS_READY]:
            self.ukf.Q = self.Q_localizing
            self.ukf.R = self.R_localizing
        else:
            self.ukf.Q = self.Q_walking
            self.ukf.R = self.R_walking

        self.predict(u=diff_transformation.pos_theta / dt_secs, dt=dt_secs)
        self.odom_t_previous = odom_t

        self.broadcast_tf_position(pose_msg.header.stamp)
        self.publish_amcl_pose(timestamp=pose_msg.header.stamp)

        return odom_t

    def field_point_cloud_callback(self, point_cloud_msg: PointCloud2):
        if self.robot_state.status not in [
            RobotState.STATUS_LOCALIZING,
            RobotState.STATUS_READY,
            RobotState.STATUS_DETERMINING_SIDE,
            RobotState.STATUS_WALKING,
        ]:
            return None, None, None

        stamp = point_cloud_msg.header.stamp
        point_cloud = pcl2.read_points_list(point_cloud_msg)
        point_cloud_array = np.array(point_cloud)
        current_transform = Transformation(pos_theta=self.ukf.x)
        offset_transform = self.map.matchPointsWithMap(current_transform, point_cloud_array)

        if offset_transform is not None:
            vo_transform = current_transform @ offset_transform
            vo_pos_theta = vo_transform.pos_theta
            self.update(vo_pos_theta)
            self.broadcast_tf_position(timestamp=stamp)
            self.broadcast_vo_transform_debug(vo_transform, point_cloud_msg)
            self.publish_amcl_pose(timestamp=stamp)

            return point_cloud_array, vo_transform, vo_pos_theta
        return None, None, None

    def broadcast_vo_transform_debug(self, vo_transform: Transformation, point_cloud: PointCloud2):
        self.br.sendTransform(
            vo_transform.position,
            vo_transform.quaternion,
            point_cloud.header.stamp,
            f"{os.environ.get('ROS_NAMESPACE', 'robot1')}/odom_vo",
            "world",
        )
        point_cloud.header.frame_id = f"{os.environ.get('ROS_NAMESPACE', '/robot1').replace('/', '')}/odom_vo"
        self.field_point_cloud_transformed_publisher.publish(point_cloud)

    def broadcast_tf_position(self, timestamp):
        if not self.initial_pose_initiated:
            return

        if self.odom_t_previous is None:
            rospy.logerr_throttle(1, "Odom not published")
            return

        # Prevent rebroadcasting same or older timestamp
        if timestamp <= self.timestamp_last:
            return
        else:
            self.timestamp_last = timestamp

        world_to_odom = Transformation(pos_theta=self.ukf.x) @ scipy.linalg.inv(self.odom_t_previous)

        self.br.sendTransform(
            world_to_odom.position,
            world_to_odom.quaternion,
            timestamp,
            f"{os.environ.get('ROS_NAMESPACE', 'robot1')}/odom",
            "world",
        )

    def publish_amcl_pose(self, timestamp):
        if not self.initial_pose_initiated:
            return
        amcl_pose = Transformation(pos_theta=self.ukf.x, pose_theta_covariance_array=self.ukf.P).pose_with_covariance_stamped
        amcl_pose.header.stamp = timestamp
        self.amcl_pose_publisher.publish(amcl_pose)

    def initial_pose_callback(self, pose_stamped: PoseWithCovarianceStamped):
        self.initial_pose = Transformation(pose_with_covariance_stamped=pose_stamped)
        self.ukf.x = self.initial_pose.pos_theta
        self.ukf.P = self.initial_pose.pose_theta_covariance_array
        self.broadcast_tf_position(pose_stamped.header.stamp)
        self.initial_pose_initiated = True

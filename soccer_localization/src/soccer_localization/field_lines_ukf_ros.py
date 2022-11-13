import os
from typing import Optional

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2

# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
from soccer_common import Transformation
from soccer_localization.field import Field
from soccer_localization.field_lines_ukf import FieldLinesUKF


class FieldLinesUKFROS(FieldLinesUKF):
    def __init__(self):
        super().__init__()

        self.odom_subscriber = rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, self.odom_callback, queue_size=1)
        self.field_point_cloud_subscriber = rospy.Subscriber("field_point_cloud", PointCloud2, self.field_point_cloud_callback, queue_size=1)
        self.initial_pose_subscriber = rospy.Subscriber("initialpose", PoseStamped, self.initial_pose_callback, queue_size=1)
        self.map = Field()

        self.initial_pose = Transformation(pos_theta=[-4, -3.15, np.pi / 2])  # TODO get this
        self.ukf.x = self.initial_pose.pos_theta

        self.odom_t_previous = None

        self.br = tf.TransformBroadcaster()

        rospy.loginfo("Soccer Localization UFK initiated")

    def odom_callback(self, pose_msg: PoseWithCovarianceStamped):
        t = pose_msg.header.stamp
        if self.odom_t_previous is None:
            self.odom_t_previous = Transformation(pose_with_covariance_stamped=pose_msg)
            return
        odom_t = Transformation(pose_with_covariance_stamped=pose_msg)

        diff_transformation: Transformation = np.linalg.inv(self.odom_t_previous) @ odom_t
        dt = odom_t.timestamp - self.odom_t_previous.timestamp
        dt_secs = dt.secs + dt.nsecs * 1e-9
        if dt_secs == 0:
            return

        self.predict(u=diff_transformation.pos_theta / dt_secs, dt=dt_secs)
        self.odom_t_previous = odom_t

        self.broadcast_tf_position()
        return odom_t

    def field_point_cloud_callback(self, point_cloud: PointCloud2):
        point_cloud = pcl2.read_points_list(point_cloud)
        point_cloud_array = np.array(point_cloud)
        current_transform = Transformation(pos_theta=self.ukf.x)
        offset_transform = self.map.matchPointsWithMap(current_transform, point_cloud_array)

        if offset_transform is not None:
            vo_transform = offset_transform @ current_transform
            vo_pos_theta = vo_transform.pos_theta
            self.update(vo_pos_theta)
            self.broadcast_tf_position()
            return point_cloud_array, vo_transform, vo_pos_theta
        return None, None, None

    def broadcast_tf_position(self):
        t = Transformation(pos_theta=self.ukf.x)
        self.br.sendTransform(t.position, t.quaternion, rospy.Time.now(), f"{os.environ.get('ROS_NAMESPACE', 'robot1')}/odom", "world")

    def initial_pose_callback(self, pose_stamped: PoseStamped):
        self.initial_pose = Transformation(pose_stamped=pose_stamped)
        self.ukf.x = self.initial_pose.pos_theta

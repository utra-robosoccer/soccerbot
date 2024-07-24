import os

import rospy
import tf
from sensor_msgs.msg import Imu
from soccer_pycontrol.model.sensors import Sensors

from soccer_common import Transformation


class SensorsROS(Sensors):
    def __init__(self):
        self.imu_subscriber = rospy.Subscriber("imu_filtered", Imu, self.imu_callback, queue_size=1)
        self.imu_ready = False
        self.tf_listener = tf.TransformListener()

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.imu_msg = msg
        self.imu_ready = True
        # TODO rework imu its a bit weird

    def get_imu(self):
        """
        Gets the IMU at the IMU link location.

        :return: calculated orientation of the center of the torso of the robot
        """

        assert self.imu_ready
        return Transformation(
            [0, 0, 0],
            [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w,
            ],
        )

    def get_pose(self, footprint_name="/base_footprint_gt"):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("world", os.environ["ROS_NAMESPACE"] + footprint_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return False

        return Transformation(position=trans, quaternion=rot).pos_theta

    def get_foot_pressure_sensors(self, floor):
        # TODO subscribe to foot pressure sensors
        pass

    # TODO should add get pose

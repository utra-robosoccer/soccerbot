import rospy
import scipy
import tf
import tf2_py
from rospy import Subscriber
from sensor_msgs.msg import CameraInfo
from soccer_object_detection.camera.camera_calculations import CameraCalculations
from tf import TransformListener

from soccer_common import Transformation


# TODO fix this up
class CameraCalculationsRos(CameraCalculations):
    def __init__(self, robot_name: str):
        super(CameraCalculationsRos, self).__init__()

        self.pose_base_link_straight = Transformation()  #: Pose of the camera

        self.robot_name = robot_name  #: Name of the robot
        self.camera_info_subscriber = Subscriber("/" + robot_name + "/camera/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = TransformListener()

        self.init_time = rospy.Time.now()

    def camera_info_callback(self, camera_info: CameraInfo):
        """
        Callback function for the camera info subscriber

        :param camera_info: from the camera info topic
        """
        self.camera_info = camera_info

    def reset_position(self, from_world_frame=False, timestamp=rospy.Time(0), camera_frame="/camera", skip_if_not_found=False):
        """
        Resets the position of the camera, it uses a series of methods that fall back on each other to get the location of the camera

        :param from_world_frame: If this is set to true, the camera position transformation will be from the world instead of the robot odom frame
        :param timestamp: What time do we want the camera tf frame, rospy.Time(0) if get the latest transform
        :param camera_frame: The name of the camera frame
        :param skip_if_not_found: If set to true, then will not wait if it cannot find the camera transform after the specified duration (1 second), it will just return
        """
        try:

            time_stamp1 = self.tf_listener.getLatestCommonTime("robot1/left_foot", "robot1/head")

            (trans1, rot) = self.tf_listener.lookupTransform("robot1/left_foot", "robot1/head", time_stamp1)
            self.pose = Transformation(position=trans1, quaternion=rot)
        except (
            tf2_py.LookupException,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_py.TransformException,
        ) as ex:
            rospy.logerr_throttle(5, "Unable to find transformation from world to ")
            pass
        # TODO lets make it relative for now
        # if from_world_frame:
        #     try:
        #         self.tf_listener.waitForTransform("world", self.robot_name + camera_frame, timestamp, rospy.Duration(nsecs=1000000))
        #         (trans, rot) = self.tf_listener.lookupTransform("world", self.robot_name + camera_frame, timestamp)
        #         self.pose = Transformation(trans, rot)
        #         return
        #     except (
        #         tf2_py.LookupException,
        #         tf.LookupException,
        #         tf.ConnectivityException,
        #         tf.ExtrapolationException,
        #         tf2_py.TransformException,
        #     ) as ex:
        #         rospy.logerr_throttle(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")
        #         pass
        # else:
        #
        #     try:
        #         # Find the odom to base_footprint and publish straight base footprint
        #         self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp, rospy.Duration(secs=1))
        #         (trans, rot) = self.tf_listener.lookupTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp)
        #         world_to_base_link = Transformation(trans, rot)
        #         e = world_to_base_link.orientation_euler
        #         e[1] = 0
        #         e[2] = 0
        #         world_to_base_link.orientation_euler = e
        #         self.pose_base_link_straight = world_to_base_link
        #
        #         # Calculate the camera transformation
        #         self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + camera_frame, timestamp, rospy.Duration(secs=1))
        #         (trans, rot) = self.tf_listener.lookupTransform(self.robot_name + "/odom", self.robot_name + camera_frame, timestamp)
        #         world_to_camera = Transformation(trans, rot)
        #
        #         camera_to_base_link = scipy.linalg.inv(world_to_base_link) @ world_to_camera
        #
        #         self.pose = camera_to_base_link
        #         return
        #     except (
        #         tf2_py.LookupException,
        #         tf.LookupException,
        #         tf.ConnectivityException,
        #         tf.ExtrapolationException,
        #         tf2_py.TransformException,
        #     ) as ex:
        #         rospy.logerr_throttle(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")

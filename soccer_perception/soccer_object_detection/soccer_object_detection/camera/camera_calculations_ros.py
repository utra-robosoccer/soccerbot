import rclpy
from geometry_msgs.msg import TransformStamped

# import tf
# import tf2_py
# from rospy import create_subscription
from sensor_msgs.msg import CameraInfo
from soccer_object_detection.camera.camera_calculations import CameraCalculations
from tf2_ros import Buffer, TransformListener

from soccer_common.transformation import Transformation

# from tf import TransformListener

# TODO fix this up
class CameraCalculationsRos(CameraCalculations):
    def __init__(self, node, robot_name: str):
        super().__init__()
        self.node = node

        self.pose_base_link_straight = Transformation()  #: Pose of the camera

        self.robot_name = robot_name  #: Name of the robot
        # self.camera_info_create_subscription = create_subscription("/camera/camera_info", CameraInfo, self.camera_info_callback)
        self.subscription = self.node.create_subscription(CameraInfo, "camera/camera_info", self.camera_info_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # self.init_time = self.get_clock().now()
        self.init_time = node.get_clock().now()

    def camera_info_callback(self, camera_info: CameraInfo):
        """
        Callback function for the camera info create_subscription

        :param camera_info: from the camera info topic
        """
        self.camera_info = camera_info

    def reset_position(self, timestamp=None, camera_frame="camera", skip_if_not_found=False):
        """
        Resets the position of the camera, it uses a series of methods that fall back on each other to get the location of the camera

        :param from_world_frame: If this is set to true, the camera position transformation will be from the world instead of the robot odom frame
        :param timestamp: What time do we want the camera tf frame, self.Time(0) if get the latest transform
        :param camera_frame: The name of the camera frame
        :param skip_if_not_found: If set to true, then will not wait if it cannot find the camera transform after the specified duration (1 second), it will just return
        """
        try:
            if timestamp is None:
                timestamp = rclpy.time.Time()

            if self.tf_buffer.can_transform("left_foot", "head", timestamp):
                transform_stamaped = self.tf_buffer.lookup_transform("left_foot", "head", timestamp)
                trans = transform_stamaped.transform.translation
                rot = transform_stamaped.transform.rotation

                self.pose = Transformation(position=[0, 0, trans.z], quaternion=[rot.x, rot.y, rot.z, rot.w])

                e = self.pose.orientation_euler
                e[0] *= 1
                self.pose.orientation_euler = e

        except Exception as e:
            self.node.get_logger().warn("Failed to transform the camera to the world frame")

        #     time_stamp1 = self.tf_listener.getLatestCommonTime("left_foot", "head")
        #
        #     (trans1, rot) = self.tf_listener.lookupTransform("left_foot", "head", time_stamp1)
        #
        #     self.pose = Transformation(position=[0,0,trans1[2]], quaternion=rot)
        #     self.pose.orientation_euler = 1 * self.pose.orientation_euler
        # except (
        #     tf2_py.LookupException,
        #     tf.LookupException,
        #     tf.ConnectivityException,
        #     tf.ExtrapolationException,
        #     tf2_py.TransformException,
        # ) as ex:
        #     self.get_logger().error(5, "Unable to find transformation from world to ")
        #     pass
        # TODO lets make it relative for now
        # if from_world_frame:
        #     try:
        #         self.tf_listener.waitForTransform("world", self.robot_name + camera_frame, timestamp, self.Duration(nsecs=1000000))
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
        #         self.get_logger().error(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")
        #         pass
        # else:
        #
        #     try:
        #         # Find the odom to base_footprint and publish straight base footprint
        #         self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp, self.Duration(secs=1))
        #         (trans, rot) = self.tf_listener.lookupTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp)
        #         world_to_base_link = Transformation(trans, rot)
        #         e = world_to_base_link.orientation_euler
        #         e[1] = 0
        #         e[2] = 0
        #         world_to_base_link.orientation_euler = e
        #         self.pose_base_link_straight = world_to_base_link
        #
        #         # Calculate the camera transformation
        #         self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + camera_frame, timestamp, self.Duration(secs=1))
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
        #         self.get_logger().error(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")

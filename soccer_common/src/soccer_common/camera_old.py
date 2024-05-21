from functools import cached_property

import numpy as np
import rospy
import scipy
import tf
import tf2_py
from rospy import Subscriber
from sensor_msgs.msg import CameraInfo
from tf import TransformListener
from tf.transformations import *

from soccer_common.transformation import Transformation


class Camera:
    """
    This is a reusable class that instantiates an instance of a Camera object that listens to the camera related topics
    related to a robot and has useful functions that use geometry to determine the 3d/2d projection and location of things

    """

    HORIZONTAL_FOV = 1.39626

    def __init__(self, robot_name: str):
        """
        Initializes the camera object

        :param robot_name: Name of the robot, to be used in subscribers
        """

        self.robot_name = robot_name  #: Name of the robot
        self.pose = Transformation()  #: Pose of the camera
        self.pose_base_link_straight = Transformation()  #: Pose of the camera
        self.camera_info = None  #: Camera info object recieved from the subscriber
        self.horizontalFOV = Camera.HORIZONTAL_FOV
        self.focal_length = 3.67  #: Focal length of the camera (meters) distance to the camera plane as projected in 3D

        self.camera_info_subscriber = Subscriber("/" + robot_name + "/camera/camera_info", CameraInfo, self.cameraInfoCallback)

        self.tf_listener = TransformListener()

        self.init_time = rospy.Time.now()

    def ready(self) -> bool:
        """
        Function to determine when the camera object has recieved the necessary information and is ready to be used

        :return: True if the camera is ready, else False
        """
        return self.pose is not None and self.resolution_x is not None and self.resolution_y is not None and self.camera_info is not None

    def reset_position(self, from_world_frame=False, timestamp=rospy.Time(0), camera_frame="/camera", skip_if_not_found=False):
        """
        Resets the position of the camera, it uses a series of methods that fall back on each other to get the location of the camera

        :param from_world_frame: If this is set to true, the camera position transformation will be from the world instead of the robot odom frame
        :param timestamp: What time do we want the camera tf frame, rospy.Time(0) if get the latest transform
        :param camera_frame: The name of the camera frame
        :param skip_if_not_found: If set to true, then will not wait if it cannot find the camera transform after the specified duration (1 second), it will just return
        """
        if from_world_frame:
            try:
                self.tf_listener.waitForTransform("world", self.robot_name + camera_frame, timestamp, rospy.Duration(nsecs=1000000))
                (trans, rot) = self.tf_listener.lookupTransform("world", self.robot_name + camera_frame, timestamp)
                print(trans)
                print(rot)
                self.pose = Transformation(trans, rot)
                return
            except (
                tf2_py.LookupException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                tf2_py.TransformException,
            ) as ex:
                rospy.logerr_throttle(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")
                pass
        else:

            try:
                # Find the odom to base_footprint and publish straight base footprint
                self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp, rospy.Duration(secs=1))
                (trans, rot) = self.tf_listener.lookupTransform(self.robot_name + "/odom", self.robot_name + "/base_footprint", timestamp)
                world_to_base_link = Transformation(trans, rot)
                e = world_to_base_link.orientation_euler
                e[1] = 0
                e[2] = 0
                world_to_base_link.orientation_euler = e
                self.pose_base_link_straight = world_to_base_link

                # Calculate the camera transformation
                self.tf_listener.waitForTransform(self.robot_name + "/odom", self.robot_name + camera_frame, timestamp, rospy.Duration(secs=1))
                (trans, rot) = self.tf_listener.lookupTransform(self.robot_name + "/odom", self.robot_name + camera_frame, timestamp)
                world_to_camera = Transformation(trans, rot)

                camera_to_base_link = scipy.linalg.inv(world_to_base_link) @ world_to_camera

                self.pose = camera_to_base_link
                return
            except (
                tf2_py.LookupException,
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                tf2_py.TransformException,
            ) as ex:
                rospy.logerr_throttle(5, f"Unable to find transformation from world to {self.robot_name + camera_frame}")
                pass

    def cameraInfoCallback(self, camera_info: CameraInfo):
        """
        Callback function for the camera info subscriber

        :param camera_info: from the camera info topic
        """
        self.camera_info = camera_info

    @cached_property
    def resolution_x(self) -> int:
        """
        The X resolution of the camera or the width of the screen in pixels

        :return: width in pixels
        """
        return self.camera_info.width

    @cached_property
    def resolution_y(self):
        """
        The Y resolution of the camera or the height of the screen in pixels

        :return: height in pixels
        """
        return self.camera_info.height

    def findFloorCoordinate(self, pos: [int]) -> [int]:
        """
        From a camera pixel, get a coordinate on the floor

        :param pos: The position on the screen in pixels (x, y)
        :return: The 3D coordinate of the pixel as projected to the floor
        """
        tx, ty = self.imageToWorldFrame(pos[0], pos[1])
        pixel_pose = Transformation(position=(self.focal_length, tx, ty))
        camera_pose = self.pose
        pixel_world_pose = camera_pose @ pixel_pose
        ratio = (camera_pose.position[2] - pixel_world_pose.position[2]) / self.pose.position[2]  # TODO Fix divide by 0 problem
        x_delta = (pixel_world_pose.position[0] - camera_pose.position[0]) / ratio
        y_delta = (pixel_world_pose.position[1] - camera_pose.position[1]) / ratio

        return [x_delta + camera_pose.position[0], y_delta + camera_pose.position[1], 0]

    def findCameraCoordinate(self, pos: [int]) -> [int]:
        """
        From a 3d position on the field, get the camera coordinate, opposite of :func:`~soccer_common.Camera.findFloorCoordinate`

        :param pos: The 3D coordinate of the object
        :return: The 2D pixel (x, y) on the camera, if the object was projected on the camera
        """
        pos3d = Transformation(pos)
        camera_pose = self.pose
        pos3d_tr = np.linalg.inv(camera_pose) @ pos3d

        return self.findCameraCoordinateFixedCamera(pos3d_tr.position)

    def findCameraCoordinateFixedCamera(self, pos: [int]) -> [int]:
        """
        Helper function for :func:`~soccer_common.Camera.findCameraCoordinate`, finds the camera coordinate if the camera were fixed at the origin

        :param pos: The 3D coordinate of the object
        :return: The 2D pixel (x, y) on the camera, if the object was projected on the camera and the camera is placed at the origin
        """

        pos = Transformation(pos)

        ratio = self.focal_length / pos.position[0]

        tx = pos.position[1] * ratio
        ty = pos.position[2] * ratio
        x, y = self.worldToImageFrame(tx, ty)
        return [x, y]

    @cached_property
    def verticalFOV(self):
        """
        The vertical field of vision of the camera.
        See `Field of View <https://en.wikipedia.org/wiki/Field_of_view>`_
        """
        return 2 * math.atan(math.tan(self.horizontalFOV * 0.5) * (self.resolution_y / self.resolution_x))

    @cached_property
    def imageSensorHeight(self):
        """
        The height of the image sensor (m)
        """
        return math.tan(self.verticalFOV / 2.0) * 2.0 * self.focal_length

    @cached_property
    def imageSensorWidth(self):
        """
        The width of the image sensor (m)
        """
        return math.tan(self.horizontalFOV / 2.0) * 2.0 * self.focal_length

    @cached_property
    def pixelHeight(self):
        """
        The height of a pixel in real 3d measurements (m)
        """
        return self.imageSensorHeight / self.resolution_y

    @cached_property
    def pixelWidth(self):
        """
        The wdith of a pixel in real 3d measurements (m)
        """
        return self.imageSensorWidth / self.resolution_x
        pass

    def imageToWorldFrame(self, pixel_x: int, pixel_y: int) -> tuple:
        """
        From image pixel coordinates, get the coordinates of the pixel as if they have been projected ot the camera plane, which is
        positioned at (0,0) in 3D world coordinates
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163680c589a_0_0

        :param pixel_x: x pixel of the camera
        :param pixel_y: y pixel of the camera
        :return: 3D position (X, Y) of the pixel in meters
        """
        return (
            (self.resolution_x / 2.0 - (pixel_x + 0.5)) * self.pixelWidth,
            (self.resolution_y / 2.0 - (pixel_y + 0.5)) * self.pixelHeight,
        )

    def worldToImageFrame(self, pos_x: float, pos_y: float) -> tuple:
        """
        Reverse function for  :func:`~soccer_common.Camera.imageToWorldFrame`, takes the 3D world coordinates of the camera plane
        and returns pixels

        :param pos_x: X position of the pixel on the world plane in meters
        :param pos_y: Y position of the pixel on the world plane in meters
        :return: Tuple (x, y) of the pixel coordinates of in the image
        """
        return (
            (self.resolution_x / 2.0 + pos_x / self.pixelWidth) - 0.5,
            (self.resolution_y / 2.0 + pos_y / self.pixelHeight) - 0.5,
        )

    def calculateBoundingBoxesFromBall(self, ball_position: Transformation, ball_radius: float = 0.07):
        """
        Takes a 3D ball transformation and returns the bounding boxes of the ball if seen on camera

        :param ball_position: 3D coordinates of the ball stored in the :class:`Transformation` format
        :param ball_radius: The radious of the ball in centimeters
        :return: The bounding boxes of the ball on the camera in the format [[x1,y1], [x1,y1]] which are the top left
        and bottom right of the bounding box respectively
        """

        camera_pose = self.pose
        pos3d_tr = np.linalg.inv(camera_pose) @ ball_position

        x = pos3d_tr.position[0]
        y = -pos3d_tr.position[1]
        z = -pos3d_tr.position[2]
        r = ball_radius

        thetay = math.atan2(y, x)
        dy = math.sqrt(x**2 + y**2)
        phiy = math.asin(r / dy)

        xyfar = [x - math.sin(thetay + phiy) * r, y + math.cos(thetay + phiy) * r]
        xynear = [x + math.sin(thetay - phiy) * r, y - math.cos(thetay - phiy) * r]

        thetaz = math.atan2(z, x)
        dz = math.sqrt(x**2 + z**2)
        phiz = math.asin(r / dz)

        xzfar = [x - math.sin(thetaz + phiz) * r, z + math.cos(thetaz + phiz) * r]
        xznear = [x + math.sin(thetaz - phiz) * r, z - math.cos(thetaz - phiz) * r]

        ball_right_point = [xyfar[0], xyfar[1], z]
        ball_left_point = [xynear[0], xynear[1], z]
        ball_bottom_point = [xzfar[0], y, xzfar[1]]
        ball_top_point = [xznear[0], y, xznear[1]]

        ball_left_point_cam = self.findCameraCoordinateFixedCamera(ball_left_point)
        ball_right_point_cam = self.findCameraCoordinateFixedCamera(ball_right_point)
        ball_top_point_cam = self.findCameraCoordinateFixedCamera(ball_top_point)
        ball_bottom_point_cam = self.findCameraCoordinateFixedCamera(ball_bottom_point)

        left_border_x = ball_left_point_cam[0]
        right_border_x = ball_right_point_cam[0]
        top_border_y = ball_top_point_cam[1]
        bottom_border_y = ball_bottom_point_cam[1]

        bounding_box = [[left_border_x, top_border_y], [right_border_x, bottom_border_y]]

        return bounding_box

    def calculateBallFromBoundingBoxes(self, ball_radius: float = 0.07, bounding_boxes: [float] = []) -> Transformation:
        """
        Reverse function for  :func:`~soccer_common.Camera.calculateBoundingBoxesFromBall`, takes the bounding boxes
        of the ball as seen on the camera and return the 3D position of the ball assuming that the ball is on the ground

        :param ball_radius: The radius of the ball in meters
        :param bounding_boxes: The bounding boxes of the ball on the camera in the format [[x1,y1], [x1,y1]] which are the top left and bottom right of the bounding box respectively
        :return: 3D coordinates of the ball stored in the :class:`Transformation` format
        """

        # bounding boxes [(y1, z1), (y2, z2)]
        r = ball_radius

        y1 = bounding_boxes[0][0]
        z1 = bounding_boxes[0][1]
        y2 = bounding_boxes[1][0]
        z2 = bounding_boxes[1][1]

        # Assuming the ball is a sphere, the bounding box must be a square, averaging the borders
        ym = (y1 + y2) / 2
        zm = (z1 + z2) / 2
        length = z2 - z1
        width = y2 - y1
        y1 = ym - (width / 2)
        z1 = zm - (length / 2)
        y2 = ym + (width / 2)
        z2 = zm + (length / 2)

        y1w, z1w = self.imageToWorldFrame(y1, z1)
        y2w, z2w = self.imageToWorldFrame(y2, z2)
        y1w = -y1w
        z1w = -z1w
        y2w = -y2w
        z2w = -z2w

        f = self.focal_length

        thetay1 = math.atan2(y1w, f)
        thetay2 = math.atan2(y2w, f)

        thetayy = (thetay2 - thetay1) / 2
        thetay = thetay1 + thetayy

        dy = r / math.sin(thetayy)

        xy = (math.cos(thetay) * dy, math.sin(thetay) * dy)

        thetaz1 = math.atan2(z1w, f)
        thetaz2 = math.atan2(z2w, f)

        thetazz = (thetaz2 - thetaz1) / 2
        thetaz = thetaz1 + thetazz

        dz = r / math.sin(thetazz)

        xz = (math.cos(thetaz) * dz, math.sin(thetaz) * dz)

        ball_x = xy[0]
        ball_y = xy[1]
        ball_z = xz[1]

        tr = Transformation([ball_x, -ball_y, -ball_z])
        tr_cam = self.pose @ tr

        return tr_cam

    def calculateHorizonCoverArea(self) -> int:
        """
        Given the camera's position, return the area that is covered by the horizon (that is not the field area) in pixels from the top position
        :return: Pixel length from the top of the image to the point where it meets the horizon
        """

        pitch = self.pose.orientation_euler[1]
        d = math.sin(pitch) * self.focal_length

        (r, h) = self.worldToImageFrame(0, -d)
        return int(min(max(0, h), self.resolution_y))


from unittest.mock import MagicMock

if __name__ == "__main__":
    rospy.init_node("camera_test")
    c = Camera("robot")
    c.reset_position = MagicMock(from_world_frame=True)
    print(c.pose)
    print(Transformation([0, 0, 0], [0, 0, 0, 1]))

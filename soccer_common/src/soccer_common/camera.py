import numpy as np
import rospy
import tf
import tf2_py
from rospy import Subscriber
from sensor_msgs.msg import CameraInfo
from tf import TransformListener
from tf.transformations import *

from soccer_common.transformation import Transformation


class Camera:
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        self.pose = Transformation()
        self.camera_info = None
        self.diagonal_fov = 1.523
        self.focal_length = 3.67  # 3.67

        self.camera_info_subscriber = Subscriber("/" + robot_name + "/camera/camera_info", CameraInfo, self.cameraInfoCallback)

        self.tf_listener = TransformListener()

        self.init_time = rospy.Time.now()

    def ready(self) -> bool:
        return self.pose is not None and self.resolution_x is not None and self.resolution_y is not None and self.camera_info is not None

    def reset_position(self, from_world_frame=False, timestamp=rospy.Time(0), camera_frame="/camera", skip_if_not_found=False):

        if from_world_frame:
            base_frame = "world"
            target_frame = self.robot_name + camera_frame
            timeout_duration = rospy.Duration(nsecs=1000000)
        else:
            base_frame = self.robot_name + "/odom"
            target_frame = self.robot_name + camera_frame
            timeout_duration = rospy.Duration(secs=1)

        # First if the timestamp transformation can be found
        if self.tf_listener.canTransform(base_frame, target_frame, timestamp):
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, target_frame, timestamp)
            self.pose = Transformation(trans, rot)
            return

        # Then wait the specified time
        try:
            self.tf_listener.waitForTransform(base_frame, target_frame, timestamp, timeout_duration)
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, target_frame, timestamp)
            self.pose = Transformation(trans, rot)
            return
        except (
            tf2_py.LookupException,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_py.TransformException,
        ) as ex:
            pass

        # Then just get the first transform
        try:
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
            self.pose = Transformation(trans, rot)
            return
        except (
            tf2_py.LookupException,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_py.TransformException,
        ) as ex:
            pass

        if skip_if_not_found:
            return

        # Then try waiting for the first transform
        try:
            self.tf_listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(60))
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, target_frame, timestamp)
            self.pose = Transformation(trans, rot)
            return
        except (
            tf2_py.LookupException,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_py.TransformException,
        ) as ex:
            rospy.logerr_throttle(10, f"Cant find TF between {base_frame} and {target_frame} at {timestamp}")
            rospy.logerr_throttle(10, str(ex))

    def cameraInfoCallback(self, camera_info: CameraInfo):
        self.camera_info = camera_info

    @property
    def resolution_x(self):
        return self.camera_info.width

    @property
    def resolution_y(self):
        return self.camera_info.height

    # From a camera pixel, get a coordinate on the floor
    def findFloorCoordinate(self, pos: [int]) -> [int]:
        tx, ty = self.imageToWorldFrame(pos[0], pos[1])
        pixel_pose = Transformation((self.focal_length, tx, ty), (0, 0, 0, 1))
        camera_pose = self.pose
        pixel_world_pose = camera_pose @ pixel_pose
        ratio = (self.pose.position[2] - pixel_world_pose.position[2]) / self.pose.position[2]  # Fix divide by 0 problem
        x_delta = (pixel_world_pose.position[0] - self.pose.position[0]) / ratio
        y_delta = (pixel_world_pose.position[1] - self.pose.position[1]) / ratio

        return [x_delta + camera_pose.position[0], y_delta + camera_pose.position[1], 0]

    # From a 3d position on the field, get the camera coordinate
    def findCameraCoordinate(self, pos: [int]) -> [int]:
        pos3d = Transformation(pos)
        camera_pose = self.pose
        pos3d_tr = np.linalg.inv(camera_pose) @ pos3d

        return self.findCameraCoordinateFixedCamera(pos3d_tr.position)

    def findCameraCoordinateFixedCamera(self, pos: [int]) -> [int]:
        pos = Transformation(pos)

        ratio = self.focal_length / pos.position[0]

        tx = pos.position[1] * ratio
        ty = pos.position[2] * ratio
        x, y = self.worldToImageFrame(tx, ty)
        return [x, y]

    def verticalFOV(self):
        f = math.sqrt(self.resolution_x**2 + self.resolution_y**2) / (2 * (1 / math.tan(self.diagonal_fov / 2)))
        return 2 * math.atan2(self.resolution_y / 2.0, f)

    def horizontalFOV(self):
        f = math.sqrt(self.resolution_x**2 + self.resolution_y**2) / (2 * (1 / math.tan(self.diagonal_fov / 2)))
        return 2 * math.atan2(self.resolution_x / 2, f)

    def imageSensorHeight(self):
        return math.tan(self.verticalFOV() / 2.0) * 2.0 * self.focal_length

    def imageSensorWidth(self):
        return math.tan(self.horizontalFOV() / 2.0) * 2.0 * self.focal_length

    def pixelHeight(self):
        return self.imageSensorHeight() / self.resolution_y

    def pixelWidth(self):
        return self.imageSensorWidth() / self.resolution_x
        pass

    def imageToWorldFrame(self, pos_x: int, pos_y: int) -> tuple:
        return (
            (self.resolution_x / 2.0 - pos_x) * self.pixelWidth(),
            (self.resolution_y / 2.0 - pos_y) * self.pixelHeight(),
        )

    def worldToImageFrame(self, pos_x: float, pos_y: float) -> tuple:
        return (
            (self.resolution_x / 2.0 + pos_x / self.pixelWidth()),
            (self.resolution_y / 2.0 + pos_y / self.pixelHeight()),
        )

    def calculateBoundingBoxesFromBall(self, ball_position: Transformation, ball_radius: float = 0.07):
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
        pitch = self.pose.orientation_euler[1]
        d = math.sin(pitch) * self.focal_length

        (r, h) = self.worldToImageFrame(0, -d)
        return int(max(0, h))

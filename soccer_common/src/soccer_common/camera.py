import math
from functools import cached_property

import numpy as np

from soccer_common.transformation import Transformation


class Camera:

    HORIZONTAL_FOV = 1.39626

    def __init__(self):
        self.pose = Transformation()
        self.camera_info = None
        self.horizontalFOV = Camera.HORIZONTAL_FOV
        self.focal_length = 3.67  #: Focal length of the camera (meters) distance to the camera plane as projected in 3D

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

        theta_y = math.atan2(y, x)
        dy = math.sqrt(x**2 + y**2)
        phi_y = math.asin(r / dy)

        xy_far = [x - math.sin(theta_y + phi_y) * r, y + math.cos(theta_y + phi_y) * r]
        xy_near = [x + math.sin(theta_y - phi_y) * r, y - math.cos(theta_y - phi_y) * r]

        theta_z = math.atan2(z, x)
        dz = math.sqrt(x**2 + z**2)
        phi_z = math.asin(r / dz)

        xz_far = [x - math.sin(theta_z + phi_z) * r, z + math.cos(theta_z + phi_z) * r]
        xz_near = [x + math.sin(theta_z - phi_z) * r, z - math.cos(theta_z - phi_z) * r]

        ball_right_point = [xy_far[0], xy_far[1], z]
        ball_left_point = [xy_near[0], xy_near[1], z]
        ball_bottom_point = [xz_far[0], y, xz_far[1]]
        ball_top_point = [xz_near[0], y, xz_near[1]]

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

        theta_y1 = math.atan2(y1w, f)
        theta_y2 = math.atan2(y2w, f)

        theta_yy = (theta_y2 - theta_y1) / 2
        theta_y = theta_y1 + theta_yy

        dy = r / math.sin(theta_yy)

        xy = (math.cos(theta_y) * dy, math.sin(theta_y) * dy)

        theta_z1 = math.atan2(z1w, f)
        theta_z2 = math.atan2(z2w, f)

        theta_zz = (theta_z2 - theta_z1) / 2
        theta_z = theta_z1 + theta_zz

        dz = r / math.sin(theta_zz)

        xz = (math.cos(theta_z) * dz, math.sin(theta_z) * dz)

        ball_x = xy[0]
        ball_y = xy[1]
        ball_z = xz[1]

        tr = Transformation([ball_x, -ball_y, -ball_z])
        tr_cam = self.pose @ tr

        return tr_cam

    # CACHED PROPERTIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

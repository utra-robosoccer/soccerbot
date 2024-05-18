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

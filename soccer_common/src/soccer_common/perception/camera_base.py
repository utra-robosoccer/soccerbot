import math
from functools import cached_property

import numpy as np
from sensor_msgs.msg import CameraInfo

from soccer_common.transformation import Transformation


class CameraBase:
    # TODO maybe put into a common percpetion
    # TODO also could be split up into different files too many things

    def __init__(self, camera_info: CameraInfo = CameraInfo(height=480, width=640)):
        # TODO why does this need pose shouldnt all the calcualtions be in the robots relative frame and
        #  then transformed into the world frame that would make it a lot less relient on
        #  knowledge of its global position

        self.camera_info = camera_info
        self.horizontalFOV = 1.39626
        self.focal_length = 3.67  #: Focal length of the camera (meters) distance to the camera plane as projected in 3D

    def image_to_world_frame(self, pixel_x: int, pixel_y: int) -> tuple:
        """
        From image pixel coordinates, get the coordinates of the pixel as if they have been projected ot the camera plane, which is
        positioned at (0,0) in 3D world coordinates
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163680c589a_0_0

        :param pixel_x: x pixel of the camera
        :param pixel_y: y pixel of the camera
        :return: 3D position (X, Y) of the pixel in meters
        """
        return (
            (self.resolution_x / 2.0 - (pixel_x + 0.5)) * self.pixel_width,
            (self.resolution_y / 2.0 - (pixel_y + 0.5)) * self.pixel_height,
        )

    def world_to_image_frame(self, pos_x: float, pos_y: float) -> tuple:
        """
        Reverse function for  :func:`~soccer_common.Camera.imageToWorldFrame`, takes the 3D world coordinates of the camera plane
        and returns pixels

        :param pos_x: X position of the pixel on the world plane in meters
        :param pos_y: Y position of the pixel on the world plane in meters
        :return: Tuple (x, y) of the pixel coordinates of in the image
        """
        return (
            (self.resolution_x / 2.0 + pos_x / self.pixel_width) - 0.5,
            (self.resolution_y / 2.0 + pos_y / self.pixel_height) - 0.5,
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
    def vertical_fov(self):
        """
        The vertical field of vision of the camera.
        See `Field of View <https://en.wikipedia.org/wiki/Field_of_view>`_
        """
        return 2 * math.atan(math.tan(self.horizontalFOV * 0.5) * (self.resolution_y / self.resolution_x))

    @cached_property
    def image_sensor_height(self):
        """
        The height of the image sensor (m)
        """
        return math.tan(self.vertical_fov / 2.0) * 2.0 * self.focal_length

    @cached_property
    def image_sensor_width(self):
        """
        The width of the image sensor (m)
        """
        return math.tan(self.horizontalFOV / 2.0) * 2.0 * self.focal_length

    @cached_property
    def pixel_height(self):
        """
        The height of a pixel in real 3d measurements (m)
        """
        return self.image_sensor_height / self.resolution_y

    @cached_property
    def pixel_width(self):
        """
        The wdith of a pixel in real 3d measurements (m)
        """
        return self.image_sensor_width / self.resolution_x

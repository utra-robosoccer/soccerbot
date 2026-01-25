import math
from functools import cached_property

from sensor_msgs.msg import CameraInfo


class CameraBase:

    def __init__(self, camera_info: CameraInfo = CameraInfo(height=480, width=640)):
        self.camera_info = camera_info
        self.diagFOV = 1.36136 #1.380555
        # self.diagFOV = 1.380555
        self.focal_length = 0.00367 #24  #: Focal length of the camera (millimeters) distance to the camera plane as projected in 3D

        # self.focal_length = 24
        self.horizontal_aspect_orig = 1920
        self.vertical_aspect_orig = 1080

    def image_to_world_frame(self, pixel_x: int, pixel_y: int) -> tuple:
        """
        From image pixel coordinates, get the coordinates of the pixel as if they have been projected ot the camera plane, which is
        positioned at (0,0) in 3D world coordinates

        :param pixel_x: x pixel of the camera
        :param pixel_y: y pixel of the camera
        :return: 3D position (X, Y) of the pixel in meters
        """
        return (
            (self.horizontal_aspect / 2.0 - (pixel_x + 0.5)) * self.pixel_width,
            (self.vertical_aspect / 2.0 - (pixel_y + 0.5)) * self.pixel_height,
        )

    def world_to_image_frame(self, pos_x: float, pos_y: float) -> tuple:
        """
        Reverse function for  :func:`imageToWorldFrame`, takes the 3D world coordinates of the camera plane
        and returns pixels

        :param pos_x: X position of the pixel on the world plane in meters
        :param pos_y: Y position of the pixel on the world plane in meters
        :return: Tuple (x, y) of the pixel coordinates of in the image
        """
        return (
            (self.horizontal_aspect / 2.0 + pos_x / self.pixel_width) - 0.5,
            (self.vertical_aspect / 2.0 + pos_y / self.pixel_height) - 0.5,
        )

    # CACHED PROPERTIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @cached_property
    def horizontal_aspect(self) -> int:
        """
        The X resolution of the camera or the width of the screen in pixels

        :return: width in pixels
        """
        return self.camera_info.width

    @cached_property
    def vertical_aspect(self):
        """
        The Y resolution of the camera or the height of the screen in pixels

        :return: height in pixels
        """
        return self.camera_info.height

    @cached_property
    def diag_aspect(self) -> float:
        """
        The diag resolution of the camera or the width of the screen in pixels

        :return: width in pixels
        """
        return math.sqrt(self.horizontal_aspect_orig**2 + self.horizontal_aspect_orig**2) # TODO should this be rounded ?

    @cached_property
    def horizontal_fov(self):
        """
        The horizontal field of vision of the camera.
        See `Field of View <https://en.wikipedia.org/wiki/Field_of_view>`_
        """
        return 2 * math.atan(math.tan(self.diagFOV * 0.5) * (self.horizontal_aspect_orig / self.diag_aspect))

    @cached_property
    def vertical_fov(self):
        """
        The vertical field of vision of the camera.
        See `Field of View <https://en.wikipedia.org/wiki/Field_of_view>`_
        """
        return  2 * math.atan(math.tan(self.diagFOV * 0.5) * (self.horizontal_aspect_orig / self.diag_aspect))

    @cached_property
    def image_sensor_height(self):
        """
        The height of the image sensor (m). Useful for converting pixels to distance
        """

        return math.tan(self.vertical_fov / 2.0) * 2.0 * self.focal_length

    @cached_property
    def image_sensor_width(self):
        """
        The width of the image sensor (m). Useful for converting pixels to distance
        """
        return math.tan(self.horizontal_fov / 2.0) * 2.0 * self.focal_length

    @cached_property
    def pixel_height(self):
        """
        The height of a pixel in real 3d measurements (m). This is how we relate pixels to real distance
        """
        return self.image_sensor_height / self.vertical_aspect

    @cached_property
    def pixel_width(self):
        """
        The width of a pixel in real 3d measurements (m). This is how we relate pixels to real distance
        """
        return self.image_sensor_width / self.horizontal_aspect

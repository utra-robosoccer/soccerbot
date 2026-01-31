import math
from functools import cached_property

import numpy as np
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

    def pixel_to_image_plane(self, u: int, v: int) -> tuple:
        """
        From image plane pixel coordinates, get the coordinates of the pixel as if they have been projected to the image plan in 2D world coordinates at z=focal length

        :param u: u pixel of the camera
        :param v: v pixel of the camera
        :return: 2D position (x, y) of the pixel in meters
        """
        # return (
        #     (self.cx - (u + 0.5)) / self.pixel_per_meter_width*-1,
        #     (self.cy - (v + 0.5)) / self.pixel_per_meter_height*-1,
        # )

        return (
            ((u + 0.5) - self.cx ) / self.pixel_per_meter_width,
            ((v + 0.5) - self.cy) / self.pixel_per_meter_height,
        )

    def image_plane_to_pixel(self, pos_x: float, pos_y: float) -> tuple:
        """
        Reverse function for  :func:`pixel_to_image_plane`, takes the 3D world coordinates of the camera plane
        and returns pixels

        :param pos_x: X position of the pixel on the world plane in meters
        :param pos_y: Y position of the pixel on the world plane in meters
        :return: Tuple (x, y) of the pixel coordinates of in the image
        """
        # return (
        #     (self.cx + pos_x * self.pixel_per_meter_width) - 0.5,
        #     (self.cy + pos_y * self.pixel_per_meter_height) - 0.5,
        # )

        return (
            (self.cx + pos_x * self.pixel_per_meter_width) - 0.5,
            (self.cy + pos_y * self.pixel_per_meter_height) - 0.5,
        )

    # CACHED PROPERTIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @cached_property
    def intrinsic_matrix(self):
        # For our Carla camera alpha_u = alpha_v = alpha
        # alpha can be computed given the cameras field of view via
        fx = (self.horizontal_aspect / 2.0) / np.tan(self.horizontal_fov / 2.)
        # alpha1 = self.focal_length*self.horizontal_aspect/self.image_sensor_width
        fy = (self.vertical_aspect / 2.0) / np.tan(self.vertical_fov / 2.)
        # alpha2 = self.focal_length * self.vertical_aspect / self.image_sensor_height
        cx = self.cx
        cy = self.cy
        return np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1.0]])

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
    def cx(self) -> int:
        """
        The center x pixel coordinate of the camera

        :return: width in pixels
        """
        return int(self.horizontal_aspect / 2)

    @cached_property
    def cy(self) -> int:
        """
        The center y pixel coordinate of the camera

        :return: width in pixels
        """
        return int(self.vertical_aspect / 2)

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
    def pixel_per_meter_height(self):
        """
        Converts pixels to meters in 3d measurements (m/pixel). This is how we relate pixels to real distance
        """
        return self.vertical_aspect / self.image_sensor_height

    @cached_property
    def pixel_per_meter_width(self):
        """
        Converts pixels to meters in 3d measurements (m/pixel). This is how we relate pixels to real distance
        """
        return self.horizontal_aspect / self.image_sensor_width

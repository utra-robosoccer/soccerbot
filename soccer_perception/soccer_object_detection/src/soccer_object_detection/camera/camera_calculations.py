import math

import numpy as np
from soccer_object_detection.camera.camera_base import CameraBase

from soccer_common import Transformation


class CameraCalculations(CameraBase):
    def __init__(self):
        super(CameraCalculations, self).__init__()
        self.pose = Transformation()

    def calculate_horizon_cover_area(self) -> int:
        """
        Given the camera's position, return the area that is covered by the horizon (that is not the field area) in pixels from the top position
        :return: Pixel length from the top of the image to the point where it meets the horizon
        """
        # TODO verify this works cause there is some weird stuff going on in the resultant image
        # TODO the relative frame needs some more thought since this will break some things so
        #  will focus on basics before that section
        # TODO need to find a way to visualize or verify this works with out sim maybe a graph that could work
        pitch = self.pose.orientation_euler[1]
        d = math.sin(pitch) * self.focal_length

        (r, h) = self.world_to_image_frame(0, -d)
        return int(min(max(0, h), self.resolution_y))

    def reset_position(self, from_world_frame=False, camera_frame="/camera", skip_if_not_found=False):
        """
        Resets the position of the camera, it uses a series of methods that fall back on each other to get the location of the camera

        :param from_world_frame: If this is set to true, the camera position transformation will be from the world instead of the robot odom frame
        :param timestamp: What time do we want the camera tf frame, self.Time(0) if get the latest transform
        :param camera_frame: The name of the camera frame
        :param skip_if_not_found: If set to true, then will not wait if it cannot find the camera transform after the specified duration (1 second), it will just return
        """

        # same hardcoded values
        if from_world_frame:
            trans = [0, 0, 0.46]
            rot = [0, 0, 0, 1]
            self.pose = Transformation(trans, rot)
        else:
            trans = [0, 0, 0.46]  # TODO find init height
            rot = [0, 0, 0, 1]
            self.pose = Transformation(trans, rot)

    # TODO maybe in localization
    def find_floor_coordinate(self, pos: [int]) -> [int]:
        """
        From a camera pixel, get a coordinate on the floor

        :param pos: The position on the screen in pixels (x, y)
        :return: The 3D coordinate of the pixel as projected to the floor
        """
        # TODO this actually might need an accruate pose, but is it truly necessay
        # TODO verify the math
        # TODO should be easy to verify if the relative conversion works since you have the answers
        tx, ty = self.image_to_world_frame(pos[0], pos[1])
        pixel_pose = Transformation(position=(self.focal_length, tx, ty))
        camera_pose = self.pose
        pixel_world_pose = camera_pose @ pixel_pose
        ratio = (camera_pose.position[2] - pixel_world_pose.position[2]) / self.pose.position[2]  # TODO Fix divide by 0 problem
        x_delta = (pixel_world_pose.position[0] - camera_pose.position[0]) / ratio
        y_delta = (pixel_world_pose.position[1] - camera_pose.position[1]) / ratio

        return [x_delta + camera_pose.position[0], y_delta + camera_pose.position[1], 0]

    def find_camera_coordinate(self, pos: [int]) -> [int]:
        """
        From a 3d position on the field, get the camera coordinate, opposite of :func:`~soccer_common.Camera.findFloorCoordinate`

        :param pos: The 3D coordinate of the object
        :return: The 2D pixel (x, y) on the camera, if the object was projected on the camera
        """
        pos3d = Transformation(pos)
        camera_pose = self.pose
        pos3d_tr = np.linalg.inv(camera_pose) @ pos3d

        return self.find_camera_coordinate_fixed_camera(pos3d_tr.position)

    def find_camera_coordinate_fixed_camera(self, pos: [int]) -> [int]:
        """
        Helper function for :func:`~soccer_common.Camera.findCameraCoordinate`, finds the camera coordinate if the camera were fixed at the origin

        :param pos: The 3D coordinate of the object
        :return: The 2D pixel (x, y) on the camera, if the object was projected on the camera and the camera is placed at the origin
        """

        pos = Transformation(pos)

        ratio = self.focal_length / pos.position[0]

        tx = pos.position[1] * ratio
        ty = pos.position[2] * ratio
        x, y = self.world_to_image_frame(tx, ty)
        return [x, y]

    # TODO should these be here or in the node?
    def calculate_bounding_boxes_from_ball(self, ball_position: Transformation, ball_radius: float = 0.07):
        """
        Takes a 3D ball transformation and returns the bounding boxes of the ball if seen on camera

        :param ball_position: 3D coordinates of the ball stored in the :class:`Transformation` format
        :param ball_radius: The radious of the ball in centimeters
        :return: The bounding boxes of the ball on the camera in the format [[x1,y1], [x1,y1]] which are the top left
        and bottom right of the bounding box respectively
        """
        # TODO make relative and make into smaller functions
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

        ball_left_point_cam = self.find_camera_coordinate_fixed_camera(ball_left_point)
        ball_right_point_cam = self.find_camera_coordinate_fixed_camera(ball_right_point)
        ball_top_point_cam = self.find_camera_coordinate_fixed_camera(ball_top_point)
        ball_bottom_point_cam = self.find_camera_coordinate_fixed_camera(ball_bottom_point)

        left_border_x = ball_left_point_cam[0]
        right_border_x = ball_right_point_cam[0]
        top_border_y = ball_top_point_cam[1]
        bottom_border_y = ball_bottom_point_cam[1]

        bounding_box = [[left_border_x, top_border_y], [right_border_x, bottom_border_y]]

        return bounding_box

    def calculate_ball_from_bounding_boxes(self, bounding_boxes: [float] = [], ball_radius: float = 0.07) -> Transformation:
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

        y1w, z1w = self.image_to_world_frame(y1, z1)
        y2w, z2w = self.image_to_world_frame(y2, z2)
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
        # print(tr) # TODO could use for head control
        return tr_cam  # tr

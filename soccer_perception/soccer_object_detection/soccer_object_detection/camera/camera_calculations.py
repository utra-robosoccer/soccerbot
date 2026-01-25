import math

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

from soccer_object_detection.camera.camera_base import CameraBase

from soccer_common.transformation import Transformation
from shape_msgs.msg import Plane
import PyKDL


class CameraCalculations(CameraBase):
    def __init__(self):
        super(CameraCalculations, self).__init__()
        self.pose = Transformation()

        # camera intriniscs and extrinsics
        self.intrinsic_matrix = self.get_intrinsic_matrix()
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)



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

    def find_floor_coordinate(self, pos: list[int]) -> list[int]:
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
        ratio = (camera_pose.position[2] - pixel_world_pose.position[2]) / camera_pose.position[2]  # TODO Fix divide by 0 problem
        x_delta = (pixel_world_pose.position[0] - camera_pose.position[0]) / ratio
        y_delta = (pixel_world_pose.position[1] - camera_pose.position[1]) / ratio

        return [x_delta + camera_pose.position[0], y_delta + camera_pose.position[1], 0]

    def uv_to_roadXYZ_camframe(self, u, v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        camera_pose = self.pose
        ypr = camera_pose.orientation_euler
        yaw = np.deg2rad(ypr[0])
        pitch = np.deg2rad(ypr[1])
        roll = np.deg2rad(ypr[2])
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array([[cr * cy + sp * sr * sy, cr * sp * sy - cy * sr, -cp * sy],
                                         [cp * sr, cp * cr, sp],
                                         [cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy]])
        self.rotation_cam_to_road = rotation_road_to_cam.T  # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0, -camera_pose.position[2], 0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3, 0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3, 3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0, 1, 0])
        uv_hom = np.array([u, v, 1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return camera_pose.position[2] * Kinv_uv_hom / denominator
    def get_intrinsic_matrix(self, ):
        # For our Carla camera alpha_u = alpha_v = alpha
        # alpha can be computed given the cameras field of view via
        alpha1 = (self.horizontal_aspect_orig / 2.0) / np.tan(self.horizontal_fov / 2.)
        # alpha1 = self.focal_length*self.horizontal_aspect/self.image_sensor_width
        alpha2 = (self.vertical_aspect_orig / 2.0) / np.tan(self.vertical_fov / 2.)
        # alpha2 = self.focal_length * self.vertical_aspect / self.image_sensor_height
        Cu = self.horizontal_aspect / 2.0
        Cv = self.vertical_aspect / 2.0
        return np.array([[alpha1, 0, Cu],
                         [0, alpha2, Cv],
                         [0, 0, 1.0]])
    def map_point(self,u, v):

        # convert the map -> cam transform into the correct orientation for the IPM
        rot = Transformation(euler=[0, 1.57, 0])
        t_map_to_cam = self.pose
        t_map_to_cam.rotation_matrix = rot.rotation_matrix @ t_map_to_cam.rotation_matrix
        t_cam_to_map = np.linalg.inv(t_map_to_cam)
        plane_msg = Plane()
        plane_msg.coef[2] = 1.0  # Normal in z direction
        # plane_msg.coef[3] = -self.pose.position[2]
        # Convert plane from general form to point normal form
        plane = self.plane_general_to_point_normal(plane_msg)

        # View plane from camera frame
        plane_base_point, plane_normal = self.transform_plane_to_frame(
            plane=plane, camera_pose=t_cam_to_map)

        points =  np.array([[u, v]])
        # Convert points to float if they aren't allready
        if points.dtype.char not in np.typecodes['AllFloat']:
            points = points.astype(np.float32)

        # Get intersection points with plane
        np_points = self.get_field_intersection_for_pixels(
            points,
            plane_normal,
            plane_base_point)
        # print("here: ", np_points)
        # print(self.pose.position, "   ", self.pose.quaternion)

        # print(t_map_to_cam.position, "   ", t_map_to_cam.quaternion)
        np_points = np.einsum(
            'ij, pj -> pi',
            t_map_to_cam.rotation_matrix,
            np_points) + t_map_to_cam.position
        np_point = np_points[0]
        return np_point

    def plane_general_to_point_normal(self, plane: Plane) -> tuple[np.ndarray, np.ndarray]:
        """
        Convert general plane form to point normal form.

        :param plane: The input plane in general form
        :returns: A tuple with the point and normal
        """
        # ax + by + cz + d = 0 where a, b, c are the normal vector
        a, b, c, d = plane.coef
        # A perpendicular array to the plane
        perpendicular = np.array([a, b, c])
        # Get closest point from (0, 0, 0) to the plane
        point = perpendicular * -d / np.dot(perpendicular, perpendicular)
        # A normal vector to the plane
        normal = perpendicular / np.linalg.norm(perpendicular)
        return point, normal

    def transform_plane_to_frame(self,
            plane: tuple[np.ndarray, np.ndarray], camera_pose
           ) -> tuple[np.ndarray, np.ndarray]:
        """
        Transform a plane from one frame to another.

        :param plane: The planes base point and normal vector as numpy arrays
        :param input_frame: Current frame of the plane
        :param output_frame: The desired frame of the plane
        :param time: Timestamp which is used to query the tf buffer and get the tranform at this moment
        :param buffer: The refrence to the used tf buffer
        :param timeout: An optinal timeout after which an exception is raised
        :returns: A Tuple containing the planes base point and normal vector in the
             new frame at the provided timestamp
        """

        # Create two points to transform the base point and the normal vector
        # The second point is generated by adding the normal to the base point
        field_normal = Transformation(position=(
            plane[0][0] + plane[1][0],
            plane[0][1] + plane[1][1],
            plane[0][2] + plane[1][2]))
        # print("1", field_normal.position)
        # field_normal = field_normal @ camera_pose
        field_normal = camera_pose @ field_normal
        # print("2", field_normal.position)

        field_point = Transformation(position=(
            plane[0][0],
            plane[0][1],
            plane[0][2]))
        # print("3", field_point.position)

        # field_point = field_point @ camera_pose
        field_point = camera_pose @ field_point
        # field_point = camera_pose.rotation_matrix @ field_point.position + camera_pose.position

        # print("4", field_point.position)

        field_point = np.array(
            field_point.position)
        field_normal = np.array(
            field_normal.position)
        # print(field_normal, field_point)
        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_point, field_normal

    def get_field_intersection_for_pixels(self,
            points: np.ndarray,
            plane_normal: np.ndarray,
            plane_base_point: np.ndarray,
            scale: float = 1.0,
            use_distortion: bool = False) -> np.ndarray:
        """
        Map a NumPy array of points in image space on the given plane.

        :param points: A nx2 array with n being the number of points
        :param plane_normal: The normal vector of the mapping plane
        :param plane_base_point: The base point of the mapping plane
        :param scale: A scaling factor used if e.g. a mask with a lower resolution is transformed
        :param use_distortion: A flag to indicate if distortion should be accounted for.
            Do not use this if you are working with pixel coordinates from a rectified image.
        :returns: A NumPy array containing the mapped points
            in 3d relative to the camera optical frame
        """
        # Apply binning and scale
        # binning_x = max(camera_info.binning_x, 1) / scale
        # binning_y = max(camera_info.binning_y, 1) / scale
        # points = points * np.array([binning_x, binning_y])

        # Create identity distortion coefficients if no distortion is used
        # if use_distortion:
        #     distortion_coefficients = np.array(camera_info.d)
        # else:
        distortion_coefficients = np.zeros(5)

        # Get the ray directions relative to the camera optical frame for each of the points
        ray_directions = np.ones((points.shape[0], 3))
        if points.shape[0] > 0:
            ray_directions[:, :2] = cv2.undistortPoints(
                points.reshape(1, -1, 2).astype(np.float32),
                # np.array([1338.64532, 0., 1026.12387, 0., 1337.89746, 748.42213, 0., 0., 1.]).reshape(3, 3),
                self.intrinsic_matrix.reshape(3, 3),
                distortion_coefficients).reshape(-1, 2)
        # print("here: ", ray_directions)
        # Calculate ray -> plane intersections
        intersections = self.line_plane_intersections(
            plane_normal, plane_base_point, ray_directions)
        return intersections

    def line_plane_intersections(self,
            plane_normal: np.ndarray,
            plane_base_point: np.ndarray,
            ray_directions: np.ndarray) -> np.ndarray:
        """
        Calculate the intersections of rays with a plane described by a normal and a point.

        :param plane_normal: The normal vector of the mapping plane
        :param plane_base_point: The base point of the mapping plane
        :param ray_directions: A nx3 array with n being the number of rays
        :returns: A nx3 array containing the 3d intersection points with n being the number of rays.
        """
        n_dot_u = np.tensordot(plane_normal, ray_directions, axes=([0], [1]))
        relative_ray_distance = plane_normal.dot(plane_base_point) / n_dot_u

        # we are casting a ray, intersections need to be in front of the camera
        relative_ray_distance[relative_ray_distance <= 0] = np.nan

        ray_directions[:, 0] = np.multiply(
            relative_ray_distance, ray_directions[:, 0])
        ray_directions[:, 1] = np.multiply(
            relative_ray_distance, ray_directions[:, 1])
        ray_directions[:, 2] = np.multiply(
            relative_ray_distance, ray_directions[:, 2])

        return ray_directions

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

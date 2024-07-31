#!/usr/bin/env python3

import cv2
import numpy as np
from cv2 import Mat
from soccer_object_localization.detector import Detector

from soccer_common.transformation import Transformation


class DetectorFieldline(Detector):
    """
    Class for detecting field lines and converting to pointclouds.
    """

    def __init__(self):
        super().__init__()
        self.point_cloud_max_distance = 5
        self.point_cloud_spacing = 30
        self.publish_point_cloud = True  # TODO is this necessary?
        self.ground_truth = False

        cv2.setRNGSeed(12345)

    def image_filter(self, image: Mat, debug=False) -> Mat:
        # TODO could do a cover horizon based on distance to green
        h = self.camera.calculate_horizon_cover_area()
        if h + 1 >= self.camera.resolution_y:
            return image

        image_crop = image[h + 1 :, :, :]
        # image_crop_blurred = cv2.GaussianBlur(image_crop, (3, 3), 0)
        image_crop_blurred = cv2.bilateralFilter(image, 9, 75, 75)

        # hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)

        if debug:
            cv2.imshow("CVT Color", image_crop)
            cv2.imshow("CVT Color Contrast", image_crop_blurred)
            cv2.waitKey(0)

        # Grass Mask
        grass_only_morph, grass_only_flipped = self.grass_mask(image_crop_blurred, h, debug)

        lines_only = cv2.bitwise_and(grass_only_flipped, grass_only_flipped, mask=grass_only_morph)
        lines_only = cv2.morphologyEx(lines_only, cv2.MORPH_CLOSE, self.circular_mask(5))

        # TODO is this needed or should this be in a unit test
        if debug:
            cv2.imshow("lines_only", lines_only)
            cv2.imwrite("/tmp/lines_only.png", lines_only)
            cv2.waitKey(0)

        return lines_only

    def img_to_points(self, lines_only: Mat) -> list:

        # No line detection simply publish all white points
        pts_x, pts_y = np.where(lines_only == 255)
        points3d = []

        # TODO own function
        if self.publish_point_cloud:
            i = 0
            for px, py in zip(pts_y, pts_x):
                i += 1
                if i % self.point_cloud_spacing == 0:
                    cam_to_point = Transformation(self.camera.find_floor_coordinate([px, py]))

                    # Exclude points too far away
                    if cam_to_point.norm_squared < self.point_cloud_max_distance**2:
                        points3d.append(cam_to_point.position)

        return points3d


if __name__ == "__main__":
    fieldline_detector = DetectorFieldline()

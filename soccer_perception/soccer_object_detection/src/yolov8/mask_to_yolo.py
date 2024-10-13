import math
import os
import os.path
import pickle
import sys
from os.path import expanduser
from unittest import TestCase

import cv2
import numpy as np
import pytest
import yaml
from cv2 import Mat


# ~/hdd/TORO_21_dataset/data/reality or simulation/imges or seg or annotation (yaml file)
# need path to folder, pick the images to use, save yolov8 format for detection and segmentation in a dir for training
class MaskToYolo(TestCase):
    def test_path(self):
        self.path_to_masks = os.path.expanduser("~/robosoccer_yolo/code/data/object_detection/images/test")
        self.dirs = os.listdir(self.path_to_masks)
        for file in self.dirs:
            print(file)

    def return_polygons(self, mask, class_id, original):  # given contours, from cv2 contours, append class ID and polygons
        h, w = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert the contours to polygons
        polygons = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 10:  # Adjust the threshold based on your data
                polygon = []
                for point in cnt:
                    x, y = point[0]
                    polygon.append(x / w)
                    polygon.append(y / h)
                    # Draw points on greyscale image for debugging
                    cv2.circle(original, (x, y), 2, (0, 0, 0), -1)

                    # Smoothened version using cv2.polylines
                    # cv2.polylines(original, [cnt], isClosed=True, color=(0, 0, 0), thickness=2)

                polygons.append((class_id, polygon))

        return polygons

    def sim_contour_pts(self, image_path, thresh, class_id):
        # Load greyscale version of image, resize to fit screen
        greyscale = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        greyscale = cv2.resize(greyscale, (0, 0), fx=0.5, fy=0.5)

        if False:
            # debug input image
            cv2.imshow("source", greyscale)
            cv2.waitKey(0)

        # convert greyscale image to mask
        field_mask = cv2.inRange(greyscale, thresh, thresh + 1)

        if False:
            # debug mask
            cv2.imshow("field mask", field_mask)
            cv2.waitKey(0)

        # get contours
        polygons = self.return_polygons(field_mask, class_id, greyscale)

        if True:
            cv2.imshow("contour pts.", greyscale)
            cv2.waitKey(0)

        return polygons

    def test_find_threshold(self):
        path_to_sim_masks = os.path.expanduser("~/robosoccer_yolo/code/data/masks/test")
        # path_to_output = os.path.expanduser('~/robosoccer_yolo/code/data/masks')

        # os.makedirs(path_to_output, exist_ok=True)

        for j in os.listdir(path_to_sim_masks):
            image_path = os.path.join(path_to_sim_masks, j)

            # Load greyscale version of image, resize to fit screen
            greyscale = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            greyscale = cv2.resize(greyscale, (0, 0), fx=0.5, fy=0.5)

            cv2.imshow("source", greyscale)
            cv2.waitKey(0)

    def test_image_to_yolo(self):
        path_to_sim_masks = os.path.expanduser("~/robosoccer_yolo/code/data/masks/test")  # ~/robosoccer_yolo/code/mask_to_yolo_test/images
        path_to_output = os.path.expanduser("~/robosoccer_yolo/code/data/masks")  # ~/robosoccer_yolo/code/mask_to_yolo_test/output

        os.makedirs(path_to_output, exist_ok=True)

        # [field, field lines]
        thresh_sim = [149, 204]
        thresh_real = [254, 127]
        class_num = [0, 1]
        polygons = []

        for j in os.listdir(path_to_sim_masks):
            image_path = os.path.join(path_to_sim_masks, j)
            polygons += self.sim_contour_pts(image_path, thresh_real[0], class_num[0])
            polygons += self.sim_contour_pts(image_path, thresh_real[1], class_num[1])

            with open("{}.txt".format(os.path.join(path_to_output, j)[:-4]), "w") as f:
                for class_id, polygon in polygons:
                    f.write(f"{class_id} ")
                    f.write(" ".join(map(str, polygon)))
                    f.write("\n")

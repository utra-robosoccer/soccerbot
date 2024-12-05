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
        self.path_to_masks = os.path.expanduser("~/hdd/robosoccer_yolo/TORSO_21_dataset/data/reality/test/segmentations")
        self.dirs = os.listdir(self.path_to_masks)
        for file in self.dirs:
            print(file)

    def return_polygons(self, mask, class_id, original):
        h, w = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Track unique polygons to avoid duplicates
        seen_polygons = set()
        temp_polygons = []

        for cnt in contours:
            copy = original.copy()
            if cv2.contourArea(cnt) > 20:
                epsilon = 0.001 * cv2.arcLength(cnt, True)  # Higher epsilon for simpler shapes
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                polygon = []
                for point in approx:  # Use approx instead of cnt for simpler polygon
                    x, y = point[0]
                    polygon.append(round(x / w, 3))
                    polygon.append(round(y / h, 3))
                    cv2.circle(copy, (x, y), 2, (0, 0, 0), -1)

                # Convert to a tuple of rounded points to identify duplicates
                polygon_tuple = tuple(round(p, 3) for p in polygon)
                if polygon_tuple not in seen_polygons:  # Only add unique polygons
                    seen_polygons.add(polygon_tuple)
                    temp_polygons.append((class_id, polygon))

                # cv2.imshow("contour pts.", copy)
                # cv2.waitKey(0)
        return temp_polygons

    def sim_contour_pts(self, image_path, thresh, class_id):
        # Load greyscale version of image, resize to fit screen
        greyscale = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        # greyscale = cv2.resize(greyscale, (0, 0), fx=0.5, fy=0.5)

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

        if False:
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
        path_to_sim_masks = os.path.expanduser(
            "~/hdd/robosoccer_yolo/TORSO_21_dataset/data/simulation/train/segmentations"
        )  # ~/robosoccer_yolo/code/mask_to_yolo_test/images
        path_to_output = os.path.expanduser(
            "~/hdd/robosoccer_yolo/yolov8_segmentation/temp"
        )  # was labels # ~/robosoccer_yolo/code/mask_to_yolo_test/output

        os.makedirs(path_to_output, exist_ok=True)

        # [field, field lines]
        thresh_sim = [149, 204]
        # thresh_real = [254, 127]
        class_num = [0, 1]
        # polygons = []

        for j in os.listdir(path_to_sim_masks):
            polygons = []
            # if j != "00021_img_fake_cam_027907_seg.PNG":
            #     continue
            image_path = os.path.join(path_to_sim_masks, j)
            polygons += self.sim_contour_pts(image_path, thresh_sim[0], class_num[0])
            polygons += self.sim_contour_pts(image_path, thresh_sim[1], class_num[1])

            # print(polygons)

            with open("{}.txt".format(os.path.join(path_to_output, j)[:-4]), "w") as f:
                for class_id, polygon in polygons:
                    f.write(f"{class_id} ")
                    f.write(" ".join(map(str, polygon)))
                    f.write("\n")

    def test_change_filename(self):
        path_to_dir = os.path.expanduser("~/hdd/robosoccer_yolo/yolov8_segmentation/temp")
        for file in os.listdir(path_to_dir):
            os.rename(os.path.join(path_to_dir, file), os.path.join(path_to_dir, file[:-8] + ".txt"))
            # print(type(file[:-8] + file[-4:]))

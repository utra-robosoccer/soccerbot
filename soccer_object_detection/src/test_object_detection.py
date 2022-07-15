import os.path
from unittest import TestCase

import cv2
import torch


def IoU(boxA, boxB):
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    # compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    # return the intersection over union value
    return iou


class Test(TestCase):
    def test_ball_image(self):
        IOU_THRESHOLD = 0.9
        CONFIDENCE_THRESHOLD = 0.7
        BALL_CLASS = 0
        src_path = os.path.dirname(os.path.realpath(__file__))
        model_path = src_path + "/small_model/July14.pt"
        test_path = src_path + "/test_image"

        model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)

        if torch.cuda.is_available():
            model.cuda()

        for file_name in os.listdir(test_path):
            img = cv2.imread(os.path.join(test_path, file_name))  # ground truth box = (68, 89) (257, 275)
            ground_truth_boxes = file_name[:-4].split("_")[1:]  # strip name and .png
            ground_truth_boxes = list(map(int, ground_truth_boxes))

            results = model(img)  # you can print(results) for more info

            for prediction in results.xyxy[0]:
                x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
                if img_class == BALL_CLASS:
                    iou = IoU([x1, y1, x2, y2], ground_truth_boxes)
                    self.assertGreater(iou, IOU_THRESHOLD, "bounding boxes are off by too much!")
                    self.assertGreater(confidence, CONFIDENCE_THRESHOLD, "model not confident enough!")

                    # uncomment to view bounding boxes
                    # img_detect = cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    # cv2.imshow("res", img_detect)
                    # cv2.waitKey(0)

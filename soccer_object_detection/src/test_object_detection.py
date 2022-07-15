import os.path
from unittest import TestCase

import cv2
import torch
from PIL import Image

from soccer_object_detection.msg import BoundingBox, BoundingBoxes


def copy_attr(a, b, include=(), exclude=()):
    # Copy attributes from b to a, options to only include [...] and to exclude [...]
    for k, v in b.__dict__.items():
        if (len(include) and k not in include) or k.startswith("_") or k in exclude:
            continue
        else:
            setattr(a, k, v)


class Test(TestCase):
    def test_ball_image(self):
        print("hi -------------------->")
        # training set: 'https://universe.roboflow.com/nihal-saxena/my-ball/dataset/1'
        # model = torch.hub.load("ultralytics/yolov5", "yolov5m")
        model_path = os.path.dirname(os.path.realpath(__file__)) + "/" + "July14.pt"
        model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)
        # checkpoint_ = torch.load('/home/nam/catkin_ws/src/soccerbot/soccer_object_detection/src/yolov5/runs/train/exp4/weights/best.pt')['model']
        # model.load_state_dict(checkpoint_.state_dict())

        # copy_attr(model, checkpoint_, include=('yaml', 'nc', 'hyp', 'names', 'stride'), exclude=())

        # model = model.fuse().autoshape()
        if torch.cuda.is_available():
            model.cuda()

        # 1. preprocess image
        img = cv2.imread("test_image/ball1.png")
        # img = cv2.imread('/home/nam/catkin_ws/src/soccerbot/soccer_object_detection/src/test_image/81-test_nagoya_game_c_00050.png')
        # img = cv2.imread('/home/nam/catkin_ws/src/soccerbot/soccer_object_detection/src/datasets/balls1/train/images/100--83-_jpg.rf.06a9f110a44529544e9976c9858f6d68.jpg')
        # img = cv2.imread('/home/nam/catkin_ws/src/soccerbot/soccer_object_detection/src/yolov5/runs/train/exp4/val_batch2_pred.jpg')
        cv2.imshow("name", img)
        cv2.waitKey(0)
        # img = image[:, :, :3]  # get rid of alpha channel
        # img = img[..., ::-1]  # convert bgr to rgb

        # 2. inference
        results = model(img)

        print(results)

        for prediction in results.xyxy[0]:
            x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
            print(x1, y1, x2, y2, confidence, img_class)
            img_detect = cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.imshow("res", img_detect)
            cv2.waitKey(0)

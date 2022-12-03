import os
import os.path
import pickle
import sys
from unittest import TestCase
from unittest.mock import MagicMock

import cv2
import numpy as np
import yaml
from cv2 import Mat

from soccer_common.mock_ros import mock_ros
from soccer_msgs.msg import GameState, RobotState

mock_ros()


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
    def test_object_detection_node(self):
        import numpy as np
        import rospy
        import tf2_ros
        from sensor_msgs.msg import Image

        from soccer_common import Camera

        Camera.reset_position = MagicMock()
        tf2_ros.TransformListener = MagicMock()
        rospy.Time.now = MagicMock(return_value=0)
        from sensor_msgs.msg import CameraInfo

        from soccer_object_detection.object_detect_node import ObjectDetectionNode

        src_path = os.path.dirname(os.path.realpath(__file__))
        model_path = src_path + "/small_model/July14.pt"
        test_path = src_path + "/test_image"

        from cv_bridge import CvBridge

        cvbridge = CvBridge()
        for file_name in os.listdir(test_path):
            img: Mat = cv2.imread(os.path.join(test_path, file_name))  # ground truth box = (68, 89) (257, 275)
            img_msg: Image = cvbridge.cv2_to_imgmsg(img)

            n = ObjectDetectionNode(model_path=model_path)
            n.pub_detection = MagicMock()
            n.pub_boundingbox = MagicMock()
            n.pub_detection.get_num_connections = MagicMock(return_value=1)
            n.pub_boundingbox.get_num_connections = MagicMock(return_value=1)
            n.pub_detection.publish = MagicMock()
            n.pub_boundingbox.publish = MagicMock()

            n.robot_state.status = RobotState.STATUS_READY
            n.game_state.gameState = GameState.GAMESTATE_PLAYING

            ci = CameraInfo()
            ci.height = img.shape[0]
            ci.width = img.shape[1]
            n.camera.camera_info = ci
            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            n.callback(img_msg)

            # Extract ground truth
            ground_truth_boxes = file_name[:-4].split("_")[1:]  # strip name and .png
            ground_truth_boxes = list(map(int, ground_truth_boxes))

            # Check assertion
            self.assertGreater(n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].probability, n.CONFIDENCE_THRESHOLD)
            bounding_boxes = [
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].xmin,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].ymin,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].xmax,
                n.pub_boundingbox.publish.call_args[0][0].bounding_boxes[0].ymax,
            ]
            iou = IoU(bounding_boxes, ground_truth_boxes)
            self.assertGreater(iou, 0.8, "bounding boxes are off by too much!")

            if "DISPLAY" in os.environ:
                mat = cvbridge.imgmsg_to_cv2(n.pub_detection.publish.call_args[0][0])
                cv2.imshow("res", mat)
                cv2.waitKey(1)
                cv2.destroyAllWindows()

    @pytest.mark.skip(reason="annotation_path not found")
    def test_visualize_annotations(self):
        src_path = os.path.dirname(os.path.realpath(__file__))

        # Data downloaded from https://github.com/bit-bots/TORSO_21_dataset
        annotation_path = f"{src_path}/../../data/test/annotations.yaml"
        annotation_pickle = f"{src_path}/../../data/test/annotation.pkl"
        image_path = f"{src_path}/../../data/test/data/images/"

        if not os.path.exists(annotation_pickle):
            with open(annotation_path) as f:
                print("Pickling annotation, will take a long time")
                yaml_data = yaml.load(f)
            with open(annotation_pickle, "wb") as f2:
                pickle.dump(yaml_data, f2)
                return

        MAX_DIMENSIONS = (1778, 1000)

        with open(annotation_pickle, "rb") as f:
            annos = pickle.load(f)["images"]
            print(
                "Press 's' and 'd' to move between images. 'A' and 'S' let you jump 100 images.\n'c' to correct a label\n'v' to save image.\n'q' closes.\n'n' toggles not in image text. 'o' to toggle showing obstacles\n'e' to toggle all annotations"
            )
            files = list(annos)
            files.sort()
            not_in_image = True
            show_obstacles = True
            show_annotations = True
            i = 0
            while True:
                f = files[i]
                img_path = os.path.join(image_path, f)
                img = cv2.imread(img_path)
                assert img is not None
                h, w, c = img.shape
                text_thickness = int(w / 200)
                line_thickness = int(w / 200)
                y = 20
                image_annos = annos[f]["annotations"]
                # sort lables to have them in the correct order.
                image_annos_sorted = []
                correct_order = {
                    "field edge": 0,
                    "goalpost": 1,
                    "left_goalpost": 2,
                    "right_goalpost": 3,
                    "crossbar": 4,
                    "robot": 5,
                    "obstacle": 6,
                    "ball": 7,
                    "L-Intersection": 8,
                    "T-Intersection": 9,
                    "X-Intersection": 10,
                }
                for a in image_annos:
                    a["order"] = correct_order[a["type"]]
                image_annos_sorted = sorted(image_annos, key=lambda a: a["order"])
                for a in image_annos_sorted:
                    if show_annotations:
                        if not a["in_image"] and "vector" in a:
                            cv2.putText(
                                img,
                                f"{a['type']} completely concealed in image",
                                (0, y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                (0, 0, 255),
                                int(text_thickness / 2),
                            )
                            y += 20
                        elif not a["in_image"]:
                            if not_in_image:
                                cv2.putText(
                                    img, f"{a['type']} not in image", (0, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), int(text_thickness / 2)
                                )
                                y += 20
                        else:
                            if a["type"] == "robot":
                                color = (255, 0, 0)
                            elif a["type"] == "ball":
                                color = (0, 0, 255)
                            elif a["type"] == "goalpost":
                                color = (0, 255, 255)
                            elif a["type"] == "left_goalpost":
                                color = (255, 0, 255)
                            elif a["type"] == "right_goalpost":
                                color = (0, 255, 255)
                            elif a["type"] == "crossbar":
                                color = (0, 0, 255)
                            elif a["type"] == "field edge":
                                color = (0, 255, 0)
                            elif a["type"] == "obstacle":
                                color = (0, 0, 0)

                            if a["type"] == "obstacle" and not show_obstacles:
                                pass
                            elif a["type"] == "robot" or a["type"] == "ball" or a["type"] == "obstacle":  # bounding boxes
                                x_start = int(a["vector"][0][0])
                                x_stop = int(a["vector"][1][0])
                                y_start = int(a["vector"][0][1])
                                y_stop = int(a["vector"][1][1])
                                contours = np.ndarray((4, 2), dtype=int)
                                contours[0][0] = x_start
                                contours[0][1] = y_start
                                contours[1][0] = x_start
                                contours[1][1] = y_stop
                                contours[2][0] = x_stop
                                contours[2][1] = y_stop
                                contours[3][0] = x_stop
                                contours[3][1] = y_start
                                cv2.drawContours(img, [contours], -1, color, line_thickness)
                            elif a["type"] == "goalpost" or a["type"] == "left_goalpost" or a["type"] == "right_goalpost" or a["type"] == "crossbar":
                                contours = np.ndarray((4, 2), dtype=int)
                                contours[0][0] = int(a["vector"][0][0])
                                contours[0][1] = int(a["vector"][0][1])
                                contours[1][0] = int(a["vector"][1][0])
                                contours[1][1] = int(a["vector"][1][1])
                                contours[2][0] = int(a["vector"][2][0])
                                contours[2][1] = int(a["vector"][2][1])
                                contours[3][0] = int(a["vector"][3][0])
                                contours[3][1] = int(a["vector"][3][1])
                                cv2.drawContours(img, [contours], -1, color, line_thickness)
                            elif a["type"] == "field edge":
                                points = []
                                for point in a["vector"]:
                                    points.append(point)
                                pts = np.array(points, np.int32)
                                pts = pts.reshape((-1, 1, 2))
                                img = cv2.polylines(img, [pts], False, color, line_thickness)
                            else:
                                color = (0, 0, 0)
                                if a["type"] == "L-Intersection":
                                    txt = "L"
                                elif a["type"] == "T-Intersection":
                                    txt = "T"
                                elif a["type"] == "X-Intersection":
                                    txt = "X"
                                else:
                                    print(a["type"])
                                    exit(1)
                                txt_size = cv2.getTextSize(txt, cv2.FONT_HERSHEY_COMPLEX, 1, text_thickness)
                                cv2.putText(
                                    img,
                                    txt,
                                    (int(a["vector"][0][0] - (txt_size[0][0] / 2)), int(a["vector"][0][1] + (txt_size[0][1] / 2))),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1,
                                    (0, 0, 0),
                                    text_thickness,
                                )
                if h > MAX_DIMENSIONS[1]:
                    scaling = MAX_DIMENSIONS[1] / h
                    img = cv2.resize(img, (int(w * scaling), int(h * scaling)))
                cv2.imshow("img", img)
                key = cv2.waitKey(0)
                if key in [100]:  # d
                    i += 1
                elif key == 68:  # D
                    i += 100
                elif key in [115]:  # s
                    i -= 1
                elif key == 83:  # S
                    i -= 100
                elif key in [27, 113]:
                    exit(0)
                elif key == 110:  # n
                    not_in_image = not not_in_image
                elif key == 111:  # o
                    show_obstacles = not show_obstacles
                elif key == 118:  # v
                    cv2.imwrite(f"../viz_{f}", img)
                elif key == 99:  # c
                    img_id = annos[f]["id"]
                    os.system(f"firefox --new-tab https://imagetagger.bit-bots.de/annotations/{img_id}/")
                elif key == 101:
                    show_annotations = not show_annotations
                i = max(0, i)
                i = min(len(files), i)
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                print(f"Current image number {i} name {f}\n")

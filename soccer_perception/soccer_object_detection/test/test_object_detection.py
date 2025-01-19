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
from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode
from soccer_object_detection.utils import check_bounding_box

from soccer_common import Transformation
from soccer_common.utils import download_dataset, wrapToPi

PLOT = True


class TestObjectDetection(TestCase):
    def test_object_detection(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        test_path = src_path + "data/images/simulation"

        download_dataset("https://drive.google.com/uc?id=11nN58j8_PBoLNRAzOEdk7fMe1UK1diCc", folder_path=test_path)

        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        n = ObjectDetectionNode(model_path=model_path)

        for file_name in os.listdir(f"{test_path}/images"):
            print(file_name)
            img: Mat = cv2.imread(os.path.join(f"{test_path}/images", file_name))  # ground truth box = (68, 89) (257, 275)
            img = cv2.resize(img, dsize=(640, 480))

            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            detection_image, bbs_msg = n.get_model_output(img)

            with open(os.path.join(f"{test_path}/labels", file_name.replace("PNG", "txt"))) as f:
                lines = f.readlines()

            # Check assertion
            # TODO is this really necessary
            # Also can it be a function
            for bounding_box in bbs_msg.bounding_boxes:
                if bounding_box.probability >= n.CONFIDENCE_THRESHOLD and int(bounding_box.Class) in [
                    Label.BALL.value,
                    Label.ROBOT.value,
                    Label.GOALPOST.value,
                    Label.TOPBAR.value,
                ]:
                    best_iou = check_bounding_box(bounding_box, lines, n.camera.camera_info.width, n.camera.camera_info.height)

                    self.assertGreater(best_iou, 0.05, f"bounding boxes are off by too much! Image= {file_name}" f" Best IOU={best_iou}")
                    if best_iou < 0.5:
                        print(f"bounding boxes lower than 0.5 Image= {file_name} Best IOU={best_iou}")
                    # if "DISPLAY" in os.environ:
                    #     cv2.rectangle(
                    #         img=img,
                    #         pt1=(bounding_box.xmin, bounding_box.ymin),
                    #         pt2=(bounding_box.xmax, bounding_box.ymax),
                    #         color=(255, 255, 255),
                    #     )
                    #     if bounding_box.obstacle_detected is True:
                    #         cv2.circle(img, (bounding_box.xbase, bounding_box.ybase), 0, (0, 255, 255), 3)

            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("Image", detection_image)
                cv2.waitKey()
                cv2.destroyAllWindows()

    def test_object_detection_vid(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        test_path = src_path + "data/videos/robocup2023"

        download_dataset("https://drive.google.com/uc?id=1UTQ6Rz0yk8jpWwWoq3eSf7DOmG_j9An3", folder_path=test_path)

        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        n = ObjectDetectionNode(model_path=model_path)

        cap = cv2.VideoCapture(test_path + "/2023-07-08-124521.webm")
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            img = cv2.resize(frame, dsize=(640, 480))

            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]  # TODO why does this disable cover horizon
            detection_image, bbs_msg = n.get_model_output(img)  # 0.01

            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("Image", detection_image)
                cv2.waitKey(24)  # TODO why is this one so much faster
        cv2.destroyAllWindows()

    # @pytest.mark.skip(reason="Only run locally")
    def test_object_detection_node_cam(self):
        src_path = os.path.dirname(os.path.realpath(__file__))

        model_path = src_path + "/../models/yolov8s_detect_best.pt"  # yolov8s_detect_best

        n = ObjectDetectionNode(model_path=model_path)

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            img = cv2.resize(frame, dsize=(640, 480))

            n.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            detection_image, bbs_msg = n.get_model_output_v8(img)  # 0.01
            print(bbs_msg)
            # if "DISPLAY" in os.environ and PLOT:
            #     cv2.imshow("Image", detection_image) # detection_image
            #     cv2.waitKey(24)  # TODO why is this one so much faster
        cv2.destroyAllWindows()

    @pytest.mark.skip(reason="Only run locally")
    def test_visualize_annotations(self):
        # TODO what does this do?
        src_path = os.path.dirname(os.path.realpath(__file__))

        # Data downloaded from https://github.com/bit-bots/TORSO_21_dataset
        annotation_path = "/home/robosoccer/hdd/dataset/dataV2/TORSO-21/simulation/train/annotations.yaml"
        annotation_pickle = "/home/robosoccer/hdd/dataset/dataV2/TORSO-21/simulation/train/annotation.pkl"
        image_path = "/home/robosoccer//hdd/dataset/dataV2/TORSO-21/simulation/train/images"

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
                if "5733" not in f:
                    print(f)
                    i += 1
                    continue
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

    def test_goalpost_detection(self):
        """
        Returns whether a point at a given field coordinate is visible to the robot
        """

        # TODO verify how this works
        def get_point_visibility(robot_pose, point_coords):
            robot_x, robot_y, robot_yaw = robot_pose
            point_x, point_y = point_coords

            point_yaw = math.atan2(point_y - robot_y, point_x - robot_x)
            camera_fov = 1.39626  # rads

            # Both yaw angles are between -pi and pi
            delta_yaw = wrapToPi(point_yaw - robot_yaw)

            # Check if the point is within the view cone
            # No equals case as the point wouldn't be fully visible
            is_point_visible = -camera_fov / 2.0 < delta_yaw < camera_fov / 2.0

            return is_point_visible

        """
            Returns a dictionary that stores booleans indicating whether each post is visible
            Visual reference: https://www.desmos.com/calculator/b9lndsb1bl
            Example: both posts of the left net are visible
            visible_posts = {
                "NEG_X_NET": {
                    "POS_Y_POST": True,
                    "NEG_Y_POST": True
                },
                "POS_X_NET": {
                    "POS_Y_POST": False,
                    "NEG_Y_POST": False
                }
            }
        """

        def get_visible_posts(robot_x, robot_y, robot_yaw):
            visible_posts = {"NEG_X_NET": {"POS_Y_POST": True, "NEG_Y_POST": True}, "POS_X_NET": {"POS_Y_POST": False, "NEG_Y_POST": False}}

            net_coords = {
                "NEG_X_NET": {"POS_Y_POST": [-4.5, 1.3], "NEG_Y_POST": [-4.5, -1.3]},
                "POS_X_NET": {"POS_Y_POST": [4.5, 1.3], "NEG_Y_POST": [4.5, -1.3]},
            }

            for net in net_coords.keys():
                post_coords = net_coords[net]
                for post in post_coords.keys():
                    visible_posts[net][post] = get_point_visibility((robot_x, robot_y, robot_yaw), net_coords[net][post])

            return visible_posts

        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        test_path = src_path + "data/images/goal_net"

        download_dataset("https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_path=test_path)

        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        n = ObjectDetectionNode(model_path=model_path)

        n.camera.pose = Transformation(position=[0, 0, 0.46])

        # Loop through test images
        for file_name in os.listdir(test_path):
            # file_name = "img173_-0.852141317992289_3.15_-1.7125376246657054.png"

            print(f"Loading {file_name} from goal_net dataset")
            file_name_no_ext = os.path.splitext(file_name)[0]
            x, y, yaw = file_name_no_ext.split("_")[1:]
            yaw = wrapToPi(float(yaw))
            if yaw < 0:
                yaw = (yaw + np.pi) % np.pi

            n.camera.pose.orientation_euler = [yaw, 0, 0]
            print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")
            visible_posts = get_visible_posts(float(x), float(y), float(yaw))
            for net in visible_posts.keys():
                for post in visible_posts[net].keys():
                    if visible_posts[net][post]:
                        print(f"{net}, {post} is visible")

            img: Mat = cv2.imread(os.path.join(test_path, file_name))

            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("Before", img)

            detection_image, bbs_msg = n.get_model_output(img)
            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("After", detection_image)
                cv2.waitKey(0)

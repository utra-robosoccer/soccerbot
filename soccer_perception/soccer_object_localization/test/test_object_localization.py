# import os
#
# # from soccer_object_detection.object_detect_node import ObjectDetectionNode
#
# # ROS
# # from soccer_object_detection.test_object_detection import IoU
# # from soccer_object_localization.detector_objects import DetectorObjects
#
# os.environ["ROS_NAMESPACE"] = "/robot1"
#
# import math
# from unittest import TestCase
# from unittest.mock import MagicMock
#
# import cv2
# import numpy as np
# import pytest
#
# # import rosbag
# # import rospy
# # import tf2_ros
# from cv2 import Mat
# from cv_bridge import CvBridge
# from sensor_msgs.msg import CameraInfo, Image
# from soccer_object_localization.detector_fieldline import DetectorFieldline
#
# from soccer_common.transformation import Transformation
# from soccer_common.utils import download_dataset, wrapToPi
# from soccer_msgs.msg import GameState, RobotState
#
# # from soccer_object_localization.detector_goalpost import DetectorGoalPost
#
#
#
# # TODO fix unit test
# class TestObjectLocalization(TestCase):
#
#     def test_fieldline_detection(self):
#         # rospy.init_node("test")
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../images/fieldlines"
#         download_dataset(url="https://drive.google.com/uc?id=1nJX6ySks_a7mESvCm3sNllmJTNpm-x2_", folder_path=test_path)
#
#         # Camera.reset_position = MagicMock()
#         # Camera.ready = MagicMock()
#
#         d = DetectorFieldline()
#         # d.robot_state.status = RobotState.STATUS_READY
#
#         # ROS
#         # d.image_publisher.get_num_connections = MagicMock(return_value=1)
#         # d.publish_point_cloud = True
#         # d.point_cloud_publisher.get_num_connections = MagicMock(return_value=1)
#
#         # cvbridge = CvBridge()
#
#         for file_name in os.listdir(test_path):
#             # file_name = "img160_-1.452993567956688_-3.15_0.7763055830612666.png"
#
#             print(file_name)
#             img: Mat = cv2.imread(os.path.join(test_path, file_name))
#
#             # TODO should be by default
#             c = CameraInfo()
#             c.height = img.shape[0]
#             c.width = img.shape[1]
#             d.camera.camera_info = c
#
#             # ROS
#             # img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
#             # d.image_publisher.publish = MagicMock()
#             lines_only = d.image_callback(img, debug=False)
#
#             if "DISPLAY" in os.environ:
#                 cv2.imshow("Before", img)
#                 cv2.imwrite("/tmp/before.png", img)
#
#                 cv2.imshow("After", lines_only)
#
#                 cv2.waitKey(0)
#         cv2.destroyAllWindows()
#
#     def test_fieldline_detection_cam(self):
#         # rospy.init_node("test")
#
#         Camera.reset_position = MagicMock()
#         Camera.ready = MagicMock()
#         d = DetectorFieldline()
#         d.robot_state.status = RobotState.STATUS_READY
#         d.image_publisher.get_num_connections = MagicMock(return_value=1)
#         # d.publish_point_cloud = True
#         # d.point_cloud_publisher.get_num_connections = MagicMock(return_value=1)
#
#         cap = cv2.VideoCapture(4)
#         if not cap.isOpened():
#             print("Cannot open camera")
#             exit()
#         cvbridge = CvBridge()
#         while True:
#             ret, frame = cap.read()
#             if not ret:
#                 print("Can't receive frame (stream end?). Exiting ...")
#                 break
#             img = cv2.resize(frame, dsize=(640, 480))
#
#             c = CameraInfo()
#             c.height = img.shape[0]
#             c.width = img.shape[1]
#             d.camera.camera_info = c
#
#             img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
#             d.image_publisher.publish = MagicMock()
#             d.image_callback(img_msg, debug=False)
#
#             if "DISPLAY" in os.environ:
#                 cv2.imshow("Before", img)
#                 # cv2.imwrite("/tmp/before.png", img)
#
#                 if d.image_publisher.publish.call_count != 0:
#                     img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
#                     cv2.imshow("After", img_out)
#                     # cv2.imwrite("/tmp/after.png", img_out)
#
#                 cv2.waitKey(1)
#         cv2.destroyAllWindows()
#
#     def test_fieldline_detection_vid(self):
#         # rospy.init_node("test")
#         os.environ["LD_LIBRARY_PATH"] = "$LD_LIBRARY_PATH:/opt/ros/noetic/lib/"
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#
#         Camera.reset_position = MagicMock()
#         Camera.ready = MagicMock()
#         d = DetectorFieldline()
#         d.robot_state.status = RobotState.STATUS_READY
#         d.image_publisher.get_num_connections = MagicMock(return_value=1)
#         # d.publish_point_cloud = True
#         # d.point_cloud_publisher.get_num_connections = MagicMock(return_value=1)
#
#         cap = cv2.VideoCapture(src_path + "/../../../soccer_object_detection/videos/2023-07-08-124521.webm")
#         if not cap.isOpened():
#             print("Cannot open camera")
#             exit()
#         cvbridge = CvBridge()
#         while True:
#             ret, frame = cap.read()
#             if not ret:
#                 print("Can't receive frame (stream end?). Exiting ...")
#                 break
#             img = cv2.resize(frame, dsize=(640, 480))
#
#             c = CameraInfo()
#             c.height = img.shape[0]
#             c.width = img.shape[1]
#             d.camera.camera_info = c
#
#             img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
#             d.image_publisher.publish = MagicMock()
#             d.image_callback(img_msg, debug=False)
#
#             if "DISPLAY" in os.environ:
#                 cv2.imshow("Before", img)
#                 # cv2.imwrite("/tmp/before.png", img)
#
#                 if d.image_publisher.publish.call_count != 0:
#                     img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
#                     cv2.imshow("After", img_out)
#                     # cv2.imwrite("/tmp/after.png", img_out)
#
#                 cv2.waitKey(1)
#         cv2.destroyAllWindows()
#
#     def test_goalpost_detection(self):
#         """
#         Returns whether a point at a given field coordinate is visible to the robot
#         """
#
#         def get_point_visibility(robot_pose, point_coords):
#             robot_x, robot_y, robot_yaw = robot_pose
#             point_x, point_y = point_coords
#
#             point_yaw = math.atan2(point_y - robot_y, point_x - robot_x)
#             camera_fov = 1.39626  # rads
#
#             # Both yaw angles are between -pi and pi
#             delta_yaw = wrapToPi(point_yaw - robot_yaw)
#
#             # Check if the point is within the view cone
#             # No equals case as the point wouldn't be fully visible
#             is_point_visible = -camera_fov / 2.0 < delta_yaw < camera_fov / 2.0
#
#             return is_point_visible
#
#         """
#             Returns a dictionary that stores booleans indicating whether each post is visible
#             Visual reference: https://www.desmos.com/calculator/b9lndsb1bl
#             Example: both posts of the left net are visible
#             visible_posts = {
#                 "NEG_X_NET": {
#                     "POS_Y_POST": True,
#                     "NEG_Y_POST": True
#                 },
#                 "POS_X_NET": {
#                     "POS_Y_POST": False,
#                     "NEG_Y_POST": False
#                 }
#             }
#         """
#
#         def get_visible_posts(robot_x, robot_y, robot_yaw):
#             visible_posts = {"NEG_X_NET": {"POS_Y_POST": True, "NEG_Y_POST": True}, "POS_X_NET": {"POS_Y_POST": False, "NEG_Y_POST": False}}
#
#             net_coords = {
#                 "NEG_X_NET": {"POS_Y_POST": [-4.5, 1.3], "NEG_Y_POST": [-4.5, -1.3]},
#                 "POS_X_NET": {"POS_Y_POST": [4.5, 1.3], "NEG_Y_POST": [4.5, -1.3]},
#             }
#
#             for net in net_coords.keys():
#                 post_coords = net_coords[net]
#                 for post in post_coords.keys():
#                     visible_posts[net][post] = get_point_visibility((robot_x, robot_y, robot_yaw), net_coords[net][post])
#
#             return visible_posts
#
#         # Setup test environment:
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../images/goal_net"
#         download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_path=test_path)
#
#         # Camera.reset_position = MagicMock()
#         # Camera.ready = MagicMock()
#         d = DetectorGoalPost()
#         d.robot_state.status = RobotState.STATUS_DETERMINING_SIDE
#         d.camera.pose = Transformation(position=[0, 0, 0.46])
#         # d.image_publisher.get_num_connections = MagicMock(return_value=1)
#
#         # cvbridge = CvBridge()
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../images/goal_net"
#
#         # Loop through test images
#         for file_name in os.listdir(test_path):
#             # file_name = "img173_-0.852141317992289_3.15_-1.7125376246657054.png"
#
#             print(f"Loading {file_name} from goal_net dataset")
#             file_name_no_ext = os.path.splitext(file_name)[0]
#             x, y, yaw = file_name_no_ext.split("_")[1:]
#             yaw = wrapToPi(float(yaw))
#             if yaw < 0:
#                 yaw = (yaw + np.pi) % (np.pi)
#
#             d.camera.pose.orientation_euler = [yaw, 0, 0]
#             print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")
#             visible_posts = get_visible_posts(float(x), float(y), float(yaw))
#             for net in visible_posts.keys():
#                 for post in visible_posts[net].keys():
#                     if visible_posts[net][post]:
#                         print(f"{net}, {post} is visible")
#
#             img: Mat = cv2.imread(os.path.join(test_path, file_name))
#
#             if "DISPLAY" in os.environ:
#                 cv2.imshow("Before", img)
#
#             c = CameraInfo()
#             c.height = img.shape[0]
#             c.width = img.shape[1]
#             d.camera.camera_info = c
#
#             # img_msg: Image = cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
#             # d.image_publisher.publish = MagicMock()
#             d.image_callback(img, debug=False)
#
#             if "DISPLAY" in os.environ:
#                 # if d.image_publisher.publish.call_count != 0:
#                 #     # img_out = cvbridge.imgmsg_to_cv2(d.image_publisher.publish.call_args[0][0])
#                 cv2.imshow("After", d.img_out)
#                 cv2.waitKey(0)
#
#     # @pytest.mark.skip
#     def test_goalpost_detection_tune(self):
#         """
#         Used for tuning vertical line detection parameters using sliders.
#         """
#
#         # rospy.init_node("test")
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../images/goal_net"
#         download_dataset(url="https://drive.google.com/uc?id=17qdnW7egoopXHvakiNnUUufP2MOjyZ18", folder_path=test_path)
#
#         d = DetectorGoalPost()
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../images/goal_net"
#         file_name = "img341_0.960470105738711_-3.15_1.3585673890175494.png"
#
#         print(f"Loading {file_name} from goal_net dataset")
#         file_name_no_ext = os.path.splitext(file_name)[0]
#         x, y, yaw = file_name_no_ext.split("_")[1:]
#         print(f"Parsed (x, y, yaw): ({x}, {y}, {yaw}) from filename.")
#
#         img: Mat = cv2.imread(os.path.join(test_path, file_name))
#
#         c = CameraInfo()
#         c.height = img.shape[0]
#         c.width = img.shape[1]
#         d.camera.camera_info = c
#
#         # Create a window
#         cv2.namedWindow("image")
#
#         def nothing(x):
#             pass
#
#         # create trackbars for hough line parameters
#         default_vline_angle_tol_deg = 3
#         default_theta = np.pi / 180
#         default_threshold = 30
#         default_min_line_length = 30
#         default_max_line_gap = 10
#         cv2.createTrackbar("vLineAngleTolDeg", "image", 0, 15, nothing)
#         cv2.createTrackbar("threshold", "image", 0, 255, nothing)
#         cv2.createTrackbar("minLineLength", "image", 0, 250, nothing)
#         cv2.createTrackbar("maxLineGap", "image", 0, 20, nothing)
#
#         # Set default value for MAX HSV trackbars.
#         cv2.setTrackbarPos("vLineAngleTolDeg", "image", default_vline_angle_tol_deg)
#         cv2.setTrackbarPos("threshold", "image", default_threshold)
#         cv2.setTrackbarPos("minLineLength", "image", default_min_line_length)
#         cv2.setTrackbarPos("maxLineGap", "image", default_max_line_gap)
#
#         while True:
#             # get current positions of all trackbars
#             vline_angle_tol_deg = cv2.getTrackbarPos("vLineAngleTolDeg", "image")
#             threshold = cv2.getTrackbarPos("threshold", "image")
#             min_line_length = cv2.getTrackbarPos("minLineLength", "image")
#             max_line_gap = cv2.getTrackbarPos("maxLineGap", "image")
#
#             vertical_lines, img_out = d.get_vlines_from_img(
#                 img,
#                 debug=False,
#                 angle_tol_deg=vline_angle_tol_deg,
#                 hough_theta=default_theta,
#                 hough_threshold=threshold,
#                 hough_min_line_length=min_line_length,
#                 hough_max_line_gap=max_line_gap,
#             )
#             cv2.imshow("image", img_out)
#
#             if cv2.waitKey(33) & 0xFF == ord("n"):
#                 file_name = "newfile"
#                 break
#
#     @pytest.mark.skip
#     def test_hsv_filter(self):
#         import cv2
#         import numpy as np
#
#         def nothing(x):
#             pass
#
#         # Create a window
#         cv2.namedWindow("image")
#
#         # create trackbars for color change
#         cv2.createTrackbar("HMin", "image", 0, 179, nothing)  # Hue is from 0-179 for Opencv
#         cv2.createTrackbar("SMin", "image", 0, 255, nothing)
#         cv2.createTrackbar("VMin", "image", 0, 255, nothing)
#         cv2.createTrackbar("HMax", "image", 0, 179, nothing)
#         cv2.createTrackbar("SMax", "image", 0, 255, nothing)
#         cv2.createTrackbar("VMax", "image", 0, 255, nothing)
#
#         # Set default value for MAX HSV trackbars.
#         cv2.setTrackbarPos("HMax", "image", 179)
#         cv2.setTrackbarPos("SMax", "image", 255)
#         cv2.setTrackbarPos("VMax", "image", 255)
#
#         # Initialize to check if HSV min/max value changes
#         hMin = sMin = vMin = hMax = sMax = vMax = 0
#         phMin = psMin = pvMin = phMax = psMax = pvMax = 0
#
#         img = cv2.imread("../../images/goal_net/img160_-1.452993567956688_-3.15_0.7763055830612666.png")
#         output = img
#         waitTime = 33
#
#         while 1:
#
#             # get current positions of all trackbars
#             hMin = cv2.getTrackbarPos("HMin", "image")
#             sMin = cv2.getTrackbarPos("SMin", "image")
#             vMin = cv2.getTrackbarPos("VMin", "image")
#
#             hMax = cv2.getTrackbarPos("HMax", "image")
#             sMax = cv2.getTrackbarPos("SMax", "image")
#             vMax = cv2.getTrackbarPos("VMax", "image")
#
#             # Set minimum and max HSV values to display
#             lower = np.array([hMin, sMin, vMin])
#             upper = np.array([hMax, sMax, vMax])
#
#             # Create HSV Image and threshold into a range.
#             hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#             mask = cv2.inRange(hsv, lower, upper)
#             output = cv2.bitwise_and(img, img, mask=mask)
#
#             # Print if there is a change in HSV value
#             if (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax):
#                 print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin, sMin, vMin, hMax, sMax, vMax))
#                 phMin = hMin
#                 psMin = sMin
#                 pvMin = vMin
#                 phMax = hMax
#                 psMax = sMax
#                 pvMax = vMax
#
#             # Display output image
#             cv2.imshow("image", output)
#
#             # Wait longer to prevent freeze for videos.
#             if cv2.waitKey(waitTime) & 0xFF == ord("q"):
#                 break
#
#         cv2.destroyAllWindows()
#
#     def test_robot_detection(self):
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         test_path = src_path + "/../../../soccer_object_detection/images/simulation"
#         download_dataset("https://drive.google.com/uc?id=11nN58j8_PBoLNRAzOEdk7fMe1UK1diCc", folder_path=test_path)
#
#         # ROS
#         # rospy.init_node("test")
#
#         Camera.reset_position = MagicMock()
#
#         src_path = os.path.dirname(os.path.realpath(__file__))
#         model_path = src_path + "/../../../soccer_object_detection/models/half_5.pt"
#
#         n = ObjectDetectionNode(model_path=model_path)
#         n.robot_state.status = RobotState.STATUS_READY
#         n.game_state.gameState = GameState.GAMESTATE_PLAYING
#
#         Camera.reset_position = MagicMock()
#         Camera.ready = MagicMock()
#         do = DetectorObjects()
#         do.robot_state.status = RobotState.STATUS_READY
#
#         cvbridge = CvBridge()
#         for file_name in os.listdir(f"{test_path}/images"):
#             print(file_name)
#             img: Mat = cv2.imread(os.path.join(f"{test_path}/images", file_name))  # ground truth box = (68, 89) (257, 275)
#             img_original_size = img.size
#             img = cv2.resize(img, dsize=(640, 480))
#             img_msg: Image = cvbridge.cv2_to_imgmsg(img)
#
#             # Mock the detections
#             n.pub_detection = MagicMock()
#             n.pub_boundingbox = MagicMock()
#             n.pub_detection.get_num_connections = MagicMock(return_value=1)
#             n.pub_boundingbox.get_num_connections = MagicMock(return_value=1)
#             n.pub_detection.publish = MagicMock()
#             n.pub_boundingbox.publish = MagicMock()
#
#             ci = CameraInfo()
#             ci.height = img.shape[0]
#             ci.width = img.shape[1]
#             n.camera.camera_info = ci
#             do.camera.camera_info = ci
#             n.camera.pose = Transformation(position=[0, 0, 0.46], orientation_euler=[0, np.pi / 8, 0])
#             do.camera.pose = n.camera.pose
#
#             # n.callback(img_msg)
#             detection_image = n.callback(img)  # send the image directly
#
#             # Check assertion
#             # if n.pub_boundingbox.publish.call_args is not None:
#             #     bounding_boxes = n.pub_boundingbox.publish.call_args[0][0]
#             #     do.objectDetectorCallback(bounding_boxes)
#
#             if "DISPLAY" in os.environ:
#                 # mat = cvbridge.imgmsg_to_cv2(n.pub_detection.publish.call_args[0][0])
#                 # cv2.imshow("Image", mat)
#                 # cv2.imshow("Image", img)
#                 cv2.imshow("Detections", detection_image)
#                 cv2.waitKey()
#
#         cv2.destroyAllWindows()

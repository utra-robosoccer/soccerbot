#!/usr/bin/env python3

import os
import time

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header
from detector import Detector
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
import math


class DetectorFieldline(Detector):
    CANNY_THRESHOLD_1 = 400
    CANNY_THRESHOLD_2 = 1000
    HOUGH_RHO = 1
    HOUGH_THETA = (np.pi / 180)
    HOUGH_THRESHOLD = 50
    HOUGH_MIN_LINE_LENGTH = 50
    HOUGH_MAX_LINE_GAP = 50

    def __init__(self):
        super().__init__()

        self.image_subscriber = rospy.Subscriber("camera/image_raw", Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher("camera/line_image", Image, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("field_point_cloud", PointCloud2, queue_size=1)
        self.goal_post_publisher = rospy.Publisher("goal_post", Bool, queue_size=1)
        self.trajectory_complete_subscriber = rospy.Subscriber("trajectory_complete", Bool,
                                                               self.trajectory_complete_callback)
        self.trajectory_complete = True

        cv2.setRNGSeed(12345)
        pass

    def trajectory_complete_callback(self, trajectory_complete: Bool):
        self.trajectory_complete = trajectory_complete

    def image_callback(self, img: Image):
        t_start = time.time()

        if not self.camera.ready() or not self.trajectory_complete:
            return

        pts = []

        self.camera.reset_position(publish_basecamera=True, timestamp=img.header.stamp)

        rgb_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        camera_info_K = np.array(self.camera.camera_info.K).reshape([3, 3])
        camera_info_D = np.array(self.camera.camera_info.D)
        image = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
        hsv = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2HSV)

        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, int(h )], [0, 0, 0], cv2.FILLED)

        # Original Method using contours

        # mask = cv2.inRange(hsv, lowerb=(45, 115, 45), upperb=(70, 255, 255))
        # (contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # bound_rects = []
        #
        #
        # for c in contours:
        #     if cv2.contourArea(c) > 1000:
        #         hull = cv2.convexHull(c)
        #         bound_rects.append(cv2.boundingRect(hull))
        #
        # # Merge largest contours
        # if len(bound_rects) > 0:
        #     def union(a, b):
        #         x = min(a[0], b[0])
        #         y = min(a[1], b[1])
        #         w = max(a[0] + a[2], b[0] + b[2]) - x
        #         h = max(a[1] + a[3], b[1] + b[3]) - y
        #         return (x, y, w, h)
        #
        #     final = bound_rects[0]
        #     for rect in bound_rects:
        #         final = union(final, rect)
        #     color = [0, 0, 255]
        #
        #     tl = final[0:2]
        #     br = (final[0] + final[2], final[1] + final[3])
        #     cv2.rectangle(image, tl, br, color, 2)
        #
        #     # Top black rectangle
        #     cv2.rectangle(image, [0, 0], [br[0], tl[1]], [0, 0, 0], cv2.FILLED, cv2.LINE_8)
        #
        #     # Bottom black rectangle
        #     # TODO Hardcoded second point needs to take in camera info
        #     cv2.rectangle(image, [tl[0], br[1]], [640, 480], [0, 0, 0], cv2.FILLED, cv2.LINE_8)

        # Field line detection
        mask2 = cv2.inRange(hsv, (0, 0, 255 - 65), (255, 65, 255))
        out = cv2.bitwise_and(image, image, mask=mask2)

        cdst = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        retval, dst = cv2.threshold(cdst, 127, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(dst, 50, 150)

        lines = cv2.HoughLinesP(edges, rho=DetectorFieldline.HOUGH_RHO,
                                theta=DetectorFieldline.HOUGH_THETA,
                                threshold=DetectorFieldline.HOUGH_THRESHOLD,
                                minLineLength=DetectorFieldline.HOUGH_MIN_LINE_LENGTH,
                                maxLineGap=DetectorFieldline.HOUGH_MAX_LINE_GAP)

        ccdst = cv2.cvtColor(cdst, cv2.COLOR_GRAY2RGB)

        if lines is None:
            return

        computed_lines = []
        vert_x_max, vert_x_min, vert_y_max, vert_y_min = 0, 1000, 0, 1000
        for l in lines:
            x1, y1, x2, y2 = l[0]
            # computing magnitude and angle of the line
            mag = np.sqrt((x2 - x1) ** 2. + (y2 - y1) ** 2.)
            if x2 == x1:
                angle = 0
            else:
                angle = np.rad2deg(np.arctan((y2 - y1) / (x2 - x1)))

            computed_lines += [(mag, angle)]

            pt1 = (x1, y1)
            pt2 = (x2, y2)
            if abs(angle) < 10:  # Horizontal
                cv2.line(ccdst, pt1, pt2, (255, 0, 0), thickness=3, lineType=cv2.LINE_AA)
            elif abs(abs(angle) - 90) < 10:  # Vertical
                cv2.line(ccdst, pt1, pt2, (0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
                vert_x_max = max(vert_x_max, x1, x2)
                vert_x_min = min(vert_x_min, x1, x2)
                vert_y_max = max(vert_y_max, y1, y2)
                vert_y_min = min(vert_y_min, y1, y2)
                print('Vertical line: ', pt1, pt2)
            else:
                cv2.line(ccdst, pt1, pt2, (0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

            if (pt2[0] - pt1[0]) == 0:
                continue
            slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = y1 - slope * x1

            for i in range(x1, x2, 15):
                y = slope * i + b
                pt = [i, y]
                pts.append(pt)

        computed_lines = np.array(computed_lines)

        # an image has a goalpost if two perpendicular lines with 0 degrees and 90 degrees intersect
        horizontal_line = len(computed_lines[(abs(computed_lines[:, 1]) < 10)]) > 0
        vertical_line = len(computed_lines[(abs(abs(computed_lines[:, 1]) - 90) < 10)]) > 0
        if vertical_line:
            w = vert_y_max - vert_y_min
            l = vert_x_max - vert_x_min
            area = w*l
            if area < 50000:
                cv2.rectangle(ccdst, [vert_x_min, vert_y_min], [vert_x_max, vert_y_max], [0, 255, 255], thickness=2)
                # boundingBoxes = [[vert_x_min, vert_y_min], [vert_x_max, vert_y_max]]
                # position = self.camera.calculateBallFromBoundingBoxes(0.5, boundingBoxes)
                #
                # br = tf2_ros.TransformBroadcaster()
                # ball_pose = TransformStamped()
                # ball_pose.header.frame_id = self.robot_name + "/base_camera"
                # ball_pose.child_frame_id = self.robot_name + "/goal_post"
                # ball_pose.header.stamp = img.header.stamp
                # ball_pose.header.seq = img.header.seq
                # ball_pose.transform.translation.x = position.get_position()[0]
                # ball_pose.transform.translation.y = position.get_position()[1]
                # ball_pose.transform.translation.z = 0
                # ball_pose.transform.rotation.x = 0
                # ball_pose.transform.rotation.y = 0
                # ball_pose.transform.rotation.z = 0
                # ball_pose.transform.rotation.w = 1
                # br.sendTransform(ball_pose)
                x_avg = (vert_x_max + vert_x_min)/2
                [floor_center_x, floor_center_y, _] = self.camera.findFloorCoordinate([x_avg, vert_y_max])
                [floor_close_x, floor_close_y, _] = self.camera.findFloorCoordinate([x_avg, vert_y_max])

                camera_pose = self.camera.pose

                distance = ((floor_center_x - camera_pose.get_position()[0]) ** 2 + (
                        floor_center_y - camera_pose.get_position()[1]) ** 2) ** 0.5
                theta = math.atan2(distance, camera_pose.get_position()[2])
                ratio = math.tan(theta) ** 2
                ratio2 = 1 / (1 + ratio)
                if 1 < ratio2 < 0:
                    print('here')

                floor_x = floor_close_x * (1 - ratio2) + floor_center_x * ratio2
                floor_y = floor_close_y * (1 - ratio2) + floor_center_y * ratio2
                br = tf2_ros.TransformBroadcaster()
                robot_pose = TransformStamped()
                robot_pose.header.frame_id = self.robot_name + "/base_camera"
                robot_pose.child_frame_id = self.robot_name + "/goal_post"
                robot_pose.header.stamp = img.header.stamp
                robot_pose.header.seq = img.header.seq
                robot_pose.transform.translation.x = floor_x
                robot_pose.transform.translation.y = floor_y
                robot_pose.transform.translation.z = 0
                robot_pose.transform.rotation.x = 0
                robot_pose.transform.rotation.y = 0
                robot_pose.transform.rotation.z = 0
                robot_pose.transform.rotation.w = 1
                br.sendTransform(robot_pose)
            print('Area: ', area)
        print('Horiz: ', len(computed_lines[(abs(computed_lines[:, 1]) < 10)]))
        print('Vert: ', len(computed_lines[(abs(abs(computed_lines[:, 1]) - 90) < 10)]))

        has_goalpost = Bool()
        has_goalpost.data = horizontal_line and vertical_line

        # Publish Goal post in image
        self.goal_post_publisher.publish(has_goalpost)

        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(ccdst)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        if self.point_cloud_publisher.get_num_connections() > 0:
            points3d = []
            for p in pts:
                points3d.append(self.camera.findFloorCoordinate(p))
            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = img.header.stamp
            header.frame_id = self.robot_name + "/base_camera"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            if self.point_cloud_publisher.get_num_connections() > 0:
                self.point_cloud_publisher.publish(point_cloud_msg)

        t_end = time.time()
        rospy.loginfo_throttle(20, "Fieldline detection rate: " + str(t_end - t_start))


if __name__ == '__main__':
    rospy.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldline()
    rospy.spin()

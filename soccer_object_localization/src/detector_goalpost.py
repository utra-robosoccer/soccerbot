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
from cv_bridge import CvBridge
import math
from detector_fieldline import DetectorFieldline


class DetectorGoalPost(DetectorFieldline):

    def __init__(self):
        super().__init__()

        self.image_subscriber = rospy.Subscriber("camera/image_raw", Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher("camera/goal_image", Image, queue_size=1)
        self.goal_post_need_subscriber = rospy.Subscriber("goal_post_need", Bool, self.goal_post_need_callback, queue_size=1)
        self.goal_post_need = False
        cv2.setRNGSeed(12345)
        pass

    def goal_post_need_callback (self, data: Bool):
        self.goal_post_need = data.data
        pass

    def image_callback(self, img: Image):
        t_start = time.time()

        if not self.camera.ready() or not self.trajectory_complete or not self.goal_post_need:
            return

        self.camera.reset_position(publish_basecamera=False, timestamp=img.header.stamp)

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        hsv = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2HSV)

        h = self.camera.calculateHorizonCoverArea()
        cv2.rectangle(image, [0, 0], [640, int(h * 7 / 10.0)], [0, 0, 0], cv2.FILLED)

        # Field line detection
        mask2 = cv2.inRange(hsv, (0, 0, 255 - 65), (255, 65, 255))
        out = cv2.bitwise_and(image, image, mask=mask2)

        kernel = np.ones((7, 7), np.uint8)
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)

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
            else:
                cv2.line(ccdst, pt1, pt2, (0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

        computed_lines = np.array(computed_lines)

        # an image has a goalpost if two perpendicular lines with 0 degrees and 90 degrees intersect
        vertical_line = len(computed_lines[(abs(abs(computed_lines[:, 1]) - 90) < 10)]) > 0

        if vertical_line:
            w = vert_y_max - vert_y_min
            l = vert_x_max - vert_x_min
            area = w * l
            if area < 50000:
                cv2.rectangle(ccdst, [vert_x_min, vert_y_min], [vert_x_max, vert_y_max], [0, 255, 255], thickness=2)
                x_avg = (vert_x_max + vert_x_min) / 2
                [floor_center_x, floor_center_y, _] = self.camera.findFloorCoordinate([x_avg, vert_y_max])
                [floor_close_x, floor_close_y, _] = self.camera.findFloorCoordinate([x_avg, vert_y_max])

                camera_pose = self.camera.pose

                distance = ((floor_center_x - camera_pose.get_position()[0]) ** 2 + (
                        floor_center_y - camera_pose.get_position()[1]) ** 2) ** 0.5
                theta = math.atan2(distance, camera_pose.get_position()[2])
                ratio = math.tan(theta) ** 2
                ratio2 = 1 / (1 + ratio)
                if 1 < ratio2 < 0:
                    print('here')  # TODO

                floor_x = floor_close_x * (1 - ratio2) + floor_center_x * ratio2
                floor_y = floor_close_y * (1 - ratio2) + floor_center_y * ratio2
                if floor_x > 0.0:

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


        if self.image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(ccdst)
            img_out.header = img.header
            self.image_publisher.publish(img_out)

        t_end = time.time()
        rospy.loginfo_throttle(20, "GoalPost detection rate: " + str(t_end - t_start))


if __name__ == '__main__':
    rospy.init_node("detector_goalpost")
    fieldline_detector = DetectorGoalPost()
    rospy.spin()

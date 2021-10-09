#!/usr/bin/env python3

import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy

import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header
from detector import Detector
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge

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

        self.image_subscriber = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.image_publisher = rospy.Publisher("camera/line_image", Image, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("field_point_cloud", PointCloud2, queue_size=1)
        self.trajectory_complete_subscriber = rospy.Subscriber("trajectory_complete", Bool, self.trajectory_complete_callback)
        self.trajectory_complete = True
        pass

    def trajectory_complete_callback(self, trajectory_complete: Bool):
        self.trajectory_complete = trajectory_complete

    def image_callback(self, img: Image):
        if not self.camera.ready() or not self.trajectory_complete:
            return

        pts = []

        self.camera.reset_position()

        rgb_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        camera_info_K = np.array(self.camera.camera_info.K).reshape([3, 3])
        camera_info_D = np.array(self.camera.camera_info.D)
        image = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
        hsv = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        (contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        bound_rects = []

        cv2.setRNGSeed(12345)

        for c in contours:
            if cv2.contourArea(c) > 1000:
                hull = cv2.convexHull(c)
                bound_rects.append(cv2.boundingRect(hull))

        # Merge largest contours
        if len(bound_rects) > 0:
            final = bound_rects[0]
            for rect in bound_rects:
                final |= rect
            color = [0, 0, 255]
            cv2.rectangle(image, final.tl(), final.br(), color, 2)

            # Top black rectangle
            cv2.rectangle(image, [0, 0], [final.br(), final.tl().y], [0, 0, 0], cv2.FILLED, cv2.LINE_8)

            # Bottom black rectangle
            # TODO Hardcoded second point needs to take in camera info
            cv2.rectangle(image, [final.br(), final.tl().y], [640, 480], [0, 0, 0], cv2.FILLED, cv2.LINE_8)

        # Field line detection
        mask2 = cv2.inRange(hsv, (0, 0, 255 - 65), (255, 65, 255))
        out = cv2.bitwise_and(image, image, mask=mask2)

        cdst = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        retval, dst = cv2.threshold(cdst, 127, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(dst, 50, 150)

        lines = cv2.HoughLinesP(edges,rho=DetectorFieldline.HOUGH_RHO,
                                theta=DetectorFieldline.HOUGH_THETA,
                                threshold=DetectorFieldline.HOUGH_THRESHOLD,
                                minLineLength=DetectorFieldline.HOUGH_MIN_LINE_LENGTH,
                                maxLineGap=DetectorFieldline.HOUGH_MAX_LINE_GAP)
        ccdst = cv2.cvtColor(cdst, cv2.COLOR_GRAY2RGB)

        if lines is None:
            return

        for l in lines:
            x1, y1, x2, y2 = l[0]
            pt1 = (x1, y1)
            pt2 = (x2, y2)
            cv2.line(ccdst, pt1, pt2, (0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

            if (pt2[0] - pt1[0]) == 0:
                continue
            slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = y1 - slope * x1

            for i in range(x1, x2, 15):
                y = slope * i + b
                pt = [i, y]
                pts.append(pt)

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
            self.point_cloud_publisher.publish(point_cloud_msg)


if __name__ == '__main__':
    rospy.init_node("detector_fieldline")
    fieldline_detector = DetectorFieldline()
    rospy.spin()
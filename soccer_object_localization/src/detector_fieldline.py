import numpy as np
import rospy
import cv2
import tf2_py
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState, Image, PointCloud2
from soccer_object_detection.msg import BoundingBoxes
from std_msgs.msg import Bool, Header
from detector import Detector
import sensor_msgs.point_cloud2 as pcl2

class FieldlineDetector(Detector):
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
        self.image_publisher = rospy.Publisher("camera/line_image", Image)
        self.point_cloud_publisher = rospy.Publisher("field_point_cloud", PointCloud2)
        self.trajectory_complete_subscriber = rospy.Subscriber("trajectory_complete", Bool, self.trajectory_complete_callback)
        pass

    def trajectory_complete_callback(self, trajectory_complete: Bool):
        self.trajectory_complete = trajectory_complete

    def image_callback(self, img: Image):
        if not self.camera.ready or not self.trajectory_complete:
            return

        lines = []
        pts = []

        self.camera.reset_position()

        image = img.data
        hsv = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        countours_poly = []
        bound_rects = []
        centers = []
        radius = []

        cv2.setRNGSeed(12345)

        for c in contours:
            if cv2.contourArea(c) > 1000:
                hull = cv2.convexHull(c)
                bound_rects.append(cv2.boundingRect(hull))

        # Merge largest contours
        final = None
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
        mask2 = cv2.inRange(hsv, [0, 0, 255 - 65], [255, 65, 255])
        out = cv2.bitwise_and(image, image, mask=mask2)

        cdst = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        dst = cv2.threshold(cdst, 126, 255, cv2.THRESH_BINARY)
        cv2.Canny(dst, 50, 150)

        lines = cv2.HoughLinesP(cdst, FieldlineDetector.HOUGH_RHO,
                                FieldlineDetector.HOUGH_THETA,
                                FieldlineDetector.HOUGH_THRESHOLD,
                                FieldlineDetector.HOUGH_MIN_LINE_LENGTH,
                                FieldlineDetector.HOUGH_MAX_LINE_GAP)
        dst = cv2.cvtColor(cdst, cv2.COLOR_GRAY2RGB)

        for l in lines:
            pt1 = [l[0], l[1]]
            pt2 = [l[2], l[3]]
            cv2.line(dst, pt1, pt2, [0, 0, 255], thickness=3, lineType=cv2.LINE_AA)

            slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = l[1] - slope * l[0]

            for i in range(l[0], l[2], 15):
                y = slope * i + b
                pt = [i, y]
                pts.append(pt)

        img_out = img
        img_out.data = dst
        self.image_publisher.publish(img_out)

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


if __name__ == "__main__":
    rospy.init_node("ball_detector")
    ball_detector = FieldlineDetector()
    rospy.spin()
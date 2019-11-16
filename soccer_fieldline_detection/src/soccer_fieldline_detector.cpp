#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_fieldline_detector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry/segment2.hpp>

SoccerFieldlineDetector::SoccerFieldlineDetector() {
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("front_camera/image_raw", 1, &SoccerFieldlineDetector::imageCallback, this);
}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    try {
        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Point start = cv::Point(300, 300);
        cv::Point end = cv::Point(500, 500);
        cv::line(image, start, end, cv::Scalar(0, 0, 0), 3);

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        int cannythreshold1 = 200;
        int cannythreshold2 = 400;
        nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
        nh.getParam("soccer_fieldline_detector/cannythreshold2", cannythreshold2);

        cv::Canny(
                image,
                dst,
                cannythreshold1,
                cannythreshold2);

        cvtColor(dst, cdst, CV_GRAY2BGR);

        // Cover Horizon (Ignore for now)

        // Organize Fieldlines
        std::vector<cv::Vec4i> lines;
        double rho = 1;
        double theta = CV_PI / 180;
        int threshold = 50;
        int minLineLength = 50;
        int maxLineGap = 50;
        nh.getParam("soccer_fieldline_detector/houghRho", rho);
        nh.getParam("soccer_fieldline_detector/houghTheta", theta);
        nh.getParam("soccer_fieldline_detector/houghThreshold", threshold);
        nh.getParam("soccer_fieldline_detector/houghMinLineLength", minLineLength);
        nh.getParam("soccer_fieldline_detector/houghMaxLineGap", maxLineGap);

        ROS_INFO_STREAM(theta);

        HoughLinesP(
                dst,
                lines,
                rho,
                theta,
                threshold,
                minLineLength,
                maxLineGap);

        std::vector<Point2> pts;

        for (auto l : lines) {
            cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);

            Point2 pt1(l[0],l[1]);
            Point2 pt2(l[2],l[3]);
            Segment2 s(pt1,pt2);
            float b = l[1] - s.slope()*l[0];

            for (int i = l[0]; i < l[2];i++) {
                float y = s.slope()*i + b;
                Point2 pt(i,y);
                pts.push_back(pt);

            }

        }

        ROS_INFO_STREAM("Line n: " + std::to_string(lines.size()));

        // Project fieldlines from 2d to 3d
        Pose3 pose_msgs;
        pose_msgs.position.x = -0.5;
        pose_msgs.position.y = -0.5;
        pose_msgs.position.z = 0.5;

        pose_msgs.orientation.w = 0.8536;
        pose_msgs.orientation.x = -0.1464;
        pose_msgs.orientation.y = 0.3536;
        pose_msgs.orientation.z = 0.3536;
        pose_msgs.orientation.w = 0.8536;

        Camera cam (pose_msgs,240,360);
        std::vector<Point3> rPts;
        for (auto p : pts) {
            Point3 p2 = cam.FindFloorCoordinate(p.x, p.y);
            rPts.push_back(p2);
        }

        // Publish fieldlines in a LaserScan data format
        ros::NodeHandle n;

//        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

        cv::imshow("original view", image);
        cv::imshow("edge view", cdst);
        cv::waitKey(1);

    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("CV Exception" << e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    ros::spin();

    return 0;
}
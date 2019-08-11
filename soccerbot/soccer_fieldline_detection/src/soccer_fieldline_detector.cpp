#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_fieldline_detector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
        nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
        cv::Canny(image, dst, cannythreshold1, 400);
        cvtColor(dst, cdst, CV_GRAY2BGR);

        // Cover Horizon (Ignore for now)

        // Organize Fieldlines
        std::vector<cv::Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 50, 10);
        for (int i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
        }
        ROS_INFO_STREAM("Line n: " + std::to_string(lines.size()));

        // Project fieldlines from 2d to 3d

        // Publish fieldlines in a LaserScan data format

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
#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_fieldline_detector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

SoccerFieldlineDetector::SoccerFieldlineDetector() {
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("front_camera/image_raw", 1, &SoccerFieldlineDetector::imageCallback, this);
}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_INFO_STREAM("received image");

    try {
        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat inverted;
        cv::bitwise_not(image, inverted);
        cv::imshow("view", inverted);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    // Cover Horizon (Ignore for now)

    // Detect Field Lines (Copy from simulink)

    // Organize Fieldlines

    // Project fieldlines from 2d to 3d

    // Publish fieldlines in a LaserScan data format
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    ros::spin();

    return 0;
}
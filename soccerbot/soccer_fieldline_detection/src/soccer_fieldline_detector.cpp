#include <ros/ros.h>
#include <soccer_fieldline_detection/soccer_fieldline_detector.hpp>

SoccerFieldlineDetector::SoccerFieldlineDetector() {
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("in_image_base_topic", 1, &SoccerFieldlineDetector::imageCallback, this);
}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    ros::spin();

    return 0;
}
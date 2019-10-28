#include <ros/ros.h>

void ballDetectorCallback() {
    ROS_INFO("received image");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");
    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe("chatter", 1000, ballDetectorCallback);
    ROS_INFO("hello i'm ball detector!");
    ros::spin();
    return 0;
}
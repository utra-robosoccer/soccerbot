#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <vector>

void ballDetectorCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    std::vector <darknet_ros_msgs::BoundingBox> b = msg->bounding_boxes;
    for (darknet_ros_msgs::BoundingBox box: b) {
        ROS_INFO("%s", box.Class.data());
    }

    ROS_INFO("received image");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("darknet_ros/bounding_boxes", 1000, ballDetectorCallback);
    ROS_INFO("hello i'm ball detector!");
    ros::spin();
    return 0;
}
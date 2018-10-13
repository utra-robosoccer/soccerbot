//
// Created by vuwij on 04/08/18.
//

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace geometry_msgs;
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "soccer_transforms_node");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);

    ros::Publisher headLocationPublisher = nh.advertise<TransformStamped>("poses/camera", 1);

    ros::Rate rate(100.0);

    while(nh.ok()) {
        try {
            TransformStamped robotHeadLocation = tfBuffer.lookupTransform("base_link", "camera", ros::Time(0));
            headLocationPublisher.publish(robotHeadLocation);
        } catch (tf2::TransformException & ex) {
            ROS_ERROR_STREAM(ex.what());
            rate.sleep();
            continue;
        }

        rate.sleep();
    }

    return 0;
}
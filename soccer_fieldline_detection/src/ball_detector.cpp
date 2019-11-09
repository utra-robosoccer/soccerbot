#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <vector>
#include <string>
#include <geometry/pose3.hpp>
#include <geometry_msgs/Pose.h>
#include <soccer_fieldline_detection/camera.hpp>

class BallDetector {
public:
    ros::NodeHandle n;
    ros::Subscriber darknet_pose_sub;
    ros::Publisher ball_pub;
    BallDetector() {
        darknet_pose_sub = n.subscribe("darknet_ros/bounding_boxes", 1000, &BallDetector::ballDetectorCallback, this);
        ball_pub = n.advertise<geometry_msgs::Pose>("ball_pose", 1);
    }

private:
    void ballDetectorCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        std::vector <darknet_ros_msgs::BoundingBox> b = msg->bounding_boxes;

        // Project fieldlines from 2d to 3d
        Pose3 camera_position;
        camera_position.position.x = -0.5;
        camera_position.position.y = -0.5;
        camera_position.position.z = 0.5;

        camera_position.orientation.w = 0.8536;
        camera_position.orientation.x = -0.1464;
        camera_position.orientation.y = 0.3536;
        camera_position.orientation.z = 0.3536;

        Camera cam (camera_position,240,360);

        for (darknet_ros_msgs::BoundingBox box: b) {
            //ROS_INFO("found a %s", box.Class.data());
            std::string objectClass = box.Class;
            if(objectClass != "person") {
                continue;
            }
            // For now take the center of the box
            int xavg = (box.xmin + box.xmax) / 2;
            int yavg = (box.ymin + box.ymax) / 2;
            Point3 floor_coordinate =  cam.FindFloorCoordinate(xavg,yavg);
            geometry_msgs::Pose ball_pose;
            ball_pose.position.x = floor_coordinate.x;
            ball_pose.position.y = floor_coordinate.y;
            ball_pose.position.z = floor_coordinate.z;
            ball_pub.publish(ball_pose);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");

    BallDetector ballDetector;

    ros::spin();
    return 0;
}
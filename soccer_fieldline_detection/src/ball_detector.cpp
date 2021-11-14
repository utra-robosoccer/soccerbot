#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <soccer_object_detection/BoundingBoxes.h>
#include <soccer_object_detection/BoundingBox.h>
#include <string>
#include <soccer_geometry/pose3.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <soccer_fieldline_detection/camera.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>
class BallDetector {
public:
    ros::NodeHandle n;
    ros::Subscriber boundingBoxesSub;
    ros::Subscriber head_motor_1;
    ros::Publisher ballPixelPublisher;
    ros::Publisher robotPosePub;
    std::unique_ptr<Camera> camera;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster br;
    std::string robotName;
    float angle = 0;
    BallDetector() : tfListener(tfBuffer) {
        boundingBoxesSub = n.subscribe("object_bounding_boxes", 1, &BallDetector::ballDetectorCallback, this);
        ballPixelPublisher = n.advertise<geometry_msgs::PointStamped>("ball_pixel", 1);
        robotPosePub = n.advertise<geometry_msgs::PoseStamped>("detected_robot_pose", 10);
        head_motor_1 = n.subscribe("joint_states", 1, &BallDetector::headTilt, this);
        geometry_msgs::TransformStamped camera_pose;

        robotName = ros::this_node::getNamespace();
        robotName.erase(0, 1);

        while (ros::ok()) {
            try {
                camera_pose = tfBuffer.lookupTransform(robotName + "/camera", robotName + "/base_footprint",
                                                       ros::Time(0), ros::Duration(1.0));
                break;
            }
            catch (tf2::TransformException &ex) {
                std::string s = ex.what();
            }
        }

        // Initalize Camera
        Pose3 camera_position;
        camera_position.position.x = camera_pose.transform.translation.x;
        camera_position.position.y = camera_pose.transform.translation.y;
        camera_position.position.z = camera_pose.transform.translation.z;
        camera_position.orientation.w = camera_pose.transform.rotation.w;
        camera_position.orientation.x = camera_pose.transform.rotation.x;
        camera_position.orientation.y = camera_pose.transform.rotation.y;
        camera_position.orientation.z = camera_pose.transform.rotation.z;
        camera = std::make_unique<Camera>(camera_position);

    }
    void headTilt(const sensor_msgs::JointState &msg) {
        int count = 0;
        for (auto i : msg.name) {
            if (i == "head_motor_1") {
                angle = msg.position[count];
            }
            count += 1;
        }
    }
private:
    void ballDetectorCallback(const soccer_object_detection::BoundingBoxes::ConstPtr &msg) {
        // Get transformation
        geometry_msgs::TransformStamped camera_pose;
        try {
            camera_pose = tfBuffer.lookupTransform(robotName + "/base_footprint", robotName + "/camera",
                                                   ros::Time(0), ros::Duration(0.1));
            Pose3 camera_position;
            camera_position.position.x = camera_pose.transform.translation.x;
            camera_position.position.y = camera_pose.transform.translation.y;
            camera_position.position.z = camera_pose.transform.translation.z;

            camera_position.orientation.w = camera_pose.transform.rotation.w;
            camera_position.orientation.x = camera_pose.transform.rotation.x;
            camera_position.orientation.y = camera_pose.transform.rotation.y;
            camera_position.orientation.z = camera_pose.transform.rotation.z;

            camera->setPose(camera_position);
        }
        catch (tf2::TransformException &ex) {
            return;
        }

        if (!camera->ready()) {
            return;
        }

        auto bounding_boxes = msg->bounding_boxes;
        int detected_robots = 0;
        for (soccer_object_detection::BoundingBox &box : bounding_boxes) {
                std::string objectClass = box.Class;
//            int xavg2 = (box.xmin + box.xmax) / 2;
//            int yavg2 = (box.ymin + box.ymax) / 2;
//            float area = (box.xmax - box.xmin) * (box.ymax - box.ymin);
//
//            Point3 floor_coordinate = camera->FindFloorCoordinate(xavg2, yavg2);
//            // Reduce false positives
//            if (area < 3500.0 and float(angle) > 0.6  ) {
//                continue;
//            }
//            else if (area < 400.0 ) {
//                continue;
//            }

                if (objectClass == "ball") {

                    float xavg = (box.xmin + box.xmax) / 2;
                    float yavg = 0.5 * box.ymax + 0.5 * box.ymin;
                    float yclose = box.ymax;

                    Point3 floor_coordinate_center = camera->FindFloorCoordinate(xavg, yavg);
                    Point3 floor_coordinate_close = camera->FindFloorCoordinate(xavg, yclose);

                    Pose3 camera_pose = camera->getPose();
                    float camera_height = camera_pose.position.z;
                    float distance = sqrt(pow(floor_coordinate_center.x - camera_pose.position.x, 2) + pow(floor_coordinate_center.y - camera_pose.position.y, 2));
                    float theta = atan2(distance, camera_height);
                    float ratio = pow(tan(theta),2);
                    float ratio2 = 1 / (1 + ratio);
                    if (ratio2 > 1 && ratio2 < 0) {
                        continue;
                    }
                    Point3 floor_coordinate = floor_coordinate_close;
                    floor_coordinate.x = floor_coordinate.x * (1 - ratio2) + floor_coordinate_center.x * ratio2;
                    floor_coordinate.y = floor_coordinate.y * (1 - ratio2) + floor_coordinate_center.y * ratio2;

                    geometry_msgs::TransformStamped ball_pose;
                    ball_pose.header.frame_id = robotName + "/base_footprint";
                    ball_pose.child_frame_id = robotName + "/ball";
                    ball_pose.header.stamp = msg->header.stamp;
                    ball_pose.header.seq = msg->header.seq;
                    ball_pose.transform.translation.x = floor_coordinate.x;
                    ball_pose.transform.translation.y = floor_coordinate.y;
                    ball_pose.transform.translation.z = floor_coordinate.z;
                    ball_pose.transform.rotation.x = 0;
                    ball_pose.transform.rotation.y = 0;
                    ball_pose.transform.rotation.z = 0;
                    ball_pose.transform.rotation.w = 1;
                    BallDetector::br.sendTransform(ball_pose);

                    geometry_msgs::PointStamped ball_pixel;
                    ball_pixel.header.frame_id = robotName + "/base_footprint";
                    ball_pixel.header.seq = msg->header.seq;
                    ball_pixel.header.stamp = msg->header.stamp;
                    ball_pixel.point.x = xavg;
                    ball_pixel.point.y = yavg;
                    ball_pixel.point.z = 0;
                    ballPixelPublisher.publish(ball_pixel);



                }

                if (objectClass == "robot") {
                    if (angle > 0.6) {
                        continue; // Probably looking at self arms
                    }

                    float xavg = (box.xmin + box.xmax) / 2;
                    float yavg = 0.5 * box.ymax + 0.5 * box.ymin;
                    float yclose = box.ymax;

                    float robot_length = abs(box.xmax - box.xmin);
                    float robot_width = abs(box.ymax - box.ymin);
                    if (robot_length <= 0 or robot_width <= 0) {
                        continue;
                    }
                    float length_to_width_ratio_in_full_view = 68 / 22;
                    float foot_ratio_of_length = 0.2;
                    float foot_pixel = box.ymax;
                    bool robot_standing_up = true;
                    if (robot_standing_up) {

                        // Side obstructed
                        if (box.xmax == 640) {
                            // right side obstructed
                            robot_width = robot_length / length_to_width_ratio_in_full_view;
                            box.xmax = box.xmin + robot_length;
                        } else if (box.xmin == 0) {
                            // left side obstructed
                            robot_width = robot_length / length_to_width_ratio_in_full_view;
                            box.xmin = box.xmax - robot_length;
                        }

                        // If entire robot in field of view
                        if (box.ymax == 480 and box.ymin == 0) {
                            // Top and bottom both obstructed
                            // TODO dont know what to do at this moment, assume 50 50 top bottom
                            float desired_robot_length = length_to_width_ratio_in_full_view * robot_width;
                            float additional_robot_length = desired_robot_length - robot_length;
                            box.ymax = box.ymax + additional_robot_length / 2;
                            box.ymin = box.ymin - additional_robot_length / 2;
                        } else if (box.ymax == 480) {
                            // bottom obstructed
                            robot_length = length_to_width_ratio_in_full_view * robot_width;
                            box.ymax = box.ymin + robot_length;
                        } else if (box.ymin == 0) {
                            // top obstructed
                            robot_length = length_to_width_ratio_in_full_view * robot_width;
                            box.ymin = box.ymax - robot_length;
                        }
                        foot_pixel = box.ymax - robot_length * foot_ratio_of_length;
                    }

                    Point3 floor_coordinate_center = camera->FindFloorCoordinate(xavg, foot_pixel);
                    Point3 floor_coordinate_close = camera->FindFloorCoordinate(xavg, yclose);

                    Pose3 camera_pose = camera->getPose();
                    float camera_height = camera_pose.position.z;
                    float distance = sqrt(pow(floor_coordinate_center.x - camera_pose.position.x, 2) + pow(floor_coordinate_center.y - camera_pose.position.y, 2));
                    float theta = atan2(distance, camera_height);
                    float ratio = pow(tan(theta),2);
                    float ratio2 = 1 / (1 + ratio);
                    if (ratio2 > 1 && ratio2 < 0) {
                        continue;
                    }
                    Point3 floor_coordinate = floor_coordinate_close;
                    floor_coordinate.x = floor_coordinate.x * (1 - ratio2) + floor_coordinate_center.x * ratio2;
                    floor_coordinate.y = floor_coordinate.y * (1 - ratio2) + floor_coordinate_center.y * ratio2;

//                    geometry_msgs::TransformStamped robot_pose;
//                    robot_pose.header.frame_id = robotName + "/base_footprint";
//                    robot_pose.child_frame_id = robotName + "/detected_robot_" + std::to_string(detected_robots);
//                    robot_pose.header.stamp = msg->header.stamp;
//                    robot_pose.header.seq = msg->header.seq;
//                    robot_pose.transform.translation.x = floor_coordinate.x;
//                    robot_pose.transform.translation.y = floor_coordinate.y;
//                    robot_pose.transform.translation.z = floor_coordinate.z;
//                    robot_pose.transform.rotation.x = 0;
//                    robot_pose.transform.rotation.y = 0;
//                    robot_pose.transform.rotation.z = 0;
//                    robot_pose.transform.rotation.w = 1;
//                    BallDetector::br.sendTransform(robot_pose);
//                    detected_robots = detected_robots + 1;

//                    geometry_msgs::PoseStamped robot_pose;
//                    robot_pose.header.frame_id = robotName + "/base_footprint";
//                    robot_pose.header.seq = msg->header.seq;
//                    robot_pose.header.stamp = msg->header.stamp;
//                    robot_pose.pose.position.x = floor_coordinate.x;
//                    robot_pose.pose.position.y = floor_coordinate.y;
//                    robot_pose.pose.position.z = 0;
//                    robot_pose.pose.orientation.x = 0;
//                    robot_pose.pose.orientation.y = 0;
//                    robot_pose.pose.orientation.z = 0;
//                    robot_pose.pose.orientation.w = 1;
//                    robotPosePub.publish(robot_pose);
                }
            }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");

    BallDetector ballDetector;

    ros::spin();
    return 0;
}
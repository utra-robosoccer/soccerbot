#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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

    ros::Publisher robotPosePub;
    std::unique_ptr<Camera> camera;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster br;
    std::string robotName;
    float angle = 0;
    BallDetector() : tfListener(tfBuffer) {
        boundingBoxesSub = n.subscribe("object_bounding_boxes", 1, &BallDetector::ballDetectorCallback, this);
        robotPosePub = n.advertise<geometry_msgs::Pose>("detected_robot_pose", 1);
        head_motor_1 = n.subscribe("joint_states", 1, &BallDetector::headTilt, this);
        geometry_msgs::TransformStamped camera_pose;

        robotName = ros::this_node::getNamespace();
        robotName.erase(0, 1);

        while (ros::ok()) {
            try {
                camera_pose = tfBuffer.lookupTransform(robotName + "/camera", robotName + "/odom",
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
            camera_pose = tfBuffer.lookupTransform(robotName + "/odom", robotName + "/camera",
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

        for (const soccer_object_detection::BoundingBox &box : msg->bounding_boxes) {
                std::string objectClass = box.Class;
                if (objectClass != "ball" && objectClass != "robot") {
                    continue;
                }
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
                assert(ratio2 >= 0 && ratio2 <= 1);

                Point3 floor_coordinate = floor_coordinate_close;
                floor_coordinate.x = floor_coordinate.x * (1 - ratio2) + floor_coordinate_center.x * ratio2;
                floor_coordinate.y = floor_coordinate.y * (1 - ratio2) + floor_coordinate_center.y * ratio2;



                // Reduce false positives
//                if (area < 3500.0 and float(angle) > 0.6  ) {
//                    continue;
//                }
//                else if (area < 400.0 ) {
//                    continue;
//                }
                if (objectClass == "ball") {
//                    std::cout << "Ball: X:" << xavg << " Y:" << yavg << " Area " << area << "  " << float(angle) << std::endl;
                    geometry_msgs::TransformStamped ball_pose;
                    ball_pose.header.frame_id = robotName + "/odom";
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
                } else {
                    if (float(angle) == float(0.6)) {
                        geometry_msgs::Pose robot_pose;
                        robot_pose.position.x = floor_coordinate.x;
                        robot_pose.position.y = floor_coordinate.y;
                        robotPosePub.publish(robot_pose);

                    }
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
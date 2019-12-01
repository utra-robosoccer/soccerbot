#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class BallDetector {
    ros::Publisher head_rotator_0;
    ros::Publisher head_rotator_1;
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    float frequency = 0.2f;
    float max_angle = M_PI / 4.f;
public:
    BallDetector() : tfListener(tfBuffer) {
        head_rotator_0 = n.advertise<std_msgs::Float64>("head_motor_0/command", 1);
        head_rotator_1 = n.advertise<std_msgs::Float64>("head_motor_1/command", 1);
    }

    void move_head (const int t){
        geometry_msgs::TransformStamped camera_pose;

        bool has_pose = false;
        try{
            camera_pose = tfBuffer.lookupTransform("camera", "base_link",
                                                   ros::Time(0));
            has_pose = true;
        }
        catch (tf2::TransformException &ex) {
        }
        if (has_pose && camera_pose.header.stamp - ros::Time::now() < ros::Duration(5)) {
            ROS_INFO_STREAM(camera_pose.header.stamp - ros::Time::now());
            return;
        }
        std_msgs::Float64 angle;
        angle.data = max_angle * std::sin(static_cast<float>(t) / 100.f * frequency);
        head_rotator_0.publish(angle);

        angle.data = 0.3f;
        head_rotator_1.publish(angle);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");

    BallDetector ballDetector;

    ros::Rate r(100);
    int t = 0;
    while (ros::ok()) {
        ballDetector.move_head(t++);
        r.sleep();
    }
    ros::spin();
    return 0;
}
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

class BallDetector {
    ros::Publisher head_rotator_0;
    ros::Publisher head_rotator_1;
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    float frequency = 0.2f;
    float max_angle = M_PI / 4.f;
    int last_t = 0;
public:
    BallDetector() : tfListener(tfBuffer) {
        head_rotator_0 = n.advertise<std_msgs::Float64>("head_motor_0/command", 1);
        head_rotator_1 = n.advertise<std_msgs::Float64>("head_motor_1/command", 1);
    }

    void move_head(){
        geometry_msgs::TransformStamped ball_pose;

        bool has_pose = false;
        try {
            ball_pose = tfBuffer.lookupTransform("ball", "torso",ros::Time(0));
            has_pose = true;
        } catch (tf2::TransformException &ex) {}

        ros::Duration last_pose = ros::Time::now() - ball_pose.header.stamp;
        if (has_pose && last_pose < ros::Duration(1)) {
            return;
        }

        std_msgs::Float64 angle;
        angle.data = max_angle * std::sin(static_cast<float>(last_t) / 100.f * frequency);
        head_rotator_0.publish(angle);

        angle.data = 0.6f;
        head_rotator_1.publish(angle);
        last_t += 1;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");

    BallDetector ballDetector;

    ros::Rate r(100);
    while (ros::ok()) {
        ballDetector.move_head();
        r.sleep();
    }
    ros::spin();
    return 0;
}
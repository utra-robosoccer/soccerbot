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

    int move_head (const int t){
        geometry_msgs::TransformStamped camera_pose;

        bool has_pose = false;
        try{
            camera_pose = tfBuffer.lookupTransform("ball", "base_link",
                                                   ros::Time(0));
            ROS_INFO_STREAM(camera_pose.header.frame_id);
            ROS_INFO_STREAM(camera_pose.header.stamp - ros::Time::now());
            has_pose = true;
        }
        catch (tf2::TransformException &ex) {
        }
        ros::Duration last_t = ros::Time::now() - camera_pose.header.stamp;
        if (has_pose && last_t < ros::Duration(5)) {
            ROS_INFO_STREAM(ros::Time::now());
            return t;
        }
        std_msgs::Float64 angle;
        angle.data = max_angle * std::sin(static_cast<float>(t) / 100.f * frequency);
        head_rotator_0.publish(angle);

        angle.data = 0.3f;
        head_rotator_1.publish(angle);
        return t+1;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");

    BallDetector ballDetector;

    ros::Rate r(100);
    int t = 0;
    while (ros::ok()) {
        t = ballDetector.move_head(t);
        r.sleep();
    }
    ros::spin();
    return 0;
}
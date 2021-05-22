#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>



#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

class BallDetector {
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Publisher head_rotator;


    float frequency = 0.2f;
    float max_angle = M_PI / 4.f;
    int last_t = 0;



public:


    BallDetector() : tfListener(tfBuffer) {
        //signal(SIGINT, BallDetector::quit);
        head_rotator = n.advertise<sensor_msgs::JointState>("all_motor", 10);


    }
    // catch names of the controllers availables on ROS network

    void move_head(){
        geometry_msgs::TransformStamped ball_pose;

        bool has_pose = false;
        try {
            ball_pose = tfBuffer.lookupTransform("ball", "torso",ros::Time(0));
            //has_pose = true;
        } catch (tf2::TransformException &ex) {
            has_pose = false;
        }
        /*ros::Duration last_pose = ros::Time::now() - ball_pose.header.stamp;
        if (last_pose < ros::Duration(1)) {
            has_pose = true;
            return;
        }*/

        if (!has_pose) {
            sensor_msgs::JointState js;
            js.name.push_back("head_motor_0");
            js.name.push_back("head_motor_1");
            js.position.push_back(max_angle * std::sin(static_cast<float>(last_t) / 100.f * frequency));
            js.position.push_back(0.6f);
            head_rotator.publish(js);
            last_t += 1;

        }
        else {
            has_pose = false;
        }


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

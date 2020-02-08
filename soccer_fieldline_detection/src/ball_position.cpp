//
// Created by manx52 on 2020-02-08.
//

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

class BallPosition {

    ros::Publisher ball_position;
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;


public:
    BallPosition() : tfListener(tfBuffer) {
        ball_position = n.advertise<geometry_msgs::Point>("ball_position", 1);

    }

    void get_ball_position(){
        geometry_msgs::TransformStamped ball_pose,robot_pose;

        bool has_ball = false;

        try {
            ball_pose = tfBuffer.lookupTransform("ball", "torso",ros::Time(0));
            has_ball = true;
        } catch (tf2::TransformException &ex) {}



        ros::Duration last_pose = ros::Time::now() - ball_pose.header.stamp;
        // has a recent ball and robot tf
        if (has_ball && last_pose < ros::Duration(1)) {
            geometry_msgs::Point position;
            position.x = ball_pose.transform.translation.x;
            position.y = ball_pose.transform.translation.y;
            position.z = ball_pose.transform.translation.z;

            ball_position.publish(position);
        }


    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_position");

    BallPosition ballPosition;

    ros::Rate r(100);
    while (ros::ok()) {
        ballPosition.get_ball_position();
        r.sleep();
    }
    ros::spin();
    return 0;

}
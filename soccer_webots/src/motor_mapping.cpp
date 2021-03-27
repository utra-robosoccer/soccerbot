//
// Created by manx52 on 2021-03-27.
//


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <webots_ros/Int32Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32


static std::string controllerName;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;



ros::ServiceClient  motor_pub_0 ;
ros::ServiceClient motor_pub_1;
ros::ServiceClient motor_pub_2 ;
ros::ServiceClient motor_pub_3;
ros::ServiceClient motor_pub_4 ;
ros::ServiceClient motor_pub_5;

ros::ServiceClient motor_pub_6 ;
ros::ServiceClient motor_pub_7 ;
ros::ServiceClient motor_pub_8 ;
ros::ServiceClient motor_pub_9 ;
ros::ServiceClient motor_pub_10 ;
ros::ServiceClient motor_pub_11;

ros::ServiceClient motor_pub_12 ;
ros::ServiceClient motor_pub_13 ;

ros::ServiceClient motor_pub_14;
ros::ServiceClient motor_pub_15 ;

ros::ServiceClient motor_pub_16 ;
ros::ServiceClient motor_pub_17 ;

ros::Subscriber  motor_sub_0 ;
ros::Subscriber motor_sub_1;
ros::Subscriber motor_sub_2 ;
ros::Subscriber motor_sub_3;
ros::Subscriber motor_sub_4 ;
ros::Subscriber motor_sub_5;

ros::Subscriber motor_sub_6 ;
ros::Subscriber motor_sub_7 ;
ros::Subscriber motor_sub_8 ;
ros::Subscriber motor_sub_9 ;
ros::Subscriber motor_sub_10 ;
ros::Subscriber motor_sub_11;

ros::Subscriber motor_sub_12 ;
ros::Subscriber motor_sub_13 ;

ros::Subscriber motor_sub_14;
ros::Subscriber motor_sub_15 ;

ros::Subscriber motor_sub_16 ;
ros::Subscriber motor_sub_17 ;
void right_leg_motor_0_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_0;
      temp_Srv_0.request.value = name->data;

    if (!motor_pub_0.call(temp_Srv_0) || !temp_Srv_0.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_leg_motor_1_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_1;
      temp_Srv_1.request.value = name->data;

    if (!motor_pub_1.call(temp_Srv_1) || !temp_Srv_1.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_leg_motor_2_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_2;
      temp_Srv_2.request.value = name->data;

    if (!motor_pub_2.call(temp_Srv_2) || !temp_Srv_2.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_leg_motor_3_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_3;
    temp_Srv_3.request.value = name->data;

    if (!motor_pub_3.call(temp_Srv_3) || !temp_Srv_3.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_leg_motor_4_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_4;
    temp_Srv_4.request.value = name->data;

    if (!motor_pub_4.call(temp_Srv_4) || !temp_Srv_4.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_leg_motor_5_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_5;
    temp_Srv_5.request.value = name->data;

    if (!motor_pub_5.call(temp_Srv_5) || !temp_Srv_5.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}

void left_leg_motor_0_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_6;
    temp_Srv_6.request.value = name->data;

    if (!motor_pub_6.call(temp_Srv_6) || !temp_Srv_6.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_leg_motor_1_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_7;
    temp_Srv_7.request.value = name->data;

    if (!motor_pub_7.call(temp_Srv_7) || !temp_Srv_7.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_leg_motor_2_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_8;
    temp_Srv_8.request.value = name->data;

    if (!motor_pub_8.call(temp_Srv_8) || !temp_Srv_8.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_leg_motor_3_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_9;
    temp_Srv_9.request.value = name->data;

    if (!motor_pub_9.call(temp_Srv_9) || !temp_Srv_9.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_leg_motor_4_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_10;
    temp_Srv_10.request.value = name->data;

    if (!motor_pub_10.call(temp_Srv_10) || !temp_Srv_10.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_leg_motor_5_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_11;
    temp_Srv_11.request.value = name->data;

    if (!motor_pub_11.call(temp_Srv_11) || !temp_Srv_11.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}

void left_arm_motor_0_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_12;
    temp_Srv_12.request.value = name->data;

    if (!motor_pub_12.call(temp_Srv_12) || !temp_Srv_12.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void left_arm_motor_1_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_13;
    temp_Srv_13.request.value = name->data;

    if (!motor_pub_13.call(temp_Srv_13) || !temp_Srv_13.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_arm_motor_0_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_14;
    temp_Srv_14.request.value = name->data;

    if (!motor_pub_14.call(temp_Srv_14) || !temp_Srv_14.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void right_arm_motor_1_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_15;
    temp_Srv_15.request.value = name->data;

    if (!motor_pub_15.call(temp_Srv_15) || !temp_Srv_15.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}

void head_motor_0_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_16;
    temp_Srv_16.request.value = name->data;

    if (!motor_pub_16.call(temp_Srv_16) || !temp_Srv_16.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}
void head_motor_1_callback(const std_msgs::Float64::ConstPtr &name) {
    webots_ros::set_float temp_Srv_17;
      temp_Srv_17.request.value = name->data;

    if (!motor_pub_17.call(temp_Srv_17) || !temp_Srv_17.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_name" );
    ros::ServiceClient timeStepClient;
    webots_ros::set_int timeStepSrv;
    ros::NodeHandle n;

    while (n.hasParam("controller_name") == false) {

        ros::spinOnce();
        ros::Duration(0.5).sleep();

    }
    ros::spinOnce();
    n.getParam("controller_name",controllerName);


    timeStepClient = n.serviceClient<webots_ros::set_int>("/" + controllerName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;




    motor_sub_0 = n.subscribe("right_leg_motor_0/command", 100, right_leg_motor_0_callback);
    motor_sub_1 = n.subscribe("right_leg_motor_1/command", 100, right_leg_motor_1_callback);
    motor_sub_2   = n.subscribe("right_leg_motor_2/command", 100, right_leg_motor_2_callback);
    motor_sub_3   = n.subscribe("right_leg_motor_3/command", 100, right_leg_motor_3_callback);
    motor_sub_4   = n.subscribe("right_leg_motor_4/command", 100, right_leg_motor_4_callback);
    motor_sub_5   = n.subscribe("right_leg_motor_5/command", 100, right_leg_motor_5_callback);

    motor_sub_6   = n.subscribe("left_leg_motor_0/command", 100, left_leg_motor_0_callback);
    motor_sub_7   = n.subscribe("left_leg_motor_1/command", 100, left_leg_motor_1_callback);
    motor_sub_8   = n.subscribe("left_leg_motor_2/command", 100, left_leg_motor_2_callback);
    motor_sub_9   = n.subscribe("left_leg_motor_3/command", 100, left_leg_motor_3_callback);
    motor_sub_10   = n.subscribe("left_leg_motor_4/command", 100, left_leg_motor_4_callback);
    motor_sub_11   = n.subscribe("left_leg_motor_5/command", 100, left_leg_motor_5_callback);

    motor_sub_12   = n.subscribe("left_arm_motor_0/command", 100, left_arm_motor_0_callback);
    motor_sub_13   = n.subscribe("left_arm_motor_1/command", 100, left_arm_motor_1_callback);

    motor_sub_14   = n.subscribe("right_arm_motor_0/command", 100, right_arm_motor_0_callback);
    motor_sub_15   = n.subscribe("right_arm_motor_1/command", 100, right_arm_motor_1_callback);

    motor_sub_16   = n.subscribe("head_motor_0/command", 100, head_motor_0_callback);
    motor_sub_17   = n.subscribe("head_motor_1/command", 100, head_motor_1_callback);



    motor_pub_0 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_0/set_position");
    motor_pub_1 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_1/set_position");
    motor_pub_2 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_2/set_position");
    motor_pub_3 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_3/set_position");
    motor_pub_4 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_4/set_position");
    motor_pub_5 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_leg_motor_5/set_position");

    motor_pub_6 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_0/set_position");
    motor_pub_7 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_1/set_position");
    motor_pub_8 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_2/set_position");
    motor_pub_9 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_3/set_position");
    motor_pub_10 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_4/set_position");
    motor_pub_11 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_leg_motor_5/set_position");

    motor_pub_12 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_arm_motor_0/set_position");
    motor_pub_13 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/left_arm_motor_1/set_position");

    motor_pub_14 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_arm_motor_0/set_position");
    motor_pub_15 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/right_arm_motor_1/set_position");

    motor_pub_16 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/head_motor_0/set_position");
    motor_pub_17 = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/head_motor_1/set_position");

    webots_ros::set_float temp_Srv;

    // main loop
    while (ros::ok()) {


        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
            ROS_ERROR("Failed to call service time_step for next step.");


        ros::spinOnce();


    }

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
    return 0;
}

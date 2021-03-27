//
// Created by manx52 on 2021-03-27.
//

#include "controller_name.h"
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

static int controllerCount;
static std::vector<std::string> controllerList;
static std::string controllerName;


ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;


// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
    controllerCount++;
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ROS_INFO("User stopped the 'keyboard_teleop' node.");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_name" );
    ros::NodeHandle n;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n.subscribe("/model_name", 100, controllerNameCallback);

    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {

        ros::spinOnce();
        ros::Duration(0.5).sleep();


    }
    ros::spinOnce();

    // if there is more than one controller available, let the user choose


    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
            controllerName = controllerList[wantedController - 1];
        else {
            ROS_ERROR("Invalid number for controller choice.");
            return 1;
        }
    }

    // leave topic once it's not necessary anymore
    nameSub.shutdown();

    //ros::Publisher controller_name;
    //controller_name = n.advertise<std_msgs::String>("controller_name", 1);
    n.setParam("controller_name",controllerName);
    timeStepClient = n.serviceClient<webots_ros::set_int>("/" + controllerName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

        // main loop
        while (ros::ok()) {


            if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
                ROS_ERROR("Failed to call service time_step for next step.");

            //std_msgs::String name;
            //name.data = controllerName;
            //controller_name.publish(name);

            ros::spinOnce();


        }

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
    return 0;
}

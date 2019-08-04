//
// Created by ljagodz on 7/28/19.
//

// Receives AMCL results from robots on soccer field.
// Sends: MapOverview.msg to dispatcher after creating a map from information given by AMCL results.

#pragma once

#include <ros/ros.h>
#include <soccer_msgs/MapOverview.h>


class LocalizationFilterNode {
public:
    LocalizationFilterNode();
    void publishMapOverview(); // To publish to topic.
    void updateMapOverview(); // Use info from robots to update the MapOverview msg.
private:
    ros::Subscriber robotSubscriber;
    ros::Publisher mapOverviewPublisher;
    soccer_msgs::MapOverview mapOverview;
    // probably some kind of array for msgs received from each robot in play.
};
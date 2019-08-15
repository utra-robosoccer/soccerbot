#include <soccer_route_planner/route_planner_node.hpp>
#include <cmath>


RoutePlannerNode::RoutePlannerNode() {
    mapOverviewSubscriber = nh.subscribe("map_overview", 1, &RoutePlannerNode::mapOverviewCallback, this);
    robotCommandSubscriber = nh.subscribe("robot_command", 1, &RoutePlannerNode::robotCommandCallback, this);
    waypointPublisher = nh.advertise<soccer_msgs::Waypoints>("waypoints", 1);

    mapOverviewStore = nullptr;
    robotCommandStore = nullptr;
}

void RoutePlannerNode::mapOverviewCallback(const soccer_msgs::MapOverviewConstPtr &mapOverview) {
    std::cerr << "test" << std::endl;
    mapOverviewStore = mapOverview;
    ROS_INFO_STREAM("Received map overview");

    if (this->robotCommandStore != nullptr) publishWaypoints();
}

void RoutePlannerNode::robotCommandCallback(const soccer_msgs::RobotCommandConstPtr &robotCommand) {
    std::cerr << "test" << std::endl;
    robotCommandStore = robotCommand;
    ROS_INFO_STREAM("Received robot command");

    if (this->mapOverviewStore != nullptr) publishWaypoints();
}

soccer_msgs::Waypoints RoutePlannerNode::createWaypoints() {
    soccer_msgs::Waypoints msg_temp;
    // build msg_temp here from stored mapOverview(s) and robotCommand(s)

    msg_temp.poseActions[0].duration = 0; // not sure how to assign this for now

//    if (this->mapOverviewStore.get()->estimates[0].pose != this->mapOverviewStore.get()->ballPose) {
//        if(this->mapOverviewStore.get()->estimates[0].pose.theta != this->robotCommandStore.get()->destPose.theta)
//            msg_temp.poseActions[0].actionLabel = 3; // Turn command
//    }


    // reset pointers so that new messages can be stored.
    mapOverviewStore = nullptr;
    robotCommandStore = nullptr;

    return msg_temp;
}

void RoutePlannerNode::publishWaypoints() {
    waypointPublisher.publish(this->createWaypoints());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "route_planner_node");

    // Create RoutePlannerNode class instance
    RoutePlannerNode routePlannerNode;
    ros::spin();
    return 0;
}

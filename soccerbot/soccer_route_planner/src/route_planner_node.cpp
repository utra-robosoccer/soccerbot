#include <soccer_route_planner/route_planner_node.hpp>
#include <soccer_route_planner/Lgraph.hpp>
#include <cmath>
#include <memory>

RoutePlannerNode::RoutePlannerNode() {
    mapOverviewSubscriber = nh.subscribe("map_overview", 1, &RoutePlannerNode::mapOverviewCallback, this);
    robotCommandSubscriber = nh.subscribe("robot_command", 1, &RoutePlannerNode::robotCommandCallback, this);
    waypointPublisher = nh.advertise<soccer_msgs::Waypoints>("waypoints", 1);

    robotCommandStore = nullptr;
}

void RoutePlannerNode::mapOverviewCallback(const soccer_msgs::MapOverviewPtr &mapOverview) {
    std::cerr << "test" << std::endl;
    mapOverviewStore = mapOverview;
    ROS_INFO_STREAM("Received map overview");
}

void RoutePlannerNode::robotCommandCallback(const soccer_msgs::RobotCommandPtr &robotCommand) {
    std::cerr << "test" << std::endl;
    robotCommandStore = robotCommand;
    ROS_INFO_STREAM("Received robot command");

    if (!mapOverviewStore)
        return;

    auto waypoints = createWaypoints();
    publishWaypoints(waypoints);
}

soccer_msgs::Waypoints RoutePlannerNode::createWaypoints() {
    soccer_msgs::Waypoints msg_temp;
    // build msg_temp here from stored mapOverview(s) and robotCommand(s)

    // Create graph etc for path planning recursive tree or some sort to go etc.

    msg_temp.poseActions[0].duration = 0; // not sure how to assign this for now

//    if (this->mapOverviewStore.get()->estimates[0].pose != this->mapOverviewStore.get()->ballPose) {
//        if(this->mapOverviewStore.get()->estimates[0].pose.theta != this->robotCommandStore.get()->destPose.theta)
//            msg_temp.poseActions[0].actionLabel = 3; // Turn command
//    }

    return msg_temp;
}

void RoutePlannerNode::publishWaypoints(soccer_msgs::Waypoints waypoints) {
    waypointPublisher.publish(waypoints);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "route_planner_node");

    // Create RoutePlannerNode class instance
    RoutePlannerNode routePlannerNode;
    ros::spin();
    return 0;
}


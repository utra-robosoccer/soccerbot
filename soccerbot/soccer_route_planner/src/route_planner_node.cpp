#include <soccer_route_planner/route_planner_node.hpp>


RoutePlannerNode::RoutePlannerNode() {
    mapOverviewSubscriber = nh.subscribe("map_overview", 1, &RoutePlannerNode::mapOverviewCallback, this);
    robotCommandSubscriber = nh.subscribe("robot_command", 1, &RoutePlannerNode::robotCommandCallback, this);
}

void RoutePlannerNode::mapOverviewCallback(const soccer_msgs::MapOverviewConstPtr &mapOverview) {
    std::cerr << "test" << std::endl;
    ROS_INFO_STREAM("Received map overview");
}

void RoutePlannerNode::robotCommandCallback(const soccer_msgs::RobotCommandConstPtr &robotCommand) {
    std::cerr << "test" << std::endl;
    ROS_INFO_STREAM("Received robot command");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "route_planner_node");

    // Create RoutePlannerNode class instance
    RoutePlannerNode routePlannerNode;
    ros::spin();
    return 0;
}

#include <soccer_route_planner/route_planner_node.hpp>
#include <soccer_route_planner/graph.hpp>
#include <soccer_route_planner/map.hpp>
#include <soccer_route_planner/rrt_star.hpp>
#include <soccer_msgs/RobotPath.h>

#include <cmath>
#include <memory>

RoutePlannerNode::RoutePlannerNode() {
    robotCommandSubscriber = nh.subscribe("robot1/command", 1, &RoutePlannerNode::robotCommandCallback, this);
    robotPoseSubscriber = nh.subscribe("robot1/pose", 1, &RoutePlannerNode::robotPoseCallback, this);
    waypointPublisher = nh.advertise<soccer_msgs::RobotPath>("robot1/robot_path", 1, true);
    graphPublisher = nh.advertise<visualization_msgs::Marker>("robot1/rrt_graph", 1, true);
}

void RoutePlannerNode::robotPoseCallback(const geometry_msgs::Pose2DPtr &robotPose) {
    robot_poses[1] = robotPose;
}

void RoutePlannerNode::robotCommandCallback(const soccer_msgs::RobotCommandPtr &robotCommand) {
    ROS_INFO_STREAM("Received robot command");

    if (robot_poses[1] == nullptr) {
        return;
    }

    soccer_msgs::RobotPath path;

    RRTStar rrt_star_alg;
    Graph solution = rrt_star_alg.RRT_Star(400, *robot_poses[1], robotCommand->goal);

    graphPublisher.publish(solution.get_graph_marker());
    waypointPublisher.publish(path);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "route_planner_node");

    // Create RoutePlannerNode class instance
    RoutePlannerNode routePlannerNode;
    ros::spin();
    return 0;
}


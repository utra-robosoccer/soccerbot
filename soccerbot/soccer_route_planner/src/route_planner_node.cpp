#include <soccer_route_planner/route_planner_node.hpp>


RoutePlannerNode::RoutePlannerNode() {
    robotCommandSubscriber = nh.subscribe("topic_namc", callback)

}

RoutePLannerNode::CallbackFunction() {

}

int main() {
    // Create RoutePLannerNode class instance

    // Ros::spin();
    return 0;
}

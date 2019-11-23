#pragma once

#include <stdio.h>
#include <soccer_route_planner/Graph.hpp>
#include <soccer_route_planner/Map.hpp>
#include <geometry_msgs/Pose2D.h>

class RRTStar {
private:
    Map M;
    Graph G;
public:
    RRTStar();

    // RRT* algorithm that returns optimal path.
    Graph RRT_Star(int n, geometry_msgs::Pose2D x_init, geometry_msgs::Pose2D x_goal);

    // Return ID of the closest node on the graph to the specified sample location.
    int nearestNeighbour(geometry_msgs::Pose2D sample);

    // Return vector of IDs of vertices in a neighbourhood of the sample location.
    // NOTE: create seperate neighbourhood size from step distance?
    std::vector<int> nearVertices(geometry_msgs::Pose2D sample);

    // Steer the sample location to within the step distance of the ID parameter.
    // NOTE: does this need to be randomized to any distance between 0 and step distance?
    geometry_msgs::Pose2D steer(int nearest_id, geometry_msgs::Pose2D sample);

    // Return true if collision free, false if not.
    bool collisionFree(geometry_msgs::Pose2D first, geometry_msgs::Pose2D second);

    // Return random FREE coordinate selected on the map.
    geometry_msgs::Pose2D sample();
};
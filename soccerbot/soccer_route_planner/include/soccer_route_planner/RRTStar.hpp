#pragma once

#include <stdio.h>
#include <soccer_route_planner/Graph.hpp>
#include <soccer_route_planner/Map.hpp>
#include <geometry_msgs/Pose2D.h>

class RRTStar {
private:
    Map M;
public:
    RRTStar();
    void RRTStar_func(int r, int n, geometry_msgs::Pose2D x_init, geometry_msgs::Pose2D x_goal);
    void nearestNeighbour();
    void nearVertices();
    void steering();
    void collisionTest();
    geometry_msgs::Pose2D sample();
};
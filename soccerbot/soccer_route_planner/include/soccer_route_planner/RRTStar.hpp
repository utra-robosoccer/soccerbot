#pragma once

#include <stdio.h>
#include <soccer_route_planner/Graph.hpp>
#include <soccer_route_planner/Map.hpp>
#include <geometry_msgs/Pose2D.h>

class RRTStar {
private:
    Map M;
    Graph G;

    int m_nNodes;
    float m_StepSize;
public:
    RRTStar();
    void RRTStar_func(int r, int n, geometry_msgs::Pose2D x_init, geometry_msgs::Pose2D x_goal);
    void nearestNeighbour();
    void nearVertices();
    void steering();
    void collisionTest();
    geometry_msgs::Pose2D sample();
};

// Procedure:
// 1. Initialization:
//      - Graph: 1 Vertex (Current Position), 0 Edges
// 2. Procedure and Maintenance:
//      a) Loop: over n_Nodes (tree density)
//      b) SampleFree: 'random point' in occupancy map that is free
//      c) Nearest: find 'closest point' in the graph closest to 'random point'
//      d) Steer: find the 'new point' that is closer to 'random point' than 'closest point', but not farther than 'closest point' by step size
//      e) ObstacleFree: check if path between 'closest point' and 'new point' is obstacle free
//      f) Near: find 'near set' of vertices.
// ..........to finish
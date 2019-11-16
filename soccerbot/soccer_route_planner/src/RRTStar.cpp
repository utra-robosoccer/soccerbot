#include <soccer_route_planner/RRTStar.hpp>

#include <stdlib.h>
#include <ctime>

RRTStar::RRTStar() {

}

void RRTStar::RRTStar_func(int r, int n, geometry_msgs::Pose2D x_init, geometry_msgs::Pose2D x_goal) {
    Graph G;
    G.add_vertex(x_init.x, x_init.y);

    int i = 0;

    while (i < n) {
//        geometry_msgs::Pose2D x_rand = sample();
        i++;
    }
}

void RRTStar::nearestNeighbour() {

}

void RRTStar::nearVertices() {

}

void RRTStar::steering() {

}

void RRTStar::collisionTest() {

}

geometry_msgs::Pose2D RRTStar::sample() {

}
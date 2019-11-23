#include <soccer_route_planner/RRTStar.hpp>

#include <stdlib.h>
#include <ctime>
#include <math.h>

double compute_dist(geometry_msgs::Pose2D first, geometry_msgs::Pose2D second);

RRTStar::RRTStar() {

}

Graph RRTStar::RRT_Star(int n, geometry_msgs::Pose2D x_init, geometry_msgs::Pose2D x_goal) {
    int root_id = G.add_vertex(x_init.x, x_init.y);
    G.assign_cost_simple(root_id, 0);
    G.assign_parent(root_id, -1); // Proxy for no parent.

    int i = 0;

    while (i < n) {
        // Select a random (unobstructed) sample from the map.
        geometry_msgs::Pose2D x_rand = sample();

        // Identify the node nearest to the sample.
        int nearest_id = nearestNeighbour(x_rand);

        // "Steer" a new node to be close enough to nearest neighbour (unless already within resolution).
        // x_new is position of node to be added to graph.
        geometry_msgs::Pose2D x_new;
        double c_nearest = compute_dist(G.get_node_info(nearest_id).location, x_rand);
        if(c_nearest > M.info.resolution) {
            x_new = steer(nearest_id, x_rand);
            c_nearest = compute_dist(G.get_node_info(nearest_id).location, x_new);
        }
        else
            x_new = x_rand;

        // If no obstacle between nearest node and new node...
        if(collisionFree(G.get_node_info(nearest_id).location, x_new)) {

            // Grab vertices in graph in a neighbourhood of x_new.
            std::vector<int> near = nearVertices(x_new);

            // Add new vertex to graph
            int new_id = G.add_vertex(x_new.x, x_new.y);

            // Update minimum cost to beat.
            int min_id = nearest_id;
            double c_min = G.get_node_info(nearest_id).cost + c_nearest; // UPDATE WITH CALCULATION.

            // Loop through vertices in a neighbourhood of the new vertex.
            // Loop finds best node to create edge to for a min-cost path.
            int num_near = near.size();
            for(int j = 0; j < num_near; j++) {
                // NOTE: need to add check if node is within step distance (or set neighbourhood radius to step).
                // Find location of neighbourhood vertex.
                geometry_msgs::Pose2D near_location = G.get_node_info(near[j]).location;
                // Check if neighbourhood vertex and new vertex have collision free connection.
                bool collision_free_near = collisionFree(near_location, x_new);
                // Compute cost between neighbourhood node and new node.
                double cost_near = G.get_node_info(near[j]).cost + compute_dist(near_location, x_new);
                // If lower cost path exists...
                if(collision_free_near && cost_near < c_min) {
                    min_id = near[j];
                    c_min = cost_near;
                }
            }
            G.add_edge(min_id, new_id);
            // c_min effectively cost of x_new.
            G.assign_cost_simple(new_id, c_min);
            G.assign_parent(new_id, min_id);

            // Loop through vertices in a neighbourhood of the new vertex (again) - REWIRE THE TREE.
            for(int j = 0; j < num_near; j++) {
                int parent_id = -1;
                geometry_msgs::Pose2D near_location = G.get_node_info(near[j]).location;
                bool collision_free_near = collisionFree(near_location, x_new);
                // NOTE: c_min = cost(x_new)
                double cost_new = c_min + compute_dist(near_location, x_new);
                if(collision_free_near && cost_new < G.get_node_info(near[j]).cost) {
                    int x_parent = G.parent(near[j]);
                    G.delete_edge(x_parent, near[j]);
                    G.add_edge(new_id, near[j]);
                }
            }
        }
        i++;
    }

    return G;
}

int RRTStar::nearestNeighbour(geometry_msgs::Pose2D sample) {
    int i = 0;
    double best_dist = 10000;
    int best_id = -1;
    int num_nodes = G.get_num_nodes();
    while(i < num_nodes) {
        double dist = compute_dist(G.get_node_info(i).location, sample);
        if(dist < best_dist) {
            best_id = i;
            best_dist = dist;
        }
    }
    return best_id;
}

std::vector<int> RRTStar::nearVertices(geometry_msgs::Pose2D sample) {
    int num_nodes = G.get_num_nodes();
    std::vector<int> nearVerts;
    for(int i = 0; i < num_nodes; i++) {
        if(compute_dist(G.get_node_info(i).location, sample) < M.info.resolution) nearVerts.push_back(i);
    }
    return nearVerts;
}

geometry_msgs::Pose2D RRTStar::steer(int nearest_id, geometry_msgs::Pose2D sample) {
    geometry_msgs::Pose2D nearest = G.get_node_info(nearest_id).location;
    double con = M.getStepSize() / sqrt((sample.x - nearest.x)*(sample.x - nearest.x) + (sample.y - nearest.y)*(sample.y - nearest.y));
    geometry_msgs::Pose2D temp;
    temp.x = nearest.x + con * abs(sample.x - nearest.x);
    temp.y = nearest.y + con * abs(sample.y - nearest.y);
    return temp;
}

bool RRTStar::collisionFree(geometry_msgs::Pose2D first, geometry_msgs::Pose2D second) {
    double diff_y = M.info.resolution * (second.y - first.y) / compute_dist(first, second);
    double diff_x = M.info.resolution * (second.x - first.x) / compute_dist(first, second);
    first.x = first.x + diff_x;
    first.y = first.y + diff_y;
    int direction_x = diff_x / abs(diff_x);
    int direction_y = diff_y / abs(diff_y);

    bool collisionFree = 1;

    while(compute_dist(first, second) <= 0 && collisionFree == 1) {
        if(M.getOccupancy(first)) {
            collisionFree = 0;
        }
        first.x = first.x + diff_x;
        first.y = first.y + diff_y;
    }
    return collisionFree;
}

geometry_msgs::Pose2D RRTStar::sample() {
    geometry_msgs::Pose2D temp;
    temp.x = rand() % M.info.width;
    temp.y = rand() % M.info.height;
    while(M.getOccupancy(temp)) {
        temp.x = rand() % M.info.width;
        temp.y = rand() % M.info.height;
    }
    return temp;
}

// NOTE: Does this need to be accurate distance or is squared distance sufficient?
double compute_dist(geometry_msgs::Pose2D first, geometry_msgs::Pose2D second) {
    return sqrt((second.x - first.x)*(second.x - first.x) + (second.y - first.y)*(second.y - first.y));
}
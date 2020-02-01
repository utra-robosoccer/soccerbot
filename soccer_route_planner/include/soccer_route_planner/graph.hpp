#pragma once

#include <stdio.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <map>
#include <visualization_msgs/Marker.h>

class Graph
{
private:
    std::vector<struct node> adj_list;
    std::map<int, struct node_info> node_info_map;

    int m_num_nodes;
    int m_id_generator;

    int find_vertex(int id); // Returns index of vertex within adjacency list.
public:
    Graph();

    int add_vertex(float x, float y); // Returns id of the vertex.
    void add_edge(int id_0, int id_1); // Create edge from node id_0 to node id_1.
    void delete_edge(int id_0, int id_1);

    struct node_info get_node_info(int id);

    // Assign cost to node specified by id.
    void assign_cost_simple(int id, double cost);
    void assign_parent(int id, int parent);

    // Return ID of the parent of node specified by id.
    int parent(int id);

    int get_num_nodes();

    visualization_msgs::Marker get_graph_marker();

};

struct node_info {
    double cost = -1;
    bool is_alive = true;
    geometry_msgs::Pose2D location;
    int parent = 0;
};

struct node {
    int id = -1;
    std::vector<int> neighbours;
};


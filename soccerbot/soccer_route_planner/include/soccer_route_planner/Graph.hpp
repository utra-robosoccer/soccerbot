#pragma once

#include <stdio.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <map>

class Graph
{
private:
    std::vector<struct node> adj_list;
    std::map<int, struct node_info> node_info_map;

    int num_nodes;
    int id_generator;

    int find_vertex(int id); // Returns index of vertex within adjacency list.
public:
    Graph();

    int add_vertex(float x, float y); // Returns id of the vertex.
    void add_edge(int id_0, int id_1); // Create edge from node id_0 to node id_1.
    void delete_edge(int id_0, int id_1);

    struct node_info get_node_info(int id);
};

struct node_info {
    bool is_alive = true;
    geometry_msgs::Pose2D location;
};

struct node {
    int id = -1;
    std::vector<int> neighbours;
};


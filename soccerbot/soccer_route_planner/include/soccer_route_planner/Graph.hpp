#pragma once

#include <stdio.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Pose2D.h>

class Graph
{
private:
    std::vector<struct node*> adj_list;
    int num_nodes;
public:
    Graph();

    int add_vertex(float x, float y); // Returns id of the vertex.
    void add_edge(int id_0, int id_1);
    void delete_vertex();
    void delete_edge();
};

struct node {
    int id;
    geometry_msgs::Pose2D location;
    struct node *next;
};



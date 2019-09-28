#pragma once

#include <stdio.h>
#include <vector>
#include <iostream>

class Graph
{
private:
    std::vector<struct lnode*> adj;
    int num_nodes;
public:
    Graph();

    void add_vertex();
    void add_edge();
    void delete_vertex();
    void delete_edge();
};

struct lnode {
    int id;
    struct lnode *next;
};



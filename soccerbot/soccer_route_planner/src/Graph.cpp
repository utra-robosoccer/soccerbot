//
// Created by ljagodz on 9/7/19.
//

#include <soccer_route_planner/Graph.hpp>

Graph::Graph() {
    num_nodes = 0;
}

int Graph::add_vertex(float x, float y) {
    struct node* temp = new struct node;

    temp->id = (adj_list[adj_list.size() - 1]->id)++;

    temp->location.x = x;
    temp->location.y = y;
    temp->location.theta = 0;

    num_nodes++;

    adj_list.push_back(temp);

    return temp->id;
}

void Graph::add_edge(int id_0, int id_1) {

}

void Graph::delete_vertex(int id) {

}

void Graph::delete_edge(int id_0, int id_1) {

}

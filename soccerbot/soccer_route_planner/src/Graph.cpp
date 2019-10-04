#include <soccer_route_planner/Graph.hpp>

Graph::Graph() {
    num_nodes = 0;
    id_generator = 0;
}

int Graph::add_vertex(float x, float y) {
    struct node new_node;

    new_node.id = id_generator;

    num_nodes++;
    id_generator++;

    adj_list.push_back(new_node);

    struct node_info temp;
    temp.location.theta = 0;
    temp.location.x = x;
    temp.location.y = y;
    temp.is_alive = true;

    node_info_map[new_node.id] = temp;

    return new_node.id;
}

void Graph::add_edge(int id_0, int id_1) {
    int index = find_vertex(id_0);
    adj_list[index].neighbours.push_back(id_1);
}

void Graph::delete_edge(int id_0, int id_1) {
    int index = find_vertex(id_0);
    int i = 0;
    while(adj_list[index].neighbours[i] != id_1) i++;
    adj_list[index].neighbours.erase(adj_list[index].neighbours.begin() + i);
}

int Graph::find_vertex(int id) {
    int i = 0;
    while(adj_list[i].id != id && i < num_nodes) i++;
    return i;
}

struct node_info Graph::get_node_info(int id) {
    return node_info_map[id];
}



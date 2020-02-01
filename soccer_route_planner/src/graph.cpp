#include <soccer_route_planner/graph.hpp>

Graph::Graph() {
    m_num_nodes = 0;
    m_id_generator = 0;
}

int Graph::add_vertex(float x, float y) {
    struct node new_node;

    new_node.id = m_id_generator;

    m_num_nodes++;
    m_id_generator++;

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
    while(adj_list[i].id != id && i < m_num_nodes) i++;
    return i;
}

int Graph::get_num_nodes() {
    return m_num_nodes;
}

struct node_info Graph::get_node_info(int id) {
    return node_info_map[id];
}

void Graph::assign_cost_simple(int id, double cost) {
    node_info_map[id].cost = cost;
}

void Graph::assign_parent(int id, int parent) {
    node_info_map[id].parent = id;
}

int Graph::parent(int id) {
    return node_info_map[id].parent;
}

visualization_msgs::Marker Graph::get_graph_marker() {
     visualization_msgs::Marker marker;

     marker.ns = "rrt_graph_thing";
     marker.action = visualization_msgs::Marker::ADD;
     marker.pose.orientation.w = 0.0;

     marker.id = 0;
     marker.type = visualization_msgs::Marker::LINE_LIST;
     marker.scale.x = 0.1;
     marker.color.r = 1.0;
     marker.color.a = 1.0;

}

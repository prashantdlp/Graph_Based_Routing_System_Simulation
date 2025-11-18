#include "Graph.hpp"
#include <cmath>
#include <stdexcept>

// Constructor 
Graph::Graph() : n(0) {}

// Destructor 
Graph::~Graph() {
    nodes.clear();
    edges.clear();
    adj_list.clear();
    poi_index.clear();
}

// Add node
void Graph::addNode(int id, double lat, double lon, const std::vector<std::string>& names) { //TODO:: names to pois

    Node* node = new Node(id, lat, lon, names);

    nodes[id] = node;
    n++;

    // Update POI 
    for (const auto& poi : names) {
        poi_index[poi].push_back(id);
    }
}
// Add edge
void Graph::addEdge(int id, int u, int v, double length, double avg_time,
                    const std::vector<double>& speed_profile, bool oneway, const std::string& road_type) {

    Edge* e = new Edge{id, u, v, length, avg_time, speed_profile, oneway, road_type, true};

    edges[id] = e;

    // Update adjacency list
    adj_list[u].push_back(e);
    inc_list[v].push_back(e);
    if (!oneway){ 
        Edge* reverse_e = new Edge{id, u, v, length, avg_time, speed_profile, oneway, road_type, true};
        adj_list[v].push_back(reverse_e);
        inc_list[v].push_back(e);
    }
}

const Node* Graph::getNode(int id) const {
    auto it = nodes.find(id);
    if (it == nodes.end()) return nullptr;
    return it->second;
}

const Edge* Graph::getEdge(int u, int v) const {

}

bool Graph::removeEdge(int edge_id) { 

    auto it = edges.find(edge_id);
    if(it == edges.end())
        return false;
    if (it != edges.end()) {
        it->second->active = false; 
    }
    return true;
}

bool Graph::modifyEdge(int edge_id, const Edge& patch) {
    auto it = edges.find(edge_id);
    if (it == edges.end()) return false;

    Edge* e = it->second;
    if(!e->active)
        e->active = true;
    if (patch.length != 0) e->length = patch.length;
    if (patch.average_time != 0) e->average_time = patch.average_time;
    if (!patch.speed_profile.empty()) e->speed_profile = patch.speed_profile;
    if (!patch.road_type.empty()) e->road_type = patch.road_type;
    if (patch.oneway.has_value()) e->oneway = patch.oneway.value();
   
    return true;
}

const std::vector<Edge*>& Graph::getAdjacentEdges(int node_id) const {
    auto it = adj_list.find(node_id);
    if (it != adj_list.end()) return it->second;
    return std::vector<Edge*>{};
}

const std::vector<Edge*>& Graph::getIncomingEdges(int node_id) const {
    auto it = inc_list.find(node_id); //
    if (it != inc_list.end()) return it->second;
    return std::vector<Edge*>{};
}
std::vector<int> Graph::getNodesWithPOI(const std::string& poi) const {
    auto it = poi_index.find(poi);
    if (it == poi_index.end()) return std::vector<int>{};
    return it->second;
}

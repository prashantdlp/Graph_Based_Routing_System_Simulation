#include "Algorithms.hpp"
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>

std::tuple<bool, double, std::vector<int>> Shortest_paths_distance( //TODO:OPTIMIZATIONS>>>
        const Graph& graph,
        int source,
        int target,
        const constraints& constraints
    ){
    int n = graph.size();

    std::vector<int> forbidden_nodes = constraints.forbidden_nodes;
    std::vector<std::string> forbidden_road_types = constraints.forbidden_road_types;

    std::unordered_set<int> forbidden(
    constraints.forbidden_nodes.begin(),
    constraints.forbidden_nodes.end()
    );
    std::unordered_set<std::string> forbidden_r(
    constraints.forbidden_road_types.begin(),
    constraints.forbidden_road_types.end()
    );

    if (source < 0 || source >= n || 
        target < 0 || target >= n ){
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<bool> visited(n, false);
    std::vector<double> distance(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);

    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        std::greater<std::pair<double, int>>
    > pq;

    distance[source] = 0.0;
    pq.push({0.0, source});
    while (!pq.empty()) {
        auto [dist, v] = pq.top();
        pq.pop();
        
        // Skip if visited
        if (visited[v]) continue;
        
        visited[v] = true;
        
        // reached target
        if (v == target) break;
        
        for (auto edge : graph.getAdjacentEdges(v)) {
            if(forbidden_r.find(edge->road_type) == forbidden_r.end() && edge->active){
                double length = edge->length;  
                int w = edge->v;
                if (!visited[w] && forbidden.find(w) == forbidden.end()) {
                    double new_dist = distance[v] + length;
                    
                    if (new_dist < distance[w]) {
                        distance[w] = new_dist;
                        parent[w] = v;
                        pq.push({new_dist, w});
                    }
                }
            }
        }
    
    }
    if (distance[target] == std::numeric_limits<double>::infinity()) {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<int> path;
    int current = target;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());
    
    return {true, distance[target], path};
}

double Edge_time(
    const Edge* edge,
    double start_time
){
    const double TIME_SLOT = 15.0;
    double remaining_distance = edge->length;
    double current_time = start_time;
    double total_time = 0.0;
    
    while (remaining_distance > 1e-9) {
        int slot = static_cast<int>(floor(current_time / TIME_SLOT));
        
        if(edge->speed_profile.empty())
            return total_time + edge->average_time;
        double speed = edge->speed_profile[slot];    
        return total_time + edge->average_time;
        // Time remaining in current slot
        double next_slot_boundary = (slot + 1) * TIME_SLOT;
        double time_remaining_in_slot = next_slot_boundary - current_time;
        
        // Distance we can cover in remaining time of current slot
        double distance_in_slot = speed * time_remaining_in_slot;
        
        if (distance_in_slot >= remaining_distance) {
            // We finish in this slot
            total_time += remaining_distance / speed;
            break;
        } else {
            // Move to next slot
            total_time += time_remaining_in_slot;
            remaining_distance -= distance_in_slot;
            current_time = next_slot_boundary;
        }
    }
    
    return total_time;
}

std::tuple<bool, double, std::vector<int>> Shortest_paths_speed(
    const Graph& graph,
    int source,
    int target,
    const constraints& constraints
    ){
    int n = graph.size();
    const double TIME_SLOT = 15.0;

    std::vector<int> forbidden_nodes = constraints.forbidden_nodes;
    std::vector<std::string> forbidden_road_types = constraints.forbidden_road_types;

    std::unordered_set<int> forbidden(
    constraints.forbidden_nodes.begin(),
    constraints.forbidden_nodes.end()
    );
    std::unordered_set<std::string> forbidden_r(
    constraints.forbidden_road_types.begin(),
    constraints.forbidden_road_types.end()
    );

    if (source < 0 || source >= n || 
        target < 0 || target >= n ){
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<bool> visited(n, false);
    std::vector<double> time_to_reach(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    double time = 0.0;

    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        std::greater<std::pair<double, int>>
    > pq;

    time_to_reach[source] = 0.0;
    pq.push({0.0, source});
    while (!pq.empty()) {
        auto [current_time,u] = pq.top();
        pq.pop();
        
        // Skip if visited
        if (visited[u]) continue;
        
        visited[u] = true;
        
        // reached target 
        if (u == target) break;
        
        for (auto edge : graph.getAdjacentEdges(u)) { //TODO: consider edge is active or not??
            if(forbidden_r.find(edge->road_type) == forbidden_r.end() && edge->active){
                double time_edge = Edge_time(edge,current_time);
                int w = edge->v;
                if (!visited[w] && forbidden.find(w) == forbidden.end()) {
                    double new_time = time_to_reach[u] + time_edge;
                    
                    if (new_time < time_to_reach[w]) {
                        time_to_reach[w] = new_time;
                        parent[w] = u;
                        pq.push({new_time, w});
                    }
                }
            }
        }
    
    }
    if (time_to_reach[target] == std::numeric_limits<double>::infinity()) {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<int> path;
    int current = target;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());
    
    return {true, time_to_reach[target], path};
}

std::tuple<bool, double, std::vector<int>> Algorithms::Shortest_paths(
        const Graph& graph,
        int source,
        int target,
        const std::string& mode,
        const constraints& constraints
    ){
        if(mode == "distance"){
            return Shortest_paths_distance(graph, source, target, constraints);
        }
        return Shortest_paths_speed(graph, source, target, constraints);
        
    }

double euclidian_distance(double lat1, double lon1, double lat2, double lon2){
    const double REFERENCE_LATITUDE = 19.07; 
    const double DEG_TO_RAD = M_PI / 180.0;
    const double METERS_PER_DEG_LAT = 1111390;

    const double METERS_PER_DEG_LON = METERS_PER_DEG_LAT * std::cos(REFERENCE_LATITUDE * DEG_TO_RAD);
    // North-South distance
    double deltaY_meters = (lat2 - lat1) * METERS_PER_DEG_LAT;    
    // East-West distance
    double deltaX_meters = (lon2 - lon1) * METERS_PER_DEG_LON;

    return std::sqrt(deltaX_meters * deltaX_meters + deltaY_meters * deltaY_meters);
}

int getNearestNodeId (const Graph& graph, double latitude, double longitude){
    for(int i=0; i<graph.size(); i++){
        const Node* tempnode = graph.getNode(i);
        if(tempnode->lat==latitude && tempnode->lon == longitude){
            return i;
        }
    }
    int nearestId =1;
    double nearestDistance = euclidian_distance(graph.getNode(nearestId)->lat, graph.getNode(nearestId)->lon, latitude, longitude);
    for(int i=1; i<graph.size(); i++){
        const Node* node = graph.getNode(i);

        if(nearestDistance> euclidian_distance(node->lat, node->lon, latitude, longitude)){
            nearestDistance = euclidian_distance(node->lat, node->lon, latitude, longitude);

            nearestId = i;
        }
    return i;
    }
}

std::vector<int> Algorithms::KNN(
    const Graph& graph,
    double latitude,
    double longitude,
    const std::string& poi,
    const Node& origin,
    int k,
    const std::string& metric
    ){
        //based on euclidian
    std::priority_queue<std::pair<double, int>> distances;
    for(auto id : graph.getNodesWithPOI(poi)){
        const Node* node = graph.getNode(id);
        double distance =0;
        if(metric == "euclidean"){
         distance = euclidian_distance(latitude, longitude, node->lat, node->lon);   }
        else if (metric == "shortest_path"){
        int nodeID = getNearestNodeId(graph, latitude, longitude);
        const Node* origin = graph.getNode(nodeID);
        constraints noconstr;
        distance = std::get<double>(Shortest_paths(graph, origin->id, node->id,"distance", noconstr));
        }
        if(id == origin.id){continue;}
        if((int)distances.size() < k){
            distances.push({distance, id});
        }
        else if(distance <distances.top().first ){
            distances.pop();
            distances.push({distance, id});
        }
    }        
    std::vector<int> knns;
    knns.reserve(distances.size());
    while(!knns.empty()){
        knns.push_back(distances.top().second);
        distances.pop();
    }
    std::reverse(knns.begin(), knns.end());
    
        //based on path
    return knns;
    }


#include "Algorithms.hpp"
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>

std::tuple<bool, double, std::vector<int>> Algorithms::Shortest_paths(
        const Graph& graph,
        int source,
        int target,
        const std::string& mode,
        const constraints& constraints
    ){
    int n = graph.size();

    std::vector<int> forbidden_nodes = constraints.forbidden_nodes;
    std::vector<std::string> forbidden_road_types = constraints.forbidden_road_types;

    std::unordered_set<int> forbidden(
    constraints.forbidden_nodes.begin(),
    constraints.forbidden_nodes.end()
    );
    std::unordered_set<int> forbidden_r(
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
            if(forbidden_r.find(edge->id) == forbidden_r.end()){
                int length = edge->length;  
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

std::vector<int> Algorithms::KNN(
    const Graph& graph,
    const std::string& poi,
    const Node& origin,
    int k,
    const std::string& metric
    ){
        //based on euclidian
    if(metric == "euclidean"){
    std::priority_queue<std::pair<double, int>> distances;
    for(auto id : graph.getNodesWithPOI(poi)){
        if(id == origin.id){continue;}
        const Node* node = graph.getNode(id);
        double distance = euclidian_distance(origin.lat, origin.lon, node->lat, node->lon);   
        if(distances.size() < k){
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
}
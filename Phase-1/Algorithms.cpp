#include "Algorithms.hpp"
#define M_PI 3.14159265358979323846
static std::tuple<bool, double, std::vector<int>> Shortest_paths(
        const Graph& graph,
        int source,
        int target,
        const std::string& mode,
        const constraints& patch
    ){

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

static std::vector<int> KNN(
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
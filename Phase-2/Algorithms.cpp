#include "Algorithms.hpp" //TODO: check if all includes are even required?? also where toi include this file? also what bout POI in phase 2.
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>
#include <set>

static std::vector<int> compute_heuristic(const Graph& graph, int target) {
    int n = graph.size();
    std::vector<int> h(n, std::numeric_limits<int>::max());
    
    std::priority_queue<
        std::pair<int, int>,
        std::vector<std::pair<int, int>>,
        std::greater<>
    > pq;
    
    h[target] = 0;
    pq.push({0, target});
    
    while (!pq.empty()) {
        auto [dist, u] = pq.top();
        pq.pop();
        
        if (dist > h[u]) continue;
        
        // Traverse INCOMING edges (reverse direction)
        for (const auto* edge : graph.getIncomingEdges(u)) {
            int source = graph.getNode(edge->u)->id;  // Source of the incoming edge
            int new_dist = dist + edge->length;
            
            if (new_dist < h[source]) {
                h[source] = new_dist;
                pq.push({new_dist, source});
            }
        }
    }
    
    return h;
}

static std::vector<std::pair<std::vector<int>, int>> k_shortest_paths( //TODO:                                                                                                                           DO: preprocesng??
        const Graph& graph,
        int source,
        int target,
        int k,
        const std::string& mode
    ){
        std::vector<std::pair<std::vector<int>, int>> result;
        int n = graph.size();
        std::vector<int> h = compute_heuristic(graph, target);
        std::priority_queue<
        std::pair<int, std::pair<int, std::vector<int>>>,
        std::vector<std::pair<int, std::pair<int, std::vector<int>>>>,
        std::greater<>
        > pq;
        std::set<std::vector<int>> visited;
        pq.push(std::make_pair(h[source],std::make_pair(0, std::vector<int>{source})));

        while (!pq.empty() && result.size() < k) {
            auto [h_prediction,data] = pq.top();
            auto [length,path_vector] = data;
            int curr = path_vector.back();
            pq.pop();
        
            // Found a path to target
            if (curr == target) { //TODO: path_)vector <int> vector??
                result.push_back({path_vector, length}); //
                continue;
            }
        
            // Skip if we've seen this exact path prefix
            if (visited.count(path_vector)) continue;
            visited.insert(path_vector);
        
            // Explore neighbors
            // for (auto& [next, weight] : graph.get_neighbors(last)) {
            for (auto edge : graph.getAdjacentEdges(curr)){
                int next =  graph.getNode(edge->v)->id;
                if(std::find(path_vector.begin(),path_vector.end(),next) != path_vector.end())
                    continue;

                std::vector<int> new_path_vector = path_vector;
                new_path_vector.push_back(next);
                int new_length = length + edge->length;
                int new_h_prediction = new_length + h[next];
                pq.push({new_h_prediction,{new_length, new_path_vector}});
            
            }
        }
    
    return result;
    }
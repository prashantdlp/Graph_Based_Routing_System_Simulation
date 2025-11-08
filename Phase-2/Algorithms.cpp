#include "Algorithms.hpp" //TODO: check if all includes are even required?? also where toi include this file?
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>
#include <set>


static std::vector<std::pair<std::vector<int>, int>> k_shortest_paths( //TODO: preprocesng??
        const Graph& graph,
        int source,
        int target,
        int k,
        const std::string& mode
    ){
        std::vector<std::pair<std::vector<int>, int>> result;
        int n = graph.size();
        std::priority_queue<
        std::pair<int, std::vector<int>>,
        std::vector<std::pair<int,std::vector<int>>>,
        std::greater<std::pair<int,std::vector<int>>>
        > pq;
        std::set<std::vector<int>> visited;

        pq.push(std::make_pair(0, std::vector<int>{source}));

        while (!pq.empty() && result.size() < k) {
            auto [length,path_vector] = pq.top();
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
                pq.push({new_length,new_path_vector});
            
            }
        }
    
    return result;
    }
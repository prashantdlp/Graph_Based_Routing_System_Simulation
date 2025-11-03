#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <vector>

// Simple version - just functions, no helper structs
class Algorithms {
public:
    // Returns: {found?, cost, path}
    static std::tuple<bool, double, std::vector<int>> dijkstra(
        const Graph& graph,
        int source,
        int target,
        const std::string& mode
    );
    
    static std::vector<int> knnEuclidean(
        const Graph& graph,
        double lat, double lon,
        const std::string& poi,
        int k
    );
};

#endif
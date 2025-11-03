#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <vector>

// Simple version - just functions, no helper structs
struct constraints{
    std::vector<int> forbidden_nodes;
    std::vector<std::string> forbidden_road_types;
};
class Algorithms {
public:
    // Returns: {found?, cost, path}
    static std::tuple<bool, double, std::vector<int>> Shortest_paths(
        const Graph& graph,
        int source,
        int target,
        const std::string& mode,
        const constraints& constraints
    );
    
    static std::vector<int> KNN(
        const Graph& graph,
        double lat, double lon,
        const std::string& poi,
        const Node& node,
        int k,
        const std::string& metric
    );
};

#endif
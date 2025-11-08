#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <vector>
#include <queue>
#include <algorithm>
// Simple version - just functions, no helper structs
struct constraints{
    std::vector<int> forbidden_nodes;
    std::vector<std::string> forbidden_road_types;
};
class Algorithms {
public:
    // Returns: {found?, cost, path}
    // static std::tuple<bool, double, std::vector<int>> Shortest_paths(
    //     const Graph& graph,
    //     int source,
    //     int target,
    //     const std::string& mode,
    //     const constraints& constraints
    // );
    
    static std::vector<std::pair<std::vector<int>, int>> k_shortest_paths(
        const Graph& graph,
        int id,
        int source,
        int target,
        int k,
        const std::string& mode
    );

};

#endif
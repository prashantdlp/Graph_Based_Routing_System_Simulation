#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <vector>
#include <tuple>
#include <string>
#include <utility>
#include  <unordered_map>
#define METERS_PER_DEG 111320.0 

struct constraints {
    std::vector<int> forbidden_nodes;
    std::vector<std::string> forbidden_road_types;
};

class Algorithms {
public:


    // --- Phase 2 ---
    static std::vector<std::pair<std::vector<int>, double>> k_shortest_paths(
        const Graph& graph,
        int source,
        int target,
        int k,
        const std::string& mode
    );

    static double approx_shortest_paths(
        const Graph& graph,
        int source,
        int target,
        int time_budget_ms,
        float acceptable_error_pct
    );

    static std::vector<std::pair<std::vector<double>, double>> k_shortest_paths_heuristic(
        const Graph& graph,
        int source,
        int target,
        int k,
        int overlap_threshold
    );
};

#endif
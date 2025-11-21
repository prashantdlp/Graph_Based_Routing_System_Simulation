#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <vector>
#include <tuple>
#include <string>
#include <utility>
#include  <unordered_map>

struct constraints {
    std::vector<int> forbidden_nodes;
    std::vector<std::string> forbidden_road_types;
};

class Algorithms {
public:
    // --- Phase 1 ---
    static std::tuple<bool, double, std::vector<int>> Shortest_paths(
         const Graph& graph,
         int source,
         int target,
         const std::string& mode,
         const constraints& constraints
    );

    static std::vector<int> KNN(
        const Graph& graph,
        double latitude,
        double longitude,
        const std::string& poi,
        int k,
        const std::string& metric
    );

    // --- Phase 2 ---
    static std::vector<std::pair<std::vector<int>, int>> k_shortest_paths(
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

    static std::vector<std::pair<std::vector<int>, int>> k_shortest_paths_heuristic(
        const Graph& graph,
        int source,
        int target,
        int k,
        int overlap_threshold
    );
};

#endif
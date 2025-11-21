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

    // --- Phase 3 ---
    static std::pair<std::vector<std::tuple<int,std::vector<int>,std::vector<int>>>,double> tsp(
        const Graph& graph,
        std::vector<std::tuple<int,int,int>> orders,
        int num_delivery_guys,
        int depot_node
    );

};

#endif
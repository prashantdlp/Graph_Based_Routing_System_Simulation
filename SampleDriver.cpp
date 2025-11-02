#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "Phase-1/Graph.h"

using json = nlohmann::json;

void process_graph_file(const std::ifstream& graph_file, Graph& G) 
{
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();
    
    // Number of nodes (optional, but we can resize in advance)
    int num_nodes = graph_json["meta"]["nodes"];
    G = Graph(num_nodes);

    // --- Parse nodes ---
    for (auto& node_json : graph_json["nodes"]) 
    {
        int id = node_json["id"];
        double lat = node_json["lat"];
        double lon = node_json["lon"];
        std::vector<std::string> pois = node_json["pois"].get<std::vector<std::string>>();
        G.addNode(id, lat, lon, pois);
    }

    // --- Parse edges ---
    for (auto& edge_json : graph_json["edges"]) 
    {
        int u = edge_json["u"];
        int v = edge_json["v"];
        double length = edge_json["length"];
        double avg_time = edge_json["average_time"];
        std::vector<double> speed_profile;
        if (edge_json.contains("speed_profile") && edge_json["speed_profile"].is_array()) 
        {
            speed_profile = edge_json["speed_profile"].get<std::vector<double>>();
        }
        bool oneway = edge_json["oneway"];
        std::string road_type = edge_json["road_type"];
        G.addEdge(u, v, length, avg_time, speed_profile, oneway, road_type);
    }
}

json process_query(const json& query) 
{
    // Placeholder for actual query processing logic
    json result;
    result["query_id"] = query["id"];
    result["status"] = "processed";
    return result;
}

int main(int argc, char* argv[]) 
{
    if (argc != 3) 
    {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json>" << std::endl;
        return 1;
    }

    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) 
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    Graph G ;
    process_graph_file(graph_file, G);

    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) 
    {
        std::cerr << "Failed to open " << argv[2] << std::endl;
        return 1;
    }

    json queries_json;
    queries_file >> queries_json;
    queries_file.close();
    
    std::ofstream output_file("output.json");
    if (!output_file.is_open()) 
    {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    for (const auto& query : queries_json) 
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        json result = process_query(query);
        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        output_file << result.dump(4) << '\n';
    }

    output_file.close();
    return 0;
}
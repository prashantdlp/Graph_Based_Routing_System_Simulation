#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "Graph.h"

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
    json result ;

    if(query["type"] == "k_shortest_paths")
    {
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];
        std::string mode = query["mode"];
        results["id"] = query["id"];
        result["paths"] = json::array();
        auto paths = k_shortest_paths(G, source, target, k, mode);
        for (const auto& p : paths) {
            json pathObj;
            pathObj["path"] = p.first;
            pathObj["length"] = p.second;
            result["paths"].push_back(pathObj);
        }
    }
    else if(query["type"] == "k_shortest_paths_heuristic")
    {
    
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];
        int overlap_threshold = query["overlap_threshold"]; 
        results["id"] = query["id"];
        result["paths"] = json::array();
        auto paths = k_shortest_paths(G, source, target, k, mode);
        for (const auto& p : paths) {
            json pathObj;
            pathObj["path"] = p.first;
            pathObj["length"] = p.second;
            result["paths"].push_back(pathObj);
        }
    }else if(query["type"] == "approx_shortest_path")
    {
        auto queries = query["queries"];
        int time_bugdet_ms = query["time_budget_ms"] ;
        float acceptable_error_pct = query["acceptable_error_pct"];  
        result["id"] = id;
        result["distances"] = json::array();
        for (const auto& q : queries) {
            int src = q["source"];
            int tgt = q["target"];
            int dist = getApproxShortestDistance(src, tgt, timeBudgetMs, acceptableErrorPct);
            json distObj;
            distObj["source"] = src;
            distObj["target"] = tgt;
            distObj["approx_shortest_distance"] = dist;
            result["distances"].push_back(distObj);
        }
    }
    return result;
}

int main(int argc, char* argv[]) 
{
    if (argc != 4) 
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

    json meta = queries_json["meta"];
    std::vector<json> results;

    for (const auto& query : queries_json["events"]) {
        auto start_time = std::chrono::high_resolution_clock::now();

        json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        results.push_back(result);
    }

    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    json output;
    output["meta"] = meta;
    output["results"] = results;
    output_file << output.dump(4) << std::endl;

    output_file.close();
    return 0;
}
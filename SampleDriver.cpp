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
    json result ;

    if(query["events"]["type"] == "remove_edge")
    {
        int id = query["events"]["edge_id"];
        result["done"] = G.removeEdge(id); 
    }
    else if (query["events"]["patch"]["type"] == "modify_edge")
    {
        int id = query["events"]["patch"]["edge_id"];
        Edge patch;
        if (query["events"]["patch"].contains("length")) 
        {
            patch.length = query["events"]["patch"]["length"];
        }
        if (query["events"]["patch"].contains("average_time")) 
        {
            patch.average_time = query["events"]["patch"]["average_time"];
        }
        if (query["events"]["patch"].contains("speed_profile")) 
        {
            patch.speed_profile = query["events"]["patch"]["speed_profile"].get<std::vector<double>>();
        }
        if (query["events"]["patch"].contains("oneway")) 
        {
            patch.oneway = query["events"]["patch"]["oneway"];
        }
        if (query["events"]["patch"].contains("road_type")) 
        {
            patch.road_type = query["events"]["patch"]["road_type"];
        }
        result["done"] = G.modifyEdge(id, patch); 
    }
    else if(query["events"]["type"] == "shortest_path")
    {
        int source = query["events"]["source"];
        int target = query["events"]["target"];
        std::string mode = query["events"]["mode"];

        constraints cons;
        if (query["events"].contains("constraints")) 
        {
            const auto& c = query["events"]["constraints"];
            if (c.contains("forbidden_nodes")) 
            {
                for (auto& node : c["forbidden_nodes"]) 
                {
                    cons.forbidden_nodes.push_back(node.get<int>());
                }
            }
            if (c.contains("forbidden_road_types")) 
            {
                for (auto& type : c["forbidden_road_types"]) 
                {
                    cons.forbidden_road_types.push_back(type.get<std::string>());
                }
            }
        }
        auto [found, cost, path] = Shortest_paths(G, source, target, mode, cons);
        
        result["id"] = query["events"]["id"];
        result["possible"] = found;
        result["mode"] = cost;
        result["path"] = path;

    }
    else if(query["events"]["type"] == "knn")
    {
        std::string poi = query["events"]["poi"];
        double lat = query["events"]["query_point"]["lat"];
        double lon = query["events"]["query_point"]["lon"];
        int k = query["events"]["k"];
        std::string metric = query["events"]["metric"];

        auto nodes = KNN(G, poi,lat ,lon, k, metric);
        result = {
            {"id": query["events"]["id"]},
            {"nodes": nodes}
        };
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

    std::vector<json> results;

    for (const auto& query : queries_json) {
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

    json output = results;
    output_file << output.dump(4) << std::endl;

    output_file.close();
    return 0;
}
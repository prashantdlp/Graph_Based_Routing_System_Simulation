#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "Graph.hpp"
#include "Algorithms.hpp"

using json = nlohmann::json;

void process_graph_file(std::ifstream& graph_file, Graph& G) 
{
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();
    
    // Number of nodes (optional, but we can resize in advance)
    int num_nodes = graph_json["meta"]["nodes"];
    G = Graph();

    // --- Parse nodes ---
    for (auto& node_json : graph_json["nodes"]) 
    {
        int id     = node_json["id"];
        double lat = node_json["lat"];
        double lon = node_json["lon"];
        std::vector<std::string> pois;
        if (node_json.contains("pois") && node_json["pois"].is_array()) {
            for (const auto &p : node_json["pois"]) {
                if (p.is_string()) {
                    pois.push_back(p.get<std::string>());
                }
            }
        }
        G.addNode(id, lat, lon, pois);
    }

    // --- Parse edges ---
    for (auto& edge_json : graph_json["edges"]) 
    {
        int id = edge_json["id"];
        int u = edge_json["u"];
        int v = edge_json["v"];
        double length = edge_json["length"];
        double avg_time = edge_json["average_time"];
        std::vector<double>speed_profile;
        if (edge_json.contains("speed_profile") && edge_json["speed_profile"].is_array())
        {
            speed_profile = edge_json["speed_profile"].get<std::vector<double>>();
        }
        bool oneway = false; 
        if(edge_json.contains("oneway"))
        {
            oneway = edge_json["oneway"];
        }    
        std::string road_type = "primary" ; 
        if(edge_json.contains("road_type"))
        {
            road_type = edge_json["road_type"];
        }
        G.addEdge(id, u, v, length, avg_time, speed_profile, oneway, road_type);
    }
}

json process_query(const json &query, std::ifstream &graph_file, Graph &G) 
{
    json result ;
    Algorithms A ;
    std::vector<std::tuple<int,int,int>> orders;

    for (const auto &o : query["orders"]) {
        int oid = o["order_id"];
        int pickup = o["pickup"];
        int dropoff = o["dropoff"];
        orders.emplace_back(oid, pickup, dropoff);
    }
    int num_delivery_guys = query["fleet"]["num_delievery_guys"];  
    int depot_node = query["fleet"]["depot_node"];
    auto ans = A.tsp(G, orders, num_delivery_guys, depot_node);

    result["assignments"] = json::array();
    for (const auto &assignment : ans.first) {
        int driver_id;
        std::vector<int> route;
        std::vector<int> order_ids;
        std::tie(driver_id, route, order_ids) = assignment;
        json obj;
        obj["driver_id"] = driver_id;
        obj["route"] = route;
        obj["order_ids"] = order_ids;
        result["assignments"].push_back(obj);
    }
    result["metrics"]["total_delivery_time_s"] = ans.second;
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
        std::cerr << "Failed to open " << argv[1] << std::endl;
        return 1;
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
        json result;
        try {
            result = process_query(query,graph_file, G);
        } 
        catch (const std::exception &e) {
            result["error"] = std::string("exception: ") + e.what();
        }
        catch (...) {
            result["error"] = "unknown exception";
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        results.push_back(result);
    }

    json output;
    output["meta"] = meta;
    output["results"] = results;
    output_file << output.dump(4) << std::endl;

    output_file.close();
    return 0;
}
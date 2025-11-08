#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <optional>

struct Edge{
    int id;
    int u,v;
    double length;
    double average_time ;
    std::vector<double> speed_profile ;
    std::optional<bool> oneway ; // bool restricted ;
    std::string road_type ;
    bool active;

    Edge() : id(-1), u(-1), v(-1), length(0.0), average_time(0.0), 
             oneway(false), active(true) {}
    Edge(int id, int u, int v, double length, double avg_time,
         const std::vector<double>& sp, bool oneway, const std::string& rt, bool active)
        : id(id), u(u), v(v), length(length), average_time(avg_time),
          speed_profile(sp), oneway(oneway), road_type(rt), active(active) {}
};

struct Node{
    // bool is_latitude ; true if latitude, false if x or y coordinates 
    int id;
    double lat ; // lat 
    double lon ; // lon 
    std::vector<std::string> pois ;

    Node() : id(-1), lat(0.0), lon(0.0) {}
    Node(int id, double lat, double lon, const std::vector<std::string>& pois)
        : id(id), lat(lat), lon(lon), pois(pois) {}
};

class Graph{
    int n ; // number of nodes
    
    // to store [lat,lon] / [x,y] and name of node [might contain multiple names for a single node]
    // id is mapped to index in this vector
    // std::vector<Node*> Nodes ; 
    std::unordered_map<int, Node*> nodes; // node ids -> Node
    std::unordered_map<int, Edge*> edges; // edge ids -> Edge
    std::unordered_map<int, std::vector< Edge*>> adj_list; // nodes ids -> vector of edge ids
    std::unordered_map<int, std::vector< Edge*>> inc_list; // nodes ids -> vector of edge ids for incoming edges
    std::unordered_map<std::string, std::vector<int>> poi_index; // poi -> vector of node ids
    // adj_matrix[u][v] gives pointer to edge from u to v
    // std::vector<std::vector<Edge*>> adj_matrix ;

public: 
    Graph();
    void addNode(int id, double lat, double lon, const std::vector<std::string>& names);
    void addEdge(int id, int u, int v, double length, double avg_time,
                 const std::vector<double>& speed_profile, bool oneway, const std::string& road_type);
    
    const Node* getNode(int id) const;
    const Edge* getEdge(int u, int v) const;

    bool removeEdge(int edge_id);
    void modifyEdge(int edge_id, const Edge& patch);
    // void restoreEdge(int edge_id, const Edge& patch);
    const std::vector<Edge*>& getAdjacentEdges(int node_id) const;
    const std::vector<Edge*>& getIncomingEdges(int node_id) const;
    std::vector<int> getNodesWithPOI(const std::string& poi) const;

    double getEdgeWeight(int edge_id, const std::string& mode, int time_slot = -1) const; //TODO: ??
    // double euclideanDistance(int node1, int node2) const; not needed phase 1
    int size() const { return n; }
    ~Graph();

};

#endif // GRAPH_H
#ifdef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>

struct Edge{
    int id;
    int u,v;
    double length;
    double average_time ;
    std::vector<double> speed_profile ;
    bool oneway ; // bool restricted ;
    std::string road_type ;
};

struct Node{
    // bool is_latitude ; true if latitude, false if x or y coordinates 
    int id;
    double lat ; // lat 
    double lon ; // lon 
    std::vector<std::string> names ;
};

class Graph{
    int n ; // number of nodes
    
    // to store [lat,lon] / [x,y] and name of node [might contain multiple names for a single node]
    // id is mapped to index in this vector
    // std::vector<Node*> Nodes ; 
    std::unordered_map<int, Node> nodes; // node ids -> Node
    std::unordered_map<int, Edge> edges; // edge ids -> Edge
    std::unordered_map<int, std::vector<int>> adj_list; // nodes ids -> vector of edge ids
    std::unordered_map<std::string, std::vector<int>> poi_index; // poi -> vector of node ids
    // adj_matrix[u][v] gives pointer to edge from u to v
    // std::vector<std::vector<Edge*>> adj_matrix ;

public: 
    Graph();
    void addNode(int id, double lat, double lon, const std::vector<std::string>& names);
    void addEdge(int u, int v, double length, double avg_time,
                 const std::vector<double>& speed_profile, bool oneway, const std::string& road_type);
    
    const Node* getNode(int id) const;
    const Edge* getEdge(int u, int v) const;

    void removeEdge(int edge_id);
    void modifyEdge(int edge_id, const Edge& patch)
    void restoreEdge(int edge_id, const Edge& patch);
    const std::vector<int>& getAdjacentEdges(int node_id) const;
    std::vector<int> getNodesWithPOI(const std::string& poi) const;

    double getEdgeWeight(int edge_id, const std::string& mode, int time_slot = -1) const; //TODO: ??
    // double euclideanDistance(int node1, int node2) const; not needed phase 1
    int size() const { return n; }
    ~Graph();

};

#endif // GRAPH_H
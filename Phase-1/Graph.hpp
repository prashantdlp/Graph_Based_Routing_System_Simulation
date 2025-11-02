#ifdef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>

struct Edge{
    double length;
    double average_time ;
    std::vector<double> speed_profile ;
    bool oneway ; // bool restricted ;
    std::string road_type ;
};

struct Node{
    // bool is_latitude ; true if latitude, false if x or y coordinates 
    double x ; // lat 
    double y ; // lon 
    std::vector<std::string> names ;
};

class Graph{
    int n ; // number of nodes
    
    // to store [lat,lon] / [x,y] and name of node [might contain multiple names for a single node]
    // id is mapped to index in this vector
    std::vector<Node*> Nodes ; 
    
    // adj_matrix[u][v] gives pointer to edge from u to v
    std::vector<std::vector<Edge*>> adj_matrix ;

public:

};

#endif // GRAPH_H
#include "Algorithms.hpp"
#include <algorithm>
#include <queue>

std::pair<std::vector<std::tuple<int,std::vector<int>,std::vector<int>>>,double> Algorithms::tsp(
        const Graph& graph,
        std::vector<std::tuple<int,int,int>> orders,
        int num_delivery_guys,
        int depot_node
    ){

    using namespace std;

    unordered_map<int, vector<double>> dist_cache; 
    auto dijkstra = [&](int src)->const vector<double>&{
        if(dist_cache.count(src)) return dist_cache[src];
        int N = graph.size();
        vector<double> dist(N, numeric_limits<double>::infinity());
        dist[src] = 0;
        using P = pair<double,int>;
        priority_queue<P, vector<P>, greater<P>> pq;
        pq.push({0, src});
        while(!pq.empty()){
            auto [d,u] = pq.top(); pq.pop();
            if(d>dist[u]) continue;
            for(auto &e : graph.getAdjacentEdges(u)){
                int v = e->id; double w = e->length;
                if(dist[v] > d + w){
                    dist[v] = d + w;
                    pq.push({dist[v], v});
                }
            }
        }
        dist_cache[src] = move(dist);
        return dist_cache[src];
    };

    auto get_distance = [&](int u, int v)->double{
        const vector<double>& du = dijkstra(u);
        return du[v];
    };

    struct Order {
        int id;
        int pu;
        int dof;
        bool picked = false;
        bool delivered = false;
        double completion_time = -1;
    };
    
    vector<Order> O;
    O.reserve(orders.size());
    for(auto &t : orders){
        int id, pu, df; tie(id,pu,df) = t;
        O.push_back({id, pu, df, false, false, -1.0});
    }
    int M = (int)O.size();

    struct DriverState {
        int id;
        int node;
        double time;
        vector<int> route;
        vector<int> carried_indices; 
        vector<int> handled_orders; 
    };
    
    vector<DriverState> drivers;
    drivers.reserve(num_delivery_guys);
    for(int i=0;i<num_delivery_guys;i++){
        DriverState d;
        d.id = i;
        d.node = depot_node;
        d.time = 0.0;
        d.route.push_back(depot_node);
        drivers.push_back(move(d));
    }

    int remaining = M;

    unordered_set<int> unpicked_indices;
    for(int i=0;i<M;i++) unpicked_indices.insert(i);

    while(remaining > 0){
        bool any_progress = false;

        for(auto &drv : drivers){
            if(remaining <= 0) break;

            int best_node = -1;
            double best_score = numeric_limits<double>::infinity();
            bool best_is_pickup = false;
            int best_order_idx = -1;

            for(int oi : drv.carried_indices){
                if(O[oi].delivered) continue;
                int node = O[oi].dof;
                double dist = get_distance(drv.node, node);
                if(dist < best_score){
                    best_score = dist;
                    best_node = node;
                    best_is_pickup = false;
                    best_order_idx = oi;
                }
            }

            for(int oi : unpicked_indices){
                if(O[oi].picked) continue;
                int node = O[oi].pu;
                double dist = get_distance(drv.node, node);

                double score = dist;
                if(!drv.carried_indices.empty()) score *= 1.02; 
                if(score < best_score){
                    best_score = score;
                    best_node = node;
                    best_is_pickup = true;
                    best_order_idx = oi;
                }
            }

            if(best_node == -1) continue;

            double travel = get_distance(drv.node, best_node);
            if(!isfinite(travel) || travel > 1e15) continue;

            drv.time += travel;
            drv.node = best_node;
            drv.route.push_back(best_node);

            if(best_is_pickup){
                int oi = best_order_idx;
                O[oi].picked = true;
                drv.carried_indices.push_back(oi);
                drv.handled_orders.push_back(O[oi].id);
                unpicked_indices.erase(oi);
                any_progress = true;
            } else {
                int oi = best_order_idx;
                if(!O[oi].delivered){
                    O[oi].delivered = true;
                    O[oi].completion_time = drv.time;
                    auto &v = drv.carried_indices;
                    v.erase(remove(v.begin(), v.end(), oi), v.end());
                    remaining--;
                    any_progress = true;
                }
            }
        }

        if(!any_progress){
            break;
        }
    } 

    double total_delivery_time = 0.0;
    for(auto &o : O){
        if(o.delivered) total_delivery_time += o.completion_time;
        else {
            total_delivery_time += 1e9;
        }
    }

    vector<tuple<int, vector<int>, vector<int>>> assignments;
    assignments.reserve(drivers.size());
    for(auto &d : drivers){
        vector<int> uniq_orders = d.handled_orders;
        sort(uniq_orders.begin(), uniq_orders.end());
        uniq_orders.erase(unique(uniq_orders.begin(), uniq_orders.end()), uniq_orders.end());
        assignments.push_back({d.id, d.route, uniq_orders});
    }

    return {assignments, total_delivery_time};
    }
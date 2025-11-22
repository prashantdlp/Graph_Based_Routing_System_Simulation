// #include "Algorithms.hpp"
// #include <algorithm>
// #include <queue>
// #include <limits>

// std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double> Algorithms::tsp(
//     const Graph &graph,
//     std::vector<std::tuple<int, int, int>> orders,
//     int num_delivery_guys,
//     int depot_node)
// {

//     using namespace std;

//     // Dijkstra with path reconstruction
//     struct DijkstraResult
//     {
//         vector<double> dist;
//         vector<int> parent;
//     };

//     unordered_map<int, DijkstraResult> dijkstra_cache;

//     auto dijkstra = [&](int src) -> const DijkstraResult &
//     {
//         if (dijkstra_cache.count(src))
//             return dijkstra_cache[src];

//         int N = graph.size();
//         DijkstraResult result;
//         result.dist.assign(N, numeric_limits<double>::infinity());
//         result.parent.assign(N, -1);
//         result.dist[src] = 0;

//         using P = pair<double, int>;
//         priority_queue<P, vector<P>, greater<P>> pq;
//         pq.push({0, src});

//         while (!pq.empty())
//         {
//             auto [d, u] = pq.top();
//             pq.pop();
//             if (d > result.dist[u])
//                 continue;

//             for (auto &e : graph.getAdjacentEdges(u))
//             {
//                 int v = e->v;
//                 double w = e->length;
//                 if (result.dist[v] > d + w)
//                 {
//                     result.dist[v] = d + w;
//                     result.parent[v] = u;
//                     pq.push({result.dist[v], v});
//                 }
//             }
//         }

//         dijkstra_cache[src] = move(result);
//         return dijkstra_cache[src];
//     };

//     // Get shortest path from u to v
//     auto get_path = [&](int u, int v) -> vector<int>
//     {
//         const auto &result = dijkstra(u);
//         if (!isfinite(result.dist[v]))
//             return {};

//         vector<int> path;
//         int curr = v;
//         while (curr != -1)
//         {
//             path.push_back(curr);
//             curr = result.parent[curr];
//         }
//         reverse(path.begin(), path.end());
//         return path;
//     };

//     auto get_distance = [&](int u, int v) -> double
//     {
//         const auto &result = dijkstra(u);
//         return result.dist[v];
//     };

//     struct Order
//     {
//         int id;
//         int pickup;
//         int dropoff;
//         bool picked = false;
//         bool delivered = false;
//         double completion_time = -1;
//     };

//     vector<Order> all_orders;
//     for (auto &[id, pu, df] : orders)
//     {
//         all_orders.push_back({id, pu, df, false, false, -1.0});
//     }
//     int M = (int)all_orders.size();

//     struct DriverState
//     {
//         int id;
//         int current_node;
//         double current_time;
//         vector<int> route; // Complete route with all intermediate nodes
//         vector<int> carrying;
//         vector<int> all_order_ids;
//     };

//     vector<DriverState> drivers;
//     for (int i = 0; i < num_delivery_guys; i++)
//     {
//         drivers.push_back({i, depot_node, 0.0, {depot_node}, {}, {}});
//     }

//     unordered_set<int> unpicked;
//     for (int i = 0; i < M; i++)
//         unpicked.insert(i);

//     int completed_orders = 0;

//     // Main greedy algorithm
//     while (completed_orders < M)
//     {
//         bool made_progress = false;

//         for (auto &driver : drivers)
//         {
//             if (completed_orders >= M)
//                 break;

//             int best_node = -1;
//             double best_cost = numeric_limits<double>::infinity();
//             bool is_pickup = false;
//             int target_order_idx = -1;

//             // Priority 1: Deliver orders already carrying
//             if (!driver.carrying.empty())
//             {
//                 for (int oi : driver.carrying)
//                 {
//                     if (all_orders[oi].delivered)
//                         continue;

//                     int dropoff_node = all_orders[oi].dropoff;
//                     double dist = get_distance(driver.current_node, dropoff_node);

//                     if (dist < best_cost)
//                     {
//                         best_cost = dist;
//                         best_node = dropoff_node;
//                         is_pickup = false;
//                         target_order_idx = oi;
//                     }
//                 }
//             }

//             // Priority 2: Pick up new orders (only if not carrying)
//             if (driver.carrying.empty())
//             {
//                 for (int oi : unpicked)
//                 {
//                     int pickup_node = all_orders[oi].pickup;
//                     double dist = get_distance(driver.current_node, pickup_node);

//                     if (dist < best_cost)
//                     {
//                         best_cost = dist;
//                         best_node = pickup_node;
//                         is_pickup = true;
//                         target_order_idx = oi;
//                     }
//                 }
//             }

//             if (best_node == -1)
//                 continue;

//             // Get the actual path from current to target
//             vector<int> path = get_path(driver.current_node, best_node);
//             if (path.empty())
//                 continue;

//             double travel_dist = get_distance(driver.current_node, best_node);
//             if (!isfinite(travel_dist) || travel_dist > 1e15)
//                 continue;

//             // Update driver state
//             driver.current_time += travel_dist;
//             driver.current_node = best_node;

//             // Append path to route (skip first node as it's already in route)
//             for (size_t i = 1; i < path.size(); i++)
//             {
//                 driver.route.push_back(path[i]);
//             }

//             if (is_pickup)
//             {
//                 int oi = target_order_idx;
//                 all_orders[oi].picked = true;
//                 driver.carrying.push_back(oi);
//                 driver.all_order_ids.push_back(all_orders[oi].id);
//                 unpicked.erase(oi);
//                 made_progress = true;
//             }
//             else
//             {
//                 int oi = target_order_idx;
//                 all_orders[oi].delivered = true;
//                 all_orders[oi].completion_time = driver.current_time;

//                 auto &carr = driver.carrying;
//                 carr.erase(remove(carr.begin(), carr.end(), oi), carr.end());

//                 completed_orders++;
//                 made_progress = true;
//             }
//         }

//         if (!made_progress)
//             break;
//     }

//     // Calculate total delivery time
//     double total_time = 0.0;
//     for (auto &order : all_orders)
//     {
//         if (order.delivered)
//         {
//             total_time += order.completion_time;
//         }
//         else
//         {
//             total_time += 1e9;
//         }
//     }

//     // Prepare output
//     vector<tuple<int, vector<int>, vector<int>>> assignments;
//     for (auto &d : drivers)
//     {
//         vector<int> unique_orders = d.all_order_ids;
//         sort(unique_orders.begin(), unique_orders.end());
//         unique_orders.erase(unique(unique_orders.begin(), unique_orders.end()),
//                             unique_orders.end());

//         assignments.push_back({d.id, d.route, unique_orders});
//     }

//     return {assignments, total_time};
// }

#include "Algorithms.hpp"
#include <algorithm>
#include <queue>
#include <random>
#include <chrono>
#include <unordered_map>
#include <set>

std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double>
Algorithms::tsp(
    const Graph &graph,
    std::vector<std::tuple<int, int, int>> orders,
    int num_delivery_guys,
    int depot_node)
{
    using namespace std;
    if (num_delivery_guys <= 0)
        return {{}, 1e9};

    // Caches
    std::unordered_map<int, std::vector<double>> dist_cache;
    std::unordered_map<int, std::vector<int>> parent_cache;

    auto dijkstra = [&](int src)
    {
        // If already cached, return directly
        if (dist_cache.count(src))
            return;

        int N = graph.size();

        std::vector<double> dist(N, std::numeric_limits<double>::infinity());
        std::vector<int> parent(N, -1);

        dist[src] = 0.0;

        using P = std::pair<double, int>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        pq.push({0.0, src});

        while (!pq.empty())
        {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[u])
                continue;

            for (auto &e : graph.getAdjacentEdges(u))
            {
                int v = e->v;
                double w = e->length;

                if (dist[v] > d + w)
                {
                    dist[v] = d + w;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // Store into cache
        dist_cache[src] = std::move(dist);
        parent_cache[src] = std::move(parent);
    };

    // --------------------------------------------------
    // shortest path: u -> v
    // --------------------------------------------------

    auto get_path = [&](int u, int v) -> std::vector<int>
    {
        // Ensure dijkstra(u) is computed
        dijkstra(u);

        const auto &dist = dist_cache[u];
        const auto &parent = parent_cache[u];

        if (!std::isfinite(dist[v]))
            return {};

        std::vector<int> path;
        int curr = v;

        while (curr != -1)
        {
            path.push_back(curr);
            curr = parent[curr];
        }

        std::reverse(path.begin(), path.end());
        return path;
    };

    double time_of_delivery = 0.0;
    vector<tuple<int, vector<int>, vector<int>>> delivery_guy_routes;

    vector<int> last_position(num_delivery_guys, depot_node);
    unordered_map<int, vector<int>> delivery_guy_assigned_orders;
    unordered_map<int, vector<int>> path_delivery_guy;

    for (int i = 0; i < orders.size(); i++)
    {
        int pickup = get<1>(orders[i]);
        int delivery = get<2>(orders[i]);

        int g = i % num_delivery_guys;

        vector<int> ans = get_path(last_position[g], pickup);
        if (ans.empty())
            return {{}, 1e9};

        vector<int> mid = get_path(pickup, delivery);
        if (mid.empty())
            return {{}, 1e9};

        last_position[g] = delivery;

        // avoid duplicate pickup node
        ans.insert(ans.end(), mid.begin() + 1, mid.end());

        path_delivery_guy[g].insert(
            path_delivery_guy[g].end(),
            ans.begin(), ans.end());

        delivery_guy_assigned_orders[g].push_back(get<0>(orders[i]));

        for (int k = 1; k < ans.size(); k++)
        {
            const Edge *e = graph.getEdge(ans[k - 1], ans[k]);
            time_of_delivery += e->average_time; // FIXED
        }
    }

    for (int i = 0; i < num_delivery_guys; i++)
    {
        delivery_guy_routes.push_back({i,
                                       delivery_guy_assigned_orders[i],
                                       path_delivery_guy[i]});
    }

    return {delivery_guy_routes, time_of_delivery};
}
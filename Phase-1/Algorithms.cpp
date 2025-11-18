#include "Algorithms.hpp"
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>

std::tuple<bool, double, std::vector<int>> Shortest_paths_distance( // TODO:OPTIMIZATIONS>>>
    const Graph &graph,
    int source,
    int target,
    const constraints &constraints)
{
    int n = graph.size();

    std::vector<int> forbidden_nodes = constraints.forbidden_nodes;
    std::vector<std::string> forbidden_road_types = constraints.forbidden_road_types;

    std::unordered_set<int> forbidden(
        constraints.forbidden_nodes.begin(),
        constraints.forbidden_nodes.end());
    std::unordered_set<std::string> forbidden_r(
        constraints.forbidden_road_types.begin(),
        constraints.forbidden_road_types.end());

    if (source < 0 || source >= n ||
        target < 0 || target >= n)
    {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<bool> visited(n, false);
    std::vector<double> distance(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);

    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        std::greater<std::pair<double, int>>>
        pq;

    distance[source] = 0.0;
    pq.push({0.0, source});
    while (!pq.empty())
    {
        auto [dist, v] = pq.top();
        pq.pop();

        // Skip if visited
        if (visited[v])
            continue;

        visited[v] = true;

        // reached target
        if (v == target)
            break;

        for (auto edge : graph.getAdjacentEdges(v))
        {
            if (forbidden_r.find(edge->road_type) == forbidden_r.end() && edge->active)
            {
                double length = edge->length;
                int w = edge->v;
                if (!visited[w] && forbidden.find(w) == forbidden.end())
                {
                    double new_dist = distance[v] + length;

                    if (new_dist < distance[w])
                    {
                        distance[w] = new_dist;
                        parent[w] = v;
                        pq.push({new_dist, w});
                    }
                }
            }
        }
    }
    if (distance[target] == std::numeric_limits<double>::infinity())
    {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<int> path;
    int current = target;
    while (current != -1)
    {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());

    return {true, distance[target], path};
}

double Edge_time(
    const Edge *edge,
    double start_time)
{
    const double TIME_SLOT = 15.0 * 60;
    double remaining_distance = edge->length;
    double current_time = start_time;
    double total_time = 0.0;

    while (remaining_distance > 1e-9)
    {
        int slot = static_cast<int>(floor(current_time / TIME_SLOT));

        if (edge->speed_profile.empty())
            return total_time + edge->average_time;
        if (slot >= static_cast<int>(edge->speed_profile.size()))
            slot = edge->speed_profile.size() - 1;
        double speed = edge->speed_profile[slot];
        // Time remaining in current slot fa
        double next_slot_boundary = (slot + 1) * TIME_SLOT;
        double time_remaining_in_slot = next_slot_boundary - current_time;

        // Distance we can cover in remaining time of current slot
        double distance_in_slot = speed * time_remaining_in_slot;

        if (distance_in_slot >= remaining_distance)
        {
            // We finish in this slot
            total_time += remaining_distance / speed;
            break;
        }
        else
        {
            // Move to next slot
            total_time += time_remaining_in_slot;
            remaining_distance -= distance_in_slot;
            current_time = next_slot_boundary;
        }
    }

    return total_time;
}

std::tuple<bool, double, std::vector<int>> Shortest_paths_speed(
    const Graph &graph,
    int source,
    int target,
    const constraints &constraints)
{
    int n = graph.size();
    const double TIME_SLOT = 15.0;

    std::vector<int> forbidden_nodes = constraints.forbidden_nodes;
    std::vector<std::string> forbidden_road_types = constraints.forbidden_road_types;

    std::unordered_set<int> forbidden(
        constraints.forbidden_nodes.begin(),
        constraints.forbidden_nodes.end());
    std::unordered_set<std::string> forbidden_r(
        constraints.forbidden_road_types.begin(),
        constraints.forbidden_road_types.end());

    if (source < 0 || source >= n ||
        target < 0 || target >= n)
    {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<bool> visited(n, false);
    std::vector<double> time_to_reach(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    double time = 0.0;

    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        std::greater<std::pair<double, int>>>
        pq;

    time_to_reach[source] = 0.0;
    pq.push({0.0, source});
    while (!pq.empty())
    {
        auto [current_time, u] = pq.top();
        pq.pop();

        // Skip if visited
        if (visited[u])
            continue;

        visited[u] = true;

        // reached target
        if (u == target)
            break;

        for (auto edge : graph.getAdjacentEdges(u))
        { // TODO: consider edge is active or not??
            if (forbidden_r.find(edge->road_type) == forbidden_r.end() && edge->active)
            {
                double time_edge = Edge_time(edge, current_time);
                int w = edge->v;
                if (!visited[w] && forbidden.find(w) == forbidden.end())
                {
                    double new_time = time_to_reach[u] + time_edge;

                    if (new_time < time_to_reach[w])
                    {
                        time_to_reach[w] = new_time;
                        parent[w] = u;
                        pq.push({new_time, w});
                    }
                }
            }
        }
    }
    if (time_to_reach[target] == std::numeric_limits<double>::infinity())
    {
        return {false, std::numeric_limits<double>::infinity(), {}};
    }
    std::vector<int> path;
    int current = target;
    while (current != -1)
    {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());

    return {true, time_to_reach[target], path};
}

std::tuple<bool, double, std::vector<int>> Algorithms::Shortest_paths(
    const Graph &graph,
    int source,
    int target,
    const std::string &mode,
    const constraints &constraints)
{
    if (mode == "minimum_distance")
    {
        return Shortest_paths_distance(graph, source, target, constraints);
    }
    return Shortest_paths_speed(graph, source, target, constraints);
}

double euclidian_distance(double lat1, double lon1, double lat2, double lon2)
{
    const double REFERENCE_LATITUDE = 19.07;
    const double DEG_TO_RAD = M_PI / 180.0;
    const double METERS_PER_DEG_LAT = 1111390;

    const double METERS_PER_DEG_LON = METERS_PER_DEG_LAT * std::cos(REFERENCE_LATITUDE * DEG_TO_RAD);
    // North-South distance
    double deltaY_meters = (lat2 - lat1) * METERS_PER_DEG_LAT;
    // East-West distance
    double deltaX_meters = (lon2 - lon1) * METERS_PER_DEG_LON;

    return std::sqrt(deltaX_meters * deltaX_meters + deltaY_meters * deltaY_meters);
}

int getNearestNodeId(const Graph &graph, double latitude, double longitude)
{
    for (int i = 0; i < graph.size(); i++)
    {
        const Node *tempnode = graph.getNode(i);
        if (tempnode->lat == latitude && tempnode->lon == longitude)
        {
            return i;
        }
    }
    int nearestId = 0;
    double minDistance = std::numeric_limits<double>::max();
    const Node *node0 = graph.getNode(0);
    minDistance = euclidian_distance(node0->lat, node0->lon, latitude, longitude);

    // double nearestDistance = euclidian_distance(graph.getNode(nearestId)->lat, graph.getNode(nearestId)->lon, latitude, longitude);
    for (int i = 1; i < graph.size(); i++)
    {
        const Node *node = graph.getNode(i);
        double distance = euclidian_distance(node->lat, node->lon, latitude, longitude);
        if (distance < minDistance)
        {
            minDistance = distance;
            nearestId = i;
        }
    }
    return nearestId;
}
std::vector<double> Dijkstra_OneToAll(const Graph &graph, int source)
{
    int n = graph.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());

    // Min-priority queue: stores {distance, u}
    using PII = std::pair<double, int>;
    std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;

    if (source >= 0 && source < n)
    {
        dist[source] = 0.0;
        pq.push({0.0, source});
    }

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u])
            continue;

        for (auto edge : graph.getAdjacentEdges(u))
        {
            // Only consider active edges. KNN metric usually implies pure distance.
            int v = edge->v;
            double weight = edge->length;
            if (dist[u] + weight < dist[v])
            {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

std::vector<int> Algorithms::KNN(
    const Graph &graph,
    double latitude,
    double longitude,
    const std::string &poi,
    int k,
    const std::string &metric)
{
    // based on euclidian
    std::vector<int> poi_nodes = graph.getNodesWithPOI(poi);
    std::vector<std::pair<double, int>> candidates;
    std::priority_queue<std::pair<double, int>> distances;

    if (metric == "euclidean")
    {
        for (int id : poi_nodes)
        {
            const Node *node = graph.getNode(id);
            if (!node)
                continue;
            double dist = euclidian_distance(latitude, longitude, node->lat, node->lon);
            candidates.push_back({dist, id});
        }
    }
    else if (metric == "shortest_path")
    {
        int originID = getNearestNodeId(graph, latitude, longitude);
        std::vector<double> network_dist = Dijkstra_OneToAll(graph, originID);
        for (int id : poi_nodes)
        {
            double d = network_dist[id];
            if (d != std::numeric_limits<double>::infinity())
            {
                candidates.push_back({d, id});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end());
    std::vector<int> knns;
    for (size_t i = 0; i < candidates.size() && i < (size_t)k; ++i)
    {
        knns.push_back(candidates[i].second);
    }
    return knns;
}
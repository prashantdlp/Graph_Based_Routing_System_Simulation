#include "Algorithms.hpp" //TODO: check if all includes are even required?? also where toi include this file? also what bout POI in phase 2.
#include <limits>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>
#include <set>
#include <chrono>

static std::vector<double> compute_heuristic(const Graph &graph, int target)
{
    int n = graph.size();
    std::vector<double> h(n, std::numeric_limits<double>::max());

    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        std::greater<>>
        pq;

    h[target] = 0.0;
    pq.push({0.0, target});

    while (!pq.empty())
    {
        auto [dist, u] = pq.top();
        pq.pop();

        if (dist > h[u])
            continue;

        for (const auto *edge : graph.getIncomingEdges(u))
        {
            int source = graph.getNode(edge->u)->id;
            double new_dist = dist + edge->length;

            if (new_dist < h[source])
            {
                h[source] = new_dist;
                pq.push({new_dist, source});
            }
        }
    }

    return h;
}

static std::vector<std::pair<std::vector<int>, double>> k_shortest_paths(
    const Graph &graph,
    int source,
    int target,
    int k,
    const std::string &mode)
{
    std::vector<std::pair<std::vector<int>, double>> result;
    result.reserve(k);
    int n = graph.size();
    std::vector<double> h = compute_heuristic(graph, target);

    std::priority_queue<
        std::pair<double, std::pair<double, std::vector<int>>>, // FIX #2: int → double
        std::vector<std::pair<double, std::pair<double, std::vector<int>>>>,
        std::greater<>>
        pq;

    std::set<std::vector<int>> completed_paths;
    pq.push({h[source], {0.0, std::vector<int>{source}}}); // FIX #2: 0 → 0.0

    double worst_path_cost = std::numeric_limits<double>::max(); // FIX #1: int::max → double::max

    while (!pq.empty() && result.size() < k)
    {
        auto [h_prediction, data] = pq.top();
        auto [length, path_vector] = data;
        int curr = path_vector.back();
        pq.pop();

        if (result.size() == k && h_prediction > worst_path_cost)
        {
            break;
        }

        if (curr == target)
        {
            if (completed_paths.count(path_vector) == 0)
            {
                result.push_back({path_vector, length});
                completed_paths.insert(path_vector);
                worst_path_cost = std::min(worst_path_cost, length); // FIX #3: ADD THIS
            }
            continue;
        }

        if (h[curr] == std::numeric_limits<double>::max())
            continue;

        std::unordered_set<int> in_path(path_vector.begin(), path_vector.end());

        for (auto edge : graph.getAdjacentEdges(curr))
        {
            int next = graph.getNode(edge->v)->id;

            if (in_path.count(next))
                continue;

            if (h[next] == std::numeric_limits<double>::max())
                continue;

            std::vector<int> new_path_vector = path_vector;
            new_path_vector.push_back(next);
            double new_length = length + edge->length;
            double new_h_prediction = new_length + h[next];

            pq.push({new_h_prediction, {new_length, new_path_vector}});
        }
    }

    return result;
}

double Algorithms::approx_shortest_paths( // TODO: implement
    const Graph &graph,
    int source,
    int target,
    int time_budget_ms,
    float acceptable_error_pct)
{
    using namespace std;
    using namespace std::chrono;

    if (source == target)
        return 0.0;

    auto start = high_resolution_clock::now();
    auto deadline = start + milliseconds((int)(time_budget_ms * 0.90));

    const int N = graph.size();
    const double INF = 1e18;

    vector<double> distF(N, INF), distB(N, INF);
    vector<bool> visitedF(N, false), visitedB(N, false);

    // --- Heuristics ---
    auto hF = [&](int u)
    {
        const Node *nu = graph.getNode(u);
        const Node *nt = graph.getNode(target);
        if (!nu || !nt)
            return 0.0;
        double dx = nu->lat - nt->lat;
        double dy = nu->lon - nt->lon;
        return sqrt(dx * dx + dy * dy);
    };

    auto hB = [&](int u)
    {
        const Node *nu = graph.getNode(u);
        const Node *ns = graph.getNode(source);
        if (!nu || !ns)
            return 0.0;
        double dx = nu->lat - ns->lat;
        double dy = nu->lon - ns->lon;
        return sqrt(dx * dx + dy * dy);
    };

    // Priority queues
    struct State
    {
        int node;
        double f;
        bool operator<(const State &o) const { return f > o.f; }
    };

    priority_queue<State> pqF, pqB;

    distF[source] = 0;
    distB[target] = 0;

    pqF.push({source, hF(source)});
    pqB.push({target, hB(target)});

    double best_meeting = INF;

    // Main loop
    while (!pqF.empty() && !pqB.empty())
    {
        if (high_resolution_clock::now() > deadline)
            break;

        // Expand smaller frontier
        bool expandForward = pqF.top().f < pqB.top().f;

        if (expandForward)
        {
            // Forward expansion
            auto [u, fval] = pqF.top();
            pqF.pop();
            if (visitedF[u])
                continue;
            visitedF[u] = true;

            if (visitedB[u])
            {
                best_meeting = min(best_meeting, distF[u] + distB[u]);
            }

            for (const Edge *e : graph.getAdjacentEdges(u))
            {
                int v = e->v;
                double w = e->length;

                if (distF[u] + w < distF[v])
                {
                    distF[v] = distF[u] + w;
                    pqF.push({v, distF[v] + hF(v)});
                    if (visitedB[v])
                        best_meeting = min(best_meeting, distF[v] + distB[v]);
                }
            }
        }
        else
        {
            // Backward expansion
            auto [u, fval] = pqB.top();
            pqB.pop();
            if (visitedB[u])
                continue;
            visitedB[u] = true;

            if (visitedF[u])
            {
                best_meeting = min(best_meeting, distF[u] + distB[u]);
            }

            for (const Edge *e : graph.getIncomingEdges(u))
            { // IMPORTANT
                int v = e->u;
                double w = e->length;

                if (distB[u] + w < distB[v])
                {
                    distB[v] = distB[u] + w;
                    pqB.push({v, distB[v] + hB(v)});
                    if (visitedF[v])
                        best_meeting = min(best_meeting, distF[v] + distB[v]);
                }
            }
        }

        // Early exact stop if meeting is optimal
        if (best_meeting < INF)
            return best_meeting; // exact shortest path found
    }

    // Approx if time expired or PQ empties
    if (best_meeting < INF)
        return best_meeting * (1.0 + acceptable_error_pct);

    return -1.0; // unreachable
}

std::vector<std::pair<std::vector<int>, int>> Algorithms::k_shortest_paths_heuristic(
    const Graph &graph,
    int source,
    int target,
    int k,
    int overlap_threshold)
{
    using namespace std;

    const double INF = 1e18;

    // ----------------- Utility: reconstruct paths -----------------
    auto reconstruct = [&](int t, const vector<int> &parent)
    {
        vector<int> path;
        for (int v = t; v != -1; v = parent[v])
            path.push_back(v);
        reverse(path.begin(), path.end());
        return path;
    };

    // ----------------- A* with edge penalties -----------------
    auto runAstar = [&](const unordered_map<int, double> &penalty) -> pair<vector<int>, double>
    {
        int N = graph.size();
        vector<double> dist(N, INF);
        vector<int> parent(N, -1);

        struct State
        {
            int node;
            double f;
            bool operator<(const State &o) const { return f > o.f; }
        };
        priority_queue<State> pq;

        auto h = [&](int u)
        {
            const Node *nu = graph.getNode(u);
            const Node *nt = graph.getNode(target);
            double dx = nu->lat - nt->lat;
            double dy = nu->lon - nt->lon;
            return sqrt(dx * dx + dy * dy);
        };

        dist[source] = 0;
        pq.push({source, h(source)});

        while (!pq.empty())
        {
            auto [u, fval] = pq.top();
            pq.pop();

            if (u == target)
                return {reconstruct(target, parent), dist[u]};

            for (const Edge *e : graph.getAdjacentEdges(u))
            {
                int v = e->v;
                double w = e->length;

                // apply penalty if edge appears in previous paths
                if (penalty.count(e->id))
                    w *= (1.0 + penalty.at(e->id));

                double nd = dist[u] + w;
                if (nd < dist[v])
                {
                    dist[v] = nd;
                    parent[v] = u;
                    pq.push({v, nd + h(v)});
                }
            }
        }
        return {{}, INF};
    };

    // ---------- STEP 1: Find true shortest path ----------
    vector<pair<vector<int>, int>> result;
    unordered_map<int, double> noPenalty;

    auto [bestPath, bestDist] = runAstar(noPenalty);
    if (bestDist >= INF)
        return result; // unreachable

    result.push_back({bestPath, (int)bestDist});

    // ---------- Store edges of previous paths ----------
    auto collectEdges = [&](const vector<int> &path)
    {
        vector<pair<int, int>> edges;
        for (int i = 0; i + 1 < path.size(); i++)
            edges.emplace_back(path[i], path[i + 1]);
        return edges;
    };

    vector<vector<pair<int, int>>> prevEdges;
    prevEdges.push_back(collectEdges(bestPath));

    // ---------- STEP 2: Find next k−1 diverse paths ----------
    for (int i = 1; i < k; i++)
    {
        unordered_map<int, double> penalty;

        // assign penalty weights to edges used previously
        for (auto &pe : prevEdges)
        {
            for (auto &[u, v] : pe)
            {
                // lookup edge id
                const auto &adj = graph.getAdjacentEdges(u);
                for (const Edge *e : adj)
                {
                    if (e->v == v)
                    {
                        penalty[e->id] += 0.50; // 50% penalty
                    }
                }
            }
        }

        // run A*
        auto [path, dist] = runAstar(penalty);
        if (dist >= INF)
            break;

        // compute overlap %
        auto e_current = collectEdges(path);

        bool good = true;
        for (auto &pe : prevEdges)
        {
            int overlapCnt = 0;
            int total = e_current.size();

            for (auto &[u1, v1] : e_current)
            {
                for (auto &[u2, v2] : pe)
                {
                    if (u1 == u2 && v1 == v2)
                        overlapCnt++;
                }
            }
            double overlapPct = (100.0 * overlapCnt) / total;

            if (overlapPct > overlap_threshold)
            {
                good = false;
                break;
            }
        }

        if (!good)
            continue;

        prevEdges.push_back(e_current);
        result.push_back({path, (int)dist});
    }

    // ---------- STEP 3: Sort paths by increasing length ----------
    sort(result.begin(), result.end(),
         [&](auto &a, auto &b)
         { return a.second < b.second; });

    return result;
}
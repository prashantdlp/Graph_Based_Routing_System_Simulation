import json
import sys
from collections import defaultdict
import heapq

# ============================================================================
# PHASE 2 VALIDATOR
# ============================================================================

class Phase2Validator:
    def __init__(self, graph_file, queries_file, output_file):
        self.graph_file = graph_file
        self.queries_file = queries_file
        self.output_file = output_file
        
        with open(graph_file, 'r') as f:
            self.graph_data = json.load(f)
        with open(queries_file, 'r') as f:
            self.queries_data = json.load(f)
        with open(output_file, 'r') as f:
            self.output_data = json.load(f)
        
        self.nodes = {n['id']: n for n in self.graph_data['nodes']}
        self.edges = {e['id']: e for e in self.graph_data['edges']}
        self.build_adjacency()
        
        self.total_queries = 0
        self.passed = 0
        self.failed = 0
        self.errors = []
    
    def build_adjacency(self):
        """Build adjacency list"""
        self.adj = defaultdict(list)
        for edge_id, edge in self.edges.items():
            u, v = edge['u'], edge['v']
            self.adj[u].append({'node': v, 'edge_id': edge_id})
            if not edge['oneway']:
                self.adj[v].append({'node': u, 'edge_id': edge_id})
    
    def dijkstra_distance(self, source, target):
        """Compute shortest path by distance"""
        dist = {node: float('inf') for node in self.nodes}
        parent = {node: None for node in self.nodes}
        dist[source] = 0
        
        pq = [(0, source)]
        visited = set()
        
        while pq:
            d, u = heapq.heappop(pq)
            
            if u in visited:
                continue
            visited.add(u)
            
            if u == target:
                break
            
            for neighbor in self.adj[u]:
                v = neighbor['node']
                edge_id = neighbor['edge_id']
                edge_len = self.edges[edge_id]['length']
                
                if dist[u] + edge_len < dist[v]:
                    dist[v] = dist[u] + edge_len
                    parent[v] = u
                    heapq.heappush(pq, (dist[v], v))
        
        if dist[target] == float('inf'):
            return None, float('inf')
        
        path = []
        curr = target
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        
        return path, dist[target]
    
    def calculate_path_distance(self, path):
        """Calculate total distance of path"""
        total = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            for neighbor in self.adj[u]:
                if neighbor['node'] == v:
                    edge_id = neighbor['edge_id']
                    total += self.edges[edge_id]['length']
                    break
        return total
    
    def is_simple_path(self, path):
        """Check if path is simple (no loops)"""
        return len(path) == len(set(path))
    
    def calculate_edge_overlap(self, path1, path2):
        """Calculate percentage of overlapping edges"""
        edges1 = set()
        for i in range(len(path1) - 1):
            u, v = path1[i], path1[i+1]
            edge = tuple(sorted([u, v]))
            edges1.add(edge)
        
        edges2 = set()
        for i in range(len(path2) - 1):
            u, v = path2[i], path2[i+1]
            edge = tuple(sorted([u, v]))
            edges2.add(edge)
        
        if not edges1 and not edges2:
            return 0
        
        overlap = len(edges1 & edges2)
        max_edges = max(len(edges1), len(edges2))
        
        return (overlap / max_edges * 100) if max_edges > 0 else 0
    
    def validate_k_shortest_paths(self, query, result):
        """Validate k-shortest paths (exact)"""
        source = query['source']
        target = query['target']
        k = query['k']
        mode = query.get('mode', 'distance')
        
        paths = result.get('paths', [])
        
        if not paths:
            optimal_path, optimal_cost = self.dijkstra_distance(source, target)
            if optimal_path and optimal_cost != float('inf'):
                return False, f"Expected k={k} paths but got empty result"
            return True, "No path exists (empty result valid)"
        
        # Validate each path
        prev_cost = -1
        for idx, path_data in enumerate(paths):
            path = path_data.get('path', [])
            reported_cost = path_data.get('length', 0)
            
            # Check path validity
            if not path:
                return False, f"Path {idx} is empty"
            
            if path[0] != source or path[-1] != target:
                return False, f"Path {idx} doesn't connect source to target"
            
            # Check if simple (no loops)
            if not self.is_simple_path(path):
                return False, f"Path {idx} is not simple (contains loops)"
            
            # Calculate actual cost
            actual_cost = self.calculate_path_distance(path)
            
            tolerance = max(0.01, actual_cost * 0.01)
            if abs(actual_cost - reported_cost) > tolerance:
                return False, f"Path {idx} reported cost {reported_cost} but actual is {actual_cost:.2f}"
            
            # Paths should be sorted by cost
            if actual_cost < prev_cost - tolerance:
                return False, f"Paths not sorted by cost (path {idx} is cheaper than path {idx-1})"
            
            prev_cost = actual_cost
        
        # Check if k is respected
        if len(paths) > k:
            return False, f"Returned {len(paths)} paths but k={k}"
        
        return True, f"K-shortest paths validated ({len(paths)} paths of k={k})"
    
    def validate_k_shortest_paths_heuristic(self, query, result):
        """Validate k-shortest paths (heuristic)"""
        source = query['source']
        target = query['target']
        k = query['k']
        overlap_threshold = query.get('overlap_threshold', 50)
        
        paths = result.get('paths', [])
        
        if not paths:
            optimal_path, _ = self.dijkstra_distance(source, target)
            if optimal_path:
                return False, f"Expected k={k} paths but got empty result"
            return True, "No path exists"
        
        # First path should be shortest
        first_cost = self.calculate_path_distance(paths[0].get('path', []))
        optimal_path, optimal_cost = self.dijkstra_distance(source, target)
        
        if optimal_path:
            tolerance = max(0.01, optimal_cost * 0.01)
            if first_cost > optimal_cost + tolerance:
                return False, f"First path is not optimal (got {first_cost:.2f}, expected ~{optimal_cost:.2f})"
        
        # Validate all paths
        for idx, path_data in enumerate(paths):
            path = path_data.get('path', [])
            
            if not path or path[0] != source or path[-1] != target:
                return False, f"Path {idx} is invalid"
            
            if not self.is_simple_path(path):
                return False, f"Path {idx} is not simple (contains loops)"
        
        if len(paths) > k:
            return False, f"Returned {len(paths)} paths but k={k}"
        
        return True, f"K-shortest heuristic validated ({len(paths)} paths of k={k})"
    
    def validate_approx_shortest_path(self, query, result):
        """Validate approximate shortest paths"""
        batch_queries = query.get('queries', [])
        time_budget_ms = query.get('time_budget_ms', 10)
        acceptable_error_pct = query.get('acceptable_error_pct', 10.0)
        
        distances = result.get('distances', [])
        
        if len(distances) != len(batch_queries):
            return False, f"Expected {len(batch_queries)} results but got {len(distances)}"
        
        processing_time = result.get('processing_time', 0)
        
        # Validate each result
        correct_count = 0
        for i, (query_item, result_item) in enumerate(zip(batch_queries, distances)):
            source = query_item['source']
            target = query_item['target']
            
            # Get expected shortest path
            _, expected_distance = self.dijkstra_distance(source, target)
            
            if expected_distance == float('inf'):
                approx_dist = result_item.get('approx_shortest_distance')
                if approx_dist is None or approx_dist == float('inf'):
                    correct_count += 1
                continue
            
            approx_dist = result_item.get('approx_shortest_distance', 0)
            
            # Check error percentage
            error_pct = abs(approx_dist - expected_distance) / expected_distance * 100
            
            if error_pct <= acceptable_error_pct + 0.5:
                correct_count += 1
        
        time_status = ""
        if processing_time > time_budget_ms:
            time_status = f" (WARNING: {processing_time}ms > budget {time_budget_ms}ms)"
        
        return True, f"Approx validated ({correct_count}/{len(distances)} within {acceptable_error_pct}% error){time_status}"
    
    def validate_query(self, query, result):
        """Validate a single query"""
        query_type = query.get('type')
        query_id = query.get('id')
        
        if result.get('id') != query_id:
            return False, f"Query ID mismatch"
        
        try:
            if query_type == 'k_shortest_paths':
                return self.validate_k_shortest_paths(query, result)
            elif query_type == 'k_shortest_paths_heuristic':
                return self.validate_k_shortest_paths_heuristic(query, result)
            elif query_type == 'approx_shortest_path':
                return self.validate_approx_shortest_path(query, result)
            else:
                return True, f"Unknown query type (skipped)"
        except Exception as e:
            return False, f"Exception: {str(e)}"
    
    def validate_all(self):
        """Validate all queries"""
        events = self.queries_data.get('events', [])
        results = self.output_data.get('results', [])
        
        if len(events) != len(results):
            print(f"ERROR: Mismatch in query/result counts ({len(events)} vs {len(results)})")
            return
        
        result_map = {r.get('id'): r for r in results}
        
        print("=" * 90)
        print(f"PHASE 2 VALIDATOR")
        print("=" * 90)
        print(f"Graph file: {self.graph_file}")
        print(f"Queries file: {self.queries_file}")
        print(f"Output file: {self.output_file}")
        print(f"Total events: {len(events)}")
        print("=" * 90)
        print()
        
        for event in events:
            self.total_queries += 1
            query_id = event.get('id')
            query_type = event.get('type', 'unknown')
            
            result = result_map.get(query_id)
            if result is None:
                self.failed += 1
                status = "✗ FAIL"
                color = "\033[91m"
                message = "No matching result"
            else:
                passed, message = self.validate_query(event, result)
                
                if passed:
                    self.passed += 1
                    status = "✓ PASS"
                    color = "\033[92m"
                else:
                    self.failed += 1
                    status = "✗ FAIL"
                    color = "\033[91m"
                    self.errors.append({
                        'query_id': query_id,
                        'type': query_type,
                        'message': message
                    })
            
            reset_color = "\033[0m"
            print(f"{color}{status}{reset_color} Query {query_id:3d} ({query_type:25s}): {message}")
        
        print()
        print("=" * 90)
        print(f"VALIDATION SUMMARY")
        print("=" * 90)
        print(f"Total Queries:  {self.total_queries}")
        print(f"Passed:         {self.passed} ({100*self.passed/self.total_queries:.1f}%)")
        print(f"Failed:         {self.failed} ({100*self.failed/self.total_queries:.1f}%)")
        print("=" * 90)
        
        if self.errors:
            print()
            print("FAILED QUERIES:")
            print("-" * 90)
            for error in self.errors:
                print(f"  Query {error['query_id']} ({error['type']})")
                print(f"    → {error['message']}")
        
        return self.passed, self.failed


def main():
    if len(sys.argv) != 4:
        print("Phase 2 Validator")
        print("=" * 90)
        print("Usage: python check_p2.py <graph.json> <queries.json> <output.json>")
        print()
        print("Arguments:")
        print("  <graph.json>      - Input graph file")
        print("  <queries.json>    - Input queries file")
        print("  <output.json>     - Output file to validate")
        sys.exit(1)
    
    graph_file = sys.argv[1]
    queries_file = sys.argv[2]
    output_file = sys.argv[3]
    
    try:
        validator = Phase2Validator(graph_file, queries_file, output_file)
        passed, failed = validator.validate_all()
        sys.exit(0 if failed == 0 else 1)
        
    except FileNotFoundError as e:
        print(f"ERROR: File not found - {e}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON - {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
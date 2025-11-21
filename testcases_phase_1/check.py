import json
import sys
import math
from collections import defaultdict, deque
import heapq

class Phase1Validator:
    def __init__(self, graph_file, queries_file, output_file):
        self.graph_file = graph_file
        self.queries_file = queries_file
        self.output_file = output_file
        
        # Load files
        with open(graph_file, 'r') as f:
            self.graph_data = json.load(f)
        with open(queries_file, 'r') as f:
            self.queries_data = json.load(f)
        with open(output_file, 'r') as f:
            self.output_data = json.load(f)
        
        # Build graph structure
        self.nodes = {n['id']: n for n in self.graph_data['nodes']}
        self.edges = {e['id']: e for e in self.graph_data['edges']}
        self.build_adjacency()
        
        # Track dynamic changes
        self.removed_edges = set()
        self.modified_edges = {}
        
        # Statistics
        self.total_queries = 0
        self.passed = 0
        self.failed = 0
        self.errors = []
    
    def build_adjacency(self):
        """Build adjacency list from edges"""
        self.adj = defaultdict(list)
        for edge_id, edge in self.edges.items():
            u, v = edge['u'], edge['v']
            self.adj[u].append({'node': v, 'edge_id': edge_id})
            if not edge['oneway']:
                self.adj[v].append({'node': u, 'edge_id': edge_id})
    
    def get_edge_length(self, edge_id):
        """Get current edge length considering modifications"""
        if edge_id in self.removed_edges:
            return float('inf')
        if edge_id in self.modified_edges:
            return self.modified_edges[edge_id].get('length', self.edges[edge_id]['length'])
        return self.edges[edge_id]['length']
    
    def get_edge_time(self, edge_id, start_time=0):
        """Calculate edge traversal time considering speed profile"""
        if edge_id in self.removed_edges:
            return float('inf')
        
        edge = self.modified_edges.get(edge_id, self.edges[edge_id])
        length = edge.get('length', self.edges[edge_id]['length'])
        
        # If speed profile exists, use it
        if 'speed_profile' in edge and edge['speed_profile']:
            speed_profile = edge['speed_profile']
            time_elapsed = 0
            distance_covered = 0
            
            while distance_covered < length:
                # Get current time slot (15-minute intervals)
                slot = int((start_time + time_elapsed) / 900) % 96  # 900s = 15min
                speed = speed_profile[slot]
                
                if speed <= 0:
                    speed = 1  # Avoid division by zero
                
                # Distance we can cover in this slot
                time_remaining_in_slot = 900 - ((start_time + time_elapsed) % 900)
                distance_in_slot = speed * time_remaining_in_slot
                
                if distance_covered + distance_in_slot >= length:
                    # We finish in this slot
                    time_elapsed += (length - distance_covered) / speed
                    break
                else:
                    # Move to next slot
                    distance_covered += distance_in_slot
                    time_elapsed += time_remaining_in_slot
            
            return time_elapsed
        else:
            # Use average time
            return edge.get('average_time', self.edges[edge_id]['average_time'])
    
    def get_edge_road_type(self, edge_id):
        """Get current road type considering modifications"""
        if edge_id in self.modified_edges:
            return self.modified_edges[edge_id].get('road_type', self.edges[edge_id]['road_type'])
        return self.edges[edge_id]['road_type']
    
    def is_edge_allowed(self, edge_id, constraints):
        """Check if edge is allowed given constraints"""
        if edge_id in self.removed_edges:
            return False
        
        if not constraints:
            return True
        
        # Check road type constraints
        forbidden_types = constraints.get('forbidden_road_types', [])
        if forbidden_types and self.get_edge_road_type(edge_id) in forbidden_types:
            return False
        
        return True
    
    def is_node_allowed(self, node_id, constraints, source, target):
        """Check if node is allowed given constraints"""
        if not constraints:
            return True
        
        # Source and target are always allowed
        if node_id == source or node_id == target:
            return True
        
        forbidden_nodes = constraints.get('forbidden_nodes', [])
        return node_id not in forbidden_nodes
    
    def dijkstra_distance(self, source, target, constraints=None):
        """Compute shortest path by distance using Dijkstra"""
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
                
                # Check constraints
                if not self.is_edge_allowed(edge_id, constraints):
                    continue
                if not self.is_node_allowed(v, constraints, source, target):
                    continue
                
                edge_len = self.get_edge_length(edge_id)
                if edge_len == float('inf'):
                    continue
                
                if dist[u] + edge_len < dist[v]:
                    dist[v] = dist[u] + edge_len
                    parent[v] = u
                    heapq.heappush(pq, (dist[v], v))
        
        # Reconstruct path
        if dist[target] == float('inf'):
            return None, float('inf')
        
        path = []
        curr = target
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        
        return path, dist[target]
    
    def dijkstra_time(self, source, target, constraints=None):
        """Compute shortest path by time using Dijkstra with time-dependent weights"""
        time_to = {node: float('inf') for node in self.nodes}
        parent = {node: None for node in self.nodes}
        time_to[source] = 0
        
        pq = [(0, source)]
        visited = set()
        
        while pq:
            t, u = heapq.heappop(pq)
            
            if u in visited:
                continue
            visited.add(u)
            
            if u == target:
                break
            
            for neighbor in self.adj[u]:
                v = neighbor['node']
                edge_id = neighbor['edge_id']
                
                # Check constraints
                if not self.is_edge_allowed(edge_id, constraints):
                    continue
                if not self.is_node_allowed(v, constraints, source, target):
                    continue
                
                edge_time = self.get_edge_time(edge_id, time_to[u])
                if edge_time == float('inf'):
                    continue
                
                if time_to[u] + edge_time < time_to[v]:
                    time_to[v] = time_to[u] + edge_time
                    parent[v] = u
                    heapq.heappush(pq, (time_to[v], v))
        
        # Reconstruct path
        if time_to[target] == float('inf'):
            return None, float('inf')
        
        path = []
        curr = target
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        
        return path, time_to[target]
    
    def verify_path(self, path, source, target, constraints=None):
        """Verify if a path is valid"""
        if not path:
            return False, "Empty path"
        
        if path[0] != source:
            return False, f"Path doesn't start at source {source}"
        
        if path[-1] != target:
            return False, f"Path doesn't end at target {target}"
        
        # Check each edge in path
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            
            # Check if node is allowed
            if not self.is_node_allowed(u, constraints, source, target):
                return False, f"Node {u} is forbidden"
            
            # Find edge between u and v
            edge_found = False
            for neighbor in self.adj[u]:
                if neighbor['node'] == v:
                    edge_id = neighbor['edge_id']
                    if not self.is_edge_allowed(edge_id, constraints):
                        return False, f"Edge {edge_id} is not allowed"
                    if edge_id in self.removed_edges:
                        return False, f"Edge {edge_id} is removed"
                    edge_found = True
                    break
            
            if not edge_found:
                return False, f"No edge between {u} and {v}"
        
        return True, "Path is valid"
    
    def calculate_path_distance(self, path):
        """Calculate total distance of a path"""
        total = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            # Find edge
            for neighbor in self.adj[u]:
                if neighbor['node'] == v:
                    edge_id = neighbor['edge_id']
                    total += self.get_edge_length(edge_id)
                    break
        return total
    
    def calculate_path_time(self, path):
        """Calculate total time of a path"""
        total = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            # Find edge
            for neighbor in self.adj[u]:
                if neighbor['node'] == v:
                    edge_id = neighbor['edge_id']
                    total += self.get_edge_time(edge_id, total)
                    break
        return total
    
    def euclidean_distance(self, lat1, lon1, lat2, lon2):
        """Calculate Euclidean distance between two points"""
        lat_diff = (lat2 - lat1) * 111000
        lon_diff = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))
        return math.sqrt(lat_diff**2 + lon_diff**2)
    
    def find_nearest_node(self, lat, lon):
        """Find nearest node to given coordinates"""
        min_dist = float('inf')
        nearest = None
        for node_id, node in self.nodes.items():
            dist = self.euclidean_distance(lat, lon, node['lat'], node['lon'])
            if dist < min_dist:
                min_dist = dist
                nearest = node_id
        return nearest
    
    def validate_query(self, query, result):
        """Validate a single query result"""
        query_type = query.get('type')
        query_id = query.get('id')
        
        if result.get('id') != query_id:
            return False, f"Query ID mismatch: expected {query_id}, got {result.get('id')}"
        
        try:
            if query_type == 'remove_edge':
                return self.validate_remove_edge(query, result)
            elif query_type == 'modify_edge':
                return self.validate_modify_edge(query, result)
            elif query_type == 'shortest_path':
                return self.validate_shortest_path(query, result)
            elif query_type == 'knn':
                return self.validate_knn(query, result)
            else:
                return True, f"Unknown query type (skipped): {query_type}"
        except KeyError as e:
            return False, f"Missing required field: {str(e)}"
        except Exception as e:
            return False, f"Exception during validation: {str(e)}"
    
    def validate_remove_edge(self, query, result):
        """Validate remove_edge operation"""
        edge_id = query['edge_id']
        
        if edge_id not in self.edges:
            if result.get('done') != False:
                return False, "Should return done=false for non-existent edge"
            return True, "Remove edge validated (non-existent)"
        
        if edge_id in self.removed_edges:
            # Already removed
            if result.get('done') != False:
                return False, "Should return done=false for already removed edge"
        else:
            # Should be removed
            if result.get('done') != True:
                return False, "Should return done=true for valid removal"
            self.removed_edges.add(edge_id)
        
        return True, "Remove edge validated"
    
    def validate_modify_edge(self, query, result):
        """Validate modify_edge operation"""
        edge_id = query['edge_id']
        patch = query.get('patch', {})
        
        if edge_id not in self.edges:
            if result.get('done') != False:
                return False, "Should return done=false for non-existent edge"
            return True, "Modify edge validated (non-existent)"
        
        if edge_id in self.removed_edges:
            # Restore edge
            self.removed_edges.remove(edge_id)
            if patch:
                self.modified_edges[edge_id] = {**self.edges[edge_id], **patch}
            else:
                # Restore original
                if edge_id in self.modified_edges:
                    del self.modified_edges[edge_id]
            
            if result.get('done') != True:
                return False, "Should return done=true for edge restoration"
        else:
            # Modify existing edge
            if not patch:
                if result.get('done') != False:
                    return False, "Should return done=false for empty patch on existing edge"
                return True, "Modify edge validated (empty patch)"
            
            if edge_id not in self.modified_edges:
                self.modified_edges[edge_id] = self.edges[edge_id].copy()
            self.modified_edges[edge_id].update(patch)
            
            if result.get('done') != True:
                return False, "Should return done=true for valid modification"
        
        return True, "Modify edge validated"
    
    def validate_shortest_path(self, query, result):
        """Validate shortest_path query"""
        source = query['source']
        target = query['target']
        mode = query['mode']
        constraints = query.get('constraints')
        
        # Compute expected shortest path
        if mode == 'distance':
            expected_path, expected_cost = self.dijkstra_distance(source, target, constraints)
        else:  # mode == 'time'
            expected_path, expected_cost = self.dijkstra_time(source, target, constraints)
        
        # Check if path exists
        if expected_cost == float('inf'):
            if result.get('possible') != False:
                return False, "Should return possible=false when no path exists"
            return True, "No path exists (correctly identified)"
        
        # Path should exist
        if result.get('possible') != True:
            return False, f"Should return possible=true (path exists with cost {expected_cost:.2f})"
        
        # Validate returned path
        returned_path = result.get('path')
        if not returned_path:
            return False, "Missing 'path' field in result"
        
        # Verify path validity
        valid, msg = self.verify_path(returned_path, source, target, constraints)
        if not valid:
            return False, f"Invalid path: {msg}"
        
        # Calculate returned path cost
        if mode == 'distance':
            returned_cost = self.calculate_path_distance(returned_path)
            cost_field = 'minimum_distance'
        else:
            returned_cost = self.calculate_path_time(returned_path)
            cost_field = 'minimum_time'
        
        # Check if cost matches
        reported_cost = result.get(cost_field)
        if reported_cost is None:
            return False, f"Missing '{cost_field}' field in result"
        
        # Allow small floating point tolerance (1%)
        tolerance = max(0.01, expected_cost * 0.01)
        if abs(returned_cost - reported_cost) > tolerance:
            return False, f"Reported cost {reported_cost:.2f} doesn't match calculated cost {returned_cost:.2f}"
        
        # Check if it's optimal (within tolerance)
        if returned_cost > expected_cost + tolerance:
            return False, f"Returned path cost {returned_cost:.2f} is not optimal (expected {expected_cost:.2f})"
        
        return True, f"Shortest path validated (cost: {returned_cost:.2f})"
    
    def validate_knn(self, query, result):
        """Validate KNN query"""
        poi = query['poi']
        query_point = query['query_point']
        k = query['k']
        metric = query['metric']
        
        # Find all nodes with this POI (case-insensitive)
        poi_nodes = [node_id for node_id, node in self.nodes.items() 
                     if poi.lower() in [p.lower() for p in node.get('pois', [])]]
        
        if len(poi_nodes) == 0:
            returned_nodes = result.get('nodes', [])
            if len(returned_nodes) == 0:
                return True, f"No nodes with POI '{poi}' (empty result correct)"
            else:
                return False, f"No nodes have POI '{poi}', but result returned {len(returned_nodes)} nodes"
        
        # Calculate distances
        if metric == 'euclidean':
            distances = []
            for node_id in poi_nodes:
                node = self.nodes[node_id]
                dist = self.euclidean_distance(
                    query_point['lat'], query_point['lon'],
                    node['lat'], node['lon']
                )
                distances.append((dist, node_id))
        else:  # shortest_path
            query_node = self.find_nearest_node(query_point['lat'], query_point['lon'])
            distances = []
            for node_id in poi_nodes:
                _, dist = self.dijkstra_distance(query_node, node_id)
                if dist != float('inf'):
                    distances.append((dist, node_id))
        
        # Sort by distance and get top k
        distances.sort()
        expected_k = min(k, len(distances))
        expected_nodes = set([node_id for _, node_id in distances[:expected_k]])
        
        # Check result
        returned_nodes = result.get('nodes')
        if returned_nodes is None:
            return False, "Missing 'nodes' field in result"
        
        # Allow any order
        returned_set = set(returned_nodes)
        
        if len(returned_nodes) != len(returned_set):
            return False, "Duplicate nodes in KNN result"
        
        if returned_set != expected_nodes:
            # Check if returned nodes are at least valid POI nodes
            if not returned_set.issubset(set(poi_nodes)):
                return False, f"Some returned nodes don't have POI '{poi}'"
            
            # If counts match but nodes differ, check if distances are tied
            if len(returned_set) == len(expected_nodes):
                # Get the max distance in expected set
                max_expected_dist = max(dist for dist, nid in distances[:expected_k])
                # Check if all returned nodes are within this distance
                returned_valid = True
                for node_id in returned_set:
                    node_dist = next((dist for dist, nid in distances if nid == node_id), float('inf'))
                    if node_dist > max_expected_dist + 0.01:  # Small tolerance
                        returned_valid = False
                        break
                
                if returned_valid:
                    return True, f"KNN validated with tied distances ({len(returned_nodes)} nodes)"
            
            return False, f"KNN mismatch: expected {expected_nodes}, got {returned_set}"
        
        return True, f"KNN validated ({len(returned_nodes)} nodes)"
    
    def validate_all(self):
        """Validate all queries"""
        events = self.queries_data.get('events', [])
        results = self.output_data.get('results', [])
        
        if len(events) != len(results):
            print(f"ERROR: Number of events ({len(events)}) doesn't match results ({len(results)})")
            return
        
        # Create mapping of query_id to result for non-linear order handling
        result_map = {r.get('id'): r for r in results}
        
        print("=" * 80)
        print(f"VALIDATING PHASE 1 OUTPUT")
        print("=" * 80)
        print(f"Graph: {self.graph_file}")
        print(f"Queries: {self.queries_file}")
        print(f"Output: {self.output_file}")
        print(f"Total events: {len(events)}")
        print("=" * 80)
        print()
        
        for i, event in enumerate(events):
            self.total_queries += 1
            query_id = event.get('id', i)
            query_type = event.get('type', 'update')
            
            # Find corresponding result
            result = result_map.get(query_id)
            if result is None:
                self.failed += 1
                status = "✗ FAIL"
                color = "\033[91m"  # Red
                message = "No matching result found in output"
                self.errors.append({
                    'query_id': query_id,
                    'type': query_type,
                    'message': message
                })
            else:
                passed, message = self.validate_query(event, result)
                
                if passed:
                    self.passed += 1
                    status = "✓ PASS"
                    color = "\033[92m"  # Green
                else:
                    self.failed += 1
                    status = "✗ FAIL"
                    color = "\033[91m"  # Red
                    self.errors.append({
                        'query_id': query_id,
                        'type': query_type,
                        'message': message
                    })
            
            reset_color = "\033[0m"
            print(f"{color}{status}{reset_color} Query {query_id} ({query_type}): {message}")
        
        print()
        print("=" * 80)
        print(f"VALIDATION SUMMARY")
        print("=" * 80)
        print(f"Total Queries: {self.total_queries}")
        print(f"Passed: {self.passed} ({100*self.passed/self.total_queries:.1f}%)")
        print(f"Failed: {self.failed} ({100*self.failed/self.total_queries:.1f}%)")
        print("=" * 80)
        
        if self.errors:
            print()
            print("FAILED QUERIES:")
            print("-" * 80)
            for error in self.errors:
                print(f"  Query {error['query_id']} ({error['type']}): {error['message']}")
        
        return self.passed, self.failed

def main():
    if len(sys.argv) != 4:
        print("Usage: python validator.py <graph.json> <queries.json> <output.json>")
        sys.exit(1)
    
    graph_file = sys.argv[1]
    queries_file = sys.argv[2]
    output_file = sys.argv[3]
    
    try:
        validator = Phase1Validator(graph_file, queries_file, output_file)
        passed, failed = validator.validate_all()
        
        # Exit with non-zero code if any tests failed
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
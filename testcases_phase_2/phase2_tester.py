import json
import random
import math

# ============================================================================
# PHASE 2 TEST CASE GENERATOR
# ============================================================================

class Phase2TestGenerator:
    def __init__(self):
        self.base_lat, self.base_lon = 19.070000, 72.870000
        self.pois_list = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
        self.road_types = ["primary", "secondary", "tertiary", "local", "expressway"]
    
    def generate_graph(self, num_nodes=100, edge_density=0.25):
        """Generate connected graph with realistic properties"""
        nodes = []
        
        for i in range(num_nodes):
            lat = self.base_lat + random.uniform(-0.05, 0.05)
            lon = self.base_lon + random.uniform(-0.05, 0.05)
            num_pois = random.randint(0, 2)
            pois = random.sample(self.pois_list, num_pois) if num_pois > 0 else []
            
            nodes.append({
                "id": i,
                "lat": round(lat, 6),
                "lon": round(lon, 6),
                "pois": pois
            })
        
        # Generate edges - ensure connectivity
        edges = []
        edge_id = 1000
        
        # Spanning tree for connectivity
        connected = {0}
        unconnected = set(range(1, num_nodes))
        
        while unconnected:
            u = random.choice(list(connected))
            v = random.choice(list(unconnected))
            
            dist = self.calculate_distance(nodes[u], nodes[v])
            avg_speed = random.uniform(20, 60)
            avg_time = dist / avg_speed
            
            speed_profile = self._generate_speed_profile(avg_speed)
            
            edges.append({
                "id": edge_id,
                "u": u,
                "v": v,
                "length": round(dist, 2),
                "average_time": round(avg_time, 2),
                "speed_profile": speed_profile,
                "oneway": random.choice([True, False]),
                "road_type": random.choice(self.road_types)
            })
            
            edge_id += 1
            connected.add(v)
            unconnected.remove(v)
        
        # Add additional random edges
        max_edges = int(num_nodes * (num_nodes - 1) * edge_density / 2)
        while len(edges) < max_edges:
            u = random.randint(0, num_nodes - 1)
            v = random.randint(0, num_nodes - 1)
            
            if u == v:
                continue
            
            exists = any((e['u'] == u and e['v'] == v) or (e['u'] == v and e['v'] == u) 
                        for e in edges)
            if exists:
                continue
            
            dist = self.calculate_distance(nodes[u], nodes[v])
            avg_speed = random.uniform(20, 60)
            avg_time = dist / avg_speed
            speed_profile = self._generate_speed_profile(avg_speed)
            
            edges.append({
                "id": edge_id,
                "u": u,
                "v": v,
                "length": round(dist, 2),
                "average_time": round(avg_time, 2),
                "speed_profile": speed_profile,
                "oneway": random.choice([True, False]),
                "road_type": random.choice(self.road_types)
            })
            edge_id += 1
        
        return {
            "meta": {
                "id": "phase2_comprehensive_test",
                "nodes": num_nodes,
                "description": "Comprehensive Phase 2 test with k-shortest paths"
            },
            "nodes": nodes,
            "edges": edges
        }
    
    def _generate_speed_profile(self, avg_speed):
        """Generate time-dependent speed profile"""
        profile = []
        for slot in range(96):
            hour = (slot * 15) // 60
            if 8 <= hour <= 10 or 17 <= hour <= 19:
                speed = avg_speed * random.uniform(0.5, 0.8)
            elif 0 <= hour <= 5:
                speed = avg_speed * random.uniform(1.1, 1.3)
            else:
                speed = avg_speed * random.uniform(0.9, 1.1)
            profile.append(round(speed, 2))
        return profile
    
    def calculate_distance(self, node1, node2):
        """Calculate Euclidean distance in meters"""
        lat1, lon1 = node1['lat'], node1['lon']
        lat2, lon2 = node2['lat'], node2['lon']
        
        lat_diff = (lat2 - lat1) * 111000
        lon_diff = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))
        
        return math.sqrt(lat_diff**2 + lon_diff**2)
    
    def generate_queries(self, graph, num_each_type=8):
        """Generate comprehensive Phase 2 queries"""
        num_nodes = len(graph['nodes'])
        events = []
        query_id = 1
        
        print("Generating Phase 2 queries...")
        
        # =============== K-SHORTEST PATHS (EXACT) ===============
        print(f"  - {num_each_type} k-shortest paths (exact, k=2-5)")
        for _ in range(num_each_type):
            src, tgt = random.sample(range(num_nodes), 2)
            k = random.randint(2, 5)
            
            events.append({
                "type": "k_shortest_paths",
                "id": query_id,
                "source": src,
                "target": tgt,
                "k": k,
                "mode": "distance"
            })
            query_id += 1
        
        # =============== K-SHORTEST PATHS (HEURISTIC) ===============
        print(f"  - {num_each_type} k-shortest paths (heuristic, k=2-7)")
        for _ in range(num_each_type):
            src, tgt = random.sample(range(num_nodes), 2)
            k = random.randint(2, 7)
            overlap_threshold = random.choice([20, 30, 40, 50, 60, 70, 80])
            
            events.append({
                "type": "k_shortest_paths_heuristic",
                "id": query_id,
                "source": src,
                "target": tgt,
                "k": k,
                "overlap_threshold": overlap_threshold
            })
            query_id += 1
        
        # =============== APPROXIMATE SHORTEST PATH (BATCH) ===============
        print(f"  - {num_each_type} approximate shortest path (batch queries)")
        for _ in range(num_each_type):
            batch_size = random.randint(3, 8)
            queries = []
            for _ in range(batch_size):
                src, tgt = random.sample(range(num_nodes), 2)
                queries.append({
                    "source": src,
                    "target": tgt
                })
            
            time_budget = random.choice([5, 10, 15, 20])
            acceptable_error = random.choice([5.0, 10.0, 15.0])
            
            events.append({
                "type": "approx_shortest_path",
                "id": query_id,
                "queries": queries,
                "time_budget_ms": time_budget,
                "acceptable_error_pct": acceptable_error
            })
            query_id += 1
        
        # =============== EDGE CASES ===============
        print(f"  - Edge cases and stress tests")
        
        # Same source/target for k-shortest
        node = random.randint(0, num_nodes - 1)
        events.append({
            "type": "k_shortest_paths",
            "id": query_id,
            "source": node,
            "target": node,
            "k": 3,
            "mode": "distance"
        })
        query_id += 1
        
        # Large k value
        src, tgt = random.sample(range(num_nodes), 2)
        events.append({
            "type": "k_shortest_paths",
            "id": query_id,
            "source": src,
            "target": tgt,
            "k": 20,
            "mode": "distance"
        })
        query_id += 1
        
        # Heuristic with k=2 (minimal variety)
        src, tgt = random.sample(range(num_nodes), 2)
        events.append({
            "type": "k_shortest_paths_heuristic",
            "id": query_id,
            "source": src,
            "target": tgt,
            "k": 2,
            "overlap_threshold": 50
        })
        query_id += 1
        
        # Single query in batch approx
        src, tgt = random.sample(range(num_nodes), 2)
        events.append({
            "type": "approx_shortest_path",
            "id": query_id,
            "queries": [{"source": src, "target": tgt}],
            "time_budget_ms": 15,
            "acceptable_error_pct": 10.0
        })
        query_id += 1
        
        print(f"\nTotal queries generated: {len(events)}")
        
        return {
            "meta": {"id": "phase2_comprehensive_queries"},
            "events": events
        }


def generate_test_suite(num_nodes=100, edge_density=0.25, num_each_type=8):
    """Generate complete Phase 2 test suite"""
    print("=" * 70)
    print("PHASE 2 TEST CASE GENERATOR")
    print("=" * 70)
    
    generator = Phase2TestGenerator()
    
    print(f"\nGenerating graph with {num_nodes} nodes...")
    graph = generator.generate_graph(num_nodes=num_nodes, edge_density=edge_density)
    print(f"  - Nodes: {len(graph['nodes'])}")
    print(f"  - Edges: {len(graph['edges'])}")
    
    print()
    queries = generator.generate_queries(graph, num_each_type=num_each_type)
    
    query_types = {}
    for event in queries['events']:
        qtype = event.get('type', 'unknown')
        query_types[qtype] = query_types.get(qtype, 0) + 1
    
    print("\nQuery type distribution:")
    for qtype, count in sorted(query_types.items()):
        print(f"  {qtype}: {count}")
    
    return graph, queries


if __name__ == "__main__":
    print()
    graph, queries = generate_test_suite(num_nodes=100, edge_density=0.25, num_each_type=8)
    
    with open('graph_p2.json', 'w') as f:
        json.dump(graph, f, indent=2)
    print("\n✓ graph_p2.json saved")
    
    with open('queries_p2.json', 'w') as f:
        json.dump(queries, f, indent=2)
    print("✓ queries_p2.json saved")
    
    print("\n" + "=" * 70)
    print("TEST CASE GENERATION COMPLETE")
    print("=" * 70)
    print(f"\nFiles created:")
    print(f"  - graph_p2.json: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges")
    print(f"  - queries_p2.json: {len(queries['events'])} queries")

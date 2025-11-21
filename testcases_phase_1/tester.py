import json
import random
import math

def generate_graph(num_nodes=30, edge_density=0.35):
    """Generate a connected graph with realistic properties"""
    
    # Generate nodes with Mumbai-like coordinates
    nodes = []
    base_lat, base_lon = 19.070000, 72.870000
    pois_list = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
    
    for i in range(num_nodes):
        # Spread nodes within ~5km radius
        lat = base_lat + random.uniform(-0.05, 0.05)
        lon = base_lon + random.uniform(-0.05, 0.05)
        # Randomly assign 0-3 POIs
        num_pois = random.randint(0, 3)
        pois = random.sample(pois_list, num_pois) if num_pois > 0 else []
        
        nodes.append({
            "id": i,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "pois": pois
        })
    
    # Generate edges - ensure connectivity first
    edges = []
    edge_id = 1000
    road_types = ["primary", "secondary", "tertiary", "local", "expressway"]
    
    # Create spanning tree for connectivity
    connected = {0}
    unconnected = set(range(1, num_nodes))
    
    while unconnected:
        u = random.choice(list(connected))
        v = random.choice(list(unconnected))
        
        # Calculate distance (approximate)
        dist = calculate_distance(nodes[u], nodes[v])
        avg_speed = random.uniform(20, 60)  # m/s
        avg_time = dist / avg_speed
        
        # Generate speed profile (96 slots for 15-min intervals)
        base_speed = avg_speed
        speed_profile = []
        for slot in range(96):
            # Simulate traffic patterns (rush hours, etc.)
            hour = (slot * 15) // 60
            if 8 <= hour <= 10 or 17 <= hour <= 19:  # Rush hours
                speed = base_speed * random.uniform(0.5, 0.8)
            elif 0 <= hour <= 5:  # Night time - faster
                speed = base_speed * random.uniform(1.1, 1.3)
            else:
                speed = base_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))
        
        road_type = random.choice(road_types)
        oneway = random.choice([True, False])
        
        edges.append({
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(dist, 2),
            "average_time": round(avg_time, 2),
            "speed_profile": speed_profile,
            "oneway": oneway,
            "road_type": road_type
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
        
        # Check if edge already exists
        exists = any((e['u'] == u and e['v'] == v) or (e['u'] == v and e['v'] == u) 
                    for e in edges)
        if exists:
            continue
        
        dist = calculate_distance(nodes[u], nodes[v])
        avg_speed = random.uniform(20, 60)
        avg_time = dist / avg_speed
        
        base_speed = avg_speed
        speed_profile = []
        for slot in range(96):
            hour = (slot * 15) // 60
            if 8 <= hour <= 10 or 17 <= hour <= 19:
                speed = base_speed * random.uniform(0.5, 0.8)
            elif 0 <= hour <= 5:
                speed = base_speed * random.uniform(1.1, 1.3)
            else:
                speed = base_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))
        
        edges.append({
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(dist, 2),
            "average_time": round(avg_time, 2),
            "speed_profile": speed_profile,
            "oneway": random.choice([True, False]),
            "road_type": random.choice(road_types)
        })
        edge_id += 1
    
    return {
        "meta": {
            "id": "phase1_comprehensive_test",
            "nodes": num_nodes,
            "description": "Comprehensive test for Phase 1 with all query types"
        },
        "nodes": nodes,
        "edges": edges
    }

def calculate_distance(node1, node2):
    """Calculate approximate distance in meters using Euclidean approximation"""
    lat1, lon1 = node1['lat'], node1['lon']
    lat2, lon2 = node2['lat'], node2['lon']
    
    # Approximate: 1 degree ≈ 111km at equator
    lat_diff = (lat2 - lat1) * 111000
    lon_diff = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))
    
    return math.sqrt(lat_diff**2 + lon_diff**2)

def generate_queries(graph, num_each_type=5):
    """Generate comprehensive test queries for Phase 1"""
    
    num_nodes = len(graph['nodes'])
    edge_ids = [e['id'] for e in graph['edges']]
    road_types = ["primary", "secondary", "tertiary", "local", "expressway"]
    pois_list = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
    
    events = []
    query_id = 1
    
    print("Generating Phase 1 queries...")
    
    # ==================== SHORTEST PATH QUERIES ====================
    
    # 1. Basic shortest path - distance mode
    print(f"  - {num_each_type} basic shortest path (distance) queries")
    for _ in range(num_each_type):
        src, tgt = random.sample(range(num_nodes), 2)
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": src,
            "target": tgt,
            "mode": "distance"
        })
        query_id += 1
    
    # 2. Basic shortest path - time mode (with speed profiles)
    print(f"  - {num_each_type} basic shortest path (time) queries")
    for _ in range(num_each_type):
        src, tgt = random.sample(range(num_nodes), 2)
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": src,
            "target": tgt,
            "mode": "time"
        })
        query_id += 1
    
    # 3. Shortest path with forbidden nodes only
    print(f"  - {num_each_type} shortest path with forbidden nodes")
    for _ in range(num_each_type):
        src, tgt = random.sample(range(num_nodes), 2)
        # Forbid 2-4 random nodes (not src or tgt)
        available = [n for n in range(num_nodes) if n not in [src, tgt]]
        num_forbidden = min(random.randint(2, 4), len(available))
        forbidden = random.sample(available, num_forbidden)
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": src,
            "target": tgt,
            "mode": "distance",
            "constraints": {
                "forbidden_nodes": forbidden
            }
        })
        query_id += 1
    
    # 4. Shortest path with forbidden road types only
    print(f"  - {num_each_type} shortest path with forbidden road types")
    for _ in range(num_each_type):
        src, tgt = random.sample(range(num_nodes), 2)
        # Forbid 1-2 road types
        forbidden_types = random.sample(road_types, random.randint(1, 2))
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": src,
            "target": tgt,
            "mode": "time",
            "constraints": {
                "forbidden_road_types": forbidden_types
            }
        })
        query_id += 1
    
    # 5. Shortest path with both constraints
    print(f"  - {num_each_type} shortest path with both constraints")
    for _ in range(num_each_type):
        src, tgt = random.sample(range(num_nodes), 2)
        available = [n for n in range(num_nodes) if n not in [src, tgt]]
        num_forbidden = min(random.randint(1, 3), len(available))
        forbidden_nodes = random.sample(available, num_forbidden)
        forbidden_types = random.sample(road_types, random.randint(1, 2))
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": src,
            "target": tgt,
            "mode": random.choice(["distance", "time"]),
            "constraints": {
                "forbidden_nodes": forbidden_nodes,
                "forbidden_road_types": forbidden_types
            }
        })
        query_id += 1
    
    # 6. Edge cases for shortest path
    print(f"  - Edge cases: same source/target, no path possible")
    # Same source and target
    node = random.randint(0, num_nodes - 1)
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": node,
        "target": node,
        "mode": "distance"
    })
    query_id += 1
    
    # Path with many forbidden nodes (possibly no path)
    src, tgt = random.sample(range(num_nodes), 2)
    available = [n for n in range(num_nodes) if n not in [src, tgt]]
    forbidden_many = random.sample(available, min(len(available) // 2, 10))
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "distance",
        "constraints": {
            "forbidden_nodes": forbidden_many
        }
    })
    query_id += 1
    
    # ==================== DYNAMIC UPDATE QUERIES ====================
    
    # 7. Remove edge operations
    print(f"  - {num_each_type} remove edge operations")
    edges_to_remove = random.sample(edge_ids, min(num_each_type, len(edge_ids)))
    for edge_id in edges_to_remove:
        events.append({
            "id": query_id,
            "type": "remove_edge",
            "edge_id": edge_id
        })
        query_id += 1
    
    # 8. Query after edge removal
    print(f"  - Shortest path after edge removal")
    src, tgt = random.sample(range(num_nodes), 2)
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "distance"
    })
    query_id += 1
    
    # 9. Try to remove already removed edge
    print(f"  - Try removing already removed edge")
    if edges_to_remove:
        events.append({
            "id": query_id,
            "type": "remove_edge",
            "edge_id": edges_to_remove[0]
        })
        query_id += 1
    
    # 10. Modify edge operations
    print(f"  - {num_each_type} modify edge operations")
    available_edges = [e for e in edge_ids if e not in edges_to_remove]
    edges_to_modify = random.sample(available_edges, min(num_each_type, len(available_edges)))
    
    for edge_id in edges_to_modify:
        orig_edge = next(e for e in graph['edges'] if e['id'] == edge_id)
        # Randomly choose what to modify
        patch = {}
        if random.random() > 0.5:
            patch["length"] = round(orig_edge['length'] * random.uniform(0.7, 1.5), 2)
        if random.random() > 0.5:
            patch["road_type"] = random.choice(road_types)
        if random.random() > 0.5:
            patch["average_time"] = round(orig_edge['average_time'] * random.uniform(0.7, 1.5), 2)
        
        events.append({
            "id": query_id,
            "type": "modify_edge",
            "edge_id": edge_id,
            "patch": patch
        })
        query_id += 1
    
    # 11. Query after modification
    print(f"  - Shortest path after edge modification")
    src, tgt = random.sample(range(num_nodes), 2)
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "time"
    })
    query_id += 1
    
    # 12. Restore removed edges (empty patch)
    print(f"  - Restore removed edges with empty patch")
    for edge_id in edges_to_remove[:min(2, len(edges_to_remove))]:
        events.append({
            "id": query_id,
            "type": "modify_edge",
            "edge_id": edge_id,
            "patch": {}
        })
        query_id += 1
    
    # 13. Restore removed edges with new values
    print(f"  - Restore removed edges with new values")
    if len(edges_to_remove) > 2:
        for edge_id in edges_to_remove[2:4]:
            orig_edge = next(e for e in graph['edges'] if e['id'] == edge_id)
            events.append({
                "id": query_id,
                "type": "modify_edge",
                "edge_id": edge_id,
                "patch": {
                    "length": round(orig_edge['length'] * 1.2, 2),
                    "road_type": random.choice(road_types)
                }
            })
            query_id += 1
    
    # 14. Try to modify non-existent edge
    print(f"  - Try modifying non-existent edge")
    fake_edge_id = max(edge_ids) + 100
    events.append({
        "id": query_id,
        "type": "modify_edge",
        "edge_id": fake_edge_id,
        "patch": {"length": 100.0}
    })
    query_id += 1
    
    # 15. Try empty patch on existing edge
    print(f"  - Try empty patch on existing edge")
    events.append({
        "id": query_id,
        "type": "modify_edge",
        "edge_id": edge_ids[0],
        "patch": {}
    })
    query_id += 1
    
    # ==================== KNN QUERIES ====================
    
    # 16. KNN with Euclidean distance - using exact node coordinates
    print(f"  - {num_each_type} KNN queries (Euclidean) with exact node coords")
    for _ in range(num_each_type):
        poi = random.choice(pois_list)
        # Pick exact node coordinates (not random offset)
        node = random.choice(graph['nodes'])
        query_lat = node['lat']
        query_lon = node['lon']
        k = random.randint(2, 5)
        
        events.append({
            "type": "knn",
            "id": query_id,
            "poi": poi,
            "query_point": {
                "lat": query_lat,
                "lon": query_lon
            },
            "k": k,
            "metric": "euclidean"
        })
        query_id += 1
    
    # 17. KNN with shortest path distance - using exact node coordinates
    print(f"  - {num_each_type} KNN queries (shortest_path) with exact node coords")
    for _ in range(num_each_type):
        poi = random.choice(pois_list)
        # Pick exact node coordinates (not random offset)
        node = random.choice(graph['nodes'])
        query_lat = node['lat']
        query_lon = node['lon']
        k = random.randint(2, 4)
        
        events.append({
            "type": "knn",
            "id": query_id,
            "poi": poi,
            "query_point": {
                "lat": query_lat,
                "lon": query_lon
            },
            "k": k,
            "metric": "shortest_path"
        })
        query_id += 1
    
    # 18. KNN edge cases - using exact node coordinates
    print(f"  - KNN edge cases with exact node coords")
    
    # Large k (more than available)
    poi = random.choice(pois_list)
    node = random.choice(graph['nodes'])
    events.append({
        "type": "knn",
        "id": query_id,
        "poi": poi,
        "query_point": {
            "lat": node['lat'],
            "lon": node['lon']
        },
        "k": 100,  # Likely more than available
        "metric": "euclidean"
    })
    query_id += 1
    
    # Query point at exact node location with shortest_path metric
    node = random.choice(graph['nodes'])
    events.append({
        "type": "knn",
        "id": query_id,
        "poi": random.choice(pois_list),
        "query_point": {
            "lat": node['lat'],
            "lon": node['lon']
        },
        "k": 3,
        "metric": "shortest_path"
    })
    query_id += 1
    
    # ==================== MIXED SEQUENCE ====================
    
    # 19. Complex sequence: modify, query, remove, query, restore, query
    print(f"  - Complex sequence of operations")
    
    # Modify an edge
    edge_id = random.choice(edge_ids)
    events.append({
        "id": query_id,
        "type": "modify_edge",
        "edge_id": edge_id,
        "patch": {"length": 500.0}
    })
    query_id += 1
    
    # Query
    src, tgt = random.sample(range(num_nodes), 2)
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "distance"
    })
    query_id += 1
    
    # Remove the same edge
    events.append({
        "id": query_id,
        "type": "remove_edge",
        "edge_id": edge_id
    })
    query_id += 1
    
    # Query again
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "distance"
    })
    query_id += 1
    
    # Restore with original values
    events.append({
        "id": query_id,
        "type": "modify_edge",
        "edge_id": edge_id,
        "patch": {}
    })
    query_id += 1
    
    # Final query
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "distance"
    })
    query_id += 1
    
    # ==================== STRESS TESTS ====================
    
    # 20. Multiple constraints shortest path
    print(f"  - Stress test with heavy constraints")
    src, tgt = random.sample(range(num_nodes), 2)
    available = [n for n in range(num_nodes) if n not in [src, tgt]]
    forbidden_nodes = random.sample(available, min(5, len(available)))
    forbidden_types = random.sample(road_types, 2)
    events.append({
        "type": "shortest_path",
        "id": query_id,
        "source": src,
        "target": tgt,
        "mode": "time",
        "constraints": {
            "forbidden_nodes": forbidden_nodes,
            "forbidden_road_types": forbidden_types
        }
    })
    query_id += 1
    
    # Shuffle events to make order non-linear
    print(f"\nTotal queries generated: {len(events)}")
    print("Shuffling query order to create non-linear sequence...")
    random.shuffle(events)
    
    # Re-assign query IDs after shuffling
    for i, event in enumerate(events, start=1):
        event['id'] = i
    
    return {
        "meta": {"id": "phase1_comprehensive_queries"},
        "events": events
    }

def generate_test_suite(num_nodes=30, edge_density=0.35, num_each_type=5):
    """Generate complete test suite"""
    print("="*60)
    print("Phase 1 Test Case Generator")
    print("="*60)
    
    # Generate graph
    print(f"\nGenerating graph with {num_nodes} nodes...")
    graph = generate_graph(num_nodes=num_nodes, edge_density=edge_density)
    print(f"  - Nodes: {len(graph['nodes'])}")
    print(f"  - Edges: {len(graph['edges'])}")
    
    # Calculate POI distribution
    poi_count = {}
    for node in graph['nodes']:
        for poi in node.get('pois', []):
            poi_count[poi] = poi_count.get(poi, 0) + 1
    print(f"  - POI distribution:")
    for poi, count in sorted(poi_count.items()):
        print(f"    {poi}: {count} nodes")
    
    # Generate queries
    print()
    queries = generate_queries(graph, num_each_type=num_each_type)
    
    # Count query types
    query_types = {}
    for event in queries['events']:
        qtype = event.get('type', 'update')
        query_types[qtype] = query_types.get(qtype, 0) + 1
    
    print("\nQuery type distribution:")
    for qtype, count in sorted(query_types.items()):
        print(f"  {qtype}: {count}")
    
    return graph, queries

# Generate and save files
if __name__ == "__main__":
    print("\n" + "="*60)
    print("GENERATING PHASE 1 TEST CASES")
    print("="*60 + "\n")
    
    # Generate test suite
    graph, queries = generate_test_suite(
        num_nodes=30,
        edge_density=0.35,
        num_each_type=5
    )
    
    # Save to files
    print("\nSaving files...")
    with open('graph.json', 'w') as f:
        json.dump(graph, f, indent=2)
    print("  ✓ graph.json saved")
    
    with open('queries.json', 'w') as f:
        json.dump(queries, f, indent=2)
    print("  ✓ queries.json saved")
    
    print("\n" + "="*60)
    print("TEST CASE GENERATION COMPLETE")
    print("="*60)
    print(f"\nFiles created:")
    print(f"  - graph.json: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges")
    print(f"  - queries.json: {len(queries['events'])} queries/operations")
    print("\nRun validator with:")
    print("  python validator.py graph.json queries.json output.json")
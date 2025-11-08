import json
import random
import math

# --- Configuration ---
NUM_NODES = 100        # Total nodes in the graph
NUM_EDGES = 250        # Total edges in the graph
NUM_QUERIES = 30       # Total queries to generate (will be a mix)
LAT_RANGE = (19.0, 19.2) # Mock latitude range
LON_RANGE = (72.8, 73.0) # Mock longitude range

# Lists from project PDF
POSSIBLE_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]

# --- Helper Functions ---

def get_distance(node1, node2):
    """Calculates approximate distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius in meters
    lat1_rad = math.radians(node1['lat'])
    lat2_rad = math.radians(node2['lat'])
    lon1_rad = math.radians(node1['lon'])
    lon2_rad = math.radians(node2['lon'])

    x = (lon2_rad - lon1_rad) * math.cos((lat1_rad + lat2_rad) / 2)
    y = (lat2_rad - lat1_rad)
    distance = math.sqrt(x * x + y * y) * R
    return round(distance, 2)

def generate_speed_profile():
    """Generates a 96-slot speed profile."""
    # Speeds from 20 to 80 km/h (approx 5.5 to 22.2 m/s)
    return [round(random.uniform(5.5, 22.2), 1) for _ in range(96)]

# --- Generator Functions ---

def generate_graph(num_nodes, num_edges):
    """Generates the graph.json structure."""
    print(f"Generating graph with {num_nodes} nodes and {num_edges} edges...")
    nodes = []
    pois_in_graph = set()
    node_ids = list(range(num_nodes))

    for i in node_ids:
        num_pois = random.randint(0, 2)
        node_pois = random.sample(POSSIBLE_POIS, k=num_pois)
        pois_in_graph.update(node_pois)
        
        nodes.append({
            "id": i,
            "lat": random.uniform(*LAT_RANGE),
            "lon": random.uniform(*LON_RANGE),
            "pois": node_pois
        })

    edges = []
    edge_ids_in_graph = []
    road_types_used = set()
    existing_edges = set()
    edge_id_counter = 1000 # Start from 1000 as per spec example
    
    # Try to add random edges
    while len(edges) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)

        if u == v or (u, v) in existing_edges or (v, u) in existing_edges:
            continue

        existing_edges.add((u, v))
        length = get_distance(nodes[u], nodes[v])
        
        # Ensure length is not zero to avoid division issues
        if length < 1.0:
            length = 1.0 
            
        speed_mps = random.uniform(8.3, 16.7) # 30-60 km/h
        avg_time = length / speed_mps
        road_type = random.choice(ROAD_TYPES)
        road_types_used.add(road_type)
        
        edge_id = edge_id_counter
        edge_ids_in_graph.append(edge_id)
        edge_id_counter += 1

        edges.append({
            "id": edge_id,
            "u": u,
            "v": v,
            "length": length,
            "average_time": round(avg_time, 2),
            "speed_profile": generate_speed_profile(),
            "oneway": random.choice([True, False]),
            "road_type": road_type
        })

    graph = {
        "meta": {
            "id": "generated_graph_phase1_full",
            "nodes": num_nodes,
            "description": "Auto-generated test graph for Phase 1"
        },
        "nodes": nodes,
        "edges": edges
    }
    return graph, node_ids, edge_ids_in_graph, list(pois_in_graph), list(road_types_used)

def generate_queries(num_queries, node_ids, edge_ids, pois, road_types):
    """Generates a mixed list of Phase 1 queries."""
    print(f"Generating {num_queries} mixed queries...")
    events = []
    query_id = 200
    
    # Make copies to track available IDs
    available_edge_ids = list(edge_ids)
    removed_edge_ids = set()

    for _ in range(num_queries):
        if not available_edge_ids and not removed_edge_ids:
            print("Warning: Ran out of edges to test dynamic updates.")
            # Fallback to only shortest path and KNN
            query_type = random.choice(["shortest_path", "knn"])
        else:
            query_type = random.choice(["shortest_path", "dynamic", "knn"])

        query_id += 1
        query = {}

        if query_type == "shortest_path":
            source, target = random.sample(node_ids, 2)
            mode = random.choice(["time", "distance"])
            query = {
                "type": "shortest_path",
                "id": query_id,
                "source": source,
                "target": target,
                "mode": mode
            }
            # 30% chance of adding constraints
            if random.random() < 0.3:
                constraints = {}
                if random.random() < 0.5: # Forbidden nodes
                    constraints["forbidden_nodes"] = random.sample(node_ids, k=min(len(node_ids), 3))
                if random.random() < 0.5 and road_types: # Forbidden road types
                    constraints["forbidden_road_types"] = random.sample(road_types, k=min(len(road_types), 1))
                if constraints:
                    query["constraints"] = constraints

        elif query_type == "knn" and pois:
            query = {
                "type": "knn",
                "id": query_id,
                "poi": random.choice(pois),
                "query_point": {
                    "lat": random.uniform(*LAT_RANGE),
                    "lon": random.uniform(*LON_RANGE)
                },
                "k": random.randint(1, 5),
                "metric": random.choice(["shortest_path", "euclidean"])
            }

        elif query_type == "dynamic":
            # 50/50 chance to remove vs. modify
            if random.random() < 0.5:
                # --- REMOVE EDGE ---
                # 70% chance to remove an active edge, 30% chance to re-remove a removed one
                if random.random() < 0.7 and available_edge_ids:
                    edge_to_remove = random.choice(available_edge_ids)
                    available_edge_ids.remove(edge_to_remove)
                    removed_edge_ids.add(edge_to_remove)
                elif removed_edge_ids:
                    edge_to_remove = random.choice(list(removed_edge_ids))
                else:
                    continue # Skip if no edges to test

                query = {
                    "type": "remove_edge",
                    "id": query_id,
                    "edge_id": edge_to_remove
                }
            else:
                # --- MODIFY EDGE ---
                # 70% chance to modify an active edge, 30% chance to "restore" a removed one
                edge_to_modify = -1
                if random.random() < 0.7 and available_edge_ids:
                    edge_to_modify = random.choice(available_edge_ids)
                elif removed_edge_ids:
                    edge_to_modify = random.choice(list(removed_edge_ids))
                    # It's now "active" again per the spec
                    removed_edge_ids.remove(edge_to_modify)
                    available_edge_ids.append(edge_to_modify)
                else:
                    continue # Skip if no edges to test

                patch = {}
                # Randomly add fields to the patch
                if random.random() < 0.5:
                    patch["length"] = round(random.uniform(50.0, 1000.0), 2)
                if random.random() < 0.5:
                    patch["average_time"] = round(random.uniform(5.0, 30.0), 2)
                if random.random() < 0.2: # Less common
                    patch["speed_profile"] = generate_speed_profile()
                
                # Test empty patch (restore)
                if not patch and edge_to_modify in removed_edge_ids:
                     pass # Empty patch is fine, it means restore

                query = {
                    "type": "modify_edge",
                    "id": query_id,
                    "edge_id": edge_to_modify,
                    "patch": patch
                }

        if query: # Add if not skipped
            events.append(query)

    queries = {
        "meta": { "id": "generated_phase1_full_queries" },
        "events": events
    }
    return queries

# --- Main execution ---

def main():
    # 1. Generate Graph
    graph_data, node_ids, edge_ids, pois_list, road_types_list = generate_graph(NUM_NODES, NUM_EDGES)
    
    try:
        with open("graph.json", "w") as f:
            json.dump(graph_data, f, indent=2)
        print("✅ Successfully generated 'graph.json'")
    except IOError as e:
        print(f"❌ Error writing 'graph.json': {e}")
        return

    # 2. Generate Queries
    query_data = generate_queries(NUM_QUERIES, node_ids, edge_ids, pois_list, road_types_list)

    try:
        with open("queries.json", "w") as f:
            json.dump(query_data, f, indent=2)
        print("✅ Successfully generated 'queries.json'")
    except IOError as e:
        print(f"❌ Error writing 'queries.json': {e}")
        return
        
    print("\nReminder: This script only generates *input* files.")
    print("Run your program to produce 'output.json' and verify its correctness.")
    print(f"Example command: ./phase1 graph.json queries.json output.json")

if __name__ == "__main__":
    main()


# ### 2. Updated `Makefile` Target

# To run this new script, you should modify your `Makefile`. You can replace your old `test-phase1` target with this one:

# ```makefile
# # ... (rest of your Makefile) ...

# # Target to run Phase 1 (Comprehensive) test cases
# # Assumes 'generate_phase1_tests.py' is in the root directory
# # This target depends on the 'phase1' executable, so 'make' will build it first.
# .PHONY: test-phase1
# test-phase1: $(PHASE1_EXEC)
# 	@echo "--- Generating Phase 1 Comprehensive Test Cases ---"
# 	# Assumes you have python3
# 	python3 generate_phase1_tests.py
# 	@echo "--- Running Phase 1 Executable ---"
# 	# Runs the executable with the required arguments
# 	./$(PHASE1_EXEC) graph.json queries.json output.json
# 	@echo "--- Phase 1 Test Run Complete ---"
# 	@echo "Check 'output.json' for results."

# # ... (rest of your Makefile) ...
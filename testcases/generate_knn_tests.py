import json
import random
import math

# --- Configuration ---
NUM_NODES = 100        # Total nodes in the graph
NUM_EDGES = 250        # Total edges in the graph
NUM_QUERIES = 15       # Total KNN queries to generate
LAT_RANGE = (19.0, 19.2) # Mock latitude range (e.g., Mumbai)
LON_RANGE = (72.8, 73.0) # Mock longitude range (e.g., Mumbai)

# POI list from the project document [cite: 177]
POSSIBLE_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

# --- Helper Functions ---

def get_distance(node1, node2):
    """Calculates approximate distance in meters between two lat/lon points."""
    # Simple Euclidean approximation, valid for small areas as per spec [cite: 107]
    R = 6371000  # Earth radius in meters
    lat1_rad = math.radians(node1['lat'])
    lat2_rad = math.radians(node2['lat'])
    lon1_rad = math.radians(node1['lon'])
    lon2_rad = math.radians(node2['lon'])

    x = (lon2_rad - lon1_rad) * math.cos((lat1_rad + lat2_rad) / 2)
    y = (lat2_rad - lat1_rad)
    distance = math.sqrt(x * x + y * y) * R
    return round(distance, 2)

# --- Generator Functions ---

def generate_graph(num_nodes, num_edges):
    """Generates the graph.json structure."""
    print(f"Generating graph with {num_nodes} nodes and {num_edges} edges...")
    nodes = []
    pois_in_graph = set()

    for i in range(num_nodes):
        # Assign 0 to 2 POIs to this node
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
    existing_edges = set()
    edge_id_counter = 1000
    
    # Ensure graph is somewhat connected
    for i in range(num_nodes - 1):
        u, v = i, i + 1
        if (u, v) in existing_edges or (v, u) in existing_edges:
            continue

        existing_edges.add((u, v))
        length = get_distance(nodes[u], nodes[v])
        # Assume average speed between 30-60 km/h (8.3-16.7 m/s)
        speed_mps = random.uniform(8.3, 16.7)
        avg_time = length / speed_mps

        edges.append({
            "id": edge_id_counter,
            "u": u,
            "v": v,
            "length": length,
            "average_time": round(avg_time, 2),
            "oneway": random.choice([True, False]),
            # road_type per spec [cite: 176]
            "road_type": random.choice(["primary", "secondary", "tertiary", "local"])
        })
        edge_id_counter += 1

    # Add remaining random edges
    while len(edges) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)

        if u == v or (u, v) in existing_edges or (v, u) in existing_edges:
            continue

        existing_edges.add((u, v))
        length = get_distance(nodes[u], nodes[v])
        speed_mps = random.uniform(8.3, 16.7)
        avg_time = length / speed_mps

        edges.append({
            "id": edge_id_counter,
            "u": u,
            "v": v,
            "length": length,
            "average_time": round(avg_time, 2),
            "oneway": random.choice([True, False]),
            # road_type per spec [cite: 176]
            "road_type": random.choice(["primary", "secondary", "tertiary", "local"])
        })
        edge_id_counter += 1

    graph = {
        "meta": {
            "id": "generated_graph_knn",
            "nodes": num_nodes,
            "description": "Auto-generated test graph for KNN"
        },
        "nodes": nodes,
        "edges": edges
    }
    return graph, list(pois_in_graph)

def generate_queries(num_queries, pois_in_graph):
    """Generates the queries.json structure for KNN tests."""
    print(f"Generating {num_queries} KNN queries...")
    events = []
    
    if not pois_in_graph:
        print("Warning: No POIs were added to the graph. Using 'restaurant' as default.")
        pois_in_graph = ["restaurant"] # Add a default to avoid errors

    for i in range(num_queries):
        query_id = 200 + i
        
        # --- Create a mix of query types ---
        if i % 4 == 0:
            # Test a valid POI, Euclidean
            poi = random.choice(pois_in_graph)
            k = random.randint(1, 5)
            metric = "euclidean"
        elif i % 4 == 1:
            # Test a valid POI, Shortest Path
            poi = random.choice(pois_in_graph)
            k = random.randint(1, 5)
            metric = "shortest_path"
        elif i % 4 == 2:
            # Test a POI that doesn't exist
            poi = "library" # Not in the official list [cite: 177]
            k = 3
            metric = "euclidean"
        else:
            # Test k > number of available nodes
            poi = random.choice(pois_in_graph)
            k = NUM_NODES + 10 # k is larger than all possible nodes
            metric = "shortest_path"

        events.append({
            "type": "knn",
            "id": query_id,
            "poi": poi,
            "query_point": {
                "lat": random.uniform(*LAT_RANGE),
                "lon": random.uniform(*LON_RANGE)
            },
            "k": k,
            "metric": metric
        })

    queries = {
        "meta": { "id": "generated_knn_queries_1" },
        "events": events
    }
    return queries

# --- Main execution ---
def main():
    # 1. Generate Graph
    graph_data, pois_list = generate_graph(NUM_NODES, NUM_EDGES)
    
    # 2. Write graph.json
    try:
        with open("graph.json", "w") as f:
            json.dump(graph_data, f, indent=2)
        print(f"✅ Successfully generated 'graph.json'")
    except IOError as e:
        print(f"❌ Error writing 'graph.json': {e}")
        return

    # 3. Generate Queries
    query_data = generate_queries(NUM_QUERIES, pois_list)

    # 4. Write queries.json
    try:
        with open("queries.json", "w") as f:
            json.dump(query_data, f, indent=2)
        print(f"✅ Successfully generated 'queries.json'")
    except IOError as e:
        print(f"❌ Error writing 'queries.json': {e}")
        return

    # Reminder: This script only generates *input* files [cite: 41].
    # You must run your program to produce 'output.json' and verify its correctness [cite: 419].
    print(f"Example command: ./phase1 graph.json queries.json output.json")

if __name__ == "__main__":
    main()

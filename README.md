# Graph_Based_Routing_System_Simulation
This project is a comprehensive C++ simulation of a modern, large-scale routing and logistics system, similar to those powering applications like Google Maps, Zomato, and Ola. It applies advanced Data Structures and Algorithms (DSA) to solve real-world graph-based problems, including shortest path finding, K-Nearest Neighbors (KNN), and complex delivery scheduling (a variant of the Travelling Salesman Problem).

The project is divided into three main phases, each building upon the last to handle increasingly complex queries and optimizations.

🎯 Project Phases

Phase 1: Core Routing Algorithms

        Implementation of standard graph algorithms on a dynamic graph.

        Queries:

            Shortest Path: Minimizing distance and time (with dynamic speed profiles) and handling constraints (forbidden nodes/roads).

            KNN: Finding K-Nearest Neighbors based on both Euclidean and shortest path distances.

        Features: Supports dynamic graph updates like adding, removing, and modifying edges.

Phase 2: Advanced & Heuristic Routing

        Implementation of advanced heuristic and approximate algorithms for complex routing queries.

        Queries:

            K-Shortest Paths (Exact): Finding the top k paths by distance.

            K-Shortest Paths (Heuristic): Finding k diverse paths, balancing path length with uniqueness (penalizing overlap).

            Approximate Shortest Paths: Handling large batches of queries under a strict time budget, balancing speed vs. accuracy (scored on MSE).

Phase 3: Delivery Fleet Scheduling

        A research-oriented exploration of a TSP-variant for instant delivery services.

        Problem: Given n delivery drivers at a central depot and m orders (each with a pickup and dropoff), assign routes to minimize total delivery time.

        Focus: Experimentation with heuristics and advanced algorithms, benchmarking performance, and analyzing trade-offs.

🔧 Building and Running

Dependencies

    A C++ compiler supporting C++11 or newer (e.g., g++ or clang++)

    make

    JSON Library: A C++ JSON library is required for parsing. The nlohmann/json library is highly recommended (it's often header-only, making it easy to integrate).

    Python 3.x: Required for running the test case generation scripts.

Compilation

A Makefile is provided to build the executables for all three phases. The executable files will be created in the project's root directory.

To build all phases:
Bash

make all

To build a specific phase:
Bash

make phase1
make phase2
make phase3

Execution

Each executable accepts three command-line arguments:

    The path to the graph JSON file.

    The path to the queries JSON file.

    The path for the output JSON file.

Usage:
Bash

# To run Phase 1
./phase1 <path/to/graph.json> <path/to/phase1_queries.json> <path/to/output1.json>

# To run Phase 2
./phase2 <path/to/graph.json> <path/to/phase2_queries.json> <path/to/output2.json>

# To run Phase 3
./phase3 <path/to/graph.json> <path/to/phase3_queries.json> <path/to/output3.json>
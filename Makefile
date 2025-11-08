# --- Compiler and Flags ---
# Use g++ as the C++ compiler
CXX = g++
# Set compiler flags: C++17 standard, all warnings, extra warnings, O2 optimization, and debug symbols
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g

# Set include paths (CPPFLAGS): root dir, Phase-1, Phase-2, Phase-3
# This allows you to #include "Header.hpp" from any file
CPPFLAGS = -I. -IPhase-1 -IPhase-2 -IPhase-3

# Set linker flags: -lm for the math library (useful for distances)
LDFLAGS = -lm

# --- Source Files ---
# Automatically find all .cpp files in each Phase directory
PHASE1_SRCS = $(wildcard Phase-1/*.cpp)
PHASE2_SRCS = $(wildcard Phase-2/*.cpp)
PHASE3_SRCS = $(wildcard Phase-3/*.cpp)

# --- Object Files ---
# Automatically determine the .o (object) file names from the .cpp source files
PHASE1_OBJS = $(patsubst %.cpp, %.o, $(PHASE1_SRCS))
PHASE2_OBJS = $(patsubst %.cpp, %.o, $(PHASE2_SRCS))
PHASE3_OBJS = $(patsubst %.cpp, %.o, $(PHASE3_SRCS))

# --- Executable Names ---
# Define the final executable names as required by the project spec
PHASE1_EXEC = phase1
PHASE2_EXEC = phase2
PHASE3_EXEC = phase3

# --- Build Targets ---

# The default target when you just type 'make'
# .PHONY ensures this target runs even if a file named 'all' exists
.PHONY: all
all: $(PHASE1_EXEC) $(PHASE2_EXEC) $(PHASE3_EXEC)

# Rule to build the 'phase1' executable
# It depends on all the object files from the Phase-1 directory
$(PHASE1_EXEC): $(PHASE1_OBJS)
	@echo "Linking $@..."
	# The $^ variable automatically gets all dependencies (all .o files)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

# Rule to build the 'phase2' executable
$(PHASE2_EXEC): $(PHASE2_OBJS)
	@echo "Linking $@..."
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

# Rule to build the 'phase3' executable
$(PHASE3_EXEC): $(PHASE3_OBJS)
	@echo "Linking $@..."
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

# --- Pattern Rule for Compilation ---
# This is a generic rule that tells 'make' how to build
# any .o file from a corresponding .cpp file.
# The .o files will be created in the same directory as their .cpp files.
%.o: %.cpp
	@echo "Compiling $<..."
	# $< is the source file (.cpp), $@ is the target file (.o)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# --- Testing Targets ---

# A generic 'test' target that runs the phase 1 test by default
.PHONY: test
test: test-phase1

# Target to run Phase 1 (KNN) test cases
# Assumes 'generate_knn_tests.py' is in the root directory
# This target depends on the 'phase1' executable, so 'make' will build it first.
.PHONY: test-phase1
test-phase1: $(PHASE1_EXEC)
	@echo "--- Generating Phase 1 KNN Test Cases ---"
	# Assumes you have python3
	python3 generate_knn_tests.py
	@echo "--- Running Phase 1 Executable ---"
	# Runs the executable with the required arguments
	./$(PHASE1_EXEC) graph.json queries.json output.json
	@echo "--- Phase 1 Test Run Complete ---"
	@echo "Check 'output.json' for results."

# Placeholder for Phase 2 tests
.PHONY: test-phase2 
test-phase2: $(PHASE2_EXEC)
	@echo "--- Running Phase 2 ---"
	@echo "Reminder: Create a 'generate_phase2_tests.py' script"
	@echo "and update this target to run your Phase 2 tests."
	# Example: ./$(PHASE2_EXEC) p2_graph.json p2_queries.json p2_output.json

# Placeholder for Phase 3 tests
.PHONY: test-phase3
test-phase3: $(PHASE3_EXEC)
	@echo "--- Running Phase 3 ---"
	@echo "Reminder: Create a 'generate_phase3_tests.py' script"
	@echo "and update this target to run your Phase 3 tests."
	# Example: ./$(PHASE3_EXEC) p3_graph.json p3_queries.json p3_output.json

# --- Cleanup ---

# Target to clean up all build artifacts and generated files
.PHONY: clean
clean:
	@echo "Cleaning up build artifacts and test files..."
	# Remove the executables
	rm -f $(PHASE1_EXEC) $(PHASE2_EXEC) $(PHASE3_EXEC)
	# Remove all object files from all phase directories
	rm -f Phase-1/*.o Phase-2/*.o Phase-3/*.o
	# Remove generated test files
	rm -f graph.json queries.json output.json
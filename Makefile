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
# This builds all three phase executables
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

# --- Cleanup ---

# Target to clean up all build artifacts
.PHONY: clean
clean:
	@echo "Cleaning up build artifacts..."
	# Remove the executables
	rm -f $(PHASE1_EXEC) $(PHASE2_EXEC) $(PHASE3_EXEC)
	# Remove all object files from all phase directories
	rm -f Phase-1/*.o Phase-2/*.o Phase-3/*.o
# Makefile for Cube Analyzers
# Usage:
#   mingw32-make          - Build all targets
#   mingw32-make -j8      - Build all targets (parallel)
#   mingw32-make clean    - Remove build artifacts
#   mingw32-make <target> - Build specific target

CXX      = g++
CXXFLAGS = -std=c++17 -O3 -mavx2 -fopenmp -Wall -Wextra
LDFLAGS  = -fopenmp


# Common object files shared by all targets
COMMON_OBJS = cube_common.o move_tables.o prune_tables.o prune_create.o

# All executable targets
TARGETS = std_analyzer.exe pair_analyzer.exe pseudo_analyzer.exe \
          pseudo_pair_analyzer.exe eo_cross_analyzer.exe table_generator.exe

# --- Default target ---
all: $(TARGETS)
	@echo Build completed successfully!

# --- Pattern rules ---
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.exe: %.o $(COMMON_OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

# --- Header dependencies ---
# Base layer
COMMON_HEADERS = cube_common.h
cube_common.o: cube_common.cpp $(COMMON_HEADERS)

# Data layer
move_tables.o: move_tables.cpp move_tables.h $(COMMON_HEADERS)
prune_tables.o: prune_tables.cpp prune_tables.h move_tables.h $(COMMON_HEADERS)
prune_create.o: prune_create.cpp prune_tables.h $(COMMON_HEADERS)

# Application layer - shared headers
ANALYZER_HEADERS = analyzer_executor.h prune_stats.h $(COMMON_HEADERS) move_tables.h prune_tables.h

# Application layer - analyzers
std_analyzer.o: std_analyzer.cpp $(ANALYZER_HEADERS) cross_solver.h
pair_analyzer.o: pair_analyzer.cpp $(ANALYZER_HEADERS)
pseudo_analyzer.o: pseudo_analyzer.cpp $(ANALYZER_HEADERS) cross_solver.h
pseudo_pair_analyzer.o: pseudo_pair_analyzer.cpp $(ANALYZER_HEADERS)
eo_cross_analyzer.o: eo_cross_analyzer.cpp $(ANALYZER_HEADERS)

# Application layer - tools
table_generator.o: table_generator.cpp $(COMMON_HEADERS) move_tables.h prune_tables.h

# --- Clean ---
clean:
	powershell -NoProfile -Command "Remove-Item -Path *.o, *.exe -ErrorAction SilentlyContinue"

.PHONY: all clean

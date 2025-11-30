# dijkstra-astar

Header-only C++ templates that implement [Dijkstra's](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) and [A\*](https://en.wikipedia.org/wiki/A\*_search_algorithm) shortest path algorithms. The code is designed to be graph-agnostic: you provide lightweight callbacks that enumerate adjacent vertices, optional goal checks, and (for A\*) admissible heuristics. Both solvers then return the shortest distance together with the sequence of vertices and edges that achieve it.

## Features
- Header-only implementation: drop `dijkstra.hpp` and/or `astar.hpp` into an existing build without modifying your project layout.
- Works with arbitrary vertex and edge types. Templated hash support lets you plug in structured keys.
- Coroutine-based adjacency interface allows you to stream outgoing edges without materialising full adjacency lists.
- Customizable termination (`FinalParam`) and optional node-visitation limits for bounded searches.
- Convenience accessors expose the total path cost, the ordered vertices, and the chosen edge descriptors.

## Requirements
- A C++14 or newer compiler
- Boost components:
  - `boost::coroutine`
  - `boost::heap`
  - `boost::multi_index` (only for Dijkstra)
  - `boost::system`
- For the provided tests: `boost::unit_test_framework`

When linking binaries that use the headers, link against the relevant Boost libraries, for example:

```bash
g++ -std=c++14 my_example.cpp \
    -lboost_coroutine -lboost_heap -lboost_multi_index -lboost_system 
```

## Usage

### Provide an adjacency callback

Your graph class exposes a method that yields outgoing edges via the coroutine `yield` helper. The `Edge` template parameter can be any descriptor that helps you identify the transition (strings, enums, structs, etc.).

```cpp
#include "dijkstra.hpp"

struct Graph {
    using EdgeLabel = std::string;

    void adjacent(dijkstra<int, EdgeLabel, int>::yield& yield, const int& from) {
        for (auto [to, cost] : adjacency[from]) {
            EdgeLabel label = std::to_string(from) + "->" + std::to_string(to);
            yield({to, label, cost});
        }
    }

    std::unordered_map<int, std::vector<std::pair<int, int>>> adjacency;
};
```

### Run the search
Instantiate the solver either with `std::function` callbacks or by passing a pointer to your graph instance together with member-function pointers.

```cpp
Graph g = build_graph();
dijkstra<int, Graph::EdgeLabel, int> solver(&g, &Graph::adjacent);

if (solver.run(/*start*/ 1, /*goal*/ 5)) {
    auto cost = solver.shortest_path_len();
    auto vertices = solver.shortest_path_vertices();  // {1, 3, 6, 5}
    auto edges = solver.shortest_path_edges();        // {"1->3", "3->6", "6->5"}
}
```

### Provide an A\* heuristic

Provide a heuristic that never overestimates the true remaining cost and (optionally) a custom goal predicate:

```cpp
#include "astar.hpp"

struct PointGrid {
    struct Step { char direction; };

    void adjacent(astar<Point, Step, double, Point>::yield& yield, const Point& p);
    double heuristic(const Point& from, const Point& goal);
    bool is_goal(const Point& v, const Point& goal) { return v == goal; }
};

PointGrid grid;
astar<Point, PointGrid::Step, double, Point> finder(
    &grid,
    &PointGrid::adjacent,
    &PointGrid::heuristic,
    &PointGrid::is_goal
);

finder.run({0, 0}, target_point);
```

## API overview

- `bool run(const Vertex& start, FinalParam final_param, int max_visited_nodes = std::numeric_limits<int>::max())` – executes the search and returns `true` when a goal is reached. Use `max_visited_nodes` to protect against extremely large or infinite graphs.
- `Dist shortest_path_len() const` – returns the total distance of the last successful search.
- `std::vector<Vertex> shortest_path_vertices() const` – ordered vertices from `start` to the goal.
- `std::vector<Edge> shortest_path_edges() const` – ordered edge descriptors corresponding to the path.
- `size_t visited_node_count() const` – total number of nodes seen so far.

Both solvers expose constructors that take either `std::function` objects or a graph pointer plus member-function pointers. For most use cases you only need to implement:

1. `adjacent(yield&, const Vertex&)` – enumerates `{vertex, edge, len}` triples.
2. `heuristic(const Vertex&, FinalParam)` – A\* only; should be admissible.
3. `is_final(const Vertex&, FinalParam)` – optional; defaults to equality with `final_param`.

If your vertex type is not hashable by default, specialize `std::hash<Vertex>` (see `test/test_dijkstra.cpp` for an example).

Define `ASTAR_DEBUG_LOGGING` before including the headers to enable verbose tracing of queue operations and node relaxations.


## License

Both headers are distributed under the 2-clause BSD license. 

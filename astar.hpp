/*
 * Copyright (c) 2021 Robert Antonio <robert@antonio.cz>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>
#include <algorithm>
#include <iostream>
#include <boost/heap/fibonacci_heap.hpp>
#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/coroutine/coroutine.hpp>
#include <sstream>

#ifdef ASTAR_DEBUG_LOGGING
#include <iostream>
#define LOG(cmd) std::cout << cmd << std::endl;
#else
#define LOG(cmd) (void)0;
#endif


template <typename Vertex, typename Edge, typename Dist, typename FinalParam = const Vertex&, typename VertexHash = std::hash<Vertex>>
class astar {

protected:
	struct queue_entry {
		Vertex vertex;
		Dist priority;
	};

	class node_comparator {
	public:
		bool operator()(const queue_entry& a, const queue_entry& b) const {
			return a.priority < b.priority;
		}
	};

   	typedef typename boost::heap::fibonacci_heap<queue_entry, boost::heap::compare<node_comparator>> queue_type;
	typedef typename queue_type::handle_type handle_type;


    struct node {
		handle_type queue_handle;
		Dist dist;
		Dist fdist;
		Vertex prev_vertex;
		Edge prev_edge;
	};

	std::unordered_map<Vertex, node, VertexHash> nodes;
	queue_type queue;

public:
    typedef struct {
        Vertex vertex;
        Edge edge;
        Dist len;
    } adjacent_edge;
    
    typedef typename boost::coroutines::coroutine<adjacent_edge>::push_type yield;

protected:
	std::function<void(yield&, const Vertex&)> adjacent;
	std::function<Dist(const Vertex&, FinalParam)> heuristic;
	std::function<bool(const Vertex&, FinalParam)> is_final;
	int max_visited_nodes;

public:
    astar(
		std::function<void(yield&, const Vertex&)> adjacent,
		std::function<Dist(const Vertex&, FinalParam)> heuristic,
		std::function<bool(const Vertex&, FinalParam)> is_final
	) :
		adjacent(adjacent),
		heuristic(heuristic),
		is_final(is_final)
    {}

    template<typename Graph>
    astar(
		Graph* graph, 
		void(Graph::*adjacent_method)(yield&, const Vertex&),
		Dist(Graph::*heuristic_method)(const Vertex&, FinalParam),
		bool(Graph::*is_final_method)(const Vertex&, FinalParam)
	) :
        adjacent(std::bind(adjacent_method, graph, std::placeholders::_1, std::placeholders::_2)),
        heuristic(std::bind(heuristic_method, graph, std::placeholders::_1, std::placeholders::_2)),
        is_final(std::bind(is_final_method, graph, std::placeholders::_1, std::placeholders::_2))
    {}

    astar(
		std::function<void(yield&, const Vertex&)> adjacent,
		std::function<Dist(const Vertex&, const Vertex&)> heuristic
	) :
		adjacent(adjacent),
		heuristic(heuristic),
		is_final([](const Vertex& v, const Vertex& final_vertex) { return v == final_vertex; })
    {}

    template<typename Graph>
    astar(
		Graph* graph, 
		void(Graph::*adjacent_method)(yield&, const Vertex&),
		Dist(Graph::*heuristic_method)(const Vertex&, const Vertex&)
	) :
        adjacent(std::bind(adjacent_method, graph, std::placeholders::_1, std::placeholders::_2)),
        heuristic(std::bind(heuristic_method, graph, std::placeholders::_1, std::placeholders::_2)),
		is_final([](const Vertex& v, const Vertex& final_vertex) { return v == final_vertex; })
    {}


	bool run(
        const Vertex& start, 
		FinalParam final_param,
		int max_visited_nodes = std::numeric_limits<int>::max()
    ) {
		nodes.clear();
		queue.clear();

		Dist fdist_start = heuristic(start, final_param);
		nodes[start] = { queue.push({ start, -fdist_start }), 0, fdist_start, start };

		while (!queue.empty()) {
			Vertex from_vertex = queue.top().vertex;
			queue.pop();

			node& from = nodes[from_vertex];
			if (is_final(from_vertex, final_param)) {
				return true;
			}
			LOG("Starting from " << from_vertex << " (dist=" << from.dist << ", fdist=" << from.fdist << ", qh=" << &*from.queue_handle << ")");
	
            typename boost::coroutines::coroutine<adjacent_edge>::pull_type adjacency_source(bind(adjacent, std::placeholders::_1, from_vertex));
			for (adjacent_edge& neighbor : adjacency_source) {
				Dist new_dist = from.dist + neighbor.len;
				LOG("  try " << neighbor.vertex << " (new_dist=" << new_dist << ")");
				auto it = nodes.find(neighbor.vertex);
				if (it == nodes.end()) {
					if (nodes.size() <= max_visited_nodes) {
						Dist fdist = heuristic(neighbor.vertex, final_param);
						LOG("    new node (dist=" << new_dist<< ", fdist=" << fdist << ")");
						nodes[neighbor.vertex] = { queue.push({ neighbor.vertex, -(new_dist + fdist) }), new_dist, fdist, from_vertex, neighbor.edge };
					}
				} else {
					node& to = it->second;
					if (new_dist < to.dist) {
						LOG("    update node (dist " << to.dist << "->" << new_dist << ", fdist=" << to.fdist << ")");
						to.dist = new_dist;
						to.prev_vertex = from_vertex;
						to.prev_edge = neighbor.edge;
						(*to.queue_handle).priority = -(new_dist + to.fdist);
						queue.increase(to.queue_handle);
					}
				}
			}
		}
		return false;
	}


	Dist shortest_path_len() const {
		auto it = nodes.find(queue.top().vertex);
		return it->second.dist;
	}

	std::vector<Vertex> shortest_path_vertices() const {
		std::vector<Vertex> path;
		auto it = nodes.find(queue.top().vertex);
		while (!(it->first == it->second.prev_vertex)) {
			path.push_back(it->first);
			it = nodes.find(it->second.prev_vertex);
		}
		path.push_back(it->first);
		std::reverse(path.begin(), path.end());
		return path;
	}

	std::vector<Edge> shortest_path_edges() const {
		std::vector<Edge> path;
		auto it = nodes.find(queue.top().vertex);
		while (!(it->first == it->second.prev_vertex)) {
			path.push_back(it->second.prev_edge);
			it = nodes.find(it->second.prev_vertex);
		}
		std::reverse(path.begin(), path.end());
        return path;
	}
	
	size_t visited_node_count() const {
		return nodes.size();
	}
};

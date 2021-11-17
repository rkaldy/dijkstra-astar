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
#include <limits>
#include <functional>
#include <algorithm>
#include <iostream>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/hashed_index.hpp>
#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/coroutine/coroutine.hpp>

#ifdef ASTAR_DEBUG_LOGGING
#include <iostream>
#define LOG(cmd) std::cout << cmd << std::endl;
#else
#define LOG(cmd) (void)0;
#endif


template <typename Vertex, typename Edge, typename Dist, typename FinalParam = const Vertex&, typename VertexHash = std::hash<Vertex>>
class astar {

protected:
    struct node;

    struct node_comparator {
        bool operator()(const node* a, const node* b) const {
            return a->dist + a->fdist > b->dist + b->fdist;
        }
    };
	
   	typedef typename boost::heap::fibonacci_heap<node*, boost::heap::compare<node_comparator>> priority_queue;
    typedef typename priority_queue::handle_type queue_handle_type;
 	priority_queue queue;

    struct node {
		Vertex vertex;
		Dist dist;
		Dist fdist;
		node* prev;
		Edge prev_edge;
		queue_handle_type queue_handle{};
	};


	typedef typename boost::multi_index_container<
		node, 
		boost::multi_index::indexed_by<
			boost::multi_index::hashed_unique<
				boost::multi_index::member<node, Vertex, &node::vertex>, 
				VertexHash
			>
		>
	> nodeset;
	nodeset nodes;


	struct node_inserter {
        priority_queue& queue;
		Dist fdist;

        node_inserter(Dist fdist, priority_queue& queue) : fdist(fdist), queue(queue)
        {}

		void operator()(node& node) {
			node.fdist = fdist;
			node.queue_handle = queue.push(&node);
		}
	};

    struct node_updater {
        Dist dist;
        node* prev;
        Edge prev_edge;
        priority_queue& queue;

        node_updater(Dist dist, node* prev, Edge prev_edge, priority_queue& queue) :
            dist(dist), prev(prev), prev_edge(prev_edge), queue(queue) 
        {}

        void operator()(node& node) {
            node.dist = dist;
            node.prev = prev;
            node.prev_edge = prev_edge;
		    queue.increase(node.queue_handle, &node);
        }
    };

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

		typename nodeset::iterator it = nodes.insert({start, 0, 0, nullptr}).first;
		nodes.modify(it, node_inserter(heuristic(start, final_param), queue));

		while (!queue.empty()) {
			node* from = queue.top();
			if (is_final(from->vertex, final_param)) {
				return true;
			}
			queue.pop();
			LOG("Starting from " << from->vertex << " (dist=" << from->dist << ", fdist=" << from->fdist << ")");
	
            typename boost::coroutines::coroutine<adjacent_edge>::pull_type adjacency_source(bind(adjacent, std::placeholders::_1, from->vertex));
			for (adjacent_edge& neighbor : adjacency_source) {
				Dist new_dist = from->dist + neighbor.len;
				auto res = nodes.insert({neighbor.vertex, new_dist, 0, from, neighbor.edge});
				typename nodeset::iterator to = res.first;
				LOG("  try " << to->vertex << " (dist=" << to->dist << ", new_dist=" << new_dist << ")");
				if (res.second) {
                    if (nodes.size() <= max_visited_nodes) {
    					nodes.modify(to, node_inserter(heuristic(neighbor.vertex, final_param), queue));
						LOG("    new node (fdist=" << to->fdist << ")");
                    }
				} 
				else if (new_dist < to->dist) {
                    nodes.modify(to, node_updater(new_dist, from, neighbor.edge, queue));
					LOG("    node updated");
				}
			}
		}
		return false;
	}


	Dist shortest_path_len() const {
		return queue.top()->dist;
	}

	std::vector<Vertex> shortest_path_vertices() const {
		std::vector<Vertex> path;
		const node* n = queue.top();
		while (n->prev) {
			path.push_back(n->vertex);
			n = n->prev;
		}
		path.push_back(n->vertex);
		std::reverse(path.begin(), path.end());
		return path;
	}

	std::vector<Edge> shortest_path_edges() const {
		std::vector<Edge> path;
		const node* n = queue.top();
		while (n->prev) {
			path.push_back(n->prev_edge);
			n = n->prev;
		}
		std::reverse(path.begin(), path.end());
        return path;
	}
	
	size_t visited_node_count() const {
		return nodes.size();
	}
};

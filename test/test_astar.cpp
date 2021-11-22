#include <boost/test/unit_test.hpp>
#include <vector>
#include <map>
#include <utility>
#include <tuple>
#include <sstream>
#include <cmath>
//#define ASTAR_DEBUG_LOGGING
#include "../astar.hpp"


using namespace std;
using namespace boost::test_tools;


const vector<tuple<int, int, int>> EDGE_DEF = {{1,2,7}, {1,3,9}, {1,6,14}, {2,3,10}, {2,4,15}, {3,4,11}, {3,6,2}, {4,5,6}, {5,6,9}};

class classic_graph {
public:
	classic_graph() {
        int u, v, len;
        for (auto edge: EDGE_DEF) {
            tie(u, v, len) = edge;
            edges[u][v] = len;
            edges[v][u] = len;
        }
    }

    void adjacent(astar<int, string, int>::yield& yield, const int& from) {
       for (auto e: edges[from]) {
           stringstream ss;
           ss << from << "->" << e.first;
           yield({e.first, ss.str(), e.second});
       }
    }

    int heuristic(const int& from, const int& to) {
        return abs(to - from);
    }

private:
    map<int, map<int, int>> edges;
};


BOOST_AUTO_TEST_CASE(classic_graph_astar) {
    classic_graph g;
    astar<int, string, int> ah(&g, &classic_graph::adjacent, &classic_graph::heuristic);
    BOOST_CHECK(ah.run(1, 5));
    BOOST_CHECK_EQUAL(ah.shortest_path_len(), 20);
    
    vector<int> vertices_expected = {1, 3, 6, 5};
    vector<int> vertices = ah.shortest_path_vertices();
    BOOST_CHECK_EQUAL_COLLECTIONS(vertices.begin(), vertices.end(), vertices_expected.begin(), vertices_expected.end());

    vector<string> edges_expected = {"1->3", "3->6", "6->5"};
    vector<string> edges = ah.shortest_path_edges();
    BOOST_CHECK_EQUAL_COLLECTIONS(edges.begin(), edges.end(), edges_expected.begin(), edges_expected.end());

	bool res = ah.run(1, 7);
    BOOST_CHECK_EQUAL(res, false);
}


typedef struct {
    int x, y;
} point;

static bool operator==(const point& a, const point& b) { return a.x == b.x && a.y == b.y; }
static bool operator!=(const point& a, const point& b) { return a.x != b.x || a.y != b.y; }
static ostream& operator<<(ostream& os, const point& p) { os << '(' << p.x << ',' << p.y << ')'; return os; }

template<> struct std::hash<point> {
    size_t operator()(const point& p) const noexcept {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};


class generated_graph {
public:
    generated_graph() {}

    void adjacent(astar<point, char, double, double>::yield& yield, const point& p) {
		double len = 1.0 + pow(1.1, p.x);
        yield({{p.x, p.y+1}, 'N', len});
        yield({{p.x, p.y-1}, 'S', len});
        yield({{p.x+1, p.y}, 'E', len});
        yield({{p.x-1, p.y}, 'W', len});
    }

    double heuristic(const point& p, double final_dist) {
        double h = final_dist - sqrt(p.x*p.x + p.y*p.y);
		return h > 0 ? h : 0;
    }

	bool is_final(const point& p, double final_dist) {
		return p.x*p.x + p.y*p.y >= final_dist*final_dist;
	}
};


BOOST_AUTO_TEST_CASE(generated_graph_astar, *boost::unit_test::tolerance(0.000001)) {
	generated_graph g;
	astar<point, char, double, double> ah(&g, &generated_graph::adjacent, &generated_graph::heuristic, &generated_graph::is_final);
	BOOST_CHECK(ah.run({0, 0}, 5));
	BOOST_TEST(ah.shortest_path_len() == 9.1698654461);

	vector<point> vertices_expected = {{0,0}, {-1,0}, {-2,0}, {-3,0}, {-4,0}, {-5,0}};
	vector<point> vertices = ah.shortest_path_vertices();
    BOOST_CHECK_EQUAL_COLLECTIONS(vertices.begin(), vertices.end(), vertices_expected.begin(), vertices_expected.end());
    
	vector<char> edges_expected = {'W', 'W', 'W', 'W', 'W'};
    vector<char> edges = ah.shortest_path_edges();
    BOOST_CHECK_EQUAL_COLLECTIONS(edges.begin(), edges.end(), edges_expected.begin(), edges_expected.end());
}

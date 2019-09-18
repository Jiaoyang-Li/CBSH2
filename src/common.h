#pragma once
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>
//#include <unordered_map>

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CARDINAL, SEMI_PAIR, PAIR, MIX, STRATEGY_COUNT };

typedef std::tuple<int, int, int> Constraint;
typedef std::tuple<int, int, int, int, int> Conflict;


struct PathEntry
{
	int location;
	bool single;
	PathEntry(int loc = -1) { location = loc; single = false; }
};

bool validMove(int curr, int next, int map_size, int num_col);

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);
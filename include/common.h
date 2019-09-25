#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef std::tuple<int, int, int> Constraint;
typedef std::tuple<int, int, int, int, int> Conflict;
typedef vector< unordered_set<int64_t> > CAT; // conflict avoidance table

struct PathEntry
{
	int location;
	bool single;
	PathEntry(int loc = -1) { location = loc; single = false; }
};

bool validMove(int curr, int next, int map_size, int num_col);

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);
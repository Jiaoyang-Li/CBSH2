#pragma once

#include "common.h"
#include "compute_heuristic.h"
#include "epea_node.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using google::dense_hash_map;
using namespace std;

class EPEASearch
{

public:
	double runtime = 0;
	double solution_cost = -2;

	int min_f;
	int num_expanded = 0;
	int num_generated = 0;

	vector < vector<int> > paths;  // agents paths

	bool runEPEASearch();
	EPEASearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al);
	EPEASearch(int TIME_LIMIT, const MapLoader& ml, const vector<vector<int>>& my_heuristics,
		const vector<int>& starts, const vector<int>& goals, 
		const vector<list<Constraint>>& constraints, int lowerbound, int screen);
	~EPEASearch();

private:
	const int TIME_LIMIT;
	const int maxCost = INT_MAX;
	int start_time;
	int screen = 1;
	int initialEstimate;

	vector<list<Constraint>> initial_constraints;
	vector<int> minLengths;
	//int lastConstraintTime = -1;
	const bool* my_map;

	typedef fibonacci_heap< EPEANode*, compare<EPEANode::compare_node> > heap_open_t;
	typedef dense_hash_map<EPEANode*, EPEANode*, EPEANode::EPEANodeHasher, EPEANode::epea_eqnode> hashtable_t;
	heap_open_t open_list;
	hashtable_t allNodes_table;

	int num_of_agents;
	const int* moves_offset;
	//int num_col;
	//int num_row;
	//AgentsLoader al;

	// used in hash table
	EPEANode* empty_node;
	EPEANode* deleted_node;

	vector<vector<int>> my_heuristics;  // this is the precomputed heuristic for this agent

	list<EPEANode*> ExpandOneAgent(list<EPEANode*>& intermediateNodes, int agentIndex);
	void Expand(EPEANode& node);
	void Clear();

	void printResults() const;

};


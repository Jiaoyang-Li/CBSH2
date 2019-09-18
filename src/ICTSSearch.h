#pragma once

#include "common.h"
#include "compute_heuristic.h"
#include "MDD.h"
#include "agents_loader.h"

class ICTSSearch
{

public:
	int runtime = 0;
	int solution_cost = -2;


	// vector < vector<int> > paths;  // agents paths

	bool runICTSSearch();
	ICTSSearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al);
	ICTSSearch::ICTSSearch(int TIME_LIMIT, const MapLoader& ml, const vector<vector<int>>& my_heuristics,
		const vector<int>& starts, const vector<list<Constraint>>& constraints,
		const vector<MDD*>& initial_mdds, const vector<int>& initial_cost, int screen);
	~ICTSSearch();

private:
	const int TIME_LIMIT;
	int screen = 1;
	int num_mdds = 0;

	std::vector<int> starts;
	std::vector<std::unordered_map<int, MDD*>> MDDTable;
	std::vector<int> initial_cost;
	vector<vector < list< pair<int, int> > >> initial_constraints;

	int num_of_agents;
	const int* moves_offset;
	int num_col;
	int map_size;


	vector<vector<int>> my_heuristics;  // this is the precomputed heuristic for this agent

	void printResults() const;



};


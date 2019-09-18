#include "ICTSSearch.h"
#include <ctime>
#include <iostream>
void ICTSSearch::printResults() const
{
	if (solution_cost >= 0) // solved
		std::cout << "Optimal,";
	else // time_out
		cout << "Timeout,";

	std::cout << runtime << "," <<
		num_mdds << "," << solution_cost << ",";
	int initialEstimate = 0;
	for (int i = 0; i < num_of_agents; i++)
		initialEstimate += initial_cost[i];
	std::cout << initialEstimate << std::endl;
}
bool ICTSSearch::runICTSSearch()
{
	if (screen > 0)
		std::cout << "   ICTS: ";
	time_t start = std::clock();
	for (int sum = 0; ; sum++)
	{
		runtime = std::clock() - start;
		if (runtime > TIME_LIMIT)
		{
			if (screen > 0)
				printResults();
			return false;
		}
		std::vector<int> increased_cost(num_of_agents, 0);
		increased_cost[0] = sum;
		std::vector<MDD*> MDDs(num_of_agents);
		while (increased_cost[0] >= 0)
		{
			for (int i = 0; i < num_of_agents; i++)
			{
				int cost = initial_cost[i] + increased_cost[i];
				std::unordered_map<int, MDD*>::const_iterator got = MDDTable[i].find(cost);
				if (got == MDDTable[i].end())
				{
					MDD * mdd = new MDD();
					mdd->buildMDD(initial_constraints[i], cost + 1, starts[i], 
						moves_offset, my_heuristics[i], map_size, num_col);
					MDDTable[i][cost] = mdd;
					num_mdds++;
				}
				MDDs[i] = MDDTable[i][cost];
			}
			MDD* mdd1 = MDDs[0];
			MDD* mdd2 = MDDs[1];
			if (mdd1->levels.size() > mdd2->levels.size())
			{
				mdd1 = MDDs[1];
				mdd2 = MDDs[0];
			}
			if (SyncMDDs(*mdd1, *mdd2)) // find solutions
			{
				solution_cost = 0;
				for (int i = 0; i < num_of_agents; i++)
				{
					solution_cost += initial_cost[i] + increased_cost[i];
				}
				runtime = std::clock() - start;
				if(screen > 0)
					printResults();
				return true;
			}
			increased_cost[0]--;
			increased_cost[1]++;
		}
	}
}


ICTSSearch::ICTSSearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al):
	TIME_LIMIT(TIME_LIMIT)
{
	moves_offset = ml.get_moves_offset();
	num_of_agents = al.num_of_agents;
	map_size = ml.cols * ml.rows;
	num_col = ml.cols;

	MDDTable.resize(num_of_agents);
	initial_cost.resize(num_of_agents);
	initial_constraints.resize(num_of_agents);

	// compute heuristic lookup table
	my_heuristics.resize(num_of_agents);
	starts.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		starts[i] = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(starts[i], goal_loc, ml.get_map(), ml.rows, ml.cols, ml.get_moves_offset());
		ch.getHVals(my_heuristics[i]);
		initial_cost[i] = my_heuristics[i][starts[i]];
	}
}

ICTSSearch::ICTSSearch(int TIME_LIMIT, const MapLoader& ml, const vector<vector<int>>& my_heuristics,
	const vector<int>& starts, const vector<list<Constraint>>& constraints, 
	const vector<MDD*>& initial_mdds, const vector<int>& initial_cost, int screen):
	TIME_LIMIT(TIME_LIMIT), my_heuristics(my_heuristics), starts(starts), initial_cost(initial_cost), screen(screen)
{
	moves_offset = ml.get_moves_offset();
	num_of_agents = my_heuristics.size();
	map_size = ml.cols * ml.rows;
	num_col = ml.cols;

	MDDTable.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (initial_mdds[i] != NULL)
			MDDTable[i][initial_cost[i]] = initial_mdds[i];
	}

	int max_timestep = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		if (initial_cost[i] > max_timestep)
			max_timestep = initial_cost[i];
	}
	initial_constraints.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		initial_constraints[i].resize(max_timestep + 1);

		for (list<Constraint>::const_iterator it = constraints[i].begin(); it != constraints[i].end(); it++)
		{
			initial_constraints[i][get<2>(*it)].push_back(make_pair(get<0>(*it), get<1>(*it)));
		}
	}
	
}

ICTSSearch::~ICTSSearch()
{
	for (int i = 0; i < num_of_agents; i++)
	{
		for (auto mdd : MDDTable[i])
		{
			delete mdd.second;
		}
	}
}
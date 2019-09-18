#pragma once
#include "map_loader.h"
#include "agents_loader.h"
#include "common.h"

#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>
#include <list>

using boost::heap::fibonacci_heap;
using boost::heap::compare;

class EPEANode
{
private:
	int num_of_agents;
	int num_of_rows;
	const int * move_offset;
	const bool* my_map;

	list<pair<int, int>> constraints; // constraint table
	

public:
	int h;
	int g;

	int index;

	int num_of_cols;

	int makespan;
	vector<int> locs;
	vector<int16_t> arrival_time;

	EPEANode* parent = NULL;

	bool alreadyExpanded = false;
	int targetDeltaF; // Starts at zero, incremented after a node is expanded once. Set on Expand.
	int remainingDeltaF; // Remaining delta F towards targetDeltaF. Reset on Expand.
	int maxDeltaF = 0; // Only computed on demand

	std::shared_ptr<vector<list<pair<int16_t, int16_t>>>> singleAgentDeltaFs; // For each agent and each direction it can go, the effect of that move on F
													// INT16_MAX means this is an illegal move. Only computed on demand.
	vector<vector<int16_t>> fLookup; // Per each agent and delta F, has 1 if that delta F is achievable by moving the agents starting from this one on,
												// 2 if it isn't, and 0 if we don't know yet.
	
	EPEANode(const MapLoader &ml, const AgentsLoader &al, const vector<vector<int>>& my_heuristics); // for root node
	EPEANode(const MapLoader &ml, const vector<int>& starts, int h); // for root node
	EPEANode();

	void deep_copy(const EPEANode &cpy); // copy
	void Update(const EPEANode &cpy); // Update stats, used in duplicate detection.
	void Clear();
	void ClearConstraintTable();
	bool validMove(int curr_loc, int next_loc);
	void calcSingleAgentDeltaFs(const vector<vector<int>>& my_heuristics, const vector<list<Constraint>>& initial_constraints);
	bool existsChildForF(int agentNum, int remainingTargetDeltaF);

	void MoveTo(int agent_id, int next_loc, const vector<vector<int>>& my_heuristics);
	vector<vector<int>> GetPlan();
	~EPEANode();

	// the following is used to comapre nodes in the OPEN list
	struct compare_node {
		bool operator()(const EPEANode* n1, const EPEANode* n2) const 
		{
			if(n1->g + n1->h + n1->targetDeltaF == n2->g + n2->h + n2->targetDeltaF)
				return n1->h + n1->targetDeltaF >= n2->h + n2->targetDeltaF;
			return n1->g + n1->h + n1->targetDeltaF > n2->g + n2->h + n2->targetDeltaF;
		}
	}; 

		// The following is used by googledensehash for checking whether two nodes are equal
		// we say that two nodes, s1 and s2, are equal if
		// both are non-NULL and have the same time_expanded (unique)
	struct epea_eqnode {
		bool operator()(const EPEANode* s1, const EPEANode* s2) const {
			if (s1 == s2)
				return true;
			else if (!s1 || !s2 || s1->makespan != s2->makespan)
				return false;
			else
			{
				for (int i = 0; i < s1->locs.size(); i++)
					if (s1->locs[i] != s2->locs[i])
						return false;
				return true;
			}
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct EPEANodeHasher {
		size_t operator()(const EPEANode* n) const {
			size_t timestep_hash = hash<int>()(n->makespan);
			int sum_of_locs = 0;
			for (int i = 0; i < n->locs.size(); i++)
				sum_of_locs += n->locs[i];
			size_t loc_hash = hash<int>()(sum_of_locs);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

	typedef fibonacci_heap< EPEANode*, compare<compare_node> >::handle_type open_handle_t;
	open_handle_t open_handle;
	bool in_openlist;
};


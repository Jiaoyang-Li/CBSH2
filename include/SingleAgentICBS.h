#pragma once

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include "common.h"
#include "LLNode.h"
#include "map_loader.h"



class SingleAgentICBS
{
public:
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
	typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;

	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;

	int start_location;
	int goal_location;

	const bool* my_map;
	int map_size;
	int num_col;
	const int* moves_offset;
	std::vector<int> my_heuristic;  // this is the precomputed heuristic for this agent

	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;


	int min_f_val = 0;  // min f-val seen so far
	int num_of_conf; // number of conflicts between this agent to all the other agents

	//returns the minimal plan length for the agent (that is, extract the latest timestep which
	// has a constraint invloving this agent's goal location).
	int extractLastGoalTimestep(int goal_location, const std::vector< std::list<std::pair<int, int> > >& cons);

	//Checks if a vaild path found (wrt my_map and constraints)
	//Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

	// Updates the path datamember
	void updatePath(const LLNode* goal, std::vector<PathEntry> &path); 

	// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
	// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
	int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const CAT& cat);

	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool findPath(std::vector<PathEntry> &path, const std::vector < std::list< std::pair<int, int> > >& constraints, 
		const CAT& cat, int lowerbound);

	bool validMove(int curr, int next) const; // whetehr curr->next is a valid move

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);

	SingleAgentICBS(int start_location, int goal_location, const bool* my_map, int map_size, const int* moves_offset, int num_col);
	~SingleAgentICBS();

};


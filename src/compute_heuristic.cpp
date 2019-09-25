#include <boost/heap/fibonacci_heap.hpp>
#include "compute_heuristic.h"
#include <iostream>
#include "LLNode.h"
#include "common.h"

using std::cout;
using std::endl;


ComputeHeuristic::ComputeHeuristic(int start_location, int goal_location, const bool* my_map, int map_rows, int map_cols,
                                   const int* moves_offset) :
    my_map(my_map), map_rows(map_rows), map_cols(map_cols), moves_offset(moves_offset), 
	start_location(start_location), goal_location(goal_location){}

bool ComputeHeuristic::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_rows * map_cols)
		return false;
	int curr_x = curr / map_cols;
	int curr_y = curr % map_cols;
	int next_x = next / map_cols;
	int next_y = next % map_cols;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}

void ComputeHeuristic::getHVals(vector<int>& res)
{
	size_t root_location = goal_location;
	res.resize(map_rows * map_cols);
	for (int i = 0; i < map_rows * map_cols; i++)
		res[i] = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> nodes;
	boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	LLNode* root = new LLNode(root_location, 0, 0, NULL, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	while (!heap.empty()) {
		LLNode* curr = heap.top();
		heap.pop();

		for (int direction = 0; direction < 5; direction++)
		{
			int next_loc = curr->loc + moves_offset[direction];
			if (validMove(curr->loc, next_loc) && !my_map[next_loc])
			{  // if that grid is not blocked
				int next_g_val = curr->g_val + 1;
				LLNode* next = new LLNode(next_loc, next_g_val, 0, NULL, 0);
				it = nodes.find(next);
				if (it == nodes.end()) 
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes.insert(next);
				}
				else 
				{  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = *it;
					if (existing_next->g_val > next_g_val) 
					{
						existing_next->g_val = next_g_val;
						heap.update(existing_next->open_handle);
					}
				}
			}
		}
	}
	// iterate over all nodes and populate the distances
	for (it = nodes.begin(); it != nodes.end(); it++) 
	{
		LLNode* s = *it;
		res[s->loc] = s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
}


ComputeHeuristic::~ComputeHeuristic() {}

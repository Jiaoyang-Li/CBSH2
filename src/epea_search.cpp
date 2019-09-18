#include "epea_search.h"
#include <ctime> 
#include <iostream>


// Expands a single agent in the nodes.
// This includes:
// - Generating the children
// - Inserting them into OPEN
// - Insert node into CLOSED
// Returns the child nodes
list<EPEANode*> EPEASearch::ExpandOneAgent(list<EPEANode*>& intermediateNodes, int agent_id)
{
	list<EPEANode*> GeneratedNodes;
	for (list< EPEANode* >::iterator it = intermediateNodes.begin(); it != intermediateNodes.end(); ++it) 
	{
		EPEANode* node = *it;
		// Try all legal moves of the agents
		for (list< pair<int16_t, int16_t> >::iterator it = node->singleAgentDeltaFs->at(agent_id).begin(); 
			it != node->singleAgentDeltaFs->at(agent_id).end(); ++it)
		{
			int next_loc = node->locs[agent_id] + moves_offset[it->first];
			if (!node->validMove(node->locs[agent_id], next_loc))
				continue;

			// validate initial constraints
			bool constrained = false;
			for (Constraint c : initial_constraints[agent_id])
			{
				if (get<0>(c) < 0) // barrier constraint
				{
					int x1 = (-get<0>(c) - 1) / node->num_of_cols, y1 = (-get<0>(c) - 1) % node->num_of_cols;
					int x2 = get<1>(c) / node->num_of_cols, y2 = get<1>(c) % node->num_of_cols;
					if (x1 == x2)
					{
						if (y1 < y2)
						{
							for (int i = 0; i <= y2 - y1; i++)
							{
								if (next_loc == x1 * node->num_of_cols + y2 - i && node->makespan + 1 == get<2>(c) - i)
								{
									constrained = true;
									break;
								}
							}	
						}
						else
						{
							for (int i = 0; i <= y1 - y2; i++)
							{
								if (next_loc == x1 * node->num_of_cols + y2 + i && node->makespan + 1 == get<2>(c) - i)
								{
									constrained = true;
									break;
								}
							}
						}
					}
					else // y1== y2
					{
						if (x1 < x2)
						{
							for (int i = 0; i <= x2 - x1; i++)
							{
								if (next_loc == (x2 - i) * node->num_of_cols + y1 && node->makespan + 1 == get<2>(c) - i)
								{
									constrained = true;
									break;
								}
							}
						}
						else
						{
							for (int i = 0; i <= x1 - x2; i++)
							{
								if (next_loc == (x2 + i) * node->num_of_cols + y1 && node->makespan + 1 == get<2>(c) - i)
								{
									constrained = true;
									break;
								}
							}
						}
					}
				}
				else if (get<2>(c) == node->makespan + 1 &&
					((get<0>(c) == node->locs[agent_id] && get<1>(c) == next_loc) ||
					(get<0>(c) == next_loc && get<1>(c) < 0)))
				{
					constrained = true;
				}
				if (constrained)
					break;
			}
			if (constrained)
				continue;

			
			// Using the data that describes its delta F potential before the move.
			if (it->second <= node->remainingDeltaF  // last move was good
				&&  node->existsChildForF(agent_id + 1, node->remainingDeltaF))
			{
				EPEANode *childNode = new EPEANode();
				childNode->deep_copy(*node); // Copy all except lookUp table
				childNode->MoveTo(agent_id, next_loc, my_heuristics);
				childNode->remainingDeltaF -= it->second; // Update target F
				GeneratedNodes.push_back(childNode);
			}
			else
				break;
		}
		if(agent_id > 0)
			delete node;
	}
	intermediateNodes.clear();
	return GeneratedNodes;
}

void EPEASearch::Clear()
{
	// releaseClosedListNodes
	hashtable_t::iterator it;
	for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) 
	{
		delete ((*it).second);  // should it be .second?
	}
	allNodes_table.clear();
	delete (empty_node);
	delete (deleted_node);
}

void EPEASearch::Expand(EPEANode& node)
{
	if (!node.alreadyExpanded)
	{
		node.calcSingleAgentDeltaFs(my_heuristics, initial_constraints);
		node.alreadyExpanded = true;

		node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
		node.remainingDeltaF = 0; // Just for the following hasChildrenForCurrentDeltaF call.
		while (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF) == false) 
			// DeltaF=0 may not be possible if all agents have obstacles between their location and the goal
		{
			node.targetDeltaF++;
			node.remainingDeltaF = node.targetDeltaF;  // Just for the following hasChildrenForCurrentDeltaF call.
		}
		if (node.targetDeltaF > node.maxDeltaF) // Node has no possible children at all
		{
			node.Clear();
			return;
		}
	}

	// If this node was already expanded, notice its h was updated, so the deltaF refers to its original H
	list<EPEANode*> intermediateNodes;
	intermediateNodes.push_back(&node);

	for (int agentIndex = 0; agentIndex < num_of_agents; agentIndex++)
	{
		intermediateNodes = ExpandOneAgent(intermediateNodes, agentIndex);
	}
	list<EPEANode*> finalGeneratedNodes = intermediateNodes;
	for (list< EPEANode* >::iterator it = finalGeneratedNodes.begin(); it != finalGeneratedNodes.end(); ++it)
	{
		(*it)->makespan++;
		(*it)->ClearConstraintTable();
		(*it)->targetDeltaF = 0;
		(*it)->parent = &node;
	}

	// Enter the generated nodes into the open list
	for (list< EPEANode* >::iterator child = finalGeneratedNodes.begin(); child != finalGeneratedNodes.end(); ++child)
	{
		if ((*child)->h + (*child)->g <= this->maxCost)
			// Assuming h is an admissable heuristic, no need to generate nodes that won't get us to the goal
			// within the budget
		{
			// try to retrieve it from the hash table
			hashtable_t::iterator it = allNodes_table.find(*child);
			// If in closed list - only reopen if F is lower or node is otherwise preferred
			if (it != allNodes_table.end()) // Notice the agents may have gotten to their location from a different direction in this node.
			{
				EPEANode* existing = (*it).second;

				bool compare;  
				if((*child)->g + (*child)->h == existing->g + existing->h)
					compare = (*child)->h < existing->h;
				else
					compare = (*child)->g + (*child)->h < existing->g + existing->h;


				if (compare)// This node has smaller f, or preferred due to other consideration.
				{
					existing->Update(**child);
					if (!existing->in_openlist) // reopen 
					{
						existing->open_handle = this->open_list.push(existing);
						existing->in_openlist = true;
					}
				}
				delete (*child);
			}
			else  // add the newly generated node to open_list and hash table
			{
				allNodes_table[(*child)] = *child;
				this->num_generated++;
				(*child)->index = num_generated;
				(*child)->open_handle = open_list.push(*child);
				(*child)->in_openlist = true;
			}
		}
	}

	if (node.alreadyExpanded == false)
	{
		// Node was cleared during expansion.
		// It's unnecessary and unsafe to continue to prepare it for the next partial expansion.
		return;
	}




	node.targetDeltaF++; // This delta F was exhausted
	node.remainingDeltaF = node.targetDeltaF;
	
	while (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF) == false)
	{
		node.targetDeltaF++;
		node.remainingDeltaF = node.targetDeltaF; // Just for the following hasChildrenForCurrentDeltaF call.
	}

	if (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF)
			&& node.h + node.g + node.targetDeltaF <= this->maxCost)
	{
		// Re-insert node into open list
		node.open_handle = open_list.push(&node);
		node.in_openlist = true;
	}
	else
	{
		node.Clear();
	}
}

void EPEASearch::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Optimal,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";

	std::cout << runtime << "," <<
		num_expanded << "," << num_generated << "," <<
		solution_cost << ",";
	if(!open_list.empty())
		cout << open_list.top()->g + open_list.top()->h + open_list.top()->targetDeltaF;
	cout << "," << initialEstimate << std::endl;
}


// Runs the algorithm until the problem is solved or time is exhausted
bool EPEASearch::runEPEASearch()
{
	if(screen > 0)
		std::cout << "   EPEA: ";
	// set timer
	std::clock_t start;
	start = std::clock();

	initialEstimate = open_list.top()->h; // g = targetDeltaF = 0 initially

	int lastF = -1;

	while (!open_list.empty())
	{
		EPEANode* curr = open_list.top();
		open_list.pop();
		lastF = curr->g + curr->h + curr->targetDeltaF;


		// Check if max time has been exceeded
		runtime = std::clock() - start;
		if (runtime > TIME_LIMIT)
		{
			min_f = lastF;
			solution_cost = -1;
			if(screen > 0)
				printResults();
			this->Clear();
			return false;
		}

		// Check if node is the goal
		if ((int)round(curr->h) == 0)
		{
			int i = 0;
			for (; i < num_of_agents; i++)
			{
				if(curr->arrival_time[i] < minLengths[i])
					break;
			}
			if (i == num_of_agents)
			{
				runtime = std::clock() - start;
				this->solution_cost = curr->g;
				this->paths = curr->GetPlan();
				if (screen > 0)
					printResults();
				this->Clear();
				return true;
			}
		}

		// Expand
		Expand(*curr);
		num_expanded++;
	}
	solution_cost = -2;
	runtime = std::clock() - start;
	if (screen > 0)
		printResults();
	this->Clear();
	return false;
}

EPEASearch::EPEASearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al):
	TIME_LIMIT(TIME_LIMIT)
{
	this->moves_offset = ml.get_moves_offset();
	this->num_of_agents = al.num_of_agents;

	// initialize allNodes_table (hash table)
	empty_node = new EPEANode();
	empty_node->locs[0] = -2;
	deleted_node = new EPEANode();
	deleted_node->locs[0] = -3;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);
	
	// compute heuristic lookup table
	my_heuristics.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) 
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.get_moves_offset());
		ch.getHVals(my_heuristics[i]);
	}

	// generate root node
	EPEANode* root_node = new EPEANode(ml, al, my_heuristics);
	root_node->open_handle = this->open_list.push(root_node);
	root_node->in_openlist = true;
	allNodes_table[root_node] = root_node;
	num_generated++;
	root_node->index = num_generated;

}

EPEASearch::EPEASearch(int TIME_LIMIT, const MapLoader& ml, const vector<vector<int>>& my_heuristics, 
	const vector<int>& starts, const vector<int>& goals, 
	const vector<list<Constraint>>& constraints, int lowerbound, int screen) :
	TIME_LIMIT(TIME_LIMIT), my_heuristics(my_heuristics), my_map(my_map), 
	initial_constraints(constraints), screen(screen)
{
	this->moves_offset = ml.get_moves_offset();
	this->num_of_agents = my_heuristics.size();

	// initialize allNodes_table (hash table)
	empty_node = new EPEANode();
	empty_node->locs[0] = -2;
	deleted_node = new EPEANode();
	deleted_node->locs[0] = -3;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);

	int h = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		h+= my_heuristics[i][starts[i]];
	}
	// generate root node
	EPEANode* root_node = new EPEANode(ml, starts, h);
	root_node->open_handle = this->open_list.push(root_node);
	root_node->in_openlist = true;
	allNodes_table[root_node] = root_node;
	num_generated++;
	root_node->index = num_generated;

	// find cost constraints in intial constraints
	minLengths.resize(num_of_agents, 0);
	for (int i = 0; i < num_of_agents; i++)
	{
		for (auto constraint : constraints[i])
		{
			if (get<2>(constraint) > minLengths[i] && get<0>(constraint) < 0) // this is a path length constraint
				minLengths[i] = get<2>(constraint);
			else if (get<2>(constraint) >= minLengths[i] &&
				get<0>(constraint) == goals[i] && get<1>(constraint) < 0) // or this is a vertex constraint at the goal location
				minLengths[i] = get<2>(constraint) + 1;
		}
	}
}


EPEASearch::~EPEASearch()
{
}

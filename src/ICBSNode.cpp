#include "ICBSNode.h"
#include <iostream>

void ICBSNode::clear()
{
	cardinalConf.clear();
	semiConf.clear();
	nonConf.clear();
	unknownConf.clear();
	conflictGraph.clear();
}

void ICBSNode::printConflictGraph(int num_of_agents) const
{
	std::cout << "Conflcit graph in Node " << time_generated << " with f=" << g_val << "+" << h_val << std::endl;
	for (auto e : conflictGraph)
	{
		int i = e.first / num_of_agents;
		int j = e.first % num_of_agents;
		std::cout << "(" << i << "," << j << ")=" << e.second << std::endl;
	}
}
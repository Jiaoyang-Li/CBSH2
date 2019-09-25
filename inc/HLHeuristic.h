#pragma once
#include "common.h"

int minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_edges);

bool KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);

int greedyMatching(const std::vector<int>& CG, int cols);

int weightedVertexCover(const std::vector<int>& CG, const int N);
int weightedVertexCover(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int> range, int& best_so_far);

class HLHeuristic
{
public:
	HLHeuristic();
	~HLHeuristic();
};


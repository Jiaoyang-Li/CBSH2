#include "HLHeuristic.h"
#include <queue>
#include <iostream>

int minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_CGedges)
{
	if (num_of_CGedges < 2)
		return num_of_CGedges;
	// Compute #CG nodes that have edges
	int num_of_CGnodes = 0;
	for (int i = 0; i <  cols; i++)
	{
		for (int j = 0; j <  cols; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	if (old_mvc == -1)
	{
		for (int i = 1; i < num_of_CGnodes; i++)
			if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i, cols))
				return i;
	}
	else
	{
		if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols))
			return old_mvc - 1;
		else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols))
			return old_mvc;
		else
			return old_mvc + 1;
	}
}

// Whether there exists a k-vertex cover solution
bool KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols)
{
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	std::vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < cols - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < cols && flag; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		std::vector<int> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < cols; j++)
		{
			if (CG_copy[node[i] * cols + j] > 0)
			{
				CG_copy[node[i] * cols + j] = 0;
				CG_copy[j * cols + node[i]] = 0;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols))
			return true;
	}
	return false;
}

int greedyMatching(const std::vector<int>& CG,  int cols)
{
	int rst = 0;
	std::vector<bool> used(cols, false);
	while(1)
	{
		int maxWeight = 0;
		int ep1, ep2;
		for (int i = 0; i < cols; i++)
		{
			if(used[i])
				continue;
			for (int j = i + 1; j < cols; j++)
			{
				if (used[j])
					continue;
				else if (maxWeight < CG[i * cols + j])
				{
					maxWeight = CG[i * cols + j];
					ep1 = i;
					ep2 = j;
				}
			}
		}
		if (maxWeight == 0)
			return rst;
		rst += maxWeight;
		used[ep1] = true;
		used[ep2] = true;
	}
}

// branch and bound
// enumerate all possible assignments and return the best one
int weightedVertexCover(const std::vector<int>& CG, const int N)
{
	int rst = 0;
	std::vector<bool> done(N, false);
	for (int i = 0; i < N; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(N);
		indices.reserve(N);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < N; k++)
			{
				if (CG[j * N + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * N + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}		
				else if (CG[k * N + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * N + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num  == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * N + indices[1]], CG[indices[1] * N + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> x(num);
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * N + indices[k]], CG[indices[k] * N + indices[j]]);
			}
		}
		int best_so_far = INT_MAX;
		rst += weightedVertexCover(x, 0, 0, G, range, best_so_far);
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = weightedVertexCover(x, 0, 0, CG, range, best_so_far);
	if( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

// recusive component of weighted vertex cover
int weightedVertexCover(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int> range, int& best_so_far)
{
	if (sum >= best_so_far)
		return INT_MAX;
	else if (i == x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = weightedVertexCover(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}
	
	int cols = x.size();
	
	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = weightedVertexCover(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost; 
	}

	return best_so_far;
}


HLHeuristic::HLHeuristic()
{
}


HLHeuristic::~HLHeuristic()
{
}

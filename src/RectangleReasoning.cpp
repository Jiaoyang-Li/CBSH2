#include "RectangleReasoning.h"
#include <algorithm>    // std::find

// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2)
{
	int s1_x = S1 / num_col;
	int s1_y = S1 % num_col;
	int s2_x = S2 / num_col;
	int s2_y = S2 % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	int R1_x, R1_y, R2_x, R2_y;
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - Rg_y) >= 0)
		{
			R1_x = s1_x;
			R2_x = Rg_x;
			R1_y = Rg_y;
			R2_y = s2_y;
		}
		else
		{
			R1_x = Rg_x;
			R2_x = s2_x;
			R1_y = s1_y;
			R2_y = Rg_y;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - Rg_x) >= 0)
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
	}

	constraints1.push_back(std::make_tuple(-1 - R1_x * num_col - R1_y, Rg, Rg_t));
	constraints2.push_back(std::make_tuple(-1 - R2_x * num_col - R2_y, Rg, Rg_t));
}

// add a pair of modified barrier constraints
void addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2,
	const MDD* mdd1, const MDD* mdd2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2)
{
	int s1_x = path1[S1_t].location / num_col;
	int s1_y = path1[S1_t].location % num_col;
	int s2_x = path2[S2_t].location / num_col;
	int s2_y = path2[S2_t].location % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	int R1_x, R1_y, R2_x, R2_y;
	if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
		addModifiedHorizontalBarrierConstraint(mdd1, Rg_x, R1_y, Rg_y, Rg_t, num_col, constraints1);
		addModifiedVerticalBarrierConstraint(mdd2, Rg_y, R2_x, Rg_x, Rg_t, num_col, constraints2);
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
		addModifiedVerticalBarrierConstraint(mdd1, Rg_y, R1_x, Rg_x, Rg_t, num_col, constraints1);
		addModifiedHorizontalBarrierConstraint(mdd2, Rg_x, R2_y, Rg_y, Rg_t, num_col, constraints2);		
	}
}

// add a vertival modified barrier constraint
void addModifiedVerticalBarrierConstraint(const MDD* mdd, int y, int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<std::tuple<int, int, int>>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		MDDNode* it = NULL;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == NULL && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			int loc2 = (Ri_x + (t2 - 1 - Ri_t) * sign) * num_col + y;
			constraints.push_back(std::make_tuple(-1 - loc1, loc2, t2 - 1));
			t1 = -1;
			continue;
		}
		else if (it != NULL && t1 < 0)
		{
			t1 = t2;
		}
		if (it != NULL && t2 == Rg_t)// add constraints [t1, t2]
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			constraints.push_back(std::make_tuple(-1 - loc1, loc, t2)); 
		}
	}
}

// add a horizontal modified barrier constraint
void addModifiedHorizontalBarrierConstraint(const MDD* mdd, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<std::tuple<int, int, int>>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		MDDNode* it = NULL;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == NULL && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			int loc2 = (Ri_y + (t2 - 1 - Ri_t) * sign) + x * num_col;
			constraints.push_back(std::make_tuple(-1 - loc1, loc2, t2 - 1));
			t1 = -1;
			continue;
		}
		else if (it != NULL && t1 < 0)
		{
			t1 = t2;
		}
		if (it != NULL && t2 == Rg_t) // add constraints [t1, t2]
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			constraints.push_back(std::make_tuple(-1 - loc1, loc, t2));
		}
	}
}

//Identify rectangle conflicts for CR/R
bool isRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t)
{
	return g1_t == abs(s1.first - g1.first) + abs(s1.second - g1.second) &&  // Manhattan-optimal
		        g2_t == abs(s2.first - g2.first) + abs(s2.second - g2.second) && // Manhattan-optimal
			(s1.first - g1.first) * (s2.first - g2.first) >= 0 &&  //Move in the same direction
			(s1.second - g1.second) * (s2.second - g2.second) >= 0; //Move in the same direction
}

//Identify rectangle conflicts for RM
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col)
{
	if (s1 == s2) // A standard cardinal conflict
		return false;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return false;
	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if ((s1_x - g1_x) * (s2_x - g2_x) < 0 || (s1_y - g1_y) * (s2_y - g2_y) < 0) // Not move in the same direction
		return false;
	else if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) // s1 always in the middle
		return false;
	else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) // s2 always in the middle
		return false;
	else if ((s1_x == g1_x && s2_y == g2_y) || (s1_y == g1_y && s2_x == g2_x)) // area = 1
		return false;
	else
		return true;
}

//Classify rectangle conflicts for CR/R
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
		int cardinal1 = 0, cardinal2 = 0;
		if ((s1.first - s2.first) * (g1.first - g2.first) <= 0)
			cardinal1++;
		if ((s1.second - s2.second) * (g1.second - g2.second) <= 0)
			cardinal2++;
		return cardinal1 + cardinal2;
}

//Classify rectangle conflicts for RM
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col)
{
	int cardinal1 = 0, cardinal2 = 0;

	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg.second) >= 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg.first) < 0))
	{
		if (Rg.first == g1_x)
			cardinal1 = 1;
		if (Rg.second == g2_y)
			cardinal2 = 1;
	}
	else
	{
		if (Rg.second == g1_y)
			cardinal1 = 1;
		if (Rg.first == g2_x)
			cardinal2 = 1;
	}

	return cardinal1 + cardinal2;
}

//Compute rectangle corner Rs
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1)
{
	int x, y;
	if (s1.first == g1.first)
		x = s1.first;
	else if (s1.first < g1.first)
		x = std::max(s1.first, s2.first);
	else
		x = std::min(s1.first, s2.first);
	if (s1.second == g1.second)
		y = s1.second;
	else if (s1.second < g1.second)
		y = std::max(s1.second, s2.second);
	else
		y = std::min(s1.second, s2.second);
	return std::make_pair(x, y);
}

//Compute rectangle corner Rg
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
	int x, y;
	if (s1.first == g1.first)
		x = g1.first;
	else if (s1.first < g1.first)
		x = std::min(g1.first, g2.first);
	else
		x = std::max(g1.first, g2.first);
	if (s1.second == g1.second)
		y = g1.second;
	else if (s1.second < g1.second)
		y = std::min(g1.second, g2.second);
	else
		y = std::max(g1.second, g2.second);
	return std::make_pair(x, y);
}

//Compute start candidates for RM
std::list<int>  getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> starts;
	for (int t = 0; t <= timestep; t++) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, timestep - t, num_col))
			starts.push_back(t);
	}
	return starts;
}

//Compute goal candidates for RM
std::list<int>  getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> goals;
	for (int t = path.size() - 1; t >= timestep; t--) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, t - timestep, num_col))
			goals.push_back(t);
	}
	return goals;
}

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dist, int num_col)
{
	return abs(loc1 / num_col - loc2 / num_col) + abs(loc1 % num_col - loc2 % num_col) == dist;
}

// whther two rectangle conflicts are idenitical
bool equalRectangleConflict(const std::tuple<int, int, int, int, int>& c1, const std::tuple<int, int, int, int, int>& c2)
{
	if (std::get<2>(c1) != std::get<2>(c2)) //Not same vertex Rg
		return false;
	else if ((std::get<0>(c1) == std::get<0>(c2) && std::get<1>(c1) == std::get<1>(c2)) ||
		(std::get<0>(c1) == std::get<1>(c2) && std::get<1>(c1) == std::get<0>(c2))) // Same set of agents
		return true;
	else
		return false;
}

// find duplicate rectangle conflicts, used to detect whether a semi-/non-cardinal rectangle conflict is unique
bool findRectangleConflict(const ICBSNode* curr, const std::tuple<int, int, int, int, int>& conflict)
{
	while (curr != NULL)
	{
		if (equalRectangleConflict(conflict, *curr->conflict))
			return true;
		curr = curr->parent;
	}
	return false;
}


int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col)
{
	int s1 = paths[std::get<0>(conflict)]->at(std::get<3>(conflict)).location;
	int s2 = paths[std::get<1>(conflict)]->at(std::get<4>(conflict)).location;
	std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
	std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
	std::pair<int, int> Rg = std::make_pair(std::get<2>(conflict) / num_col, std::get<2>(conflict) % num_col);
	std::pair<int, int> Rs = getRs(S1, S2, Rg);
	return std::get<3>(conflict) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
}

#pragma once
#include <tuple>
#include <vector>
#include <list>

#include "LLNode.h"
#include "ICBSNode.h"
#include "MDD.h"
// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a pair of modified barrier constraints
void addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, 
	const MDD* mdd1, const MDD* mdd2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a vertival modified barrier constraint
void addModifiedVerticalBarrierConstraint(const MDD* mdd, int y, int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<std::tuple<int, int, int>>& constraints);

// add a horizontal modified barrier constraint
void addModifiedHorizontalBarrierConstraint(const MDD* mdd, int x, int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<std::tuple<int, int, int>>& constraints);

//Identify rectangle conflicts
bool isRectangleConflict(const std::pair<int,int>& s1, const std::pair<int, int>& s2, 
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col);// for RM

//Classify rectangle conflicts
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2);// for CR and R
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col);// for RM

//Compute rectangle corners
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2);
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1);

//Compute start and goal candidates for RM
std::list<int> getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);
std::list<int> getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dt, int num_col);

// whther two rectangle conflicts are idenitical
bool equalRectangleConflict(const std::tuple<int, int, int, int, int>& c1, const std::tuple<int, int, int, int, int>& c2);

// find duplicate rectangle conflicts, used to detect whether a semi-/non-cardinal rectangle conflict is unique
bool findRectangleConflict(const ICBSNode* curr, const std::tuple<int, int, int, int, int>& conflict); 


int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);
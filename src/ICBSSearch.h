#pragma once
#include "HLHeuristic.h"
#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"
#include <ctime>
#include "HTable.h"
#include "MDD.h"

class ICBSSearch
{
public:
	
	int runtime = 0;
	double runtime_lowlevel = 0;
	double runtime_conflictdetection = 0;
	double runtime_computeh = 0;
	double runtime_updatepaths = 0;
	double runtime_updatecons = 0;

	int max_num_of_mdds = 10000;
	double wA = -1;

	ICBSNode* dummy_start;
	ICBSNode* goal_node = NULL;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;
	uint64_t HL_num_heuristics = 0;

	uint64_t cardinal_pair_num = 0; // total number of runs of 2-agent path finding for cardinal conflicts
	uint64_t cardinal_pair_runtime = 0; // total runtime spent by 2-agent path finding for cardinal conflicts
	uint64_t not_cardinal_pair_num = 0; // total number of runs of 2-agent path finding  for not-cardinal conflicts
	uint64_t not_cardinal_pair_runtime = 0; // total runtime spent by 2-agent path finding  for not-cardinal conflicts
	uint64_t mvc_runtime = 0; // total runtime spent by solving mvc
	uint64_t cardinal_heuristic_num = 0; // number of runs that 2-agent path finding offers heuristics larger than 1  for cardinal conflicts
	uint64_t cardinal_heuristic_value = 0; // heuristics by 2-agent path finding for cardinal conflicts
	uint64_t not_cardinal_heuristic_num = 0; // number of runs that 2-agent path finding offers heuristics larger than 0  for not-cardinal conflicts
	uint64_t not_cardinal_heuristic_value = 0; // heuristics by 2-agent path finding  for not-cardinal conflicts
	uint64_t build_mdds_runtime = 0; // heuristics by 2-agent path finding  for not-cardinal conflicts

	bool solution_found = false;
	int solution_cost = -2;;
	double min_f_val;
	double focal_list_threshold;

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool runICBSSearch();

	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, 
		heuristics_type h_type, bool PC, bool rectangleReasoning,
		bool EPEA4PAIR, int CGSolver, bool booking, int time_limit,bool bookingMDD, int screen, bool log);
	ICBSSearch(const MapLoader* ml, vector<SingleAgentICBS*>& search_engines, const vector<list<Constraint>>& constraints,
		vector<vector<PathEntry>>& paths_found_initially, double f_w, int initial_h, 
		heuristics_type h_type, bool PC, bool rectangleReasoning, int cost_upperbound, int time_limit, int screen);
	void clearSearchEngines();
	~ICBSSearch();

	// Save results
	void saveResults(const std::string &fileName, const std::string &instanceName) const;
	void saveLogs(const std::string &fileName) const;

private:

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	std::vector<MDDTable> mddTable;
	int num_released_mdds = 0;
	std::vector<std::vector<HTable>> hTable;
	bool booking = false;
	int bookingHitTimes = 0;
	uint64_t bookingSearchtime = 0;

	bool PC; // prioritize conflicts or not
	bool rectangleReasoning = false; // using rectangle reasoning
	bool EPEA4PAIR; // use EPEA* to solve 2-agent path finding for PAIR
	int CGSolver; // solver for conflict graoh
	bool log = false; // log the heurictis at different depths of the CT tree
	bool bookingMDD = false; // book mdds
	int screen;
	//int nodesThreshold; // threshold of nodes that we run 2-agent searches. After the threshold, we only use cardinal conflicts for agents.
	heuristics_type h_type;
	const int time_limit;
	//const int node_limit = INT_MAX;
	double focal_w = 1.0;
	const int cost_upperbound = INT_MAX;
	
	//uint64_t nodesHaveHVals = 0;

	// Logs
	vector<int> sum_h_vals; // sum of heuristics for the CT nodes at level t
	vector<int> sum_f_vals; // sum of f values for the CT nodes at level t
	vector<int> num_CTnodes; // number of CT nodes at level t that has heuristics
	vector<int> sum_runtime; // sum of runtime for computing heuristics for the CT nodes at level t
	list<pair<int, int>> log_min_f; // changes of lowerbound in terms of expanded nodes: <lowerbound, #expanded nodes>


	vector<list<Constraint>> initial_constraints;
	const MapLoader* ml;
	std::clock_t start;

	int num_of_agents;


	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found
	vector<MDD*> mdds_initially;  // contain initial paths found
	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd


	// high level search
	bool findPathForSingleAgent(ICBSNode*  node, int ag, int lowerbound = 0);
	bool generateChild(ICBSNode* child, ICBSNode* curr);

	//conflicts
	void findConflicts(ICBSNode& curr);
	void chooseConflict(ICBSNode &parent);
	void classifyConflicts(ICBSNode &parent);
	void copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
		std::list<std::shared_ptr<Conflict>>& copy, int excluded_agent) const;
	void removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts, 
		list<pair<int, int>>& highPriorityPairs,
		const vector<vector<PathEntry>*>& paths, int num_col);

	// add heuristics for the high-level search
	int computeHeuristics(ICBSNode& curr);
	bool buildConflictGraph(ICBSNode& node);
	int getEdgeWeight(int a1, int a2, const vector<list<Constraint>> & constraints, ICBSNode& node, bool cardinal, bool& hit);

	// build MDD
	MDD * buildMDD(ICBSNode& curr, int id);
	void releaseMDDMemory(int id);

	//update information
	int collectConstraints(ICBSNode* curr, int agent_id, std::vector <std::list< std::pair<int, int> > >& cons_vec); // return the minimal length of the path
	inline void updatePaths(ICBSNode* curr);
	void updateFocalList();
	void updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();
	void copyConflictGraph(ICBSNode& child, const ICBSNode& parent);

	// print and save
	void printPaths() const;
	void printStrategy() const;
	void printResults() const;
	void printConflicts(const ICBSNode &curr) const;
	
	bool validateSolution() const;
};


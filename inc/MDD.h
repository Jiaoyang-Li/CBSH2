#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"


class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == NULL)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	int level;

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	std::list<MDDNode*> children;
	std::list<MDDNode*> parents;
	//MDDNode* parent;
};



class MDD
{
public:
	std::vector<std::list<MDDNode*>> levels;

	bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints, 
		int numOfLevels, const SingleAgentICBS & solver);
	bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
		int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);

	MDDNode* find(int location, int level) const;
	void deleteNode(MDDNode* node);
	void clear();
	bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

	MDD(){};
	MDD(const MDD & cpy);
	~MDD();
};


struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a;
	const ICBSNode* n;

	ConstraintsHasher() {};
	ConstraintsHasher(int a, const ICBSNode* n) : a(a), n(n) {};

	struct EqNode
	{
		bool operator() (const ConstraintsHasher& c1, const ConstraintsHasher& c2) const
		{
			std::set<Constraint> cons1, cons2;
			const ICBSNode* curr = c1.n;
			while (curr->parent != NULL)
			{
				if (curr->agent_id == c1.a)
					for (auto con : curr->constraints)
						cons1.insert(con);
				curr = curr->parent;
			}
			curr = c2.n;
			while (curr->parent != NULL)
			{
				if (curr->agent_id == c1.a)
					for (auto con : curr->constraints)
						cons2.insert(con);
				curr = curr->parent;
			}
			if (cons1.size() != cons2.size())
				return false;

			if (!equal(cons1.begin(), cons1.end(), cons2.begin()))
				return false;
			else
				return true;
		}
	};

	struct Hasher
	{
		std::size_t operator()(const ConstraintsHasher& entry) const
		{
			const ICBSNode* curr = entry.n;
			size_t cons_hash = 0;
			while (curr->parent != NULL)
			{
				if (curr->agent_id == entry.a)
				{
					for (auto con : curr->constraints)
					{
						cons_hash += 3 * std::hash<int>()(std::get<0>(con)) + 5 * std::hash<int>()(std::get<1>(con)) + 7 * std::hash<int>()(std::get<2>(con));
					}
				}
				curr = curr->parent;
			}
			return (cons_hash << 1);
		}
	};
};




typedef unordered_map<ConstraintsHasher, MDD*, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode> MDDTable;


class SyncMDDNode
{
public:
	SyncMDDNode(int currloc, SyncMDDNode* parent)
	{
		location = currloc;
		if (parent != NULL)
		{
			//level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	//int level;

	bool operator == (const SyncMDDNode & node) const
	{
		return (this->location == node.location);
	}


	std::list<SyncMDDNode*> children;
	std::list<SyncMDDNode*> parents;
	std::list<const MDDNode*> coexistingNodesFromOtherMdds;

};


class SyncMDD
{
public:
	std::vector<std::list<SyncMDDNode*>> levels;

	SyncMDDNode* find(int location, int level) const;
	void deleteNode(SyncMDDNode* node, int level);
	void clear();

	SyncMDD(const MDD & cpy);
	~SyncMDD();
};


// Match and prune MDD according to another MDD.
bool SyncMDDs(const MDD &mdd1, const MDD& mdd2);
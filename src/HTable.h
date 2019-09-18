#pragma once

#include <set>
#include "ICBSNode.h"


struct HTableEntry // look-up table entry 
{
	int a1;
	int a2;
	ICBSNode* n;

	HTableEntry(){};
	HTableEntry(int a1, int a2, ICBSNode* n): a1(a1), a2(a2), n(n) {};

	bool operator==(const HTableEntry& other) const
	{
		std::set<Constraint> cons1[2], cons2[2];
		const ICBSNode* curr = n;
		while (curr->parent != NULL)
		{
			if(curr->agent_id == a1)
				for (auto con : curr->constraints)
					cons1[0].insert(con);
			else if (curr->agent_id == a2)
				for (auto con : curr->constraints)
					cons2[0].insert(con);
			curr = curr->parent;
		}
		curr = other.n;
		while (curr->parent != NULL)
		{
			if (curr->agent_id == a1)
				for (auto con : curr->constraints)
					cons1[1].insert(con);
			else if (curr->agent_id == a2)
				for (auto con : curr->constraints)
					cons2[1].insert(con);
			curr = curr->parent;
		}
		if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
			return false;
		
		if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
			return false;
		if (!equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin()))
			return false;
		return true;
	}
};

template <>
struct hash<HTableEntry>
{
	std::size_t operator()(const HTableEntry& entry) const
	{
		ICBSNode* curr = entry.n;
		size_t cons1_hash = 0, cons2_hash = 0;
		while (curr->parent != NULL)
		{
			if (curr->agent_id == entry.a1)
			{
				for (auto con : curr->constraints)
				{
					cons1_hash += 3 * std::hash<int>()(std::get<0>(con)) + 5 * std::hash<int>()(std::get<1>(con)) + 7 * std::hash<int>()(std::get<2>(con));
				}
			}
			else if (curr->agent_id == entry.a2)
			{
				for (auto con : curr->constraints)
				{
					cons2_hash += 3 * std::hash<int>()(std::get<0>(con)) + 5 * std::hash<int>()(std::get<1>(con)) + 7 * std::hash<int>()(std::get<2>(con));
				}
			}
			curr = curr->parent;
		}
		return (cons1_hash << 1) ^ (cons2_hash << 1);
	}
};

typedef std::unordered_map<HTableEntry, int> HTable;
#include "LLNode.h"

LLNode::LLNode() : loc(0), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false) {}

LLNode::LLNode(int loc, int g_val, int h_val, LLNode* parent, int timestep, int num_internal_conf, bool in_openlist) :
	loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
	num_internal_conf(num_internal_conf), in_openlist(in_openlist) {}

LLNode::LLNode(const LLNode& other) 
{
	loc = other.loc;
	g_val = other.g_val;
	h_val = other.h_val;
	parent = other.parent;
	timestep = other.timestep;
	in_openlist = other.in_openlist;
	open_handle = other.open_handle;
	focal_handle = other.focal_handle;
	num_internal_conf = other.num_internal_conf;
}



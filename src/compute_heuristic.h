#pragma once

#include <vector>
#include <utility>
#include <stdlib.h>

using namespace std;

class ComputeHeuristic 
{
 public:
  int start_location;
  int goal_location;

  const bool* my_map;
  int map_rows;
  int map_cols;
  const int* moves_offset;
  
  ComputeHeuristic(int start_location, int goal_location, const bool* my_map, int map_rows, int map_cols, const int* moves_offset);
 
 bool validMove(int curr, int next) const;

  void getHVals(vector<int>& res);

  ~ComputeHeuristic();

};


// Load's a 2D map.
#pragma once

#include <string>
#include <vector>

class MapLoader 
{
 public:
 
  int rows;
  int cols;

  int start_loc;
  int goal_loc;

  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
 

  MapLoader(){}
  MapLoader(std::string fname, int rows, int cols, int obstacles);
  
  inline bool is_blocked (int row, int col) const { return my_map[row * this->cols + col]; }
  inline bool is_blocked (int loc) const { return my_map[loc]; }
  inline size_t map_size() const { return rows * cols; }
  void printMap ();
  const bool* get_map () const; 
  const int* get_moves_offset() const;
  inline int linearize_coordinate(int row, int col) const { return ( this->cols * row + col); }
  inline int row_coordinate(int id) const { return id / this->cols; }
  inline int col_coordinate(int id) const { return id % this->cols; }  

  ~MapLoader();

  private:
	  int* moves_offset;
	  bool* my_map;

	  void generateConnectedRandomGrid(int rows, int cols, int obstacles); // initialize new [rows x cols] map with random obstacles
	  bool addObstacle(int obstacle); // add this obsatcle only if the map is still connected
	  bool isConnected(int start, int goal); // run BFS to find a path between start and goal, return true if a path exists.
	  void saveToFile(std::string fname);
};


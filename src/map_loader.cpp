#include "map_loader.h"
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <queue>
#include <vector>

using namespace boost;
using namespace std;

bool MapLoader::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = obstacle / cols;
	int obstacle_y = obstacle % cols;
	int x[4] = {obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= rows || y[start] < 0 || y[start] >= cols || my_map[x[start] * cols + y[start]])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= rows || y[goal] < 0 || y[goal] >= cols || my_map[x[goal] * cols + y[goal]])
			goal++;
		else if (isConnected(x[start] * cols + y[start], x[goal] * cols + y[goal])) // cannot find a path from start to goal 
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

 bool MapLoader::isConnected(int start, int goal)
{
	std::queue<int> open;
	std::vector<bool> closed(cols * rows, false);
	open.push(start);
	closed[start] = true;
	while (!open.empty())
	{
		int curr = open.front(); open.pop();
		if (curr == goal)
			return true;
		for (int i = 0; i < 4; i++)
		{
			int next = curr + moves_offset[i];
			if (next < 0 || next >= cols *rows || my_map[next] || closed[next])
				continue;
			int curr_x = curr / cols;
			int curr_y = curr % cols;
			int next_x = next / cols;
			int next_y = next % cols;
			if (abs(next_x - curr_x) + abs(next_y - curr_y) != 1) // invalid move
				continue;
			open.push(next);
			closed[next] = true;
		}
	}
	return false;
}

void MapLoader::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	std::cout <<"Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << std::endl;
	int i, j;
	this->rows = rows + 2;
	this->cols = cols + 2;
	this->my_map = new bool[this->rows* this->cols];
	for (i=0; i< this->rows* this->cols; i++)
		this->my_map[i] = false;
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	moves_offset = new int[MapLoader::MOVE_COUNT];
	moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[MapLoader::valid_moves_t::NORTH] = -this->cols;
	moves_offset[MapLoader::valid_moves_t::EAST] = 1;
	moves_offset[MapLoader::valid_moves_t::SOUTH] = this->cols;
	moves_offset[MapLoader::valid_moves_t::WEST] = -1;

	// add padding
	i = 0;
	for (j=0; j<this->cols; j++)
		this->my_map[linearize_coordinate(i,j)] = true;
	i= this->rows-1;
	for (j=0; j<this->cols; j++)
		this->my_map[linearize_coordinate(i,j)] = true;
	j=0;
	for (i=0; i<this->rows; i++)
		this->my_map[linearize_coordinate(i,j)] = true;
	j= this->cols-1;
	for (i=0; i<this->rows; i++)
		this->my_map[linearize_coordinate(i,j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % (this->rows* this->cols);
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}

MapLoader::MapLoader(string fname, int rows = 0, int cols = 0, int obstacles = 0)
{
	string line;
	ifstream myfile (fname.c_str());
	if (myfile.is_open()) 
	{
		getline (myfile,line);
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		tokenizer< char_separator<char> >::iterator beg=tok.begin();
		int rows = atoi ( (*beg).c_str() ); // read number of rows
		beg++;
		int cols = atoi ( (*beg).c_str() ); // read number of cols
		bool* my_map= new bool[rows*cols];
		for (int i=0; i<rows*cols; i++)
			my_map[i] = false;
		// read map (and start/goal locations)
		for (int i=0; i<rows; i++) 
		{
			getline (myfile, line);
			for (int j=0; j<cols; j++)
			{
				my_map[cols*i + j] = (line[j] != '.');
			}
		}

		myfile.close();
		this->rows = rows;
		this->cols = cols;
		this->my_map = my_map;
		// initialize moves_offset array
		moves_offset = new int[MapLoader::MOVE_COUNT];
		moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
		moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
		moves_offset[MapLoader::valid_moves_t::EAST] = 1;
		moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
		moves_offset[MapLoader::valid_moves_t::WEST] = -1;
	}
	else if (rows > 0 && cols > 0 && obstacles >= 0) // generate random grid
	{
		generateConnectedRandomGrid(rows, cols, obstacles);
		saveToFile(fname);
	}
	else
	{
		cerr << "Map file " << fname << " not found." << std::endl;
		exit(10);
	}
}



void MapLoader::printMap () 
{
	for (int i = 0; i<rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (this->my_map[i * cols + j])
				std::cout << '@';
			else
				std::cout << '.';
		}
		std::cout << std::endl;
	}
}



const bool* MapLoader::get_map() const {
  return my_map;
}

const int* MapLoader::get_moves_offset() const {
	return moves_offset;
}

MapLoader::~MapLoader() {
  delete[] this->my_map;
  delete[] this->moves_offset;
}

void MapLoader::saveToFile(std::string fname) 
{
	ofstream myfile;
	myfile.open (fname);
	myfile << rows << "," << cols << endl;
	for (int i=0; i<rows; i++) 
	{
		for (int j=0; j<cols; j++) 
		{
			if ( my_map[linearize_coordinate(i,j)] == true)
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}



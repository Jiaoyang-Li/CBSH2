/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <iostream>


#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>(), "output file for schedule")
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")		
		("PC,p", po::value<bool>()->default_value(true), "conflict prioirtization")
		("heuristics,h", po::value<std::string>()->default_value("NONE"), "heuristics for the high-level search (NONE, CARDINAL,SEMI-PAIR, PAIR, MIX)")
		("wA*,w", po::value<double>(), "weight on the heuristic")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<int>()->default_value(7200), "cutoff time (seconds)")
		("MaxMDDs", po::value<int>(), "maximum number of MDDs saved for each pair of agents")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("rectangleReasoning", po::value<bool>()->default_value(false), "Using rectangle reasoning")
		("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("log,l", po::value<std::string>(), "log the heurictis at different depths of the CT tree to the file")
		("EPEA4PAIR,e", po::value<bool>()->default_value(true), "use EPEA for PAIR")
		("CGSolver", po::value<int>()->default_value(2), "solver for conflict graph (0: mvc; 1:mm; 2: wmvc)")\
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instacnes")
		("booking", po::value<bool>()->default_value(true), "booking the results of 2-agent search")
		("bookingMDDs", po::value<bool>()->default_value(false), "booking MDDs")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	// read the map file and construct its two-dim array
	MapLoader ml(vm["map"].as<string>(), vm["rows"].as<int>(), 
		vm["cols"].as<int>(), vm["obs"].as<int>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>(), vm["warehouseWidth"].as<int>());
 
	srand(vm["seed"].as<int>());

	heuristics_type h;
	if (vm["heuristics"].as<string>() == "NONE")
		h = heuristics_type::NONE;
	else if (vm["heuristics"].as<string>() == "CARDINAL")
		h = heuristics_type::CARDINAL;
	else if (vm["heuristics"].as<string>() == "PAIR")
		h = heuristics_type::PAIR;
	else if (vm["heuristics"].as<string>() == "SEMI-PAIR")
		h = heuristics_type::SEMI_PAIR;
	else if (vm["heuristics"].as<string>() == "MIX")
		h = heuristics_type::MIX;
	else
	{
		std::cout <<"WRONG HEURISTICS NAME!" << std::endl;
		return -1;
	}
	bool log = vm.count("log");
	ICBSSearch icbs(ml, al, 1.0, h, vm["PC"].as<bool>(), vm["rectangleReasoning"].as<bool>(), 
		vm["EPEA4PAIR"].as<bool>(), vm["CGSolver"].as<int>(), vm["booking"].as<bool>(),
		vm["cutoffTime"].as<int>() * 1000, vm["bookingMDDs"].as<bool>(), vm["screen"].as<int>(), log);
	if(vm.count("MaxMDDs"))
		icbs.max_num_of_mdds = vm["MaxMDDs"].as<int>();
	if (vm.count("wA*"))
		icbs.wA=vm["wA*"].as<double>();
	bool res;
	res = icbs.runICBSSearch();
	if(vm.count("output"))
		icbs.saveResults(vm["output"].as<std::string>(), vm["agents"].as<string>());
	if (log)
		icbs.saveLogs(vm["log"].as<std::string>());
	icbs.clearSearchEngines();
	return 0;

}

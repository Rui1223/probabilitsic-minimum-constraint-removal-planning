#ifndef GREEDYEXECUTEREPLANNER_H
#define GREEDYEXECUTEREPLANNER_H

#include <iostream>
#include <vector>

#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"

class GreedyExecuteReplanner_t
{
	int m_nReplan;
	double m_ExecutionTime;
	int m_pathLength;
	std::vector<int> m_ExecutedPath; // record the actual executed path

public:
	// Constructor
	GreedyExecuteReplanner_t() {}
	GreedyExecuteReplanner_t(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver);

	// function to execute a given input path
	bool execute(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver, std::vector<int> &path);
	// The function to update the map giving the transition labels
	bool updateMap(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver, std::vector<int> &labels);

	// Destrcutor
	~ExecuteReplanner_t();

};

#endif




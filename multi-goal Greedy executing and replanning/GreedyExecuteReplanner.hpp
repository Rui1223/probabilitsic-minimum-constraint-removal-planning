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
	bool m_needReplan; // indicate whether we need to replan
	bool m_isDoomed; // check whether the ground truth is doomed

public:
	// Constructor
	GreedyExecuteReplanner_t() {}
	GreedyExecuteReplanner_t(ConnectedGraph_t &g, PmcrGreedySolver_t originalGsolver);

	// function to execute a given input path
	bool execute(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver, std::vector<int> &path);
	// The function to update the map giving the transition labels
	bool updateMap(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver, std::vector<int> &labels);

	// getters
	int getnReplan() { return m_nReplan; }
	double getExecutionTime() { return m_ExecutionTime; }
	int getPathLength() { return m_pathLength; }
	std::vector<int> getExecutedPath() { return m_ExecutedPath; }
	bool getIsDoomed() { return m_isDoomed; }

	// Destrcutor
	~GreedyExecuteReplanner_t();

};

#endif




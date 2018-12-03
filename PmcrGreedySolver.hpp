/* This hpp file define the MCR problem and the greedy search algorithm applied to this problem */

#ifndef PMCRGREEDYSOLVER_H
#define PMCRGREEDYSOLVER_H

#include <queue>
#include "LabeledGraph.hpp"
#include "PmcrNode.hpp"

class PmcrGreedySolver_t
{
	// Input that a greedy solver needs
	LabeledGraph_t m_lgraph; 
	int m_start; // the id of the start node
	int m_goal; // the id of the goal node
	std::priority_queue<PmcrNode_t> m_open;
	std::vector<PmcrNode_t> m_closed;
	std::vector<int> m_path;

public:
	PmcrGreedySolver_t(LabeledGraph_t g, int start, int goal);
	bool greedy_search();
	void back_track_path();
	void print_path();
	void print_test();
};

#endif
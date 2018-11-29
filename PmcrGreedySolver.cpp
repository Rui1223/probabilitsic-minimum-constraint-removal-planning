#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>

#include "PmcrNode.hpp"
#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

PmcrGreedySolver_t::PmcrGreedySolver_t(LabeledGraph_t g, int start, int goal)
{
	m_lgraph = g;
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
	m_open.push(PmcrNode_t(start, {0}, nullptr, {0.0, 0.1, 0.6, 0.3}));
}

void PmcrGreedySolver_t::greedy_search()
{
	while(!m_open.empty())
	{
		PmcrNode_t current = m_open.top();
		m_open.pop();
		m_closed.push_back(current);
		if (current.id = m_goal)
			printf("goal found!")
			// should return a path here
			//backTrackPath();
	}
}

void PmcrGreedySolver_t::printtest()
{
	printf("Check whether there is something in the closed list\n");
	for (auto &node : m_closed)
		std::cout << node.getID() << "\t" << node.getCardinality() << "\t" << node.getWeights();
		std::cout << std::endl;

}
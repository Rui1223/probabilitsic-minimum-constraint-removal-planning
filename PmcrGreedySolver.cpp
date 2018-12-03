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
	m_open.push(PmcrNode_t(start, {0}, nullptr, m_lgraph.getLabelWeights()));
	m_path = std::vector<int>();
}

bool PmcrGreedySolver_t::greedy_search()
{
	while(!m_open.empty())
	{
		PmcrNode_t current = m_open.top();
		m_open.pop();
		m_closed.push_back(current);
		if (current.getID() == m_goal)
		{
			printf("goal found!\n");
			// should return a path here
			back_track_path();
		}
		// look at each neighbor of the current node
		std::vector<int> neigbhors = m_lgraph.getNodeNeighbors()[current.getID()];
	}
}

void PmcrGreedySolver_t::back_track_path()
{
	// start from the goal
	PmcrNode_t current = m_closed[-1];
	while (current.getID() != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current.getID());
		current = current.getParent();
	} 
	// finally put the start into the path
	m_path.push_back(current.getID());
	print_path();
}

void PmcrGreedySolver_t::print_path()
{
	for (auto const &waypoint : m_path)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";
}

void PmcrGreedySolver_t::print_test()
{
	printf("Check whether there is something in the closed list\n");
	for (auto &node : m_closed)
		std::cout << node.getID() << "\t" << node.getCardinality() << "\t" << node.getWeights();
		std::cout << std::endl;

}
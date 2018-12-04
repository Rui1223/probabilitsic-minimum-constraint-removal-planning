#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>

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

void PmcrGreedySolver_t::greedy_search()
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
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors()[current.getID()];
		for (auto const &neighbor : neighbors)
		{
			// no need to come back to parent, since it will
			// always prune that parent (superset will always be pruned without check)
			if ( neighbor == current.getID() ) {continue;}
			// check neighbor's label
			std::vector<int> currentLabels = 
				label_union(current.getLabels(), 
					m_lgraph.getEdgeLabels()[current.getID()][neighbor]); 
			bool isPrune = check_prune(neighbor, currentLabels)
		}
	}
}

bool check_prune(int neighborID, std::vector<int> labels)
// This function checks whether any node in OPEN and CLOSED with the same id of the query node 
// is necessary to be pruned (including the query node). If it is, return true. Otherwise false.
{

}

std::vector<int> PmcrGreedySolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());
	return v;
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
#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <string> // std::string, std::to_string

#include "PmcrNode.hpp"
#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

PmcrGreedySolver_t::PmcrGreedySolver_t(ConnectedGraph_t &g, int start, int goal)
{
	m_lgraph = g;
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
	m_open.push(new PmcrNode_t(m_start, {}, nullptr, m_lgraph.compute_weight({})));
	m_path = std::vector<int>();
	m_lowestWeights = std::vector<double>(m_lgraph.getnNodes(), 1.0);

}

PmcrGreedySolver_t::~PmcrGreedySolver_t()
{
	while (!m_open.empty())
	{
		PmcrNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void PmcrGreedySolver_t::greedy_search()
{
	// need a list to record the lowest weight so far for each node
	m_lowestWeights[m_start] = 0.0;

	while(!m_open.empty())
	{
		PmcrNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has the smallest recorded weight
		if (current->getWeights() > m_lowestWeights[current->getID()])
		{
			// This node has been beated by some nodes with the same id but less weight
			// No need to put it into the closed list
			continue;
		}
		// This node has the smallest recorded weight
		m_currentWeight = current->getWeights();
		m_currentLabels = current->getLabels();
		m_closed.push_back(current);
		if (current->getID() == m_goal)
		{
			printf("goal found!\n");
			// print the labels & weights for the found path
			std::cout << "The weight of the path is " << m_currentWeight << "\n";
			std::cout << "<";
			for (auto const &e : m_currentLabels)
			{
				std::cout << e << " ";
			}
			std::cout << ">\n";
			// should return a path here
			back_track_path();
			return;
		}
		// look at each neighbor of the current node
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors()[current->getID()];
		for (auto const &neighbor : neighbors)
		{
			// no need to come back to parent, since it will
			// always prune that parent (superset will always be pruned without check)
			// But if current is m_start, it has no parent			
			if (current->getID() != m_start and neighbor == current->getParent()->getID()) 
			{ 
				continue; 
			}
			// check neighbor's label
			std::vector<int> currentLabels = 
				label_union(current->getLabels(), 
					m_lgraph.getEdgeLabels()[current->getID()][neighbor]);
			double currentWeight = m_lgraph.compute_weight(currentLabels);

			// check whether we need to prune this neighbor (based on weight)
			// If the neighbor has less weight, update lowest weight record put into open
			// Otherwise, discard
			if (currentWeight < m_lowestWeights[neighbor])
			{
				m_lowestWeights[neighbor] = currentWeight;
				m_open.push(new PmcrNode_t(neighbor, currentLabels, current, currentWeight));
			}
		}
	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	printf("failed to find a solution in this problem!\n");
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
	PmcrNode_t *current = m_closed[m_closed.size()-1];
	while (current->getID() != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->getID());
		current = current->getParent();
	} 
	// finally put the start into the path
	m_path.push_back(current->getID());
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

void PmcrGreedySolver_t::print_closedList()
{
	printf("Check whether there is something in the closed list\n");
	for (auto &node : m_closed)
	{
		if(node->getID() != m_start)
		{
			std::cout << node->getID() << "\t" << node->getWeights() << "\t"
				<< node->getParent()->getID();
			std::cout << std::endl;	
		}
	
	}	


}

void PmcrGreedySolver_t::write_solution(std::string file_dir, double t)
{

	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		file_ << m_start << " " << m_goal << " " << t << "\n";
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		// write the label and weight for the solution
		file_ << m_currentWeight << "\n";
		
		if (!m_currentLabels.empty())
		{
			int pp = 0;
			while (pp < m_currentLabels.size()-1)
			{
				file_ << m_currentLabels[pp] << ",";
				pp++;
			}
			file_ << m_currentLabels[pp];
		}
		file_ << "\n";
		file_.close();
	}


}
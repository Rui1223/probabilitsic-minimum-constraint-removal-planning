#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <string> // std::string, std::to_string

// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
// #include "ToyGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "PmcrNode.hpp"
#include "Timer.hpp"

PmcrGreedySolver_t::PmcrGreedySolver_t(ConnectedGraph_t &g, int start, int goal)
{
	Timer tt;
	tt.reset();
	m_lgraph = g;
	std::cout << "Time to load the graph for Gsolver: " << tt.elapsed() << " seconds\n";
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
	m_open.push(new PmcrNode_t(m_start, computeH(m_start), {}, nullptr, 
		m_lgraph.compute_survival_currentLabels({})));
	m_path = std::vector<int>();
	m_highestSurvival = std::vector<double>(m_lgraph.getnNodes(), 0.0);
	// m_expanded = std::vector<bool>(m_lgraph.getnNodes(), false);
	// m_expanded[m_start] = true;
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
	// need a list to record the highest survivability so far for each node
	m_highestSurvival[m_start] = 1.0;

	while(!m_open.empty())
	{
		PmcrNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has the highest recorded survivability
		if (current->getSurvival() < m_highestSurvival[current->getID()])
		{
			// This node has been beated by some nodes with the same id but less weight
			// No need to put it into the closed list
			delete current;
			continue;
		}
		// This node has the highest recorded survivability
		m_currentSurvival = current->getSurvival();
		m_currentLabels = current->getLabels();
		m_closed.push_back(current);

		if (current->getID() == m_goal)
		{
			printf("goal found!\n");
			// print the labels & weights for the found path
			std::cout << "The survivability of the path is " << m_currentSurvival << "\n";
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
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors(current->getID());
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
					m_lgraph.getEdgeLabels(current->getID(), neighbor));
			double currentSurvival = m_lgraph.compute_survival_currentLabels(currentLabels);


			// check whether we need to prune this neighbor (based on survivability)
			// If the neighbor has higher survivability, update the highest survivability
			// record and put into open
			// Otherwise, discard
			if (currentSurvival > m_highestSurvival[neighbor])
			{
				m_highestSurvival[neighbor] = currentSurvival;
				m_open.push(new PmcrNode_t(neighbor, computeH(neighbor), currentLabels, 
																		current, currentSurvival));
			}
		}
	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	m_currentSurvival = 0.0;
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


int PmcrGreedySolver_t::computeH(int indx)
{
	int col = m_lgraph.getnCol();
	int indx_row = indx / col;
	int indx_col = indx % col;
	int goal_row = m_goal / col;
	int goal_col = m_goal % col;
	// manhattan distance as heuristic
	return abs(indx_row - goal_row) + abs(indx_col - goal_col);
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
			std::cout << node->getID() << "\t" << node->getSurvival() << "\t"
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
		// write the label and survivability for the solution
		file_ << m_currentSurvival << "\n";
		
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
#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <fstream>
#include <string> // std::string, std::to_string
#include <cstdlib> // std::rand, std::srand

#include "ConnectedGraph.hpp"
// #include "ToyGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "PmcrNode.hpp"
#include "Timer.hpp"

// random generator function:
int myrandom (int i) { return std::rand()%i; }

PmcrGreedySolver_t::PmcrGreedySolver_t(ConnectedGraph_t &g)
{
	Timer tt;
	tt.reset();
	m_lgraph = g;
	std::cout << "Time to load the graph for Gsolver: " << tt.elapsed() << " seconds\n";
	m_start = m_lgraph.getmStart();
	m_goal = m_lgraph.getmGoal();

	// essential elements for greedy search
	m_open.push(new PmcrNode_t(m_start, computeFGH(m_start), {}, nullptr, 
		m_lgraph.compute_survival_currentLabels({})));
	m_path = std::vector<int>();
	m_highestSurvival = std::vector<double>(m_lgraph.getnNodes(), -1.0);
	m_highestSurvival[m_start] = 1.0;
	m_expanded = std::vector<bool>(m_lgraph.getnNodes(), false);
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
	

	while(!m_open.empty())
	{
		PmcrNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->getID()] == true)
		{
			// This node has been expanded with the highest survivability for its id
			// No need to put it into the closed list
			delete current;
			continue;
		}
		// get the current node's labels and survivability
		m_currentSurvival = current->getSurvival();
		m_currentLabels = current->getLabels();

		m_closed.push_back(current);
		m_expanded[current->getID()] = true;

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
		// randomly shuffle the neighbors
		std::random_shuffle( neighbors.begin(), neighbors.end(), myrandom );
		for (auto const &neighbor : neighbors)
		{			
			if (m_expanded[neighbor] == true) 
			{ 
				continue; 
			}
			// check neighbor's label and compute corresponding survivability
			std::vector<int> neighborLabels = 
				label_union(current->getLabels(), 
					m_lgraph.getEdgeLabels(current->getID(), neighbor));
			double neighborSurvival = m_lgraph.compute_survival_currentLabels(neighborLabels);

			// check whether we need to prune this neighbor (based on survivability)
			// If the neighbor has higher survivability, update the highest survivability
			// record and put into open
			// Otherwise, discard
			if (neighborSurvival > m_highestSurvival[neighbor])
			{
				m_highestSurvival[neighbor] = neighborSurvival;
				m_open.push(new PmcrNode_t(neighbor, computeFGH(neighbor), neighborLabels, 
																		current, neighborSurvival));
			}
		}
	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	// In MCR-like setting, it should not happen.
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


std::vector<int> PmcrGreedySolver_t::computeFGH(int indx)
{
	int col = m_lgraph.getnCol();
	int indx_row = indx / col;
	int indx_col = indx % col;
	int start_row = m_start / col;
	int start_col = m_start % col;
	int goal_row = m_goal / col;
	int goal_col = m_goal % col;
	// manhattan distance as distance metric
	int h = abs(indx_row - goal_row) + abs(indx_col - goal_col);
	int g = abs(indx_row - start_row) + abs(indx_col - start_col);
	int f = h + g;
	std::vector<int> fgh{f, g, h};
	return fgh;
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
		// line 1: write in time & survivability
		file_ << t << " " << m_currentSurvival << "\n";

		// line 2: write in the labels that the optimal solution carries
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

		// line 3: write in the optimal path
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		
		file_.close();
	}


}
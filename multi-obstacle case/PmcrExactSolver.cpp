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
#include "PmcrExactSolver.hpp"
#include "PmcrNode.hpp"
#include "Timer.hpp"

// random generator function:
int myrandom2 (int i) { return std::rand()%i; }

PmcrExactSolver_t::PmcrExactSolver_t(ConnectedGraph_t &g)
{
	Timer tt;
	tt.reset();
	m_start = g.getmStart();
	m_goal = g.getmGoal();
	m_col = g.getnCol();

	// essential elements for exact search
	m_G = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_G[m_start] = 0;
	m_open.push(new PmcrNode_t(m_start, m_G[m_start], computeH(m_start), {}, nullptr, 
		g.compute_survival_currentLabels({})));
	m_path = std::vector<int>();
	// m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_visited = std::vector<bool>(g.getnNodes(), false);
	m_visited[m_start] = true;
	int n_nodes = g.getnNodes();
	for (int hh=0; hh < n_nodes; hh++)
	{
		m_recordSet.push_back(std::vector<std::vector<int>>());
	}

}

PmcrExactSolver_t::~PmcrExactSolver_t()
{
	while (!m_open.empty())
	{
		PmcrNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void PmcrExactSolver_t::exact_search(ConnectedGraph_t &g)
{
	int counter1 = 0;

	while(!m_open.empty())
	{
		PmcrNode_t *current = m_open.top();
		m_open.pop();
		// // Now check if the current node has been expanded
		// if (m_expanded[current->getID()] == true)
		// {
		// 	// This node has been expanded with the highest survivability for its id
		// 	// No need to put it into the closed list
		// 	delete current;
		// 	continue;
		// }
		// get the current node's labels and survivability
		m_currentSurvival = current->getSurvival();
		m_currentLabels = current->getLabels();

		m_closed.push_back(current);
		//m_expanded[current->getID()] = true;

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
		std::vector<int> neighbors = g.getNodeNeighbors(current->getID());
		// randomly shuffle the neighbors
		std::random_shuffle( neighbors.begin(), neighbors.end(), myrandom2 );
		for (auto const &neighbor : neighbors)
		{			
			// if (m_expanded[neighbor] == true) 
			// { 
			// 	continue; 
			// }
			// check neighbor's label and compute corresponding survivability
			std::vector<int> neighborLabels = 
				label_union(current->getLabels(), 
					g.getEdgeLabels(current->getID(), neighbor));
			double neighborSurvival = g.compute_survival_currentLabels(neighborLabels);

			// check whether we need to prune this neighbor (based on label)
			// Every time we look at a neighbor, check if the labels it carries
			// is a super set of any of the set in the m_recordSet

			// The first time visited
			if (m_visited[neighbor] == false)
			{
				m_G[neighbor] = current->getG() + g.getEdgeCost(current->getID(), neighbor);
				m_open.push(new PmcrNode_t(neighbor, m_G[neighbor], computeH(neighbor), neighborLabels, 
																		current, neighborSurvival));
				m_recordSet[neighbor].push_back(neighborLabels);
				m_visited[neighbor] = true;
				continue;
			}
			else // not the first time visited
			{
				if (!check_superset(neighbor, neighborLabels))
				{
					// std::cout << "counter:" << counter1++ << "\n";
					// // print neighborLabels
					// std::cout << "<";
					// for (auto const &nl : neighborLabels)
					// {
					// 	std::cout << nl << ",";
					// }
					// std::cout << ">\n";
					// std::cout << "--------------\n";
					// // print m_record[neighbor]
					// for (auto const &labelset: m_recordSet[neighbor])
					// {
					// 	std::cout << "<";
					// 	for (auto const &l : labelset)
					// 	{
					// 		std::cout << l << " ";
					// 	}
					// 	std::cout << ">\n";
					// }
					// std::cout << "\n";			
					// You reach the same node again with a different label set (not a superset)
					m_open.push(new PmcrNode_t(neighbor, 
						current->getG() + g.getEdgeCost(current->getID(), neighbor), 
							computeH(neighbor), neighborLabels,	current, neighborSurvival));
					m_recordSet[neighbor].push_back(neighborLabels);
				}
			}

		}
	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	// In MCR-like setting, it should not happen.
	m_currentSurvival = 0.0;
	printf("failed to find a solution in this problem!\n");
}

bool PmcrExactSolver_t::check_superset(int neighbor, std::vector<int> neighborLabels)
{
	bool isSuperset = false;
	for (auto const s: m_recordSet[neighbor])
	{
		if ( check_subset(neighborLabels, s) ) { return true; }
	}

	return isSuperset;
}


std::vector<int> PmcrExactSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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


int PmcrExactSolver_t::computeH(int indx)
{
	int indx_row = indx / m_col;
	int indx_col = indx % m_col;
	int goal_row = m_goal / m_col;
	int goal_col = m_goal % m_col;
	// manhattan distance as distance metric
	int h = abs(indx_row - goal_row) + abs(indx_col - goal_col);
	// int g = abs(indx_row - start_row) + abs(indx_col - start_col);
	// int f = h + g;
	// std::vector<int> fgh{f, g, h};
	return h;
}

void PmcrExactSolver_t::back_track_path()
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


void PmcrExactSolver_t::print_path()
{
	for (auto const &waypoint : m_path)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";
}

void PmcrExactSolver_t::print_closedList()
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

void PmcrExactSolver_t::write_solution(std::string file_dir, double t)
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

bool check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}

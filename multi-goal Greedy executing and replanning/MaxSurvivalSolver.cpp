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
#include "MaxSurvivalSolver.hpp"
#include "PmcrNode.hpp"
#include "Timer.hpp"

// random generator function:
int myrandom2 (int i) { return std::rand()%i; }

MaxSurvivalSolver_t::MaxSurvivalSolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSetD, 
			std::vector<int> targetPosesD, std::map<int, std::pair<int, double>> labelWeights)
{
	// first initialize the things you update from the last replanning
	m_start = start;
	m_goalSetD = goalSetD;
	m_targetPosesD = targetPosesD;
	m_labelWeights = labelWeights;

	// these parameters never change. Just get it from the graph problem
	m_col = g.getnCol();
	m_targetObs = g.getmTargetObs();
	m_nlabelsPerObs = g.getnLabelsPerObs();
	m_nObstacles = g.getnObstacles();

	m_G = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_G[m_start] = 0;	
	m_open.push(new PmcrNode_t(m_start, m_G[m_start], computeH(m_start), {}, nullptr, 
		compute_survival_currentLabels({})));

	// need a list to record the highest survivability so far for each node
	m_highestSurvival = std::vector<double>(g.getnNodes(), -1.0);
	m_highestSurvival[m_start] = 1.0;
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_solvable = true;
}

MaxSurvivalSolver_t::~MaxSurvivalSolver_t()
{
	while (!m_open.empty())
	{
		PmcrNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void MaxSurvivalSolver_t::MaxSurvival_search(ConnectedGraph_t &g)
{
	m_goalSet = m_goalSetD;
	m_targetPoses = m_targetPosesD;

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

		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->getID()) != m_goalSet.end() )
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
			if (m_expanded[neighbor] == true) 
			{ 
				continue; 
			}
			// check neighbor's label and compute corresponding survivability
			std::vector<int> neighborLabels = 
				label_union(current->getLabels(), 
					g.getEdgeLabels(current->getID(), neighbor));
			double neighborSurvival = compute_survival_currentLabels(neighborLabels);

			// check whether we need to prune this neighbor (based on survivability)
			// If the neighbor has higher survivability, update the highest survivability
			// record and put into open
			// Otherwise, discard
			if (neighborSurvival > m_highestSurvival[neighbor])
			{
				m_highestSurvival[neighbor] = neighborSurvival;
				m_G[neighbor] = current->getG()+g.getEdgeCost(current->getID(), neighbor);
				m_open.push(new PmcrNode_t(neighbor, m_G[neighbor], 
									computeH(neighbor), neighborLabels, current, neighborSurvival));
				continue;
			}
			if (neighborSurvival == m_highestSurvival[neighbor])
			{
				if (current->getG()+g.getEdgeCost(current->getID(), neighbor) < m_G[neighbor])
				{
					m_G[neighbor] = current->getG()+g.getEdgeCost(current->getID(), neighbor);
					m_open.push(new PmcrNode_t(neighbor, m_G[neighbor], computeH(neighbor), neighborLabels, 
																			current, neighborSurvival));
				}
			}
		}

	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	// The ground truth is not solvable
	m_solvable = false;
	std::cout << "The ground truth is not solvable!\n";

	return;		
}

int MaxSurvivalSolver_t::computeH(int indx)
{
	int indx_row = indx / m_col;
	int indx_col = indx % m_col;
	// manhattan distance as distance metric	
	int goal_row;
	int goal_col;
	int min_H = std::numeric_limits<int>::max();
	int temp_h;
	// the heuristic value now is the minimum manhattan distance among all goals
	for (auto const &gs : m_goalSet)
	{
		goal_row = gs / m_col;
		goal_col = gs % m_col;
		temp_h = abs(indx_row - goal_row) + abs(indx_col - goal_col);
		if (temp_h < min_H) { min_H = temp_h; }
	}
	return min_H;
}

std::vector<int> MaxSurvivalSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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

void MaxSurvivalSolver_t::back_track_path()
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
	// std::cout << "path: \n";
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	// std::cout << "\n";

}

double MaxSurvivalSolver_t::compute_survival_currentLabels(std::vector<int> currentLabels)
{
	double currentSurvival = 1.0;
	std::vector<double> CollisionPerObs(m_nObstacles, 0.0);
	for (auto const &label : currentLabels)
	{
		CollisionPerObs[m_labelWeights[label].first] += m_labelWeights[label].second;
	}
	for (auto const &collision_prob : CollisionPerObs)
	{
		currentSurvival *= (1 - collision_prob); 
	}

	return currentSurvival;
}


void MaxSurvivalSolver_t::print_closedList()
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

void MaxSurvivalSolver_t::write_solution(std::string file_dir, double t)
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

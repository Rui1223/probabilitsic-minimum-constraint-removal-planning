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

PmcrGreedySolver_t::PmcrGreedySolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSetD, 
	std::vector<int> targetPosesD, std::map<int, std::pair<int, double>> labelWeights)
{
	// first initialize the things you update from the last replanning
	m_start = start;
	m_goalSetD = goalSetD;
	m_targetPosesD = targetPosesD;
	m_labelWeights = labelWeights;

	// these two parameters never change. Just get it from the graph problem
	m_col = g.getnCol();
	m_targetObs = g.getmTargetObs();
	m_nlabelsPerObs = g.getnLabelsPerObs();
	m_nObstacles = g.getnObstacles();

	// essential elements for greedy search
	m_G = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_G[m_start] = 0;	
	m_open.push(new PmcrNode_t(m_start, m_G[m_start], computeH(m_start), {}, nullptr, 
		compute_survival_currentLabels({})));

	m_paths = std::vector<std::vector<int>>();
	// need a list to record the highest survivability so far for each node
	m_highestSurvival = std::vector<double>(g.getnNodes(), -1.0);
	m_highestSurvival[m_start] = 1.0;
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	// Initialize 3 key variables we will be using
	m_MaxSurvival = 1.0;
	m_highestSuccess = 0.0;
	m_lowestReachability = m_highestSuccess*1.0 / m_MaxSurvival;

	m_solvable = true;
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
	std::cout << "Destroy a GreedySolver.\n";
}

void PmcrGreedySolver_t::greedy_search(ConnectedGraph_t &g)
{
	m_goalSet = m_goalSetD;
	m_targetPoses = m_targetPosesD;

	int goal_idx;
	std::vector<int>::iterator it;
	
	while(!m_goalSet.empty())
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
		// update highest_survival & lowest_reachability
		m_MaxSurvival = m_currentSurvival;
		m_lowestReachability = m_highestSuccess*1.0 / m_MaxSurvival;
		// Prune goals in goalSet which have lower reachability than what we required in order to
		// acheive a higher success rate
		prune_goalSet(g);

		m_closed.push_back(current);
		m_expanded[current->getID()] = true;

		// look at each neighbor of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->getID());
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
				m_open.push(new PmcrNode_t(neighbor, m_G[neighbor], computeH(neighbor), neighborLabels, 
																		current, neighborSurvival));
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

		// Now check whether we reach a goal
		it = std::find(m_goalSet.begin(), m_goalSet.end(), current->getID());
		// reach the goal
		if ( it != m_goalSet.end() )
		{
			std::cout << "Encounter a goal\n";
			goal_idx = std::distance(m_goalSet.begin(), it);
			// print the goal & its corresponding target pose
			std::cout << "Goal: " << current->getID() << "\n";
			std::cout << "Target Pose: " << m_targetPoses[goal_idx] << "\n";
			// check if the labels the path from start to current carries contains the label of the
			// goal you are reaching. If it is, it fails. (Since you cannot reach the goal without
			// colliding it.)
			if ( std::find(m_currentLabels.begin(), m_currentLabels.end(), m_targetPoses[goal_idx]) 
																			!= m_currentLabels.end() )
			{
				// directly prune the goal
				std::cout << "has to reach the goal while colliding with that goal pose,prune.\n\n";
				m_goalSet.erase(it);
				m_targetPoses.erase(m_targetPoses.begin()+goal_idx);
				continue;
			}
			// Otherwise it is a valid goal and let's back track of the path
			std::cout << "It's a valid goal\n";
			// update m_highestSuccess & m_lowestReachability
			m_highestSuccess = 
				m_currentSurvival * m_labelWeights[m_targetPoses[goal_idx]].second * 1.0;
			m_lowestReachability = m_highestSuccess*1.0 / m_MaxSurvival;
			// print the success rate, survivability & labels for the found path
			std::cout << "Success rate of the path: " << m_highestSuccess << "\n";
			std::cout << "Survivability of the path: " << m_currentSurvival << "\n";
			std::cout << "Labels of the path: " << "<";
			for (auto const &e : m_currentLabels)
			{
				std::cout << e << " ";
			}
			std::cout << ">\n";

			// should return a path here
			back_track_path();
			std::cout << "-----------------------------------------\n\n";

			m_optimalGoal = m_goalSet[goal_idx];
			m_optimalPose = m_targetPoses[goal_idx];
			m_optimalSurvival = m_currentSurvival;
			m_optimalLabels = m_currentLabels;

			// delete that goal from the goalSet
			m_goalSet.erase(it);
			m_targetPoses.erase(m_targetPoses.begin()+goal_idx);
			// Prune the goal set based on the latest computed m_lowestReachability
			prune_goalSet(g);
		}

	}
	// You are reaching here because there is no remaining goals
	std::cout << "We have found all possible goals!\n";
	// Three situations here
	//(1) find a feasible path (survivability != 0)
	//(2) find a doomed path (survivability == 0)
	//(3) (all goals are not qualified, return no path in m_paths)
	if (m_optimalSurvival == 0.0 or m_optimalPath.empty())
	{
		m_solvable = false;
		std::cout << "The ground truth is not solvable!\n";
	}

	return;
}

void PmcrGreedySolver_t::prune_goalSet(ConnectedGraph_t &g)
{
	int deletions = 0;
	for (int gg=0; gg < (m_goalSet.size()+deletions); gg++)
	{
		if (m_labelWeights[m_targetPoses[gg-deletions]].second < m_lowestReachability)
		{
			m_goalSet.erase(m_goalSet.begin() + gg - deletions);
			m_targetPoses.erase(m_targetPoses.begin() + gg - deletions);
			deletions++;
		}
	}
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

void PmcrGreedySolver_t::back_track_path()
{	
	std::vector<int> path_candidate;
	// start from the goal
	PmcrNode_t *current = m_closed[m_closed.size()-1];
	while (current->getID() != m_start)
	{
		// keep backtracking the path until you reach the start
		path_candidate.push_back(current->getID());
		current = current->getParent();
	} 
	// finally put the start into the path
	path_candidate.push_back(current->getID());

	// print the current path
	for (auto const &waypoint : path_candidate)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";

	m_paths.push_back(path_candidate);

	m_optimalPath = m_paths[m_paths.size()-1];
}


// void PmcrGreedySolver_t::print_path()
// {
// 	for (auto const &waypoint : m_path)
// 	{
// 		std::cout << waypoint << " ";
// 	}
// 	std::cout << "\n";
// }

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
		// first line
		file_ << t << " " << m_highestSuccess << " " << m_optimalSurvival
				<< " " << m_optimalGoal << " " << m_optimalPose << " " << m_paths.size() << "\n";

		// second line
		if (!m_optimalLabels.empty())
		{
			int pp = 0;
			while (pp < m_optimalLabels.size()-1)
			{
				file_ << m_optimalLabels[pp] << ",";
				pp++;
			}
			file_ << m_optimalLabels[pp];
		}
		file_ << "\n";

		// 3rd line: only write in the optimal path
		for (auto const &waypoint : m_optimalPath)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		file_.close();
	}
}

double PmcrGreedySolver_t::compute_survival_currentLabels(std::vector<int> currentLabels)
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
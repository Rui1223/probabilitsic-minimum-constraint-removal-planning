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
#include "AstarSolver.hpp"
#include "Timer.hpp"

// random generator function:
int myrandomAstar (int i) { return std::rand()%i; }

AstarSolver_t::AstarSolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSet, 
		std::vector<int> targetPoses, std::map<int, std::pair<int, double>> labelWeights)
{
	// first initialize the things you update from the last replanning
	m_start = start;
	m_goalSet = goalSet;
	m_targetPoses = targetPoses;
	m_labelWeights = labelWeights;

	// these parameters never change. Just get it from the graph problem
	m_col = g.getnCol();
	m_targetObs = g.getmTargetObs();
	m_nlabelsPerObs = g.getnLabelsPerObs();
	m_nObstacles = g.getnObstacles();

	// essential elements for Astar search
	m_G = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_G[m_start] = 0;	
	m_open.push(new AstarNode_t(m_start, computeH(m_start), m_G[m_start]+computeH(m_start), nullptr));
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_solvable = true;
}

AstarSolver_t::~AstarSolver_t()
{
	while (!m_open.empty())
	{
		AstarNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }	
}

void AstarSolver_t::Astar_search(ConnectedGraph_t &g)
{

	while (!m_open.empty())
	{
		AstarNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->m_id] == true)
		{
			// This node has been expanded with the highest f value for its id
			// No need to put it into the closed list
			delete current;
			continue;
		}
		m_closed.push_back(current);
		m_expanded[current->m_id] = true;

		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id) != m_goalSet.end() )
		{
			std::cout << "Goal is connected all the way to the start\n";
			back_track_path(); // construct your path
			return;
		}
		//get neighbors of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		// randomly shuffle the neighbors
		std::random_shuffle( neighbors.begin(), neighbors.end(), myrandomAstar );
		for (auto const &neighbor : neighbors)
		{
			// check if the node has been visited or extended before
			if (m_expanded[neighbor]) { continue; }
			// check whether the edge between current and neighbor collide with a true pose
			std::vector<int> edgeLabels = g.getEdgeLabels(current->m_id, neighbor);	
			if( !collision_check(g, edgeLabels) )
			{
				// No collision! a valid edge. Add the neighbor to open list
				if (m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor) < m_G[neighbor])
				{
					m_G[neighbor] = m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor);
					m_open.push(new AstarNode_t(neighbor, computeH(neighbor), m_G[neighbor]+computeH(neighbor), current));
				}
			}	

		}		

	}
	// You are reaching here since the open list is empty and the goal is not found
	// The ground truth is not solvable
	m_solvable = false;
	std::cout << "The ground truth is not solvable!\n";

	return;
}

bool AstarSolver_t::collision_check(ConnectedGraph_t &g, std::vector<int> &edgeLabels)
{
	bool isCollision = false;
	for (auto const &l : edgeLabels)
	{
		if (m_labelWeights[l].second == 1.0)
		{
			isCollision = true;
			break;
		}
	}
	return isCollision;
}

int AstarSolver_t::computeH(int indx)
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

void AstarSolver_t::back_track_path()
{
	// start from the goal
	AstarNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);

	// print the path if you want
	// std::cout << "path: \n";
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	// std::cout << "\n";
}

void AstarSolver_t::print_closedList()
{
	std::cout << "Check whether there is something in the closed list\n";
	for (auto &node : m_closed)
	{
		if(node->m_id != m_start)
		{
			std::cout << node->m_id << "\t" << node->m_parent->m_id;
			std::cout << std::endl;	
		}
	
	}
}

void AstarSolver_t::write_solution(std::string file_dir, double t)
{
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		// 1st line : time
		file_ << t << "\n";
		// 2nd line: only write in the optimal path
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		file_.close();
	}	
}
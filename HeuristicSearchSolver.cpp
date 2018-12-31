/*The hpp file declares a heuristic search solver used in fixed label search algorithm*/

#include <cassert>
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string> // std::string, std::to_string

#include "HeuristicSearchSolver.hpp"
#include "LabeledGraph.hpp"


HeuristicSearchSolver_t::HeuristicSearchSolver_t(ConnectedGraph_t &g, int start, 
	int goal, std::vector<int> &l, 
	double w) : m_lgraph(g), m_start(start), m_goal(goal), m_currentLabels(l), m_currentWeight(w)
{
	m_open.push(new HeuristicNode_t(m_start, computeH(m_start), nullptr));
}

bool HeuristicSearchSolver_t::Heuristic_search()
{
	std::vector<bool> visited(m_lgraph.getnNodes(), false);
	visited[m_start] = true;

	while (!m_open.empty())
	{
		HeuristicNode_t *current = m_open.top();
		m_open.pop();
		m_closed.push_back(current);
		if (current->m_id == m_goal)
		{
			std::cout << "Goal is connected all the way to the start\n";
			back_track_path(); // construct your path
			print_path();
			return true;
		}
		//get neighbors of the current node
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors()[current->m_id];
		for (auto const &neighbor : neighbors)
		{
			// check if the node has been visited or extended before
			if (visited[neighbor]) { continue; }
			// check whether the edge between current and neighbor node form a valid edge in the 
			// subgraph
			bool isSubset = check_subset(m_lgraph.getEdgeLabels()[current->m_id][neighbor]);
			if (isSubset)
			{
				// the neighbor is a true neighbor in the current subgraph
				// add the neighbor to open list
				m_open.push(new HeuristicNode_t(neighbor, computeH(neighbor), current));
				visited[neighbor] = true;
			} 
		}
	}
	// You are reaching here because the open list is empty and goal is not found
	std::cout << "Coundn't found the goal at the current subgraph\n";
	std::cout << "-----------------------------------------------\n";
	return false;	
}

int HeuristicSearchSolver_t::computeH(int indx)
{
	int col = m_lgraph.getnCol();
	int indx_row = indx / col;
	int indx_col = indx % col;
	int goal_row = m_goal / col;
	int goal_col = m_goal / col;
	// manhattan distance as heuristic
	return abs(indx_row - goal_row) + abs(indx_col - goal_col);
}

bool HeuristicSearchSolver_t::check_subset(std::vector<int> labels)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(m_currentLabels.begin(), m_currentLabels.end(),
						labels.begin(), labels.end()) );
}

void HeuristicSearchSolver_t::back_track_path()
{
	// start from the goal
	HeuristicNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);
}

void HeuristicSearchSolver_t::print_closedList()
{
	printf("Check whether there is something in the closed list\n");
	for (auto &node : m_closed)
	{
		if(node->m_id != m_start)
		{
			std::cout << node->m_id << "\t" << node->m_H << "\t"
				<< node->m_parent->m_id;
			std::cout << std::endl;	
		}
	}
	std::cout << "Finishing printing the closed list\n";	
}

void HeuristicSearchSolver_t::print_path()
{
	std::cout << m_path << "\n";
}


HeuristicSearchSolver_t::~HeuristicSearchSolver_t()
{
	while (!m_open.empty())
	{
		HeuristicNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v)
{
	out << "<"; 
	for (auto const &item : v)
	{
		out << item << " ";
	}
	out << ">";
	return out;
}

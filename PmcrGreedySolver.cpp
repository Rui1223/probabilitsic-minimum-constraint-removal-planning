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

PmcrGreedySolver_t::PmcrGreedySolver_t(LabeledGraph_t &g, int start, int goal)
{
	m_lgraph = g;
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
	m_open.push(new PmcrNode_t(m_start, {}, nullptr, m_lgraph.compute_weight({})));
	m_path = std::vector<int>();

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

	while(!m_open.empty())
	{
		PmcrNode_t *current = m_open.top();
		m_open.pop();
		m_currentWeight = current->getWeights();
		m_currentLabels = current->getLabels();
		m_closed.push_back(current);
		if (current->getID() == m_goal)
		{
			printf("goal found!\n");
			// print the labels & weights for the found path
			std::cout << "The weight of the path is " << current->getWeights() << "\n";
			std::cout << "<";
			for (auto const &e : current->getLabels())
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
			if ( current->getID() != m_start and neighbor == current->getParent()->getID()) 
				{continue;}
			// check neighbor's label
			std::vector<int> currentLabels = 
				label_union(current->getLabels(), 
					m_lgraph.getEdgeLabels()[current->getID()][neighbor]);
			double currentWeights = m_lgraph.compute_weight(currentLabels);

			bool isPrune = check_prune(neighbor, currentWeights);
			if (!isPrune)
			{
				// Congrats! This neighbor node stand up to all pruning tests
				// Let's add it to the open list, welcome!
				m_open.push(new PmcrNode_t(neighbor, currentLabels, current, currentWeights));

			}
		}
		// finish search all the neighbors of expanded node at the iteration
	}
	// You are reaching here because the m_open is empty and you haven't reached the goal
	printf("failed to find a solution in this problem!\n");
}

bool PmcrGreedySolver_t::check_prune(int neighborID, double weights)
// This function checks whether any node in OPEN and CLOSED with the same id of the query node 
// is necessary to be pruned (including the query node). If it is, return true. Otherwise false.
{
	bool isPrune = false;
	// check if there are nodes in CLOSED with the same neighborID
	isPrune = search_closedList(neighborID, weights, isPrune);
	if (isPrune)
	{
		return isPrune;
	}
	// now check if there are nodes in OPEN with the same neighborID
	isPrune = search_openList(neighborID, weights, isPrune);
	return isPrune;
}

bool PmcrGreedySolver_t::search_closedList(int neighborID, double weights, bool isPrune)
{
	for (auto &node : m_closed)
	{
		if (node->getID() == neighborID)
		{
			if (weights >= node->getWeights())
			{
				isPrune = true;
				return isPrune;
			}			
		}

	}
	// You are reaching here because either there is no node with the same neighborID
	// or the weight of the current neighbor is smaller than all the nodes with the 
	// same neighborID in the closed list.
	return isPrune; // should be false
}


bool PmcrGreedySolver_t::search_openList(int neighborID, double weights, bool isPrune)
{
	// search for all elements in the priority queue (annoying)
	// the APIs for priority queue are not as rich as those provided for std::vector
	while(!m_open.empty())
	{
		PmcrNode_t *node = m_open.top();
		m_open.pop(); // pop out that node so as to keep searching
		if (node->getID() == neighborID)
		{
			if (weights >= node->getWeights())
			{
				isPrune = true;
				m_open.push(node);
				push_virtualOpen();
				return isPrune;				
			}
			// if the weight of current neighbor is smaller than the node with the same neighborID
			// in open list for the first time, it must be smaller than all the nodes with
			// the same neighborID, no need for further comparison
			// In addition, the node with the same id won't show up twice
			push_virtualOpen();
			return isPrune; // should be false
		}
		// only put the node poped from open list with different ID than neighborID
		m_virtualOpen.push_back(node);
	}
	// You are reaching here since the open list is empty and you didn't find any node with
	// the same id as that of neighbor.
	push_virtualOpen();
	return isPrune; // should be false
}
void PmcrGreedySolver_t::push_virtualOpen()
{
	for (auto &e : m_virtualOpen)
	{
		m_open.push(e);
	}
	// empty virtualOpen
	m_virtualOpen.erase(m_virtualOpen.begin(), m_virtualOpen.end());
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

void PmcrGreedySolver_t::write_solution(int n)
{

	std::ofstream file_("./graph_2/GreedySearch_solution" + std::to_string(n) + ".txt");
	if (file_.is_open())
	{
		file_ << m_start << " " << m_goal << "\n";
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
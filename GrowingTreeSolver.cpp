/* This hpp files declare a solver used to solve weighted MCR problem under the assumption
1. The edge sets are connected
2. The number of labels are fixed (not sure)
*/

#include <cassert>
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstring> // std::string, std::to_string
#include <map>
//#include <cassert>
#include <functional>
#include <set>

#include "GrowingTreeSolver.hpp"
#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"
#include "ToyGraph.hpp"
#include "Timer.hpp"

GrowingTreeSolver_t::GrowingTreeSolver_t(ConnectedGraph_t &g, int start, int goal)
{
	Timer tt;
	tt.reset();
	m_lgraph = g;
	std::cout << "Time to load the graph for Gr_solver: " << tt.elapsed() << " seconds\n";
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;

	m_frontier.push(new TreeNode_t(m_start, nullptr, {}, m_lgraph.compute_weight({})));
	m_path = std::vector<int>();
	m_labelsHistory = 
		std::vector<std::vector<std::vector<int>>>(m_lgraph.getnNodes(), std::vector<std::vector<int>>());
	m_labelsHistory[m_start].push_back(std::vector<int>());
}

GrowingTreeSolver_t::~GrowingTreeSolver_t()
{
	while (!m_open.empty())
	{
		TreeNode_t* a1 = m_open.front();
		delete a1;
		m_open.pop();
	}
	while (!m_frontier.empty())
	{
		TreeNode_t* a1 = m_frontier.front();
		delete a1;
		m_frontier.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void GrowingTreeSolver_t::GrowingTreeSearch()
{
	int k = 1;
	int node_add = 1;
	// growing the search tree for each labelSet from the sorted labelMap
	for (auto const &pair: m_lgraph.getLabelMap())
	{
		m_currentLabels = pair.first;
		m_currentWeight = pair.second;
		//std::cout << "current set of labels: " << m_currentLabels << "\n";
		//std::cout << "currrent weight: " << m_currentWeight << "\n";
		//std::cout << "start the " << k << "th search\n";

		m_open = m_frontier; // The frontier nodes are the nodes to be expanded next
		// clear frontier
		m_frontier = std::queue<TreeNode_t*>();

		while(!m_open.empty())
		{
			bool putInFrontier = false;
			TreeNode_t *current = m_open.front();
			m_open.pop();

			if (current->m_id == m_goal)
			{
				// Congrats! You reach the goal!
				// The goal will go into the closed list
				m_closed.push_back(current);
				std::cout << "Found at " << k << "th search.\n"; 
				std::cout << "Goal is connected all the way to the start\n";
				std::cout << "The weight of the path is " << m_currentWeight << "\n";
				std::cout << "<";
				for (auto const &e : m_currentLabels)
				{
					std::cout << e << " ";
				}
				std::cout << ">\n";
				// should return a path here
				back_track_path();
				print_path();
				return;
			}

			// look at neighbors to decide whether the current node should be 
			// put into closed or frontier
			std::vector<int> neighbors = m_lgraph.getNodeNeighbors(current->m_id);
			for (auto const &neighbor : neighbors)
			{
				bool isPrune = false;
				// no need to come back to parent, since it will
				// always prune that parent (superset will always be pruned without check)
				// But if current is m_start, it has no parent			
				if (current->m_id != m_start and neighbor == current->m_parent->m_id) 
				{ 
					continue;
				}
				// check neighbor's label
				std::vector<int> currentLabels = 
					label_union(current->m_labels, 
						m_lgraph.getEdgeLabels(current->m_id, neighbor));
				double currentWeight = m_lgraph.compute_weight(currentLabels);

				// check whether we need to prune this neighbor based on labels 
				// (unlike greedy search where you check pruning condition by weight)
				if (currentLabels == m_currentLabels)
				{
					// check if this node appears in a subset of the currentLabels
					for (auto const &l : m_labelsHistory[neighbor])
					{
						// std::cout << "<";
						// for (auto const &e : l)
						// {
						// 	std::cout << e << ",";
						// }
						// std::cout << ">\n";
						// std::cout << "---------\n";
						if (check_subset(l))
						{
							isPrune = true;
							break; // prune

						}
					}
					if (!isPrune)
					{
						// push to open list
						m_open.push(new TreeNode_t(neighbor, current, currentLabels, currentWeight));
						node_add++;
						//std::cout << neighbor << "\n";
						// update labelsHistory
						m_labelsHistory[neighbor].push_back(currentLabels);
					}
				}
				else // which means we are at a frontier node
				{
					putInFrontier = true;
				}
			}
			//std::cout << node_add << "\n";
			//// finish checking all the neighbors ////
			if (putInFrontier)
			{
				m_frontier.push(current);
			}
			else
			{
				m_closed.push_back(current);
			}

		}
		k++;
	}
	m_currentWeight = 1.0;
	std::cout << "Couldn't find the goal...\n";
}

std::vector<int> GrowingTreeSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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

bool GrowingTreeSolver_t::check_subset(std::vector<int> labels)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	bool isSubset = std::includes(m_currentLabels.begin(), m_currentLabels.end(),
						labels.begin(), labels.end());
	return isSubset;
}

void GrowingTreeSolver_t::back_track_path()
{
	// start from the goal
	TreeNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);
}

void GrowingTreeSolver_t::print_path()
{
	for (auto const &waypoint : m_path)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";
}

void GrowingTreeSolver_t::write_solution(std::string file_dir, double t)
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

